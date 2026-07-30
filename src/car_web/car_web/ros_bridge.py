import base64
import io
import math
import threading
import time

from action_msgs.msg import GoalStatus
from car_interfaces.msg import CarMcuStatus, CarStatus
from car_interfaces.srv import EmergencyStop
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from PIL import Image
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from car_web.pose_tracking import map_odom_anchor, project_odom
from car_web.navigation_state import (
    ACTIVE_NAVIGATION_STATES,
    UNFINISHED_NAVIGATION_STATES,
    initial_navigation_state,
)


def _yaw_quaternion(yaw):
    """将平面偏航转换为 ROS 四元数"""
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _flip_top_bottom(image):
    """兼容 Ubuntu 22.04 Pillow 9 和较新 Pillow 的翻转常量"""
    transpose = getattr(Image, "Transpose", Image)
    return image.transpose(transpose.FLIP_TOP_BOTTOM)


def _message_pose(position, orientation):
    """从ROS位置和四元数读取平面位姿"""
    yaw = math.atan2(
        2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )
    return {
        "x": float(position.x),
        "y": float(position.y),
        "yaw": yaw,
    }


class CarWebBridge(Node):
    """集中维护网页需要的 ROS 状态、发布器、服务和 Nav2 action"""

    def __init__(self):
        """创建状态缓存、ROS 通信实体和周期诊断任务"""
        super().__init__("car_web")
        # 这些参数由 HTTP 进程读取，但必须先在 ROS 节点中显式声明
        self.declare_parameter("map_file", "maps/map.yaml")
        self.declare_parameter("web_host", "0.0.0.0")
        self.declare_parameter("web_port", 8000)
        self.declare_parameter("mcu_online_timeout_sec", 3.0)
        self.declare_parameter("sensor_timeout_sec", 3.0)
        self.declare_parameter("initial_pose_retry_timeout_sec", 15.0)
        self.declare_parameter("navigation_start_timeout_sec", 15.0)
        self.declare_parameter("navigation_start_retry_count", 1)
        self.declare_parameter("navigation_retry_delay_sec", 2.0)
        self.declare_parameter("wheel_distance", 0.175)
        self._mcu_timeout = self.get_parameter(
            "mcu_online_timeout_sec"
        ).value
        self._sensor_timeout = self.get_parameter("sensor_timeout_sec").value
        self._initial_pose_retry_timeout = float(
            self.get_parameter("initial_pose_retry_timeout_sec").value
        )
        self._wheel_distance = float(
            self.get_parameter("wheel_distance").value
        )
        self._lock = threading.RLock()
        self._last_mcu = 0.0
        self._last_scan = 0.0
        self._last_odom = 0.0
        self._last_move_command = 0.0
        self._last_nav_command = 0.0
        self._last_smoothed_command = 0.0
        self._last_navigation_diagnostic = 0.0
        self._last_plan = 0.0
        self._last_lidar_tcp = 0.0
        self._lidar_tcp_connected = False
        self._mcu = None
        self._status = None
        self._move_command = None
        self._nav_command = None
        self._smoothed_command = None
        self._map = None
        self._odom_pose = None
        self._last_odom_pose = 0.0
        self._map_odom_anchor = None
        self._requested_map_pose = None
        self._map_tf_authoritative = False
        self._pose_mode = "manual"
        self._last_map_tf = 0.0
        self._robot_pose = None
        self._last_robot_pose = 0.0
        self._last_amcl_pose = 0.0
        self._last_slam_pose = 0.0
        self._amcl_pose_received = False
        self._pending_initial_pose = None
        self._initial_pose_retry_deadline = 0.0
        self._nav = initial_navigation_state()
        self._goal_handle = None
        self._goal_request_future = None
        self._navigation_generation = 0
        self._waypoint_offset = 0
        self._pause_requested = False
        self._navigation_terminal_callback = None

        self._manual_publisher = self.create_publisher(
            Twist, "/cmd_vel_manual", 10
        )
        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._display_publisher = self.create_publisher(
            String, "/car/display_message", 10
        )
        self.create_subscription(
            CarMcuStatus,
            "/car/mcu_status",
            self._on_mcu,
            qos_profile_sensor_data,
        )
        self.create_subscription(CarStatus, "/car/status", self._on_status, 10)
        self.create_subscription(
            LaserScan, "/scan", self._on_scan,
            QoSProfile(
                depth=5, reliability=ReliabilityPolicy.BEST_EFFORT
            ),
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl_pose,
            QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/pose",
            self._on_slam_pose,
            QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.create_subscription(
            Twist,
            "/cmd_vel_nav",
            self._on_nav_command,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel",
            self._on_smoothed_command,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel_move",
            self._on_move_command,
            qos_profile_sensor_data,
        )
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(
            Bool,
            "/laser_tcp_connected",
            self._on_lidar_tcp,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self._on_map,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._e_stop_client = self.create_client(
            EmergencyStop, "/car/e_stop"
        )
        self._single_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self._multi_client = ActionClient(
            self, FollowWaypoints, "/follow_waypoints"
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(0.25, self._update_robot_pose)
        self.create_timer(0.5, self._retry_initial_pose)
        self.create_timer(0.5, self._publish_display_status)
        self.create_timer(1.0, self._diagnose_navigation_commands)

    def _on_mcu(self, message):
        """缓存最新 MCU 状态及其本机接收时间"""
        with self._lock:
            self._mcu = message
            self._last_mcu = time.monotonic()

    def _on_status(self, message):
        """缓存 Jetson 运动仲裁节点发布的小车状态"""
        with self._lock:
            self._status = message

    def _on_scan(self, _):
        """记录雷达扫描到达时间用于传感器就绪判断"""
        with self._lock:
            self._last_scan = time.monotonic()

    def _on_odom(self, message):
        """缓存里程计并在需要时建立地图与里程计位姿锚点"""
        pose = _message_pose(
            message.pose.pose.position,
            message.pose.pose.orientation,
        )
        now = time.monotonic()
        with self._lock:
            self._last_odom = now
            self._last_odom_pose = now
            self._odom_pose = pose
            if (
                self._map_odom_anchor is None
                and self._requested_map_pose is not None
            ):
                self._map_odom_anchor = map_odom_anchor(
                    self._requested_map_pose,
                    self._odom_pose,
                )

    @staticmethod
    def _finite_pose(pose):
        """判断平面位姿的三个数值是否全部有限"""
        return all(
            math.isfinite(float(pose[key]))
            for key in ("x", "y", "yaw")
        )

    def _accept_localization_pose(self, pose, source, now):
        """接收 AMCL 或 SLAM 位姿并同步地图里程计锚点"""
        if not self._finite_pose(pose):
            return
        localized = {
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "yaw": float(pose["yaw"]),
            "source": source,
        }
        self._robot_pose = localized
        self._requested_map_pose = {
            key: localized[key] for key in ("x", "y", "yaw")
        }
        if self._odom_pose is not None:
            self._map_odom_anchor = map_odom_anchor(
                self._requested_map_pose,
                self._odom_pose,
            )
        self._last_robot_pose = now

    def _on_amcl_pose(self, message):
        """导航模式下接收 AMCL 位姿"""
        pose = _message_pose(
            message.pose.pose.position,
            message.pose.pose.orientation,
        )
        now = time.monotonic()
        with self._lock:
            self._last_amcl_pose = now
            self._amcl_pose_received = True
            if self._pose_mode == "navigation":
                self._accept_localization_pose(pose, "amcl", now)

    def _on_slam_pose(self, message):
        """建图模式下接收 slam_toolbox 位姿"""
        pose = _message_pose(
            message.pose.pose.position,
            message.pose.pose.orientation,
        )
        now = time.monotonic()
        with self._lock:
            self._last_slam_pose = now
            if self._pose_mode == "mapping":
                self._accept_localization_pose(pose, "slam", now)

    def _on_move_command(self, message):
        """缓存发送到底盘前的最终速度"""
        with self._lock:
            self._move_command = message
            self._last_move_command = time.monotonic()

    def _on_plan(self, message):
        """记录最近一次非空全局路径的到达时间"""
        if not message.poses:
            return
        with self._lock:
            self._last_plan = time.monotonic()

    def _on_nav_command(self, message):
        """缓存局部控制器生成的原始导航速度"""
        with self._lock:
            self._nav_command = message
            self._last_nav_command = time.monotonic()

    def _on_smoothed_command(self, message):
        """缓存速度平滑器输出"""
        with self._lock:
            self._smoothed_command = message
            self._last_smoothed_command = time.monotonic()

    def _on_lidar_tcp(self, message):
        """缓存雷达 TCP 客户端连接状态"""
        with self._lock:
            self._lidar_tcp_connected = bool(message.data)
            self._last_lidar_tcp = time.monotonic()

    def _on_map(self, message):
        """缓存最新实时栅格地图"""
        with self._lock:
            self._map = message

    def _update_robot_pose(self):
        """按当前模式从 TF 或里程计锚点更新网页小车位姿"""
        with self._lock:
            use_map_tf = self._map_tf_authoritative
            pose_mode = self._pose_mode
        transform = None
        if use_map_tf:
            try:
                transform = self._tf_buffer.lookup_transform(
                    "map", "base_footprint", rclpy.time.Time()
                )
            except TransformException:
                transform = None
        now = time.monotonic()
        if transform is not None:
            with self._lock:
                self._last_map_tf = now
        if pose_mode in {"mapping", "navigation"}:
            return
        if transform is None:
            with self._lock:
                odom_fresh = (
                    self._odom_pose is not None
                    and now - self._last_odom_pose <= self._sensor_timeout
                )
                if self._map_odom_anchor is not None and odom_fresh:
                    self._robot_pose = project_odom(
                        self._map_odom_anchor,
                        self._odom_pose,
                    )
                    self._robot_pose["source"] = "odom_anchor"
                    self._last_robot_pose = now
                elif now - self._last_robot_pose > self._sensor_timeout:
                    self._robot_pose = None
            return

    def _publish_display_status(self):
        """向 ESP32 OLED 发布简短运行状态"""
        status = self.status()
        message = String()
        if not status["car_online"]:
            message.data = "等待小车"
        elif status["navigation"]["state"] == "running":
            message.data = "正在导航"
        elif status["navigation"]["state"] in {"pausing", "paused"}:
            message.data = "导航已暂停"
        elif status["sensor_ready"]:
            message.data = "小车在线"
        else:
            message.data = "等待雷达"
        self._display_publisher.publish(message)

    def _diagnose_navigation_commands(self):
        """分段检查导航速度链并节流输出中文诊断"""
        now = time.monotonic()
        with self._lock:
            if self._nav["state"] != "running":
                return
            if now - self._last_navigation_diagnostic < 5.0:
                return
            last_nav = self._last_nav_command
            last_smoothed = self._last_smoothed_command
            last_move = self._last_move_command
            last_plan = self._last_plan
            nav_command = self._nav_command
            smoothed_command = self._smoothed_command
            move_command = self._move_command
            mcu = self._mcu
            mcu_fresh = now - self._last_mcu <= self._mcu_timeout
            status = self._status
        message = ""
        nav_nonzero = self._twist_nonzero(nav_command)
        smoothed_nonzero = self._twist_nonzero(smoothed_command)
        move_nonzero = self._twist_nonzero(move_command)
        if now - last_plan > 2.0:
            message = "导航未生成全局路径，目标可能不可达"
        elif now - last_nav > 2.0 or not nav_nonzero:
            message = "全局路径已生成，局部控制器未生成有效速度"
        elif now - last_smoothed > 2.0 or not smoothed_nonzero:
            message = "局部控制器已有速度，速度平滑器未生成有效输出"
        elif now - last_move > 2.0 or not move_nonzero:
            if status is not None and status.e_stop_ok:
                message = "速度仲裁已因急停停止导航输出"
            elif not mcu_fresh:
                message = "速度仲裁已因底盘状态过期停止导航输出"
            else:
                message = "速度平滑器已有输出，速度仲裁未生成有效输出"
        elif (
            mcu_fresh
            and mcu
            and mcu.obstacle_ok
        ):
            message = "前方近障正在拦截导航前进"
        elif (
            mcu_fresh
            and mcu
            and move_nonzero
            and abs(float(mcu.left_wheel_speed)) < 0.01
            and abs(float(mcu.right_wheel_speed)) < 0.01
        ):
            message = "底盘已收到导航速度，但左右车轮仍未转动"
        if not message:
            return
        with self._lock:
            self._last_navigation_diagnostic = now
        self.get_logger().warning(message)

    @staticmethod
    def _twist_nonzero(message):
        """判断速度消息是否包含可视为运动的分量"""
        return (
            message is not None
            and (
                abs(float(message.linear.x)) >= 0.005
                or abs(float(message.angular.z)) >= 0.05
            )
        )

    def status(self):
        """生成网页状态接口使用的线程安全快照"""
        now = time.monotonic()
        with self._lock:
            mcu_fresh = now - self._last_mcu <= self._mcu_timeout
            scan_ok = now - self._last_scan <= self._sensor_timeout
            odom_ok = now - self._last_odom <= self._sensor_timeout
            lidar_tcp_connected = (
                self._lidar_tcp_connected
                and now - self._last_lidar_tcp <= self._sensor_timeout
            )
            status = self._status
            mcu = self._mcu
            move_command = getattr(self, "_move_command", None)
            move_command_fresh = (
                now - getattr(self, "_last_move_command", 0.0)
                <= self._sensor_timeout
            )
            navigation = dict(self._nav)
            hold_localized_pose = (
                getattr(self, "_pose_mode", "manual")
                in {"mapping", "navigation"}
            )
            robot_pose = (
                dict(self._robot_pose)
                if (
                    self._robot_pose
                    and (
                        hold_localized_pose
                        or now - self._last_robot_pose <= self._sensor_timeout
                    )
                )
                else None
            )
        car_online = bool(
            mcu_fresh
            and mcu
            and mcu.wifi_connect_ok
            and mcu.agent_connect_ok
        )
        if not car_online:
            connection_phase = "waiting_chassis"
        elif not lidar_tcp_connected or not scan_ok:
            connection_phase = "waiting_lidar"
        else:
            connection_phase = "ready"
        target_linear = (
            float(move_command.linear.x)
            if move_command_fresh and move_command else 0.0
        )
        target_angular = (
            float(move_command.angular.z)
            if move_command_fresh and move_command else 0.0
        )
        left_speed = (
            float(getattr(mcu, "left_wheel_speed", 0.0))
            if mcu_fresh else 0.0
        )
        right_speed = (
            float(getattr(mcu, "right_wheel_speed", 0.0))
            if mcu_fresh else 0.0
        )
        wheel_distance = max(
            float(getattr(self, "_wheel_distance", 0.175)), 0.001
        )
        measured_linear = (left_speed + right_speed) * 0.5
        measured_angular = (right_speed - left_speed) / wheel_distance
        return {
            "car_online": car_online,
            "lidar_tcp_connected": lidar_tcp_connected,
            "connection_phase": connection_phase,
            "scan_ok": scan_ok,
            "odom_ok": odom_ok,
            "sensor_ready": scan_ok and odom_ok,
            "e_stop": bool(status.e_stop_ok) if status else False,
            "obstacle": bool(mcu.obstacle_ok) if mcu else False,
            "obstacle_distance": (
                float(mcu.obstacle_distance) if mcu else 0.0
            ),
            "fault_bits": int(status.fault_bits) if status else 0,
            "navigation": navigation,
            "robot_pose": robot_pose,
            "motion": {
                "target_linear": target_linear,
                "target_angular": target_angular,
                "measured_linear": measured_linear,
                "measured_angular": measured_angular,
                "left_wheel_speed": left_speed,
                "right_wheel_speed": right_speed,
                "command_fresh": move_command_fresh,
                "feedback_fresh": mcu_fresh,
            },
        }

    def stack_state(self):
        """根据 ROS 节点名称判断建图和导航栈是否存在"""
        names = set(self.get_node_names())
        return {
            "mapping": any("slam_toolbox" in name for name in names),
            "navigation": any(
                name in names
                for name in {"amcl", "controller_server", "bt_navigator"}
            ),
        }

    def publish_move(self, linear, angular):
        """向手动速度话题发布米每秒和弧度每秒命令"""
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self._manual_publisher.publish(message)

    def _publish_initial_pose(self, pose):
        """发布一帧带协方差的地图初始位姿"""
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.position.x = float(pose["x"])
        message.pose.pose.position.y = float(pose["y"])
        quaternion = _yaw_quaternion(float(pose["yaw"]))
        (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        ) = quaternion
        message.pose.covariance[0] = 0.04
        message.pose.covariance[7] = 0.04
        message.pose.covariance[35] = 0.0305
        self._initial_pose_publisher.publish(message)

    def set_initial_pose(self, pose, retry=False):
        """更新网页位姿锚点并按需重复发送初始位姿"""
        requested = {
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "yaw": float(pose["yaw"]),
        }
        now = time.monotonic()
        with self._lock:
            self._requested_map_pose = requested
            if self._odom_pose is not None:
                self._map_odom_anchor = map_odom_anchor(
                    requested,
                    self._odom_pose,
                )
            displayed = dict(requested)
            displayed["source"] = "odom_anchor"
            self._robot_pose = displayed
            self._last_robot_pose = now
            self._pending_initial_pose = dict(requested) if retry else None
            self._initial_pose_retry_deadline = (
                now + self._initial_pose_retry_timeout
                if retry else 0.0
            )
        self._publish_initial_pose(requested)

    def _retry_initial_pose(self):
        """在超时前重发初始位姿直到 AMCL 确认"""
        now = time.monotonic()
        with self._lock:
            pose = (
                dict(self._pending_initial_pose)
                if self._pending_initial_pose is not None
                else None
            )
            deadline = self._initial_pose_retry_deadline
        if pose is None:
            return
        if self.localization_ready():
            with self._lock:
                self._pending_initial_pose = None
                self._initial_pose_retry_deadline = 0.0
            self.get_logger().info("AMCL已确认当前位置，停止重复发送初始位姿")
            return
        if now >= deadline:
            with self._lock:
                self._pending_initial_pose = None
                self._initial_pose_retry_deadline = 0.0
            self.get_logger().warning("AMCL在限定时间内未确认当前位置")
            return
        self._publish_initial_pose(pose)

    def stop_initial_pose_retry(self):
        """停止尚未完成的初始位姿重发"""
        with self._lock:
            self._pending_initial_pose = None
            self._initial_pose_retry_deadline = 0.0

    def begin_localization(self):
        """切换到以 AMCL 和 map TF 为准的导航位姿模式"""
        with self._lock:
            self._map_tf_authoritative = True
            self._pose_mode = "navigation"
            self._last_map_tf = 0.0
            self._last_amcl_pose = 0.0
            self._amcl_pose_received = False

    def end_localization(self):
        """退出导航位姿模式并清理初始位姿重试状态"""
        with self._lock:
            self._map_tf_authoritative = False
            self._pose_mode = "manual"
            self._last_map_tf = 0.0
            self._last_amcl_pose = 0.0
            self._amcl_pose_received = False
            self._pending_initial_pose = None
            self._initial_pose_retry_deadline = 0.0

    def begin_mapping(self):
        """清理旧地图状态并切换到 SLAM 位姿模式"""
        with self._lock:
            self._map_tf_authoritative = True
            self._pose_mode = "mapping"
            self._last_map_tf = 0.0
            self._map = None
            self._map_odom_anchor = None
            self._requested_map_pose = None
            self._robot_pose = None
            self._last_robot_pose = 0.0
            self._last_amcl_pose = 0.0
            self._last_slam_pose = 0.0
            self._amcl_pose_received = False
            self._pending_initial_pose = None
            self._initial_pose_retry_deadline = 0.0

    def end_mapping(self):
        """退出 SLAM 位姿模式并恢复手动里程计跟踪"""
        with self._lock:
            self._map_tf_authoritative = False
            self._pose_mode = "manual"
            self._last_map_tf = 0.0

    def emergency_stop(self, stop, reason="Web操作"):
        """同步调用急停服务并返回底层处理结果"""
        if not self._e_stop_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("急停服务尚未就绪")
        request = EmergencyStop.Request()
        request.stop = bool(stop)
        request.reason = str(reason)
        request.source = "car_web"
        future = self._e_stop_client.call_async(request)
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done() or future.result() is None:
            raise RuntimeError("急停服务调用超时")
        return {
            "success": bool(future.result().success),
            "message": future.result().message,
        }

    def _pose_message(self, pose):
        """把网页平面位姿转换为 map 坐标系 PoseStamped"""
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = float(pose["x"])
        message.pose.position.y = float(pose["y"])
        quaternion = _yaw_quaternion(float(pose.get("yaw", 0.0)))
        (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ) = quaternion
        return message

    def navigation_ready(self):
        """检查单点、多点 action 和 map TF 是否全部就绪"""
        return (
            self._single_client.server_is_ready()
            and self._multi_client.server_is_ready()
            and self.map_transform_ready()
        )

    def map_transform_ready(self):
        """判断最近一次 map 到底盘 TF 是否仍然新鲜"""
        now = time.monotonic()
        with self._lock:
            return now - self._last_map_tf <= self._sensor_timeout

    def localization_ready(self):
        """判断 AMCL 位姿与 map TF 是否共同确认定位完成"""
        now = time.monotonic()
        with self._lock:
            amcl_confirmed = (
                self._amcl_pose_received
                and now - self._last_amcl_pose <= self._sensor_timeout
            )
            pose_fresh = (
                self._robot_pose is not None
                and self._robot_pose.get("source") == "amcl"
                and now - self._last_robot_pose <= self._sensor_timeout
            )
        return amcl_confirmed and self.map_transform_ready() and pose_fresh

    def _navigation_error(self, message, generation=None):
        """只为当前代次写入导航失败或暂停状态"""
        terminal_state = "failed"
        with self._lock:
            if (
                generation is not None
                and generation != self._navigation_generation
            ):
                return
            self._goal_handle = None
            self._goal_request_future = None
            if self._pause_requested:
                self._nav["state"] = "paused"
                self._nav["message"] = ""
                terminal_state = "paused"
            else:
                self._nav["state"] = "failed"
                self._nav["message"] = str(message)
        self._notify_navigation_terminal(terminal_state)

    def set_navigation_terminal_callback(self, callback):
        """注册导航进入终态后的位姿记录回调"""
        with self._lock:
            self._navigation_terminal_callback = callback

    def _notify_navigation_terminal(self, state):
        """安全调用导航终态回调而不影响 action 状态机"""
        with self._lock:
            callback = self._navigation_terminal_callback
        if callback is None:
            return
        try:
            callback(str(state))
        except Exception as error:
            self.get_logger().warning(
                f"导航终态位姿记录回调失败：{error}"
            )

    def navigate(self, poses, waypoint_offset=0, total_waypoints=None):
        """按目标数量选择单点或多点 action 并异步发送"""
        if not poses:
            raise RuntimeError("目标队列为空")
        waypoint_offset = max(int(waypoint_offset), 0)
        full_total = max(
            int(
                total_waypoints
                if total_waypoints is not None
                else waypoint_offset + len(poses)
            ),
            waypoint_offset + len(poses),
        )
        with self._lock:
            if self._nav["state"] in ACTIVE_NAVIGATION_STATES:
                raise RuntimeError("已有导航任务正在执行")
            historical_missed = sorted(
                {
                    int(index)
                    for index in self._nav.get("missed_waypoints", [])
                    if 0 <= int(index) < waypoint_offset
                }
            )
            self._navigation_generation += 1
            generation = self._navigation_generation
            self._waypoint_offset = waypoint_offset
            self._pause_requested = False
            self._last_plan = 0.0
            self._last_nav_command = 0.0
            self._last_smoothed_command = 0.0
            self._last_move_command = 0.0
            self._nav = {
                "state": "sending",
                "message": "",
                "current_waypoint": waypoint_offset,
                "total_waypoints": full_total,
                "completed_waypoints": waypoint_offset,
                "missed_waypoints": historical_missed,
            }
        try:
            if len(poses) == 1:
                if not self._single_client.wait_for_server(timeout_sec=5.0):
                    raise RuntimeError("NavigateToPose action尚未就绪")
                goal = NavigateToPose.Goal()
                goal.pose = self._pose_message(poses[0])
                future = self._single_client.send_goal_async(
                    goal,
                    feedback_callback=lambda feedback: (
                        self._single_feedback(feedback, generation)
                    ),
                )
            else:
                if not self._multi_client.wait_for_server(timeout_sec=5.0):
                    raise RuntimeError("FollowWaypoints action尚未就绪")
                goal = FollowWaypoints.Goal()
                goal.poses = [self._pose_message(pose) for pose in poses]
                future = self._multi_client.send_goal_async(
                    goal,
                    feedback_callback=lambda feedback: (
                        self._multi_feedback(feedback, generation)
                    ),
                )
        except Exception as error:
            self._navigation_error(error, generation)
            if isinstance(error, RuntimeError):
                raise
            raise RuntimeError(f"导航目标发送失败：{error}") from error
        with self._lock:
            self._goal_request_future = future
        future.add_done_callback(
            lambda result: self._goal_response(result, generation)
        )

    def _single_feedback(self, _, generation=None):
        """更新单点导航的当前目标进度"""
        with self._lock:
            if (
                generation is not None
                and generation != self._navigation_generation
            ):
                return
            self._nav["current_waypoint"] = self._waypoint_offset
            self._nav["completed_waypoints"] = self._waypoint_offset

    def _multi_feedback(self, feedback, generation=None):
        """把 FollowWaypoints 相对序号转换为完整队列序号"""
        with self._lock:
            if (
                generation is not None
                and generation != self._navigation_generation
            ):
                return
            current = self._waypoint_offset + int(
                feedback.feedback.current_waypoint
            )
            self._nav["current_waypoint"] = current
            self._nav["completed_waypoints"] = current

    def _goal_response(self, future, generation=None):
        """处理 Nav2 接受或拒绝目标的异步结果"""
        with self._lock:
            if (
                generation is not None
                and generation != self._navigation_generation
            ):
                return
        try:
            goal_handle = future.result()
        except Exception as error:
            self._navigation_error(
                f"导航目标发送失败：{error}", generation
            )
            return
        with self._lock:
            self._goal_request_future = None
        if goal_handle is None or not goal_handle.accepted:
            terminal_state = "rejected"
            with self._lock:
                if self._pause_requested:
                    self._nav["state"] = "paused"
                    self._nav["message"] = ""
                    terminal_state = "paused"
                else:
                    self._nav["state"] = "rejected"
                    self._nav["message"] = "导航目标被拒绝"
            self._notify_navigation_terminal(terminal_state)
            return
        with self._lock:
            self._goal_handle = goal_handle
            canceling = self._nav["state"] in {"canceling", "pausing"}
            if not canceling:
                self._nav["state"] = "running"
        if canceling:
            goal_handle.cancel_goal_async()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._navigation_result(result, generation)
        )

    def _navigation_result(self, future, generation=None):
        """归并 Nav2 终态和失败点并更新网页导航状态"""
        with self._lock:
            if (
                generation is not None
                and generation != self._navigation_generation
            ):
                return
        try:
            wrapped = future.result()
        except Exception as error:
            self._navigation_error(
                f"导航结果读取失败: {error}", generation
            )
            return
        if wrapped is None:
            self._navigation_error("导航结果为空", generation)
            return
        missed = []
        if hasattr(wrapped.result, "missed_waypoints"):
            missed = [
                self._waypoint_offset + int(index)
                for index in wrapped.result.missed_waypoints
            ]
        state = (
            "succeeded"
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED
            else "canceled"
            if wrapped.status == GoalStatus.STATUS_CANCELED
            else "failed"
        )
        with self._lock:
            self._goal_handle = None
            self._goal_request_future = None
            if state != "succeeded" and self._pause_requested:
                state = "paused"
            if state == "failed" and self._nav["total_waypoints"] > 0:
                failed_index = min(
                    self._nav["current_waypoint"],
                    self._nav["total_waypoints"] - 1,
                )
                missed.append(failed_index)
                self._nav["completed_waypoints"] = max(
                    self._nav["completed_waypoints"],
                    failed_index + 1,
                )
            missed = sorted(
                {
                    *self._nav.get("missed_waypoints", []),
                    *missed,
                }
            )
            if state == "succeeded" and missed:
                state = "partial"
            self._nav["state"] = state
            self._nav["missed_waypoints"] = missed
            if (
                state in {"succeeded", "partial"}
                and self._nav["total_waypoints"] > 0
            ):
                self._nav["current_waypoint"] = (
                    self._nav["total_waypoints"] - 1
                )
                self._nav["completed_waypoints"] = (
                    self._nav["total_waypoints"]
                )
            self._nav["message"] = (
                f"失败点: {[index + 1 for index in missed]}"
                if missed else ""
            )
            self._pause_requested = state == "paused"
        labels = {
            "succeeded": "导航任务已完成",
            "partial": "导航任务部分完成",
            "canceled": "导航任务已取消",
            "paused": "导航任务已暂停",
            "failed": "导航任务执行失败",
        }
        message = labels.get(state, "导航任务状态已更新")
        if state in {"failed", "partial"}:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)
        self._notify_navigation_terminal(state)

    def pause_navigation(self):
        """急停时取消当前 action 但保留队列进度"""
        with self._lock:
            goal_handle = self._goal_handle
            request_pending = self._goal_request_future is not None
            state = self._nav["state"]
            if state == "paused":
                self.publish_move(0.0, 0.0)
                return True
            active = state in {"sending", "running", "pausing"}
            if active:
                self._pause_requested = True
                self._nav["state"] = "pausing"
                self._nav["message"] = ""
        if not active:
            self.publish_move(0.0, 0.0)
            return False
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        self.publish_move(0.0, 0.0)
        return goal_handle is not None or request_pending

    def cancel_navigation(self):
        """取消当前 action 并保持目标数据供上层决定是否清除"""
        terminal_state = None
        with self._lock:
            goal_handle = self._goal_handle
            request_pending = self._goal_request_future is not None
            active = self._nav["state"] in UNFINISHED_NAVIGATION_STATES
            if active:
                self._pause_requested = False
                self._nav["state"] = "canceling"
                self._nav["message"] = ""
        if goal_handle is None and not request_pending:
            with self._lock:
                if active:
                    self._nav["state"] = "canceled"
                    terminal_state = "canceled"
            self.publish_move(0.0, 0.0)
            if terminal_state is not None:
                self._notify_navigation_terminal(terminal_state)
            return False
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        self.publish_move(0.0, 0.0)
        return True

    def reset_navigation(self):
        """使旧异步回调失效并恢复初始导航状态"""
        with self._lock:
            self._navigation_generation += 1
            self._goal_handle = None
            self._goal_request_future = None
            self._waypoint_offset = 0
            self._pause_requested = False
            self._nav = initial_navigation_state()

    def resize_navigation_queue(self, total_waypoints):
        """目标增删后约束现有进度和失败序号"""
        total = max(int(total_waypoints), 0)
        with self._lock:
            completed = max(
                int(self._nav.get("completed_waypoints", 0)),
                0,
            )
            current = max(
                int(self._nav.get("current_waypoint", 0)),
                0,
            )
            self._nav["total_waypoints"] = total
            self._nav["completed_waypoints"] = min(completed, total)
            self._nav["current_waypoint"] = min(
                current,
                max(total - 1, 0),
            )
            self._nav["missed_waypoints"] = [
                index
                for index in self._nav.get("missed_waypoints", [])
                if 0 <= int(index) < total
            ]

    def robot_pose(self):
        """返回当前可显示位姿，手动模式下拒绝过期数据"""
        now = time.monotonic()
        with self._lock:
            hold_localized_pose = (
                getattr(self, "_pose_mode", "manual")
                in {"mapping", "navigation"}
            )
            if (
                not self._robot_pose
                or (
                    not hold_localized_pose
                    and now - self._last_robot_pose > self._sensor_timeout
                )
            ):
                return None
            return dict(self._robot_pose)

    def map_payload(self):
        """把 OccupancyGrid 编码为浏览器可直接显示的 PNG 数据"""
        with self._lock:
            message = self._map
        robot_pose = self.robot_pose()
        if message is None:
            return {"available": False, "robot_pose": robot_pose}
        width = int(message.info.width)
        height = int(message.info.height)
        resolution = float(message.info.resolution)
        if width <= 0 or height <= 0:
            return {
                "available": False,
                "robot_pose": robot_pose,
                "reason": "地图尺寸无效",
            }
        if not math.isfinite(resolution) or resolution <= 0.0:
            return {
                "available": False,
                "robot_pose": robot_pose,
                "reason": "地图分辨率无效",
            }
        if len(message.data) != width * height:
            return {
                "available": False,
                "robot_pose": robot_pose,
                "reason": "地图数据长度与尺寸不一致",
            }
        pixels = bytearray()
        for value in message.data:
            pixels.append(205 if value < 0 else 0 if value >= 65 else 254)
        try:
            image = Image.frombytes("L", (width, height), bytes(pixels))
            image = _flip_top_bottom(image)
            buffer = io.BytesIO()
            image.save(buffer, format="PNG")
        except (AttributeError, OSError, ValueError) as error:
            self.get_logger().error(f"地图图像编码失败：{error}")
            return {
                "available": False,
                "robot_pose": robot_pose,
                "reason": "地图图像编码失败",
            }
        return {
            "available": True,
            "source": "live",
            "image": "data:image/png;base64," + base64.b64encode(
                buffer.getvalue()
            ).decode("ascii"),
            "width": width,
            "height": height,
            "resolution": resolution,
            "origin": {
                "x": float(message.info.origin.position.x),
                "y": float(message.info.origin.position.y),
            },
            "robot_pose": robot_pose,
        }
