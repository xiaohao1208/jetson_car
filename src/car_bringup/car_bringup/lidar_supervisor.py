import os
import signal
import subprocess
import time
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from car_interfaces.msg import CarMcuStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class LidarSupervisorNode(Node):
    """根据雷达 TCP 状态、本地虚拟串口和 /scan 新鲜度管理 ydlidar_node

    雷达板和 ESP32 底盘是两条独立链路。雷达链路遵循书上的流程：
    雷达串口 -> 雷达转接板 WiFi TCP -> laser_tcp_server -> /tmp/tty_laser
    -> ydlidar_node -> /scan

    /tmp/tty_laser 可以在 TCP 客户端接入前先创建，但默认必须等雷达 TCP
    客户端接入后再启动 ydlidar_node。否则 SDK 会打开空的伪串口并反复报
    Device Failed
    """

    def __init__(self):
        """读取监督参数并创建雷达、扫描和底盘状态输入"""
        super().__init__("lidar_supervisor")

        # car_bringup 安装目录中的默认 YDLIDAR 参数文件
        default_params_file = os.path.join(
            get_package_share_directory("car_bringup"), "config", "ydlidar_car.yaml"
        )
        self.declare_parameter("laser_connected_topic", "/laser_tcp_connected")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("mcu_status_topic", "/car/mcu_status")
        self.declare_parameter("ydlidar_params_file", default_params_file)
        self.declare_parameter("serial_port", "/tmp/tty_laser")
        self.declare_parameter("serial_wait_timeout_sec", 10.0)
        self.declare_parameter("require_tcp_before_start", True)
        self.declare_parameter("chassis_first_startup", True)
        self.declare_parameter("chassis_first_timeout_sec", 20.0)
        self.declare_parameter("tcp_state_timeout_sec", 3.0)
        self.declare_parameter("start_delay_sec", 1.0)
        self.declare_parameter("initial_scan_timeout_sec", 20.0)
        self.declare_parameter("scan_timeout_sec", 5.0)
        self.declare_parameter("restart_backoff_sec", 3.0)
        self.declare_parameter("max_restart_backoff_sec", 20.0)
        self.declare_parameter("stop_timeout_sec", 3.0)

        # 雷达 TCP 连接状态话题名称
        self._laser_connected_topic = (
            self.get_parameter("laser_connected_topic")
            .get_parameter_value().string_value
        )
        # 雷达扫描话题名称
        self._scan_topic = (
            self.get_parameter("scan_topic")
            .get_parameter_value().string_value
        )
        # ESP32 原始状态话题名称
        self._mcu_status_topic = (
            self.get_parameter("mcu_status_topic")
            .get_parameter_value().string_value
        )
        # 启动第三方 ydlidar_node 使用的参数文件
        self._ydlidar_params_file = (
            self.get_parameter("ydlidar_params_file").get_parameter_value().string_value
        )
        # laser_tcp_server 创建的本地虚拟串口路径
        self._serial_port = (
            self.get_parameter("serial_port")
            .get_parameter_value().string_value
        )
        # 虚拟串口长时间未出现后日志升级为警告的等待时长
        self._serial_wait_timeout = (
            self.get_parameter("serial_wait_timeout_sec").get_parameter_value().double_value
        )
        # 启动 ydlidar_node 前是否必须确认雷达 TCP 已连接
        self._require_tcp_before_start = (
            self.get_parameter("require_tcp_before_start").get_parameter_value().bool_value
        )
        # 是否优先等待 ESP32 底盘连接完成
        self._chassis_first_startup = (
            self.get_parameter("chassis_first_startup").get_parameter_value().bool_value
        )
        # 底盘优先门控的可选放行超时，零表示永不自动放行
        self._chassis_first_timeout = (
            self.get_parameter("chassis_first_timeout_sec").get_parameter_value().double_value
        )
        # 雷达 TCP Bool 状态的最大可信年龄
        self._tcp_state_timeout = (
            self.get_parameter("tcp_state_timeout_sec").get_parameter_value().double_value
        )
        # TCP 和串口就绪后启动驱动前的稳定等待时长
        self._start_delay = (
            self.get_parameter("start_delay_sec")
            .get_parameter_value().double_value
        )
        # 首次启动驱动后等待第一帧扫描的最长时间
        self._initial_scan_timeout = (
            self.get_parameter("initial_scan_timeout_sec").get_parameter_value().double_value
        )
        # 正常运行后允许扫描中断的最长时间
        self._scan_timeout = (
            self.get_parameter("scan_timeout_sec")
            .get_parameter_value().double_value
        )
        # 驱动首次重启退避时长
        self._base_restart_backoff = (
            self.get_parameter("restart_backoff_sec").get_parameter_value().double_value
        )
        # 连续重启失败时允许增长到的最大退避时长
        self._max_restart_backoff = (
            self.get_parameter("max_restart_backoff_sec").get_parameter_value().double_value
        )
        # SIGINT 后等待 ydlidar_node 正常退出的时间
        self._stop_timeout = (
            self.get_parameter("stop_timeout_sec")
            .get_parameter_value().double_value
        )

        # 最近一次 /laser_tcp_connected 的连接值
        self._laser_connected = False
        # 最近一次 /laser_tcp_connected 的接收时间，用于判断 TCP 状态是否新鲜
        self._last_laser_state_time: Optional[float] = None
        # 最近一次收到 /scan 的 monotonic 时间，用于运行中超时重启
        self._last_scan_time: Optional[float] = None
        # 当前 ydlidar_node 子进程句柄，None 表示驱动未运行
        self._process: Optional[subprocess.Popen] = None
        # ydlidar_node 启动时间，用于区分“启动后没出第一帧”和“运行中断流”
        self._process_started_at: Optional[float] = None
        # 本次驱动启动后是否见过 /scan，决定使用 initial_scan_timeout 还是 scan_timeout
        self._scan_seen_since_start = False
        # supervisor 自身启动时间，用于底盘优先门控的超时判断
        self._node_started_at = time.monotonic()
        # 如果不要求底盘优先，则初始化为 True，监督循环会直接检查串口和 TCP
        self._chassis_seen = not self._chassis_first_startup
        # 延迟启动 ydlidar_node 的目标时间，避免 TCP 刚连上时马上发送初始化命令
        self._pending_start_at: Optional[float] = None
        # 连续重启失败时逐步增加等待时间，避免刷屏和频繁占用串口
        self._current_restart_backoff = max(0.1, self._base_restart_backoff)
        # 上一次记录的 TCP 就绪状态，用于仅在变化时输出日志
        self._last_connected_state: Optional[bool] = None
        # 最近一次虚拟串口等待日志时间
        self._last_serial_wait_log_time = 0.0
        # 最近一次底盘门控等待日志时间
        self._last_chassis_wait_log_time = 0.0
        # 最近一次雷达 TCP 等待日志时间
        self._last_tcp_wait_log_time = 0.0
        # 上一次记录的虚拟串口存在状态
        self._last_serial_ready_state: Optional[bool] = None

        self.create_subscription(
            Bool,
            self._laser_connected_topic,
            self._laser_connected_callback,
            QoSProfile(depth=10),
        )
        self.create_subscription(
            LaserScan,
            self._scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CarMcuStatus,
            self._mcu_status_topic,
            self._mcu_status_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(0.5, self._supervise)

        self.get_logger().info(
            "雷达监督节点已启动: 串口 %s，TCP状态 %s，驱动参数 %s，底盘优先=%s，启动前要求TCP=%s"
            % (
                self._serial_port,
                self._laser_connected_topic,
                self._ydlidar_params_file,
                self._chassis_first_startup,
                self._require_tcp_before_start,
            )
        )

    def destroy_node(self):
        """先停止受管雷达驱动再销毁 ROS 节点"""
        self._stop_driver("节点退出，停止 ydlidar_node")
        super().destroy_node()

    def _laser_connected_callback(self, msg: Bool):
        """记录雷达 TCP 状态和本机接收时间"""
        # laser_tcp_server 周期发布该状态，这里记录接收时间而不是消息时间戳
        self._laser_connected = bool(msg.data)
        self._last_laser_state_time = time.monotonic()

    def _scan_callback(self, _: LaserScan):
        """记录最新扫描并在首帧到达时清除重启退避"""
        # 收到本帧扫描的单调时间
        now = time.monotonic()
        self._last_scan_time = now
        if self._process is not None and not self._scan_seen_since_start:
            # 第一帧 /scan 说明雷达链路真正可用，重启退避恢复到初始值
            self._scan_seen_since_start = True
            self._current_restart_backoff = max(0.1, self._base_restart_backoff)
            self.get_logger().info("已收到 /scan，雷达驱动进入正常工作状态")

    def _mcu_status_callback(self, message: CarMcuStatus):
        """在底盘 WiFi 与 Agent 同时正常时永久打开首次门控"""
        if self._chassis_seen:
            return
        # 内部门控仍要求 WiFi 和 Agent 都正常
        if not message.wifi_connect_ok or not message.agent_connect_ok:
            return
        self._chassis_seen = True
        self.get_logger().info(
            "底盘 WiFi 和 micro-ROS Agent 均正常，允许启动雷达驱动"
        )

    def _tcp_connected_recent(self, now: float) -> bool:
        """判断雷达 TCP 在线状态是否仍在可信时间窗口内"""
        if not self._laser_connected or self._last_laser_state_time is None:
            return False
        return now - self._last_laser_state_time <= self._tcp_state_timeout

    def _chassis_gate_open(self, now: float) -> bool:
        """判断底盘门控已满足或调试超时已经放行"""
        if self._chassis_seen:
            return True
        if self._chassis_first_timeout <= 0.0:
            return False
        # 单独调试雷达时可设置超时放行，整车默认不走这个分支
        if now - self._node_started_at >= self._chassis_first_timeout:
            self._chassis_seen = True
            self.get_logger().warn(
                f"{self._chassis_first_timeout:.1f}s 内未收到 /car/mcu_status，"
                "放开雷达驱动启动以便单独调试雷达"
            )
            return True
        return False

    def _serial_ready(self, now: float) -> bool:
        """检查伪串口路径并对等待日志进行节流"""
        if os.path.exists(self._serial_port):
            # 串口路径存在只说明 TCP 桥创建了伪终端，不代表雷达板已经 TCP 接入
            if self._last_serial_ready_state is not True:
                self._last_serial_ready_state = True
                self.get_logger().info(f"本地雷达串口已就绪：{self._serial_port}")
            return True

        if self._last_serial_ready_state is not False:
            # 第一次发现串口不存在时立即提示，后续按 10 秒节流
            self._last_serial_ready_state = False
            self._last_serial_wait_log_time = now
            self.get_logger().warn(
                f"本地雷达串口不存在：{self._serial_port}，等待雷达桥接服务创建虚拟串口"
            )
        elif now - self._last_serial_wait_log_time > 10.0:
            self._last_serial_wait_log_time = now
            # 监督节点启动后等待串口的累计时间
            waited = now - self._node_started_at
            # 等待超过配置阈值后使用警告级别，否则使用普通信息级别
            level = (
                self.get_logger().warn
                if waited >= self._serial_wait_timeout
                else self.get_logger().info
            )
            level(
                f"等待本地雷达串口 {self._serial_port}，"
                "它应由雷达桥接服务在启动时创建"
            )
        return False

    def _supervise(self):
        """按固定顺序监督子进程、底盘、TCP、串口和扫描新鲜度"""
        # 监督循环按固定顺序检查：子进程退出 -> TCP 状态 -> 底盘门控 -> 串口 -> 扫描超时
        # 本轮监督判断使用的单调时间
        now = time.monotonic()
        self._reap_driver(now)

        # 雷达 TCP 状态值和接收时间是否都可信
        tcp_ready = self._tcp_connected_recent(now)
        if tcp_ready != self._last_connected_state:
            # 只在状态变化时打日志，避免等待雷达板时刷屏
            self._last_connected_state = tcp_ready
            if tcp_ready:
                self.get_logger().info("雷达 TCP 已连接")
            else:
                if self._require_tcp_before_start:
                    self.get_logger().warn("雷达 TCP 未连接或状态超时，等待雷达板接入后再启动雷达驱动")
                else:
                    self.get_logger().warn("雷达 TCP 未连接或状态超时，雷达驱动会保持等待本地串口数据")

        if not self._chassis_gate_open(now):
            self._pending_start_at = None
            if self._process is not None:
                self._stop_driver("底盘首次连接尚未完成，暂停 ydlidar_node")
            elif now - self._last_chassis_wait_log_time > 5.0:
                self._last_chassis_wait_log_time = now
                self.get_logger().info("等待底盘首次 /car/mcu_status 后再启动雷达驱动")
            return

        if not self._serial_ready(now):
            self._pending_start_at = None
            if self._process is not None:
                self._stop_driver("本地雷达串口消失，停止 ydlidar_node，等待 laser_tcp_server 恢复")
            return

        if self._require_tcp_before_start and not tcp_ready:
            # TCP 未连接时不启动 ydlidar_node，避免驱动打开空伪串口后报 Device Failed
            self._pending_start_at = None
            self._scan_seen_since_start = False
            self._current_restart_backoff = max(0.1, self._base_restart_backoff)
            if self._process is not None:
                self._stop_driver("雷达 TCP 已断开，停止 ydlidar_node，等待雷达板重新接入")
            else:
                if now - self._last_tcp_wait_log_time > 10.0:
                    self._last_tcp_wait_log_time = now
                    self.get_logger().info("等待雷达板 TCP 客户端连接到 Jetson 8889")
            return

        if self._process is None:
            if self._pending_start_at is None:
                # 第一次满足条件时只记录计划启动时间，真正启动等 start_delay 后执行
                self._pending_start_at = now + max(0.0, self._start_delay)
            if now >= self._pending_start_at:
                self._start_driver(now)
            return

        if self._scan_timed_out(now):
            # 有过 /scan 后使用短超时，首次启动没见过 /scan 使用更长初始化超时
            # 当前启动阶段适用的扫描超时
            timeout = (
                self._scan_timeout
                if self._scan_seen_since_start
                else self._initial_scan_timeout
            )
            if not tcp_ready:
                if now - self._last_tcp_wait_log_time > 10.0:
                    self._last_tcp_wait_log_time = now
                    self.get_logger().info(
                        f"雷达驱动已 {timeout:.1f}s 未产生 /scan，"
                        "但雷达 TCP 未连接，保持驱动运行并等待雷达板接入"
                    )
                return
            self.get_logger().warn(
                f"雷达驱动已 {timeout:.1f}s 未产生有效 /scan，重启驱动"
            )
            self._stop_driver("雷达扫描超时，重启 ydlidar_node")
            self._schedule_restart(now)

    def _start_driver(self, now: float):
        """在独立进程组中启动第三方 ydlidar_node"""
        # 通过 ros2 run 启动第三方 ydlidar_node，不修改 ydlidar 源码和 launch
        # 启动第三方 ydlidar_node 的 ros2 run 命令
        command = [
            "ros2",
            "run",
            "ydlidar",
            "ydlidar_node",
            "--ros-args",
            "-r",
            "__node:=ydlidar_node",
            "--params-file",
            self._ydlidar_params_file,
        ]
        try:
            # 创建新的进程会话，监督节点停止时可以一次性结束 ros2 run 及其子进程
            self._process = subprocess.Popen(command, start_new_session=True)
        except OSError as exc:
            self.get_logger().error(f"启动雷达驱动失败：{exc}")
            self._process = None
            self._schedule_restart(now)
            return

        self._process_started_at = now
        self._scan_seen_since_start = False
        self._pending_start_at = None
        self.get_logger().info("雷达驱动已启动，等待 /scan")

    def _reap_driver(self, now: float):
        """非阻塞回收已退出驱动并安排下一次重启"""
        # poll 非阻塞检查子进程是否已经退出，避免监督循环卡住
        if self._process is None:
            return
        # ydlidar_node 当前退出码，None 表示仍在运行
        return_code = self._process.poll()
        if return_code is None:
            return
        self.get_logger().warn(f"雷达驱动已退出，退出码={return_code}")
        self._process = None
        self._process_started_at = None
        self._scan_seen_since_start = False
        self._schedule_restart(now)

    def _scan_timed_out(self, now: float) -> bool:
        """区分首次扫描超时和正常运行阶段扫描超时"""
        # 没有进程启动时间就没有超时判断对象
        if self._process_started_at is None:
            return False
        # 已经见过 /scan 后按最后一帧时间判断，启动阶段按进程启动时间判断
        if self._scan_seen_since_start and self._last_scan_time is not None:
            return now - self._last_scan_time > self._scan_timeout
        return now - self._process_started_at > self._initial_scan_timeout

    def _schedule_restart(self, now: float):
        """使用有上限的指数退避安排驱动重启"""
        # 指数退避避免雷达板掉线时频繁启动/停止 ydlidar_node
        # 本次驱动重启前实际等待的退避时长
        delay = min(self._current_restart_backoff, self._max_restart_backoff)
        self._pending_start_at = now + delay
        self._current_restart_backoff = min(
            max(0.1, delay * 2.0),
            max(0.1, self._max_restart_backoff),
        )
        self.get_logger().info(f"{delay:.1f}s 后重试启动雷达驱动")

    def _stop_driver(self, reason: str):
        """按 SIGINT、SIGTERM、SIGKILL 顺序停止驱动进程组"""
        # 停止顺序为 SIGINT -> SIGTERM -> SIGKILL，优先让 ydlidar_node 自己释放串口
        if self._process is None:
            return
        # 当前待停止的 ydlidar_node 子进程
        process = self._process
        self.get_logger().warn(reason)
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=self._stop_timeout)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=1.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        finally:
            self._process = None
            self._process_started_at = None
            self._scan_seen_since_start = False


def main(args=None):
    """运行雷达监督节点直到 ROS 关闭"""
    rclpy.init(args=args)
    # 雷达驱动监督节点实例
    node = LidarSupervisorNode()
    try:
        # supervisor 是长生命周期节点，所有恢复逻辑都在定时器里执行
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
