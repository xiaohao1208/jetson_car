import time

from car_interfaces.msg import CarMcuStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class TopicChecker(Node):
    """在短窗口内检查底盘、雷达、里程计和正式 TF 链"""

    def __init__(self):
        """创建诊断订阅并初始化消息计数"""
        super().__init__("car_check_topics")
        self.counts = {
            "mcu": 0,
            "scan": 0,
            "odom": 0,
            "plan": 0,
            "nav_cmd": 0,
            "smooth_cmd": 0,
            "move_cmd": 0,
        }
        self.laser_tcp_connected = False
        self.nonzero = {
            "nav_cmd": False,
            "smooth_cmd": False,
            "move_cmd": False,
        }
        self.wheel_speeds = (0.0, 0.0)
        self.obstacle = False
        self.create_subscription(
            CarMcuStatus, "/car/mcu_status",
            self._on_mcu, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan",
            lambda _: self._count("scan"), qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/odom", lambda _: self._count("odom"), 10
        )
        self.create_subscription(
            Bool, "/laser_tcp_connected", self._on_tcp, 10
        )
        self.create_subscription(
            Path, "/plan", lambda _: self._count("plan"), 10
        )
        self.create_subscription(
            Twist, "/cmd_vel_nav",
            lambda message: self._on_twist("nav_cmd", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist, "/cmd_vel",
            lambda message: self._on_twist("smooth_cmd", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist, "/cmd_vel_move",
            lambda message: self._on_twist("move_cmd", message),
            qos_profile_sensor_data,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _count(self, key):
        """累计指定诊断话题的消息数量"""
        self.counts[key] += 1

    def _on_tcp(self, message):
        """记录雷达 TCP 连接状态"""
        self.laser_tcp_connected = bool(message.data)

    def _on_mcu(self, message):
        """记录 MCU 消息、轮速和近障状态"""
        self._count("mcu")
        self.wheel_speeds = (
            float(message.left_wheel_speed),
            float(message.right_wheel_speed),
        )
        self.obstacle = bool(message.obstacle_ok)

    def _on_twist(self, key, message):
        """记录速度链消息数量及是否出现非零输出"""
        self._count(key)
        if (
            abs(float(message.linear.x)) >= 0.005
            or abs(float(message.angular.z)) >= 0.05
        ):
            self.nonzero[key] = True


def main(args=None):
    """采样五秒并输出底盘到导航速度链的中文诊断结果"""
    rclpy.init(args=args)
    node = TopicChecker()
    start = time.monotonic()
    while rclpy.ok() and time.monotonic() - start < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    try:
        node.tf_buffer.lookup_transform(
            "odom", "base_footprint", rclpy.time.Time()
        )
        tf_ok = True
    except TransformException:
        tf_ok = False
    print(
        "诊断结果: "
        f"mcu={node.counts['mcu']} scan={node.counts['scan']} "
        f"odom={node.counts['odom']} lidar_tcp={node.laser_tcp_connected} "
        f"odom_tf={tf_ok} plan={node.counts['plan']} "
        f"nav_cmd={node.counts['nav_cmd']}/{node.nonzero['nav_cmd']} "
        f"smooth_cmd={node.counts['smooth_cmd']}/"
        f"{node.nonzero['smooth_cmd']} "
        f"move_cmd={node.counts['move_cmd']}/"
        f"{node.nonzero['move_cmd']} "
        f"left_wheel={node.wheel_speeds[0]:.3f} "
        f"right_wheel={node.wheel_speeds[1]:.3f} "
        f"obstacle={node.obstacle}"
    )
    node.destroy_node()
    rclpy.shutdown()
