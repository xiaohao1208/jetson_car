import sys
import time

from car_interfaces.msg import CarMcuStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from car_bringup.chassis_gate import ChassisReadyGate


class WaitForChassisNode(Node):
    """等待 ESP32 底盘状态稳定后退出"""

    def __init__(self):
        """读取底盘门控参数并订阅 MCU 状态"""
        super().__init__("wait_for_chassis")
        self.declare_parameter("mcu_status_topic", "/car/mcu_status")
        self.declare_parameter("ready_count", 3)
        self.declare_parameter("wait_timeout_sec", 0.0)
        self.declare_parameter("log_period_sec", 5.0)

        # 被监听的 ESP32 原始状态话题
        self._mcu_status_topic = str(
            self.get_parameter("mcu_status_topic").value
        )
        # 需要连续满足 WiFi 和 Agent 正常的状态帧数量
        self._gate = ChassisReadyGate(
            self.get_parameter("ready_count").value
        )
        # 零表示始终等待，正数超时后以失败状态退出
        self._wait_timeout = max(
            0.0, float(self.get_parameter("wait_timeout_sec").value)
        )
        # 等待日志的最小输出周期
        self._log_period = max(
            1.0, float(self.get_parameter("log_period_sec").value)
        )
        # 使用单调时间避免系统时间变化影响启动门控
        self._start_time = time.monotonic()
        self._last_log_time = 0.0
        # None 表示继续等待，零表示成功，其它值表示失败
        self._exit_code = None

        self.create_subscription(
            CarMcuStatus,
            self._mcu_status_topic,
            self._mcu_status_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(0.5, self._timer_callback)
        self.get_logger().info(
            "等待底盘 ESP32 连接 micro-ROS，topic=%s，连续有效帧=%d，超时=%.1fs"
            % (
                self._mcu_status_topic,
                self._gate.ready_count,
                self._wait_timeout,
            )
        )

    @property
    def done(self):
        """返回底盘等待流程是否已经得到终态"""
        return self._exit_code is not None

    @property
    def exit_code(self):
        """返回成功、超时或中断对应的进程退出码"""
        return self._exit_code

    def _mcu_status_callback(self, message):
        """累计连续有效 MCU 状态并在满足门槛后放行"""
        ready = self._gate.update(
            bool(message.wifi_connect_ok),
            bool(message.agent_connect_ok),
        )
        if ready:
            self.get_logger().info(
                "底盘 ESP32 已稳定连接，连续收到 %d 帧有效状态"
                % self._gate.consecutive_ready
            )
            self._exit_code = 0
            return

        if message.wifi_connect_ok and message.agent_connect_ok:
            return
        now = time.monotonic()
        if now - self._last_log_time >= self._log_period:
            self._last_log_time = now
            self.get_logger().warn(
                "已收到 MCU 状态，但 WiFi 或 micro-ROS Agent 尚未稳定"
            )

    def _timer_callback(self):
        """检查总等待超时并按固定周期输出等待日志"""
        if self.done:
            return
        now = time.monotonic()
        elapsed = now - self._start_time
        if self._wait_timeout > 0.0 and elapsed >= self._wait_timeout:
            self.get_logger().error(
                "%.1fs 内未等到底盘稳定连接，雷达链路保持关闭"
                % self._wait_timeout
            )
            self._exit_code = 2
            return
        if now - self._last_log_time >= self._log_period:
            self._last_log_time = now
            self.get_logger().info(
                "等待底盘稳定连接后再开放雷达 TCP 8889"
            )


def main(args=None):
    """运行底盘门控节点并使用门控结果作为进程退出码"""
    rclpy.init(args=args)
    node = WaitForChassisNode()
    exit_code = 130
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.exit_code is not None:
            exit_code = node.exit_code
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
