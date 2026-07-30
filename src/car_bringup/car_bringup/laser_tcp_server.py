#!/usr/bin/env python3

import os
import pty
import select
import socket
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class LaserTcpServerNode(Node):
    """把雷达 WiFi TCP 数据流映射成本地 /tmp/tty_laser 伪串口

    雷达驱动仍按“本地串口设备”工作，雷达转接板只负责把串口字节透明搬到
    TCP。这个节点位于中间，必须保证字节顺序不变，并在连接断开时及时发布
    /laser_tcp_connected=false，让 supervisor 停止或等待 ydlidar_node
    """

    def __init__(self):
        """读取桥接参数并创建雷达 TCP 连接状态发布器"""
        super().__init__("laser_tcp_server")

        self.declare_parameter("tcp_host", "192.168.4.1")
        self.declare_parameter("tcp_port", 8889)
        self.declare_parameter("serial_port", "/tmp/tty_laser")
        self.declare_parameter("idle_timeout_sec", 30.0)
        self.declare_parameter("status_topic", "/laser_tcp_connected")

        # TCP 监听地址，热点模式下固定监听 Jetson 热点 IP，避免绑定到外网接口
        self.tcp_host = self.get_parameter("tcp_host").get_parameter_value().string_value
        # 雷达转接板配置中的 server_port 必须和这里一致
        self.tcp_port = self.get_parameter("tcp_port").get_parameter_value().integer_value
        # ydlidar_node 打开的本地路径，本节点会把它重建为指向伪终端 slave 的符号链接
        self.serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        # 长时间无收发数据说明雷达板或 WiFi 卡住，主动断开让板子重新连
        self.idle_timeout = (
            self.get_parameter("idle_timeout_sec")
            .get_parameter_value().double_value
        )
        # supervisor/Web 共同使用的 TCP 连接状态 topic
        self.status_topic = self.get_parameter("status_topic").get_parameter_value().string_value

        # 使用普通 Bool topic 而不是服务，便于 Web 和 supervisor 都按新鲜度判断
        self.status_publisher = self.create_publisher(Bool, self.status_topic, 10)
        # 最近一次发布 TCP 连接状态的墙钟时间
        self._last_status_publish_time = 0.0
        # 记录上次发布值，用于节流重复状态
        self._last_published_connected = None

    def publish_connection_state(self, connected, force=False):
        """发布雷达 TCP 连接状态并节流未变化的重复消息"""
        # Web 和 lidar_supervisor 使用该 topic 判断 TCP 是否接入，/scan 新鲜度单独判断
        if not rclpy.ok():
            self._last_published_connected = connected
            return
        # 本轮状态发布判断使用的墙钟时间
        now = time.time()
        if (
            not force
            and self._last_published_connected == connected
            and now - self._last_status_publish_time < 1.0
        ):
            return
        # 待发布的 TCP 连接布尔消息
        msg = Bool()
        msg.data = connected
        self.status_publisher.publish(msg)
        self._last_status_publish_time = now
        self._last_published_connected = connected

    def close_client(self, mypoll, client, reason):
        """从 poll 注销并关闭当前客户端，然后发布断开状态"""
        # close_client 统一处理 poll 反注册、socket 关闭、日志和状态发布，避免分支漏清理
        if client is None:
            return None
        # 断开客户端的远端地址，读取失败时保持为空
        address = None
        try:
            address = client.getpeername()
        except OSError:
            pass
        try:
            mypoll.unregister(client.fileno())
        except (KeyError, OSError, ValueError):
            pass
        try:
            client.close()
        except OSError:
            pass
        # 日志中可选的远端地址后缀
        suffix = f" {address}" if address is not None else ""
        self.get_logger().warn(f"雷达 TCP 客户端断开{suffix}：{reason}")
        self.publish_connection_state(False, force=True)
        return None

    def _create_serial_link(self):
        """创建伪终端并把固定串口路径链接到 slave 设备"""
        # pty.openpty 返回 master/slave，ydlidar_node 读写 slave，Python 从 master 转发字节
        # 伪终端 master 和 slave 文件描述符
        master, slave = pty.openpty()
        # ydlidar_node 实际打开的伪终端 slave 系统路径
        slave_name = os.ttyname(slave)
        # 上次异常退出可能留下 /tmp/tty_laser，必须先删掉旧链接/文件再创建
        if os.path.lexists(self.serial_port):
            os.remove(self.serial_port)
        os.symlink(slave_name, self.serial_port)
        # slave 由 ydlidar_node 通过符号链接重新打开，服务端不保留多余描述符
        os.close(slave)
        return master, slave_name

    def run(self):
        """在 TCP socket 和伪终端之间双向透明转发原始字节"""
        # TCP server 用非阻塞 socket 配合 poll，同时监听网络和伪串口两个方向
        # 监听雷达转接板连接的 TCP 服务 socket
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.tcp_host, self.tcp_port))
        server.listen(1)
        server.setblocking(False)

        # 虚拟串口 master 描述符和 slave 系统路径
        master, slave_name = self._create_serial_link()
        self.get_logger().info(
            f"TCP监听：{self.tcp_host}:{self.tcp_port}，"
            f"虚拟串口：{self.serial_port} 对应 {slave_name}"
        )

        # 同时监控监听 socket、客户端 socket 和虚拟串口的 poll 对象
        mypoll = select.poll()
        mypoll.register(server.fileno(), select.POLLIN)
        mypoll.register(master, select.POLLIN)
        # 当前雷达板 TCP 客户端，只允许一个客户端，后来的连接会替换旧连接
        client = None
        # 当前雷达客户端 socket 的文件描述符
        client_fd = None
        # 最近一次任意方向转发成功的时间，用于 idle timeout
        last_exchange_data_time = time.time()
        # 是否已经记录“无客户端时丢弃串口数据”的提示
        logged_no_client_serial_data = False
        # 是否已经记录 ydlidar 到雷达板方向开始转发
        logged_serial_to_tcp = False
        # 是否已经记录雷达板到 ydlidar 方向开始转发
        logged_tcp_to_serial = False
        self.publish_connection_state(False, force=True)

        try:
            self.get_logger().info(
                f"等待雷达板 TCP 客户端连接到 Jetson 端口 {self.tcp_port}，"
                f"雷达驱动可先打开 {self.serial_port}"
            )
            while rclpy.ok():
                self.publish_connection_state(client is not None)
                # 本轮 poll 返回的文件描述符事件列表
                fdlist = mypoll.poll(256)
                # 本轮空闲超时判断使用的墙钟时间
                now = time.time()
                # 当前就绪文件描述符及其事件位
                for fd, event in fdlist:
                    if fd == server.fileno():
                        # 新雷达板连接到达，替换旧客户端可以处理雷达板重启后端口变化
                        try:
                            # 新客户端 socket 及其远端地址
                            new_client, client_address = server.accept()
                        except BlockingIOError:
                            continue
                        new_client.setblocking(False)
                        if client is not None:
                            client = self.close_client(mypoll, client, "新客户端已连接，替换旧连接")
                        client = new_client
                        client_fd = client.fileno()
                        mypoll.register(client_fd, select.POLLIN)
                        last_exchange_data_time = now
                        logged_serial_to_tcp = False
                        logged_tcp_to_serial = False
                        self.get_logger().info(f"来自{client_address}的雷达 TCP 连接已建立")
                        self.publish_connection_state(True, force=True)
                        continue

                    if event & (select.POLLHUP | select.POLLERR | select.POLLNVAL):
                        # poll 报错通常表示 socket 已断开，立即清理并等待重新连接
                        if client is not None and fd == client_fd:
                            client = self.close_client(
                                mypoll,
                                client,
                                "套接字轮询异常",
                            )
                            client_fd = None
                        continue

                    if fd == master:
                        try:
                            # ydlidar_node 写入虚拟串口的原始字节
                            data = os.read(master, 256)
                        except OSError as exc:
                            self.get_logger().warn(f"读取虚拟串口失败：{exc}")
                            continue
                        if not data:
                            continue
                        if client is None:
                            # 雷达板未接入时丢弃 ydlidar 初始化字节，避免重连后收到过期命令
                            if not logged_no_client_serial_data:
                                logged_no_client_serial_data = True
                                self.get_logger().info(
                                    "已收到雷达驱动写入虚拟串口的数据，"
                                    "当前无雷达 TCP 客户端，数据先丢弃并继续等待雷达板"
                                )
                            continue
                        try:
                            # ydlidar_node -> 伪串口 master -> TCP -> 雷达转接板
                            client.sendall(data)
                            if not logged_serial_to_tcp:
                                logged_serial_to_tcp = True
                                self.get_logger().info(
                                    "已开始从雷达驱动向雷达 TCP 客户端转发数据"
                                )
                            last_exchange_data_time = now
                        except OSError as exc:
                            client = self.close_client(
                                mypoll,
                                client,
                                f"发送到 TCP 客户端失败：{exc}",
                            )
                            client_fd = None
                        continue

                    if client is not None and fd == client_fd:
                        # 雷达转接板 -> TCP -> 伪串口 master -> ydlidar_node
                        try:
                            # 雷达转接板通过 TCP 发来的原始串口字节
                            data = client.recv(256)
                        except BlockingIOError:
                            continue
                        except OSError as exc:
                            client = self.close_client(
                                mypoll,
                                client,
                                f"读取 TCP 客户端失败：{exc}",
                            )
                            client_fd = None
                            continue
                        if not data:
                            client = self.close_client(mypoll, client, "客户端关闭连接")
                            client_fd = None
                            continue
                        try:
                            os.write(master, data)
                            if not logged_tcp_to_serial:
                                logged_tcp_to_serial = True
                                self.get_logger().info(
                                    "已开始从雷达 TCP 客户端向雷达驱动转发数据"
                                )
                            last_exchange_data_time = now
                        except OSError as exc:
                            self.get_logger().warn(f"写入虚拟串口失败：{exc}")

                # 有连接但长期没有任何字节交换时，主动断开让雷达板重新建立 TCP
                if client is not None and now - last_exchange_data_time > self.idle_timeout:
                    client = self.close_client(
                        mypoll,
                        client,
                        f"{self.idle_timeout:.1f}s 没有数据交换",
                    )
                    client_fd = None
        finally:
            # 退出时必须发布 false，否则 Web/OLED 可能继续显示雷达 TCP 在线
            client = self.close_client(mypoll, client, "服务正在关闭")
            try:
                mypoll.unregister(server.fileno())
            except (KeyError, OSError, ValueError):
                pass
            server.close()
            # 关闭虚拟串口 master，让系统回收对应 /dev/pts 设备
            try:
                os.close(master)
            except OSError:
                pass
            # 只删除本进程创建且仍指向同一 slave 的链接，避免影响并行启动的新实例
            try:
                if (
                    os.path.islink(self.serial_port)
                    and os.path.realpath(self.serial_port) == slave_name
                ):
                    os.unlink(self.serial_port)
            except OSError:
                pass
            self.publish_connection_state(False, force=True)


def main():
    """运行雷达 TCP 转伪串口节点直到 ROS 关闭"""
    rclpy.init()
    # 雷达 TCP 到本地虚拟串口的桥接节点实例
    node = LaserTcpServerNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
