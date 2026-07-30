import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from car_bringup.hotspot import ensure_hotspot


def _include(package, filename, arguments=None, condition=None):
    """构造一个可带参数和条件的包内启动文件引用"""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package), "launch", filename
            )
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def _as_bool(value):
    """把 ROS 启动参数的常见文本形式转换为布尔值"""
    return value.lower() in {"1", "true", "yes", "on"}


def _configure_hotspot(context):
    """在其它节点构造前同步确认固定热点，禁用时完全不修改网络"""
    if not _as_bool(LaunchConfiguration("start_hotspot").perform(context)):
        return []
    try:
        device = ensure_hotspot(
            LaunchConfiguration("hotspot_interface").perform(context),
            LaunchConfiguration("hotspot_connection").perform(context),
            LaunchConfiguration("hotspot_ssid").perform(context),
            LaunchConfiguration("hotspot_password").perform(context),
            LaunchConfiguration("hotspot_address").perform(context),
            int(LaunchConfiguration("hotspot_channel").perform(context)),
            float(
                LaunchConfiguration("hotspot_timeout_sec").perform(context)
            ),
        )
        print(f"小车热点已就绪，网卡={device}")
    except (OSError, RuntimeError, ValueError) as error:
        if _as_bool(
            LaunchConfiguration("hotspot_required").perform(context)
        ):
            raise RuntimeError(f"小车热点启动失败: {error}") from error
        print(f"警告：小车热点未就绪，继续启动ROS节点：{error}")
    return []


def generate_launch_description():
    """按底盘优先顺序构造热点、Agent、雷达和网页整车启动"""
    share = get_package_share_directory("car_bringup")
    with open(
        os.path.join(share, "config", "bringup.yaml"),
        "r",
        encoding="utf-8",
    ) as stream:
        config = yaml.safe_load(stream) or {}

    def value(name, fallback):
        """读取 bringup.yaml 配置并转换为启动参数文本"""
        return str(config.get(name, fallback))

    start_agent = LaunchConfiguration("start_agent")
    start_web = LaunchConfiguration("start_web")

    def lidar_actions():
        """构造雷达 TCP 桥接和驱动监督两个节点"""
        return [
            Node(
                package="car_bringup",
                executable="laser_tcp_server",
                name="laser_tcp_server",
                parameters=[
                    {
                        "tcp_host": value(
                            "laser_tcp_host", "192.168.4.1"
                        ),
                        "tcp_port": int(value("laser_tcp_port", 8889)),
                        "serial_port": value(
                            "laser_serial_port", "/tmp/tty_laser"
                        ),
                    }
                ],
                output="screen",
            ),
            Node(
                package="car_bringup",
                executable="lidar_supervisor",
                name="lidar_supervisor",
                parameters=[
                    {
                        "serial_port": value(
                            "laser_serial_port", "/tmp/tty_laser"
                        ),
                        "chassis_first_startup": True,
                        "chassis_first_timeout_sec": 0.0,
                        "require_tcp_before_start": True,
                    }
                ],
                output="screen",
            ),
        ]

    def configure_lidar_chain(context):
        """按启动参数选择直接调试或等待底盘门控的雷达链路"""
        if not _as_bool(
            LaunchConfiguration("start_lidar").perform(context)
        ):
            return []
        if not _as_bool(
            LaunchConfiguration("lidar_wait_for_chassis").perform(context)
        ):
            return [
                LogInfo(
                    msg=(
                        "lidar_wait_for_chassis=false，"
                        "直接启动雷达链路，仅用于单独调试"
                    )
                ),
                *lidar_actions(),
            ]

        wait_for_chassis = Node(
            package="car_bringup",
            executable="wait_for_chassis",
            name="wait_for_chassis",
            parameters=[
                {
                    "mcu_status_topic": "/car/mcu_status",
                    "ready_count": LaunchConfiguration(
                        "lidar_chassis_ready_count"
                    ),
                    "wait_timeout_sec": LaunchConfiguration(
                        "lidar_chassis_wait_timeout_sec"
                    ),
                    "log_period_sec": LaunchConfiguration(
                        "lidar_chassis_wait_log_period_sec"
                    ),
                }
            ],
            output="screen",
        )

        def on_gate_exit(event, _):
            """底盘门控退出后决定保持关闭或延时启动雷达"""
            if event.returncode != 0:
                return [
                    LogInfo(
                        msg=(
                            "底盘连接门控未成功，"
                            "雷达 TCP 和雷达驱动保持关闭"
                        )
                    )
                ]
            delay = max(
                0.0,
                float(
                    LaunchConfiguration(
                        "lidar_start_after_chassis_sec"
                    ).perform(context)
                ),
            )
            return [
                LogInfo(
                    msg=(
                        "底盘连接完成，"
                        f"{delay:.1f}s 后启动雷达 TCP 和雷达驱动"
                    )
                ),
                TimerAction(period=delay, actions=lidar_actions()),
            ]

        return [
            LogInfo(
                msg=(
                    "雷达链路等待底盘 ESP32 稳定连接，"
                    "门控通过前不开放 TCP 8889"
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_chassis,
                    on_exit=on_gate_exit,
                )
            ),
            wait_for_chassis,
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_hotspot", default_value="true"),
            DeclareLaunchArgument("hotspot_required", default_value="true"),
            DeclareLaunchArgument(
                "hotspot_interface",
                default_value=value("hotspot_interface", "auto"),
            ),
            DeclareLaunchArgument(
                "hotspot_connection",
                default_value=value(
                    "hotspot_connection", "jetson-car-hotspot"
                ),
            ),
            DeclareLaunchArgument(
                "hotspot_ssid",
                default_value=value("hotspot_ssid", "jetson"),
            ),
            DeclareLaunchArgument(
                "hotspot_password",
                default_value=value("hotspot_password", "88888888"),
            ),
            DeclareLaunchArgument(
                "hotspot_address",
                default_value=value(
                    "hotspot_address", "192.168.4.1/24"
                ),
            ),
            DeclareLaunchArgument(
                "hotspot_channel",
                default_value=value("hotspot_channel", 6),
            ),
            DeclareLaunchArgument(
                "hotspot_timeout_sec",
                default_value=value("hotspot_timeout_sec", 15.0),
            ),
            DeclareLaunchArgument("start_agent", default_value="true"),
            DeclareLaunchArgument(
                "agent_port", default_value=value("agent_port", 8888)
            ),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument(
                "lidar_wait_for_chassis",
                default_value=value("lidar_wait_for_chassis", True),
            ),
            DeclareLaunchArgument(
                "lidar_chassis_ready_count",
                default_value=value("lidar_chassis_ready_count", 3),
            ),
            DeclareLaunchArgument(
                "lidar_chassis_wait_timeout_sec",
                default_value=value(
                    "lidar_chassis_wait_timeout_sec", 0.0
                ),
            ),
            DeclareLaunchArgument(
                "lidar_chassis_wait_log_period_sec",
                default_value=value(
                    "lidar_chassis_wait_log_period_sec", 5.0
                ),
            ),
            DeclareLaunchArgument(
                "lidar_start_after_chassis_sec",
                default_value=value(
                    "lidar_start_after_chassis_sec", 0.5
                ),
            ),
            DeclareLaunchArgument("start_web", default_value="true"),
            DeclareLaunchArgument(
                "map", default_value=value("map", "maps/map.yaml")
            ),
            OpaqueFunction(function=_configure_hotspot),
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                arguments=[
                    "udp4", "--port", LaunchConfiguration("agent_port")
                ],
                condition=IfCondition(start_agent),
                output="screen",
            ),
            _include("car_description", "description.launch.py"),
            _include("car_move", "move.launch.py"),
            _include("car_base", "base.launch.py"),
            OpaqueFunction(function=configure_lidar_chain),
            _include(
                "car_web",
                "web.launch.py",
                {"map": LaunchConfiguration("map")},
                condition=IfCondition(start_web),
            ),
        ]
    )
