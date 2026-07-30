import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """读取网页配置并允许启动参数覆盖地图路径"""
    config = os.path.join(
        get_package_share_directory("car_web"), "config", "web.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value="maps/map.yaml"),
            Node(
                package="car_web",
                executable="server",
                name="car_web",
                parameters=[
                    config,
                    {"map_file": LaunchConfiguration("map")},
                ],
                output="screen",
            ),
        ]
    )
