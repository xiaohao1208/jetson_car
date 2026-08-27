import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动空闲等待网页或命令行 Action 的标定节点。"""

    config = os.path.join(
        get_package_share_directory("car_calibrate"), "config", "calibrate.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="car_calibrate",
                executable="calibrate_node",
                name="car_calibrate",
                parameters=[config],
                output="screen",
            )
        ]
    )
