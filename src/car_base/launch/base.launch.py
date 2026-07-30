import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动编码器与 IMU 融合的底盘里程计"""
    config = os.path.join(
        get_package_share_directory("car_base"), "config", "base.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="car_base",
                executable="car_base_node",
                name="car_base",
                parameters=[config],
                output="screen",
            )
        ]
    )
