import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("car_observation"),
        "config",
        "observation_collection.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="car_observation",
                executable="observation_collector",
                name="car_observation_collect",
                parameters=[config],
                output="screen",
            )
        ]
    )
