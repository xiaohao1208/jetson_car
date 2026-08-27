import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """启动 slam_toolbox，并可选启动项目 RViz 配置"""
    package_share = get_package_share_directory("car_mapping")
    params_file = os.path.join(package_share, "config", "slam_toolbox.yaml")
    rviz_config = os.path.join(package_share, "rviz", "mapping.rviz")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="car_mapping",
                executable="scan_warmup_relay",
                name="scan_warmup_relay",
                parameters=[
                    {
                        "input_topic": "/scan",
                        "output_topic": "/mapping_scan",
                        "warmup_sec": 1.0,
                    }
                ],
                output="screen",
            ),
            Node(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[params_file, {"use_sim_time": False}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="mapping_rviz",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
