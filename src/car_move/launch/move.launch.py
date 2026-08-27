import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动速度仲裁、安全拦截与底盘状态节点"""
    config = os.path.join(
        get_package_share_directory("car_move"), "config", "move.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="car_move",
                executable="car_move_node",
                # 节点名必须与 config/move.yaml 的顶层键一致
                name="car_move",
                parameters=[config],
                output="screen",
            )
        ]
    )
