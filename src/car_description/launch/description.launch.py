from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    """启动机器人模型发布器和关节状态发布器"""
    # car_description的share目录
    package_share = get_package_share_directory("car_description")
    # 小车模型默认配置路径
    config_path = os.path.join(package_share, "config", "description.yaml")

    with open(config_path, "r", encoding="utf-8") as stream:
        # 获取配置
        config = yaml.safe_load(stream) or {}
    config_model = config.get("model", "urdf/car.urdf.xacro")

    default_model = config_model
    if not os.path.isabs(default_model):
        default_model = os.path.join(package_share, default_model)

    # 声明模型路径参数
    declare_arg_model = DeclareLaunchArgument("model", default_value=default_model)
    # 获取xacro文件展开后的urdf内容，传给robot_state_publisher发布
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", LaunchConfiguration("model")]),
        value_type=str
    )
    # 状态发布节点，只发布base_footprint以下的静态TF
    car_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen"
    )
    # 关节状态发布节点
    car_joint_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )

    return LaunchDescription([
        declare_arg_model,
        car_state_publisher_node,
        car_joint_publisher_node
    ])
