import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import yaml


def load_navigation_modes():
    """读取启动文件和网页后台共享的导航模式定义"""
    source_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "navigation_modes.yaml"
    )
    if not source_path.is_file():
        source_path = (
            Path(get_package_share_directory("car_navigation"))
            / "config"
            / "navigation_modes.yaml"
        )
    with source_path.open(encoding="utf-8") as stream:
        modes = yaml.safe_load(stream)
    if not isinstance(modes, dict) or "classic" not in modes:
        raise RuntimeError("导航模式配置缺少经典导航")
    return modes


# 公开模式只改变局部控制器插件，NavFn和其余导航安全链保持一致
NAVIGATION_MODES = load_navigation_modes()


def navigation_mode_rewrites(mode):
    """把公开模式转换为局部控制器参数覆写，并拒绝未知模式"""
    if mode not in NAVIGATION_MODES:
        raise RuntimeError(
            f"未知navigation_mode={mode}，允许值: {', '.join(NAVIGATION_MODES)}"
        )
    selected = NAVIGATION_MODES[mode]
    required = ("controller", "failure_tolerance", "progress_checker")
    missing = [key for key in required if key not in selected]
    if missing:
        raise RuntimeError(
            f"navigation_mode={mode}缺少配置: {', '.join(missing)}"
        )
    return {
        "controller_server.ros__parameters.FollowPath.plugin":
            selected["controller"],
        "controller_server.ros__parameters.failure_tolerance":
            str(selected["failure_tolerance"]),
        "controller_server.ros__parameters.progress_checker.plugin":
            selected["progress_checker"],
    }


def parse_navigation_start_delay(value):
    """校验定位激活后附加的导航启动延迟"""
    try:
        delay = float(value)
    except ValueError as error:
        raise RuntimeError("navigation_start_delay_sec必须是数字") from error
    if not math.isfinite(delay) or delay < 0.0:
        raise RuntimeError("navigation_start_delay_sec不能小于0")
    return delay


def parse_localization_start_timeout(value):
    """校验定位生命周期门禁的等待上限"""
    try:
        timeout = float(value)
    except ValueError as error:
        raise RuntimeError(
            "localization_start_timeout_sec必须是数字"
        ) from error
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise RuntimeError("localization_start_timeout_sec必须大于0")
    return timeout


def launch_stack(context):
    """先启动定位入口，确认生命周期激活后再启动导航入口"""
    mode = LaunchConfiguration("navigation_mode").perform(context)
    navigation_delay = parse_navigation_start_delay(
        LaunchConfiguration(
            "navigation_start_delay_sec"
        ).perform(context)
    )
    localization_timeout = parse_localization_start_timeout(
        LaunchConfiguration(
            "localization_start_timeout_sec"
        ).perform(context)
    )
    package_share = get_package_share_directory("car_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")
    source_params = os.path.join(package_share, "config", "nav2_params.yaml")
    navigation_rewrites = navigation_mode_rewrites(mode)
    navigation_rewrites[
        "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml"
    ] = os.path.join(
        package_share,
        "behavior_trees",
        "navigate_to_pose_no_recovery.xml",
    )
    configured_params = RewrittenYaml(
        source_file=source_params,
        root_key="",
        param_rewrites=navigation_rewrites,
        convert_types=True,
    )
    common_arguments = {
        "use_sim_time": "False",
        "params_file": configured_params,
        "autostart": "True",
        "use_composition": "False",
    }
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "localization_launch.py")
        ),
        launch_arguments={
            **common_arguments,
            "map": LaunchConfiguration("map"),
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "navigation_launch.py")
        ),
        launch_arguments=common_arguments.items(),
    )

    localization_gate = Node(
        package="car_navigation",
        executable="wait_for_localization",
        name="wait_for_localization",
        parameters=[{"timeout_sec": localization_timeout}],
        output="screen",
    )

    def on_localization_gate_exit(event, launch_context):
        """门禁成功后启动导航；失败则结束本轮并让网页执行重试"""
        if launch_context.is_shutdown:
            return []
        if event.returncode != 0:
            reason = (
                "地图定位栈未激活，已取消启动planner和controller"
            )
            return [
                LogInfo(msg=f"[ERROR] {reason}"),
                EmitEvent(event=Shutdown(reason=reason)),
            ]
        actions = [
            LogInfo(
                msg=(
                    "地图定位栈已激活，"
                    f"{navigation_delay:.1f}s 后启动Nav2导航栈"
                )
            )
        ]
        if navigation_delay > 0.0:
            actions.append(
                TimerAction(
                    period=navigation_delay,
                    actions=[navigation],
                )
            )
        else:
            actions.append(navigation)
        return actions

    return [
        localization,
        RegisterEventHandler(
            OnProcessExit(
                target_action=localization_gate,
                on_exit=on_localization_gate_exit,
            )
        ),
        localization_gate,
    ]


def generate_launch_description():
    """声明导航启动参数并构造可选 RViz 的启动描述"""
    package_share = get_package_share_directory("car_navigation")
    rviz_config = os.path.join(package_share, "rviz", "navigation.rviz")
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value="maps/map.yaml"),
            DeclareLaunchArgument("navigation_mode", default_value="classic"),
            DeclareLaunchArgument(
                "navigation_start_delay_sec",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "localization_start_timeout_sec",
                default_value="10.0",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            OpaqueFunction(function=launch_stack),
            Node(
                package="rviz2",
                executable="rviz2",
                name="navigation_rviz",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                output="screen",
            ),
        ]
    )
