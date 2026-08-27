# 必选：一条命令获取、验证并部署 car_rl 模型。

import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _project_root():
    """从显式环境或 car_rl 安装位置确定 Jetson 工作区绝对路径。"""
    configured = os.environ.get("JETSON_CAR_ROOT", "").strip()
    if configured:
        root = Path(configured).expanduser().resolve()
        if not root.is_dir():
            raise RuntimeError("JETSON_CAR_ROOT 不是有效目录")
        return root

    share = Path(get_package_share_directory("car_rl")).resolve()
    for candidate in (share, *share.parents):
        if candidate.name == "install":
            return candidate.parent
    raise RuntimeError("无法从 car_rl 安装位置确定 Jetson 项目目录")


def _deploy(context):
    """同步执行一次性部署程序，使失败状态传递给 ros2 launch。"""
    project_root = _project_root()
    prefix = Path(get_package_prefix("car_rl")).resolve()
    worker = prefix / "lib" / "car_rl" / "get_model_worker"
    model_tool = prefix / "lib" / "car_rl" / "model_tool"
    if not worker.is_file():
        raise RuntimeError(f"未找到模型部署程序：{worker}")
    if not model_tool.is_file():
        raise RuntimeError(f"未找到 model_tool：{model_tool}")

    command = [
        str(worker),
        "--project-root",
        str(project_root),
        "--model-tool",
        str(model_tool),
        "--class",
        LaunchConfiguration("class").perform(context).strip(),
    ]
    archive = LaunchConfiguration("archive").perform(context).strip()
    if archive:
        command.extend(["--archive", archive])
    result = subprocess.run(command, check=False)
    if result.returncode != 0:
        raise RuntimeError(f"模型部署失败，退出码={result.returncode}")
    return [LogInfo(msg="car_rl 模型获取和部署完成")]


def generate_launch_description():
    """声明可选模型路径并执行完整部署事务。"""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "class",
                default_value="controller",
                description="模型种类，当前只支持 controller",
            ),
            DeclareLaunchArgument(
                "archive",
                default_value="",
                description="可选的 car_rl_model.zip 绝对路径",
            ),
            OpaqueFunction(function=_deploy),
        ]
    )
