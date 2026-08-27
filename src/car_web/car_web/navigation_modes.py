import json
from pathlib import Path
import subprocess
import threading
import time

from ament_index_python.packages import get_package_share_directory
import yaml


class NavigationModeRegistry:
    """检查经典和强化学习导航模式是否可以启动"""

    def __init__(self):
        """读取共享模式定义并初始化短时状态缓存"""
        config_path = (
            Path(get_package_share_directory("car_navigation"))
            / "config"
            / "navigation_modes.yaml"
        )
        with config_path.open(encoding="utf-8") as stream:
            self._modes = yaml.safe_load(stream)
        self._cache = None
        self._cache_time = 0.0
        self._lock = threading.RLock()

    def name(self, mode):
        """返回公开模式的中文名称"""
        definition = self._modes.get(mode, {})
        return definition.get("label", str(mode))

    def contains(self, mode):
        """返回模式 ID 是否存在于共享定义中"""
        return mode in self._modes

    def status(self):
        """获取模型工具状态并缓存十秒，避免频繁启动子进程"""
        now = time.monotonic()
        with self._lock:
            if (
                self._cache is not None
                and now - self._cache_time < 10.0
            ):
                return dict(self._cache)
        try:
            result = subprocess.run(
                ["ros2", "run", "car_rl", "model_tool", "status", "--json"],
                check=True,
                capture_output=True,
                text=True,
                timeout=8.0,
            )
            status = json.loads(result.stdout.strip().splitlines()[-1])
        except (
            OSError,
            subprocess.SubprocessError,
            ValueError,
            IndexError,
        ):
            status = {
                "available": False,
                "controller_available": False,
                "reason": "强化学习状态服务暂时不可用",
            }
        with self._lock:
            self._cache = status
            self._cache_time = time.monotonic()
        return dict(status)

    def unavailable_reason(self, mode):
        """返回模式不可启动的中文原因，空字符串表示可启动"""
        if mode == "classic":
            return ""
        if mode not in self._modes:
            return f"未知导航模式: {mode}"
        status = self.status()
        definition = self._modes[mode]
        if "controller_available" in status:
            controller_ready = bool(status["controller_available"])
        else:
            controller_ready = (
                status.get("bundle")
                and status.get("backend")
                and status.get("controller_engine")
            )
        if (
            not definition.get("requires_controller_engine")
            or controller_ready
        ):
            return ""
        return (
            status.get("reason")
            or "强化学习模型或推理环境尚未就绪"
        )
