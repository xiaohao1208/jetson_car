import math
from pathlib import Path
import subprocess
import threading
import time
import uuid

from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
import rclpy
from rclpy.executors import MultiThreadedExecutor
import uvicorn
import yaml

from car_web.map_store import MapPoseStore
from car_web.map_lifecycle import (
    remove_temporary_map,
    replace_saved_map,
)
from car_web.navigation_modes import NavigationModeRegistry
from car_web.navigation_state import UNFINISHED_NAVIGATION_STATES
from car_web.process_manager import OwnedProcessManager
from car_web.ros_bridge import CarWebBridge
from car_web.stack_commands import (
    mapping_launch_command,
    navigation_launch_command,
)


def _workspace_root():
    """从安装位置或源码位置解析当前项目根目录"""
    """从安装前缀反推出 colcon 工作区，避免写死当前用户目录"""
    share = Path(get_package_share_directory("car_web"))
    for parent in share.parents:
        if parent.name == "install":
            return parent.parent
    return Path.cwd()


def _resolve_map_path(value):
    """把网页参数中的地图路径解析为绝对路径"""
    path = Path(value).expanduser()
    return path if path.is_absolute() else _workspace_root() / path


class AccessLogThrottle:
    """限制重复GET访问日志的输出频率"""

    def __init__(self, interval_sec=5.0):
        """初始化按客户端、方法和路径分组的访问日志节流器"""
        self._interval_sec = float(interval_sec)
        self._last_output = {}
        self._lock = threading.Lock()

    def should_log(self, method, path, status_code, client, now=None):
        """判断本次访问是否应记录且错误响应始终记录"""
        if method != "GET" or status_code >= 400:
            return True
        current = time.monotonic() if now is None else float(now)
        key = (client, method, path, int(status_code))
        with self._lock:
            previous = self._last_output.get(key)
            if (
                previous is not None
                and current - previous < self._interval_sec
            ):
                return False
            self._last_output[key] = current
        return True


class WebApplication:
    """组合 ROS 桥、进程管理、地图存储和 HTTP 接口"""

    def __init__(self, bridge):
        """组合现有子系统并注册保持兼容的 HTTP 接口"""
        self.bridge = bridge
        self.processes = OwnedProcessManager()
        self._mapping_command = mapping_launch_command()
        self.processes.cleanup_orphaned(self._mapping_command)
        self.map_path = _resolve_map_path(
            bridge.get_parameter("map_file").value
        )
        self.pose_store = MapPoseStore(self.map_path)
        self._pose_save_lock = threading.Lock()
        saved_pose = self.pose_store.load()
        self._last_saved_pose = dict(saved_pose) if saved_pose else None
        self._last_saved_map_id = (
            self.pose_store.map_id() if saved_pose else None
        )
        if saved_pose:
            self.bridge.set_initial_pose(saved_pose, retry=False)
        self.goals = []
        self._lock = threading.RLock()
        self._mapping_operation_lock = threading.Lock()
        self._mapping_operation = "idle"
        self._last_mapping_error = ""
        self._navigation_mode = "classic"
        self._navigation_phase = "idle"
        self._navigation_error = ""
        self._navigation_generation = 0
        self._navigation_monitor_thread = None
        self._navigation_process_lock = threading.Lock()
        self._navigation_start_timeout = max(
            0.1,
            float(
                bridge.get_parameter(
                    "navigation_start_timeout_sec"
                ).value
            ),
        )
        self._navigation_start_retry_count = max(
            0,
            int(
                bridge.get_parameter(
                    "navigation_start_retry_count"
                ).value
            ),
        )
        self._navigation_retry_delay = max(
            0.0,
            float(
                bridge.get_parameter(
                    "navigation_retry_delay_sec"
                ).value
            ),
        )
        self.navigation_modes = NavigationModeRegistry()
        self._access_log_throttle = AccessLogThrottle(5.0)
        self.bridge.set_navigation_terminal_callback(
            self._on_navigation_terminal
        )
        self.app = FastAPI(title="ROS2 Car", version="0.1.0")
        self._configure_access_logging()
        self._configure_exception_handlers()
        self._configure_static_files()
        self._configure_routes()

    def _configure_access_logging(self):
        """安装只改变日志频率而不改变请求处理的中间件"""
        @self.app.middleware("http")
        async def access_logging(request: Request, call_next):
            """保持请求处理不变并按客户端和接口节流访问日志"""
            response = await call_next(request)
            client = request.client.host if request.client else "-"
            if self._access_log_throttle.should_log(
                request.method,
                request.url.path,
                response.status_code,
                client,
            ):
                self.bridge.get_logger().info(
                    f"网页访问，客户端={client}，"
                    f"请求={request.method} {request.url.path}，"
                    f"状态={response.status_code}"
                )
            return response

    def _configure_exception_handlers(self):
        """把未处理异常统一转换为中文 JSON 响应"""
        @self.app.exception_handler(Exception)
        async def unhandled_exception(_: Request, error: Exception):
            """把未处理异常统一转换为可解析的 JSON 响应"""
            self.bridge.get_logger().error(
                f"网页服务发生未处理异常：{error}"
            )
            return JSONResponse(
                status_code=500,
                content={
                    "ok": False,
                    "detail": "服务内部错误，请查看Jetson日志",
                },
            )

    def _configure_static_files(self):
        """挂载网页模板和静态资源目录"""
        share = Path(get_package_share_directory("car_web"))
        self._index = share / "templates" / "index.html"
        self.app.mount(
            "/static",
            StaticFiles(directory=str(share / "static")),
            name="static",
        )

    @staticmethod
    def _ok(**payload):
        """构造带 ok=true 的统一成功响应"""
        return {"ok": True, **payload}

    @staticmethod
    def _validate_pose(body):
        """解析并校验有限的地图 X、Y 和弧度偏航"""
        try:
            pose = {
                "x": float(body["x"]),
                "y": float(body["y"]),
                "yaw": float(body.get("yaw", 0.0)),
            }
        except (KeyError, TypeError, ValueError) as error:
            raise HTTPException(400, "位姿必须包含有限的x、y、yaw") from error
        if not all(math.isfinite(value) for value in pose.values()):
            raise HTTPException(400, "位姿不能包含NaN或Inf")
        return pose

    def _navigation_mode_name(self, mode):
        """返回导航模式的中文显示名称"""
        return self.navigation_modes.name(mode)

    def _rl_status(self):
        """返回经过短时缓存的强化学习部署状态"""
        return self.navigation_modes.status()

    def _check_navigation_mode(self, mode):
        """拒绝未知或当前模型条件不满足的导航模式"""
        reason = self.navigation_modes.unavailable_reason(mode)
        if not reason:
            return
        status_code = 400 if not self.navigation_modes.contains(mode) else 409
        raise HTTPException(status_code, reason)

    def _mapping_status(self, stacks=None):
        """合并受管进程和 ROS 图得到稳定建图阶段"""
        stacks = stacks or self.bridge.stack_state()
        owned = self.processes.running("mapping")
        external = bool(stacks["mapping"] and not owned)
        with self._lock:
            operation = self._mapping_operation
            error = self._last_mapping_error
        if operation in {"saving", "stopping"}:
            state = operation
        elif owned and stacks["mapping"]:
            state = "running"
        elif owned:
            state = "starting"
        elif error:
            state = "error"
        else:
            state = "idle"
        return {
            "mapping_active": owned,
            "mapping_owned": owned,
            "mapping_state": state,
            "external_mapping_detected": external,
            "last_mapping_error": error,
        }

    def _require_navigation_ready(self):
        """要求导航进程、Nav2 Action 和地图 TF 全部就绪"""
        running, phase, error = self._navigation_runtime_status()
        if not running:
            raise HTTPException(409, "导航栈尚未启动")
        if phase in {"starting", "retrying", "stopping", "error"}:
            raise HTTPException(
                409,
                error or "后台导航尚未准备就绪",
            )
        if not self.bridge.navigation_ready():
            raise HTTPException(409, "后台导航尚未准备就绪")
        state = self.bridge.status()
        if state["e_stop"]:
            raise HTTPException(409, "请先解除急停")
        return state

    def _require_navigation_editable(self):
        """要求当前阶段允许修改导航目标"""
        state = self._require_navigation_ready()
        if (
            state["navigation"].get("state")
            in UNFINISHED_NAVIGATION_STATES
        ):
            raise HTTPException(409, "导航任务尚未结束，不能修改位置或目标")
        return state

    def _require_initial_pose_editable(self):
        """要求当前阶段允许设置当前位置"""
        if self.processes.running("mapping"):
            raise HTTPException(409, "建图进行中，不能设置当前位置")
        running, phase, _ = self._navigation_runtime_status()
        if phase in {"starting", "retrying", "stopping"}:
            raise HTTPException(409, "后台导航尚未准备就绪")
        if running:
            self._require_navigation_editable()

    def _remaining_navigation_goals(self, navigation):
        """返回排除已完成和已失败目标后的待执行队列"""
        with self._lock:
            goals = list(self.goals)
        try:
            completed = int(navigation.get("completed_waypoints", 0))
        except (TypeError, ValueError):
            completed = 0
        completed = max(0, min(completed, len(goals)))
        return goals, completed, goals[completed:]

    @staticmethod
    def _normalized_pose(pose):
        """把位姿转换为稳定浮点字段并归一化偏航"""
        try:
            normalized = {
                key: float(pose[key])
                for key in ("x", "y", "yaw")
            }
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("位姿数据不完整") from error
        if not all(math.isfinite(value) for value in normalized.values()):
            raise ValueError("位姿包含无效数值")
        return normalized

    @staticmethod
    def _same_pose(first, second):
        """使用保存精度判断两个位姿是否等价"""
        return (
            first is not None
            and second is not None
            and all(
                abs(first[key] - second[key]) <= 1.0e-6
                for key in ("x", "y", "yaw")
            )
        )

    def _persist_pose(self, pose, reason):
        """在地图存在且位姿变化时原子记录当前位置"""
        try:
            normalized = self._normalized_pose(pose)
            with self._pose_save_lock:
                map_id = self.pose_store.map_id()
                if map_id is None:
                    return None
                if (
                    map_id == self._last_saved_map_id
                    and self._same_pose(
                        normalized,
                        self._last_saved_pose,
                    )
                ):
                    return normalized
                payload = self.pose_store.save(normalized)
                self._last_saved_map_id = payload["map_id"]
                self._last_saved_pose = dict(normalized)
            return normalized
        except (
            KeyError,
            OSError,
            RuntimeError,
            TypeError,
            ValueError,
            yaml.YAMLError,
        ) as error:
            self.bridge.get_logger().warning(
                f"位姿记录失败，阶段={reason}，原因={error}"
            )
            return None

    def _record_current_pose(self, reason, allow_mapping=False):
        """读取桥接层当前位姿并按阶段策略持久化"""
        if (
            not allow_mapping
            and self.processes.running("mapping")
        ):
            return None
        pose = self.bridge.robot_pose()
        if pose is None:
            return None
        return self._persist_pose(pose, reason)

    def _on_navigation_terminal(self, state):
        """在导航任务到达终态时记录小车当前位置"""
        self._record_current_pose(f"导航任务结束，状态={state}")

    def _navigation_runtime_status(self):
        """合并进程和桥接层就绪状态并识别意外退出"""
        running = self.processes.running("navigation")
        unexpected_exit = False
        with self._lock:
            if self._navigation_phase == "ready" and not running:
                self._navigation_phase = "error"
                self._navigation_error = "导航进程意外退出"
                unexpected_exit = True
            phase = self._navigation_phase
            error = self._navigation_error
        if phase == "idle" and running and self.bridge.navigation_ready():
            with self._lock:
                if self._navigation_phase == "idle":
                    self._navigation_phase = "ready"
                    self._navigation_error = ""
                phase = self._navigation_phase
                error = self._navigation_error
        if unexpected_exit:
            self.bridge.end_localization()
            self.bridge.get_logger().error("导航进程意外退出")
        return running, phase, error

    def _navigation_busy(self):
        """判断任一导航启动、运行或停止阶段是否占用资源"""
        running, phase, _ = self._navigation_runtime_status()
        return (
            running
            or self.bridge.stack_state()["navigation"]
            or phase in {
                "starting",
                "retrying",
                "ready",
                "stopping",
            }
        )

    def _navigation_generation_matches(self, generation):
        """判断后台线程是否仍属于当前导航启动世代"""
        with self._lock:
            return generation == self._navigation_generation

    def _set_navigation_phase(self, generation, phase, error=""):
        """只允许当前世代更新导航阶段和错误原因"""
        with self._lock:
            if generation != self._navigation_generation:
                return False
            self._navigation_phase = str(phase)
            self._navigation_error = str(error)
            return True

    def _navigation_command(self, mode):
        """构造导航进程命令且不在网页服务内复制 launch 细节"""
        return navigation_launch_command(self.map_path, mode)

    def _start_navigation_process(self, mode, saved_pose):
        """启动导航进程、进入定位态并恢复保存位姿"""
        pid = self.processes.start(
            "navigation",
            self._navigation_command(mode),
        )
        self.bridge.begin_localization()
        if saved_pose:
            self.bridge.set_initial_pose(saved_pose, retry=True)
        return pid

    def _wait_navigation_stack_exit(self, generation, timeout_sec=5.0):
        """等待受管进程和 ROS 导航节点全部退出"""
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while time.monotonic() < deadline:
            if not self._navigation_generation_matches(generation):
                return True
            if (
                not self.processes.running("navigation")
                and not self.bridge.stack_state()["navigation"]
            ):
                return True
            time.sleep(0.1)
        return (
            not self.processes.running("navigation")
            and not self.bridge.stack_state()["navigation"]
        )

    def _wait_navigation_retry_delay(self, generation):
        """可被世代变化提前终止地等待自动重试间隔"""
        deadline = time.monotonic() + self._navigation_retry_delay
        while time.monotonic() < deadline:
            if not self._navigation_generation_matches(generation):
                return False
            time.sleep(min(0.1, deadline - time.monotonic()))
        return self._navigation_generation_matches(generation)

    def _stop_failed_navigation_start(self, generation):
        """停止失败启动产生的进程、定位和任务状态"""
        with self._navigation_process_lock:
            if not self._navigation_generation_matches(generation):
                return True
            self.bridge.stop_initial_pose_retry()
            stopped = self.processes.stop("navigation")
            self.bridge.end_localization()
            self.bridge.reset_navigation()
        if not stopped and self.processes.running("navigation"):
            return False
        return self._wait_navigation_stack_exit(generation)

    def _fail_navigation_start(self, generation, message):
        """记录当前世代不可恢复的导航启动错误"""
        if self._set_navigation_phase(
            generation,
            "error",
            message,
        ):
            self.bridge.get_logger().error(message)

    def _monitor_navigation_start(self, generation, mode, saved_pose):
        """等待导航就绪并按配置完成清理和有限次数重试"""
        retry_index = 0
        while self._navigation_generation_matches(generation):
            deadline = time.monotonic() + self._navigation_start_timeout
            failure = "导航栈启动超时"
            while time.monotonic() < deadline:
                if not self._navigation_generation_matches(generation):
                    return
                if not self.processes.running("navigation"):
                    failure = "导航进程在启动完成前退出"
                    break
                if self.bridge.navigation_ready():
                    if self._set_navigation_phase(
                        generation,
                        "ready",
                    ):
                        self.bridge.get_logger().info("导航后台已准备就绪")
                    return
                time.sleep(0.25)

            if retry_index >= self._navigation_start_retry_count:
                self._set_navigation_phase(generation, "stopping")
                clean = self._stop_failed_navigation_start(generation)
                if not self._navigation_generation_matches(generation):
                    return
                if not clean:
                    failure = f"{failure}，导航进程未能完全退出"
                self._fail_navigation_start(generation, failure)
                return

            retry_index += 1
            if not self._set_navigation_phase(generation, "retrying"):
                return
            self.bridge.get_logger().warning(
                f"{failure}，正在清理并进行第{retry_index}次重试"
            )
            if not self._stop_failed_navigation_start(generation):
                self._fail_navigation_start(
                    generation,
                    "导航进程未能完全退出，已取消自动重试",
                )
                return
            if not self._wait_navigation_retry_delay(generation):
                return
            try:
                with self._navigation_process_lock:
                    if not self._navigation_generation_matches(generation):
                        return
                    self._start_navigation_process(mode, saved_pose)
            except RuntimeError as error:
                self._fail_navigation_start(
                    generation,
                    f"导航自动重试启动失败：{error}",
                )
                return

    def _start_navigation_monitor(self, generation, mode, saved_pose):
        """创建不阻塞 FastAPI 请求线程的启动监督线程"""
        monitor = threading.Thread(
            target=self._monitor_navigation_start,
            args=(generation, mode, saved_pose),
            daemon=True,
            name=f"navigation-start-{generation}",
        )
        with self._lock:
            self._navigation_monitor_thread = monitor
        monitor.start()

    def _replace_saved_map(self, temporary_prefix):
        """验证并原子替换正式地图及其位姿缓存"""
        replace_saved_map(
            self.map_path,
            temporary_prefix,
            getattr(self, "pose_store", None),
        )

    @staticmethod
    def _remove_temporary_map(temporary_prefix):
        """删除一次地图保存留下的 YAML 和 PGM 临时文件"""
        remove_temporary_map(temporary_prefix)

    def _configure_routes(self):
        """注册保持兼容的页面、状态、运动、建图和导航接口"""
        @self.app.get("/")
        def index():
            """返回小车控制页面"""
            return FileResponse(self._index)

        @self.app.get("/api/status")
        def status():
            """汇总底盘、传感器、建图、导航和 RL 状态"""
            payload = self.bridge.status()
            stacks = self.bridge.stack_state()
            mapping = self._mapping_status(stacks)
            (
                navigation_active,
                navigation_phase,
                navigation_error,
            ) = self._navigation_runtime_status()
            payload.update(
                {
                    "navigation_active": navigation_active,
                    "navigation_owned": navigation_active,
                    "navigation_ready": (
                        navigation_active
                        and navigation_phase == "ready"
                        and self.bridge.navigation_ready()
                    ),
                    "navigation_busy": (
                        navigation_active
                        or stacks["navigation"]
                        or navigation_phase in {
                            "starting",
                            "retrying",
                            "ready",
                            "stopping",
                        }
                    ),
                    "navigation_phase": navigation_phase,
                    "navigation_error": navigation_error,
                    "navigation_mode": self._navigation_mode,
                    "map_exists": self.map_path.is_file(),
                    "rl": self._rl_status(),
                    **mapping,
                }
            )
            return self._ok(**payload)

        @self.app.get("/api/map")
        def map_data():
            """返回实时地图或已保存地图及其位姿和目标点"""
            payload = self.bridge.map_payload()
            if (
                not payload.get("available")
                and self.map_path.is_file()
                and not self.processes.running("mapping")
            ):
                saved_payload = self.pose_store.map_payload()
                if saved_payload.get("available"):
                    payload = saved_payload
                    payload["robot_pose"] = self.bridge.robot_pose()
            with self._lock:
                payload["goals"] = list(self.goals)
            payload["saved_pose"] = self.pose_store.load()
            return self._ok(**payload)

        @self.app.post("/api/move")
        def move(body: dict):
            """校验并发布网页手动速度"""
            state = self.bridge.status()
            if not state["car_online"]:
                raise HTTPException(409, "小车离线，拒绝运动命令")
            try:
                linear = float(body.get("linear", 0.0))
                angular = float(body.get("angular", 0.0))
            except (TypeError, ValueError) as error:
                raise HTTPException(400, "速度参数无效") from error
            if not math.isfinite(linear) or not math.isfinite(angular):
                raise HTTPException(400, "速度不能包含NaN或Inf")
            self.bridge.publish_move(linear, angular)
            if abs(linear) < 1.0e-9 and abs(angular) < 1.0e-9:
                self._record_current_pose("手动移动结束")
            return self._ok()

        @self.app.post("/api/emergency-stop")
        def emergency_stop(body: dict):
            """设置急停状态并暂停正在执行的导航任务"""
            stop = bool(body.get("stop", True))
            try:
                result = self.bridge.emergency_stop(
                    stop,
                    str(body.get("reason", "Web操作")),
                )
            except RuntimeError as error:
                raise HTTPException(503, str(error)) from error
            paused = False
            if stop and result.get("success"):
                paused = self.bridge.pause_navigation()
                self._record_current_pose("急停")
            result["navigation_paused"] = paused
            return self._ok(**result)

        @self.app.post("/api/mapping/start")
        def start_mapping():
            """在传感器就绪且导航停止时启动建图"""
            if self._navigation_busy():
                raise HTTPException(409, "导航栈运行中，不能同时建图")
            if self.processes.running("mapping"):
                return self._ok(already_running=True, owned=True)
            state = self.bridge.status()
            if not state["sensor_ready"]:
                raise HTTPException(409, "需要新鲜的/scan和/odom才能建图")
            self.bridge.publish_move(0.0, 0.0)
            self._record_current_pose("开始建图")
            try:
                pid = self.processes.start(
                    "mapping",
                    self._mapping_command,
                )
            except RuntimeError as error:
                raise HTTPException(409, str(error)) from error
            with self._lock:
                self._mapping_operation = "starting"
                self._last_mapping_error = ""
            self.bridge.begin_mapping()
            return self._ok(pid=pid)

        @self.app.post("/api/mapping/save-stop")
        def save_and_stop_mapping():
            """原子保存当前地图并停止本网页拥有的建图进程"""
            if not self._mapping_operation_lock.acquire(blocking=False):
                raise HTTPException(409, "建图结束操作正在进行")
            save_error = ""
            stopped = False
            temporary_prefix = None
            try:
                self.bridge.publish_move(0.0, 0.0)
                with self._lock:
                    self._mapping_operation = "saving"
                    self._last_mapping_error = ""
                try:
                    map_payload = self.bridge.map_payload()
                    if not map_payload.get("available"):
                        raise RuntimeError(
                            map_payload.get("reason") or "尚未收到可保存的地图"
                        )
                    self.map_path.parent.mkdir(parents=True, exist_ok=True)
                    temporary_prefix = self.map_path.parent / (
                        f".{self.map_path.stem}-{uuid.uuid4().hex}-pending"
                    )
                    subprocess.run(
                        [
                            "ros2",
                            "run",
                            "nav2_map_server",
                            "map_saver_cli",
                            "-f",
                            str(temporary_prefix),
                        ],
                        check=True,
                        capture_output=True,
                        text=True,
                        timeout=15.0,
                    )
                    self._replace_saved_map(temporary_prefix)
                    self._record_current_pose(
                        "结束建图",
                        allow_mapping=True,
                    )
                except (
                    OSError,
                    RuntimeError,
                    subprocess.SubprocessError,
                ) as error:
                    save_error = str(error)
                finally:
                    if temporary_prefix is not None:
                        self._remove_temporary_map(temporary_prefix)

                with self._lock:
                    self._mapping_operation = "stopping"
                try:
                    stopped = self.processes.stop("mapping")
                except OSError as error:
                    with self._lock:
                        self._mapping_operation = "error"
                        self._last_mapping_error = f"建图停止失败: {error}"
                    raise HTTPException(
                        500, self._last_mapping_error
                    ) from error
                if not stopped and self.bridge.stack_state()["mapping"]:
                    with self._lock:
                        self._mapping_operation = "error"
                        self._last_mapping_error = (
                            "建图进程不属于当前网页，无法安全停止"
                        )
                    raise HTTPException(
                        409, "建图进程不属于当前网页，无法安全停止"
                    )
                self.bridge.end_mapping()
                with self._lock:
                    self._mapping_operation = "idle"
                    self._last_mapping_error = save_error
                if save_error:
                    return self._ok(
                        save_succeeded=False,
                        stopped=True,
                        message=f"建图已结束，地图保存失败: {save_error}",
                    )
                return self._ok(
                    save_succeeded=True,
                    stopped=True,
                    path=str(self.map_path),
                    message="地图已保存，建图已结束",
                )
            finally:
                self._mapping_operation_lock.release()

        @self.app.post("/api/navigation/start")
        def start_navigation(body: dict):
            """启动选定导航模式并异步等待定位栈和 Nav2 就绪"""
            mode = str(body.get("mode", "classic"))
            if not self.navigation_modes.contains(mode):
                raise HTTPException(400, f"未知导航模式: {mode}")
            (
                navigation_running,
                navigation_phase,
                navigation_error,
            ) = self._navigation_runtime_status()
            if navigation_phase == "stopping":
                raise HTTPException(409, "导航栈正在停止")
            if navigation_phase in {"starting", "retrying"}:
                if mode != self._navigation_mode:
                    raise HTTPException(
                        409,
                        "当前正在使用"
                        f"{self._navigation_mode_name(self._navigation_mode)}"
                        "，请结束导航后再切换",
                    )
                return self._ok(
                    already_running=True,
                    owned=True,
                    mode=self._navigation_mode,
                    phase=navigation_phase,
                )
            if navigation_running:
                if navigation_error:
                    raise HTTPException(409, navigation_error)
                if mode != self._navigation_mode:
                    raise HTTPException(
                        409,
                        "当前正在使用"
                        f"{self._navigation_mode_name(self._navigation_mode)}"
                        "，请结束导航后再切换",
                    )
                return self._ok(
                    already_running=True,
                    owned=True,
                    mode=self._navigation_mode,
                    phase=navigation_phase,
                )
            if self.bridge.stack_state()["navigation"]:
                raise HTTPException(
                    409,
                    navigation_error
                    or "检测到尚未退出的导航节点",
                )
            self._check_navigation_mode(mode)
            if not self.map_path.is_file():
                raise HTTPException(409, f"地图不存在: {self.map_path}")
            if self.processes.running("mapping"):
                raise HTTPException(409, "建图栈运行中，不能同时导航")
            self.bridge.publish_move(0.0, 0.0)
            saved_pose = (
                self._record_current_pose("开始导航")
                or self.pose_store.load()
            )
            with self._lock:
                self._navigation_generation += 1
                generation = self._navigation_generation
                self._navigation_phase = "starting"
                self._navigation_error = ""
                self._navigation_mode = mode
            try:
                with self._navigation_process_lock:
                    pid = self._start_navigation_process(mode, saved_pose)
            except RuntimeError as error:
                self._fail_navigation_start(
                    generation,
                    f"导航进程启动失败：{error}",
                )
                raise HTTPException(409, str(error)) from error
            self._start_navigation_monitor(generation, mode, saved_pose)
            return self._ok(
                pid=pid,
                restored_pose=bool(saved_pose),
                mode=mode,
                phase="starting",
            )

        @self.app.post("/api/navigation/stop")
        def stop_navigation():
            """停止导航栈并清除本轮导航目标"""
            with self._lock:
                self._navigation_generation += 1
                generation = self._navigation_generation
                self._navigation_phase = "stopping"
                self._navigation_error = ""
            with self._navigation_process_lock:
                self.bridge.cancel_navigation()
                self._record_current_pose("结束导航")
                stopped = self.processes.stop("navigation")
                self.bridge.end_localization()
                self.bridge.reset_navigation()
            clean = self._wait_navigation_stack_exit(generation)
            with self._lock:
                self.goals.clear()
                self._navigation_mode = "classic"
                if clean:
                    self._navigation_phase = "idle"
                else:
                    self._navigation_phase = "error"
                    self._navigation_error = "导航进程未能完全退出"
            return self._ok(
                stopped=bool(stopped or clean),
                goals=[],
                phase=self._navigation_phase,
            )

        @self.app.post("/api/navigation/initial-pose")
        def initial_pose(body: dict):
            """保存并向定位系统发布用户设置的当前位置"""
            self._require_initial_pose_editable()
            live_map = self.bridge.map_payload().get("available")
            if not self.map_path.is_file() and not live_map:
                raise HTTPException(409, "没有地图，不能设置当前位置")
            pose = self._validate_pose(body)
            if self.map_path.is_file():
                if self._persist_pose(pose, "设置当前位置") is None:
                    raise HTTPException(500, "当前位置保存失败")
            self.bridge.set_initial_pose(
                pose,
                retry=self.processes.running("navigation"),
            )
            return self._ok(pose=pose)

        @self.app.post("/api/navigation/goals")
        def add_goal(body: dict):
            """把一个目标点追加到按顺序执行的导航队列"""
            self._require_navigation_editable()
            pose = self._validate_pose(body)
            with self._lock:
                self.goals.append(pose)
                goals = list(self.goals)
            self.bridge.resize_navigation_queue(len(goals))
            return self._ok(goals=goals)

        @self.app.delete("/api/navigation/goals")
        def clear_goals():
            """在无活动任务时清空全部导航目标"""
            navigation = self.bridge.status()["navigation"]
            if navigation.get("state") in UNFINISHED_NAVIGATION_STATES:
                raise HTTPException(409, "导航任务尚未结束，不能清空目标")
            with self._lock:
                self.goals.clear()
            self.bridge.reset_navigation()
            return self._ok(goals=[])

        @self.app.delete("/api/navigation/goals/last")
        def remove_last_goal():
            """在无活动任务时移除最后一个导航目标"""
            navigation = self.bridge.status()["navigation"]
            if navigation.get("state") in UNFINISHED_NAVIGATION_STATES:
                raise HTTPException(409, "导航任务尚未结束，不能清除目标")
            with self._lock:
                if not self.goals:
                    raise HTTPException(409, "没有可清除的目标点")
                removed = self.goals.pop()
                goals = list(self.goals)
            if goals:
                self.bridge.resize_navigation_queue(len(goals))
            else:
                self.bridge.reset_navigation()
            return self._ok(goals=goals, removed=removed)

        @self.app.post("/api/navigation/run")
        def run_goals():
            """从首个未完成目标开始顺序执行导航"""
            state = self._require_navigation_ready()
            goals, completed, remaining = self._remaining_navigation_goals(
                state["navigation"]
            )
            if not remaining:
                raise HTTPException(409, "没有未完成的目标点")
            self._record_current_pose("开始导航任务")
            try:
                self.bridge.navigate(
                    remaining,
                    waypoint_offset=completed,
                    total_waypoints=len(goals),
                )
            except RuntimeError as error:
                raise HTTPException(409, str(error)) from error
            return self._ok(
                count=len(remaining),
                completed=completed,
                total=len(goals),
            )

        @self.app.post("/api/navigation/resume")
        def resume_navigation():
            """急停解除后继续原有导航任务"""
            state = self._require_navigation_ready()
            navigation = state["navigation"]
            if navigation.get("state") != "paused":
                raise HTTPException(409, "当前没有已暂停的导航任务")
            goals, completed, remaining = self._remaining_navigation_goals(
                navigation
            )
            if not remaining:
                raise HTTPException(409, "没有可继续的目标点")
            self._record_current_pose("继续导航任务")
            try:
                self.bridge.navigate(
                    remaining,
                    waypoint_offset=completed,
                    total_waypoints=len(goals),
                )
            except RuntimeError as error:
                raise HTTPException(409, str(error)) from error
            return self._ok(
                count=len(remaining),
                completed=completed,
                total=len(goals),
            )

        @self.app.post("/api/navigation/cancel")
        def cancel_navigation():
            """取消当前动作但保留目标队列"""
            canceled = self.bridge.cancel_navigation()
            return self._ok(canceled=canceled)

    def shutdown(self):
        """记录最终位姿并停止本网页实例拥有的后台进程"""
        with self._lock:
            self._navigation_generation += 1
            self._navigation_phase = "stopping"
        self.bridge.publish_move(0.0, 0.0)
        self._record_current_pose("Web退出")
        self.bridge.cancel_navigation()
        self.processes.stop_all()


def main(args=None):
    """启动 ROS 桥接线程和无访问刷屏的 Uvicorn 服务"""
    rclpy.init(args=args)
    bridge = CarWebBridge()
    application = WebApplication(bridge)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    host = str(bridge.get_parameter("web_host").value)
    port = int(bridge.get_parameter("web_port").value)
    try:
        uvicorn.run(
            application.app,
            host=host,
            port=port,
            log_level="info",
            access_log=False,
        )
    finally:
        application.shutdown()
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()
