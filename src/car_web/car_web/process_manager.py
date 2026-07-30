import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


class OwnedProcessManager:
    """管理当前 robot_bringup 会话启动的进程组"""

    def __init__(self, state_file=None):
        """加载受管进程身份并清理上一轮失去父进程的任务"""
        runtime_root = Path(
            state_file or f"/tmp/car_web-{os.getuid()}/processes.json"
        )
        self._state_file = runtime_root
        self._processes = {}
        self._lock = threading.RLock()
        self._load_state()
        self._reconcile_previous_launch()

    @staticmethod
    def _process_start_time(pid):
        """读取 Linux 进程启动 tick 以防 PID 复用"""
        try:
            fields = Path(f"/proc/{pid}/stat").read_text(
                encoding="utf-8"
            ).split()
            return fields[21]
        except (OSError, IndexError):
            return None

    @staticmethod
    def _process_command(pid):
        """读取目标进程未经 Shell 拼接的参数数组"""
        try:
            data = Path(f"/proc/{pid}/cmdline").read_bytes()
        except OSError:
            return []
        return [
            part.decode("utf-8", errors="replace")
            for part in data.split(b"\0")
            if part
        ]

    @staticmethod
    def _process_group_alive(process_group):
        """检查进程组中是否仍存在非僵尸进程"""
        for proc_path in Path("/proc").iterdir():
            if not proc_path.name.isdigit():
                continue
            try:
                fields = (proc_path / "stat").read_text(
                    encoding="utf-8"
                ).rsplit(") ", 1)[1].split()
                state = fields[0]
                group = int(fields[2])
            except (OSError, IndexError, ValueError):
                continue
            if group == process_group and state != "Z":
                return True
        return False

    @staticmethod
    def _command_matches(actual, expected):
        """按顺序匹配关键命令参数并允许可执行文件使用绝对路径"""
        if not actual or not expected:
            return False
        position = 0
        for token in actual:
            expected_token = expected[position]
            matches = token == expected_token
            if "/" not in expected_token:
                matches = matches or Path(token).name == expected_token
            if matches:
                position += 1
                if position == len(expected):
                    return True
        return False

    def _record_alive(self, record):
        """同时核对 PID、启动时间和命令以确认进程身份"""
        pid = int(record.get("pid", 0))
        expected_start = str(record.get("start_time", ""))
        expected_command = list(record.get("command", []))
        return (
            pid > 1
            and self._process_start_time(pid) == expected_start
            and self._command_matches(
                self._process_command(pid), expected_command
            )
        )

    def _load_state(self):
        """加载上次持久化且仍能确认身份的进程记录"""
        try:
            payload = json.loads(
                self._state_file.read_text(encoding="utf-8")
            )
        except (OSError, ValueError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        for name, record in payload.items():
            if isinstance(record, dict) and self._record_alive(record):
                self._processes[str(name)] = record

    def _save_state(self):
        """通过临时文件原子保存当前受管进程记录"""
        self._state_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._state_file.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(self._processes, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        os.replace(temporary, self._state_file)

    def _remove_record(self, name):
        """删除指定受管任务并立即持久化"""
        with self._lock:
            self._processes.pop(name, None)
            self._save_state()

    def _signal_and_wait(self, record, timeout=5.0):
        """逐级向已确认身份的独立进程组发信号并等待退出"""
        pid = int(record["pid"])
        process_group = int(record.get("process_group", pid))
        if not self._record_alive(record):
            return not self._process_group_alive(process_group)
        if process_group != pid or process_group <= 1:
            return False
        for sig, wait_time in (
            (signal.SIGINT, timeout),
            (signal.SIGTERM, max(1.0, timeout)),
            (signal.SIGKILL, 2.0),
        ):
            try:
                os.killpg(pid, sig)
            except ProcessLookupError:
                return True
            deadline = time.monotonic() + wait_time
            while time.monotonic() < deadline:
                if not self._process_group_alive(process_group):
                    return True
                time.sleep(0.05)
        return not self._process_group_alive(process_group)

    def _reconcile_previous_launch(self):
        """停止不属于当前 robot_bringup 父进程的旧受管任务"""
        current_parent = os.getppid()
        stale = []
        with self._lock:
            for name, record in self._processes.items():
                if int(record.get("owner_parent_pid", 0)) != current_parent:
                    stale.append((name, record))
        for name, record in stale:
            if self._signal_and_wait(record):
                self._remove_record(name)

    def cleanup_orphaned(self, expected_command):
        """清理父进程已经退出的本项目旧启动命令"""
        cleaned = []
        expected = list(expected_command)
        for proc_path in Path("/proc").iterdir():
            if not proc_path.name.isdigit():
                continue
            pid = int(proc_path.name)
            if pid <= 1 or pid == os.getpid():
                continue
            try:
                status_lines = (proc_path / "status").read_text(
                    encoding="utf-8"
                ).splitlines()
                parent_line = next(
                    line for line in status_lines if line.startswith("PPid:")
                )
                parent_pid = int(parent_line.split()[1])
            except (OSError, StopIteration, ValueError):
                continue
            if parent_pid != 1:
                continue
            actual = self._process_command(pid)
            if not self._command_matches(actual, expected):
                continue
            try:
                process_group = os.getpgid(pid)
            except ProcessLookupError:
                continue
            if process_group != pid:
                continue
            record = {
                "pid": pid,
                "start_time": self._process_start_time(pid),
                "command": expected,
                "process_group": process_group,
            }
            if self._signal_and_wait(record):
                cleaned.append(pid)
        return cleaned

    def start(self, name, command):
        """启动命令并持久记录进程身份"""
        with self._lock:
            if self.running(name):
                raise RuntimeError(f"{name}已经由当前Web实例启动")
            process = subprocess.Popen(command, start_new_session=True)
            start_time = None
            deadline = time.monotonic() + 1.0
            while start_time is None and time.monotonic() < deadline:
                start_time = self._process_start_time(process.pid)
                if start_time is None:
                    time.sleep(0.01)
            if start_time is None:
                process.terminate()
                raise RuntimeError(f"{name}进程启动后无法确认身份")
            self._processes[name] = {
                "pid": process.pid,
                "start_time": start_time,
                "command": list(command),
                "owner_parent_pid": os.getppid(),
                "process_group": process.pid,
            }
            self._save_state()
            return process.pid

    def running(self, name):
        """返回任务是否仍运行并自动清除失效记录"""
        with self._lock:
            record = self._processes.get(name)
            if record is None:
                return False
            if self._record_alive(record):
                return True
            self._processes.pop(name, None)
            self._save_state()
            return False

    def stop(self, name, timeout=5.0):
        """逐级停止已验证身份的受管进程组"""
        with self._lock:
            record = self._processes.get(name)
        if record is None:
            return False
        stopped = self._signal_and_wait(record, timeout)
        if stopped:
            self._remove_record(name)
        return stopped

    def stop_all(self):
        """尽力停止本实例记录的全部进程组"""
        with self._lock:
            names = list(self._processes)
        for name in names:
            try:
                self.stop(name)
            except (OSError, subprocess.SubprocessError):
                pass
