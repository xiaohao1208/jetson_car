"""Web 访问日志的低噪声节流策略。"""

import threading
import time


class AccessLogThrottle:
    """按方法、路径和状态限制重复 GET 日志。"""

    def __init__(self, interval_sec=5.0):
        self._interval_sec = float(interval_sec)
        self._last_output = {}
        self._lock = threading.Lock()

    def should_log(self, method, path, status_code, now=None):
        """错误和写请求始终记录，其余请求按时间窗口节流。"""

        if method != "GET" or status_code >= 400:
            return True
        current = time.monotonic() if now is None else float(now)
        key = (method, path, int(status_code))
        with self._lock:
            previous = self._last_output.get(key)
            if previous is not None and current - previous < self._interval_sec:
                return False
            self._last_output[key] = current
        return True
