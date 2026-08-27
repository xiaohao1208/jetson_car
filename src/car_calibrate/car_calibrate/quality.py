"""不依赖 ROS 的标定数据质量统计。"""

from __future__ import annotations

import math

import numpy as np


def classify_imu_freshness(
    age: float,
    warning_timeout: float,
    abort_timeout: float,
):
    """把 IMU 数据年龄分为新鲜、短时空窗和必须中止。"""

    age = float(age)
    warning_timeout = float(warning_timeout)
    abort_timeout = float(abort_timeout)
    if not all(math.isfinite(value) for value in (
        age, warning_timeout, abort_timeout
    )):
        raise ValueError("IMU 数据年龄和超时必须为有限值")
    if age < 0.0 or warning_timeout <= 0.0 or abort_timeout <= warning_timeout:
        raise ValueError("IMU 超时必须满足 0 < 告警门限 < 中止门限")
    if age > abort_timeout:
        return "abort"
    if age > warning_timeout:
        return "stale"
    return "fresh"


def select_mcu_timeout(
    linear: float,
    angular: float,
    stationary_timeout: float,
    motion_timeout: float,
):
    """根据当前请求是否运动返回 MCU 超时门限及中文阶段标签。"""

    moving = abs(float(linear)) > 1.0e-9 or abs(float(angular)) > 1.0e-9
    if moving:
        return float(motion_timeout), "运动"
    return float(stationary_timeout), "静止"


def update_wheel_stall_timers(
    starts,
    *,
    now: float,
    request_changed: float,
    linear: float,
    angular: float,
    wheel_separation: float,
    left_feedback: float,
    right_feedback: float,
    startup_grace: float,
    response_speed: float,
    stall_timeout: float,
):
    """更新左右轮无响应计时器，返回新状态和已超时的车轮。"""

    timers = {
        "left": float(starts.get("left", 0.0)),
        "right": float(starts.get("right", 0.0)),
    }
    targets = {
        "left": float(linear) - float(angular) * float(wheel_separation) * 0.5,
        "right": float(linear) + float(angular) * float(wheel_separation) * 0.5,
    }
    feedback = {
        "left": float(left_feedback),
        "right": float(right_feedback),
    }
    request_age = float(now) - float(request_changed)
    if request_age <= float(startup_grace) or not any(
        abs(value) > 1.0e-9 for value in targets.values()
    ):
        return {"left": 0.0, "right": 0.0}, ()

    failed = []
    for wheel in ("left", "right"):
        commanded = abs(targets[wheel]) > 1.0e-9
        responding = abs(feedback[wheel]) >= float(response_speed)
        if not commanded or responding:
            timers[wheel] = 0.0
            continue
        if not timers[wheel]:
            timers[wheel] = float(now)
        elif float(now) - timers[wheel] >= float(stall_timeout):
            failed.append(wheel)
    return timers, tuple(failed)


def frequency_window(message_times, start: float, end: float):
    """返回窗口内各话题频率和最大相邻间隔。"""

    duration = float(end) - float(start)
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("频率统计窗口必须为有限正数")
    frequencies = {}
    maximum_gaps = {}
    for name, raw_values in message_times.items():
        values = sorted(
            float(value)
            for value in raw_values
            if start <= float(value) <= end and math.isfinite(float(value))
        )
        frequencies[name] = len(values) / duration
        maximum_gaps[name] = max(
            (right - left for left, right in zip(values, values[1:])),
            default=math.inf,
        )
    return frequencies, maximum_gaps


def stationary_imu_metrics(values):
    """计算静止陀螺仪的稳健零偏、噪声、尾部残差和短时漂移。"""

    samples = np.asarray(list(values), dtype=np.float64)
    if len(samples) < 20 or not np.isfinite(samples).all():
        raise ValueError("IMU 静止样本必须包含至少 20 个有限值")
    bias = float(np.median(samples))
    residual = samples - bias
    midpoint = len(samples) // 2
    drift = abs(
        float(np.median(samples[:midpoint]))
        - float(np.median(samples[midpoint:]))
    )
    return {
        "bias": bias,
        "noise": float(np.std(residual)),
        "residual_p99": float(np.percentile(np.abs(residual), 99)),
        "drift": drift,
        "samples": int(len(samples)),
    }


def stationary_imu_failure(
    metrics,
    *,
    bias_limit: float,
    noise_limit: float,
    residual_p99_limit: float,
    drift_limit: float,
):
    """返回静止 IMU 门禁失败原因；空字符串表示可安全修正零偏。"""

    if abs(float(metrics["bias"])) > float(bias_limit):
        return "bias"
    if float(metrics["noise"]) > float(noise_limit):
        return "noise"
    if float(metrics["residual_p99"]) > float(residual_p99_limit):
        return "residual"
    if float(metrics["drift"]) > float(drift_limit):
        return "drift"
    return ""
