"""从自动标定 CSV 行估算训练所需动力学参数。"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Iterable

import numpy as np


REQUIRED_COLUMNS = (
    "timestamp_sec",
    "command_linear_mps",
    "command_angular_radps",
    "left_target_radps",
    "right_target_radps",
    "left_feedback_radps",
    "right_feedback_radps",
    "encoder_left_ticks",
    "encoder_right_ticks",
    "imu_yaw_rad",
    "phase_id",
    "segment",
    "trial_id",
    "request_timestamp_sec",
    "left_feedback_mps",
    "right_feedback_mps",
    "imu_bias_corrected_radps",
)


class CalibrationQualityError(ValueError):
    """原始数据无法通过自动标定质量门禁。"""


@dataclass(frozen=True)
class EstimateResult:
    """聚合 YAML 字段以及训练端可复算的 trial 分布。"""

    calibration: dict
    distributions: dict[str, dict[str, float | int]]
    valid_trials: dict[str, int]


def sha256_file(path: Path) -> str:
    """流式计算文件 SHA-256。"""

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _percentiles(values: Iterable[float]) -> dict[str, float | int]:
    array = np.asarray(list(values), dtype=np.float64)
    array = array[np.isfinite(array)]
    if not len(array):
        raise CalibrationQualityError("标定指标没有有限样本")
    return {
        "p10": float(np.percentile(array, 10)),
        "p50": float(np.percentile(array, 50)),
        "p90": float(np.percentile(array, 90)),
        "count": int(len(array)),
    }


def _two_sample_crossing(time, values, threshold, direction):
    compare = np.greater_equal if direction > 0 else np.less_equal
    hits = compare(values, threshold)
    for index in range(max(0, len(hits) - 1)):
        if bool(hits[index]) and bool(hits[index + 1]):
            return float(time[index])
    return None


def _slope_percentile(time, values, start, end, positive):
    mask = (values - start) * np.sign(end - start)
    magnitude = abs(end - start)
    selected = (mask >= 0.10 * magnitude) & (mask <= 0.90 * magnitude)
    if np.count_nonzero(selected) < 3:
        return None
    filtered = np.asarray(values, dtype=np.float64).copy()
    if len(filtered) >= 3:
        filtered[1:-1] = np.median(
            np.stack((filtered[:-2], filtered[1:-1], filtered[2:])), axis=0
        )
    derivatives = np.gradient(filtered, time)
    candidates = derivatives[selected]
    candidates = candidates[candidates > 0] if positive else -candidates[candidates < 0]
    return float(np.percentile(candidates, 95)) if len(candidates) else None


def _encoder_angular_series(hold, rows):
    """由命令/目标轮速比例恢复编码器轮速差对应的车体角速度。"""

    command = np.asarray(
        [float(row["command_angular_radps"]) for row in hold],
        dtype=np.float64,
    )
    target_delta = np.asarray([
        float(row["right_target_radps"])
        - float(row["left_target_radps"])
        for row in hold
    ])
    valid = (
        np.isfinite(command)
        & np.isfinite(target_delta)
        & (np.abs(command) > 1.0e-6)
        & (np.abs(target_delta) > 1.0e-6)
    )
    if np.count_nonzero(valid) < 3:
        return None
    scale = float(np.median(np.abs(command[valid] / target_delta[valid])))
    if not np.isfinite(scale) or scale <= 0.0:
        return None
    return scale * np.asarray([
        float(row["right_feedback_radps"])
        - float(row["left_feedback_radps"])
        for row in rows
    ])


def _trial_metrics(
    rows: list[dict[str, str]],
) -> tuple[dict[str, list[float]] | None, str]:
    rows = sorted(rows, key=lambda row: float(row["timestamp_sec"]))
    baseline = [row for row in rows if row["segment"] == "baseline"]
    hold = [row for row in rows if row["segment"] == "hold"]
    recovery = [row for row in rows if row["segment"] == "recovery"]
    if not baseline or not hold or not recovery:
        return None, "缺少 baseline/hold/recovery 分段"
    hold_time = np.asarray([float(row["timestamp_sec"]) for row in hold])
    if hold_time[-1] - hold_time[0] < 2.5:
        return None, "hold 有效时长不足 2.5s"
    request_time = float(hold[0]["request_timestamp_sec"])
    phase = hold[0]["phase_id"]
    result: dict[str, list[float]] = defaultdict(list)
    for side in ("left", "right"):
        key = f"{side}_feedback_radps"
        target_key = f"{side}_target_radps"
        baseline_rows = [
            row for row in baseline
            if float(row["timestamp_sec"]) >= float(baseline[-1]["timestamp_sec"]) - 0.5
        ]
        steady_rows = [
            row for row in hold
            if float(row["timestamp_sec"]) >= hold_time[-1] - 0.8
        ]
        y0 = float(np.median([float(row[key]) for row in baseline_rows]))
        y_inf = float(np.median([float(row[key]) for row in steady_rows]))
        target = float(np.median([float(row[target_key]) for row in steady_rows]))
        delta = y_inf - y0
        if abs(delta) < 0.2 or abs(target) < 0.2:
            return None, f"{side} 轮响应幅度不足"
        values = np.asarray([float(row[key]) for row in hold])
        direction = 1 if delta > 0 else -1
        t5 = _two_sample_crossing(hold_time, values, y0 + 0.05 * delta, direction)
        t63 = _two_sample_crossing(hold_time, values, y0 + 0.632 * delta, direction)
        if t5 is None or t63 is None or t63 <= t5 or t5 < request_time:
            return None, f"{side} 轮响应交点无效"
        result["action_delay_sec"].append(t5 - request_time)
        result["motor_time_constant_sec"].append((t63 - t5) / 0.948707)
        result[f"{side}_motor_gain"].append(y_inf / target)

    actual_linear_hold = np.asarray(
        [
            0.5 * (
                float(row["left_feedback_mps"])
                + float(row["right_feedback_mps"])
            )
            for row in hold
        ]
    )
    actual_linear_recovery = np.asarray(
        [
            0.5 * (
                float(row["left_feedback_mps"])
                + float(row["right_feedback_mps"])
            )
            for row in recovery
        ]
    )
    recovery_time = np.asarray([float(row["timestamp_sec"]) for row in recovery])
    if phase == "A2":
        accel = _slope_percentile(
            hold_time, actual_linear_hold, actual_linear_hold[0],
            np.median(actual_linear_hold[-5:]), True,
        )
        decel = _slope_percentile(
            recovery_time, actual_linear_recovery,
            actual_linear_recovery[0],
            np.median(actual_linear_recovery[-5:]), False,
        )
        if accel is None or decel is None:
            return None, "线加减速度区间无效"
        result["linear_accel_mps2"].append(accel)
        result["linear_decel_mps2"].append(decel)
    elif phase == "A3":
        actual_angular_hold = _encoder_angular_series(hold, hold)
        actual_angular_recovery = _encoder_angular_series(hold, recovery)
        if actual_angular_hold is None or actual_angular_recovery is None:
            return None, "编码器角速度比例无效"
        direction = 1.0 if np.median(actual_angular_hold[-5:]) >= 0.0 else -1.0
        accel = _slope_percentile(
            hold_time,
            direction * actual_angular_hold,
            direction * actual_angular_hold[0],
            direction * np.median(actual_angular_hold[-5:]),
            True,
        )
        decel = _slope_percentile(
            recovery_time,
            direction * actual_angular_recovery,
            direction * actual_angular_recovery[0],
            direction * np.median(actual_angular_recovery[-5:]),
            False,
        )
        if accel is None or decel is None:
            return None, "角加减速度区间无效"
        result["angular_accel_radps2"].append(accel)
        result["angular_decel_radps2"].append(decel)
    return dict(result), ""


def estimate_rows(
    rows: list[dict[str, str]],
    *,
    measured_at: str,
    source_log_sha256: str,
    min_valid_trials_per_level: int = 4,
    min_metric_samples: int = 8,
) -> EstimateResult:
    """验证完整 A2/A3 分档并生成 schema v1 标定结果。"""

    if not rows:
        raise CalibrationQualityError("标定 CSV 为空")
    missing = set(REQUIRED_COLUMNS) - set(rows[0])
    if missing:
        raise CalibrationQualityError(f"CSV 缺少列: {sorted(missing)}")
    times = np.asarray([float(row["timestamp_sec"]) for row in rows])
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise CalibrationQualityError("timestamp_sec 必须有限且严格递增")
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        if row["phase_id"] in {"A2", "A3"} and row["trial_id"]:
            grouped[row["trial_id"]].append(row)
    metrics: dict[str, list[float]] = defaultdict(list)
    level_counts: dict[str, int] = defaultdict(int)
    rejected: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    for trial_id, trial_rows in grouped.items():
        level = trial_id.rsplit("-", 1)[0]
        values, reason = _trial_metrics(trial_rows)
        if values is None:
            rejected[level][reason] += 1
            continue
        level_counts[level] += 1
        for key, items in values.items():
            metrics[key].extend(items)
    expected_levels = [
        "A2-+0.030", "A2-+0.050", "A2-+0.080", "A2-+0.100",
        "A3--0.523599", "A3-+0.523599", "A3--0.785398",
        "A3-+0.785398", "A3--1.047198", "A3-+1.047198",
    ]
    failed_levels = [
        level for level in expected_levels
        if level_counts.get(level, 0) < min_valid_trials_per_level
    ]
    if failed_levels:
        details = []
        for level in failed_levels:
            reasons = "、".join(
                f"{reason}×{count}"
                for reason, count in sorted(rejected[level].items())
            ) or "未采集到 trial"
            details.append(
                f"{level}={level_counts.get(level, 0)}/"
                f"{min_valid_trials_per_level}（{reasons}）"
            )
        raise CalibrationQualityError("有效 trial 不足: " + "; ".join(details))
    fields = (
        "motor_time_constant_sec", "action_delay_sec", "left_motor_gain",
        "right_motor_gain", "linear_accel_mps2", "linear_decel_mps2",
        "angular_accel_radps2", "angular_decel_radps2",
    )
    distributions = {field: _percentiles(metrics[field]) for field in fields}
    insufficient = [
        field for field, stats in distributions.items()
        if int(stats["count"]) < min_metric_samples or float(stats["p50"]) <= 0.0
    ]
    if insufficient:
        raise CalibrationQualityError(f"标定指标样本不足或非正数: {insufficient}")
    calibration = {
        "schema_version": 1,
        "measured_at": measured_at,
        "total_mass_kg": None,
        **{field: float(distributions[field]["p50"]) for field in fields},
        "wheel_friction_center": None,
        "source_log_sha256": source_log_sha256,
    }
    return EstimateResult(calibration, distributions, dict(level_counts))
