"""自动标定 Action Server、实车安全状态机和数据采集。"""

from __future__ import annotations

import csv
from datetime import datetime
import math
import os
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
from car_interfaces.action import RunCalibration
from car_interfaces.msg import CalibrationStatus, CarMcuStatus, CarStatus
from car_interfaces.srv import EmergencyStop
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
import yaml

from .estimator import CalibrationQualityError, estimate_rows, sha256_file
from .quality import (
    classify_imu_freshness,
    frequency_window,
    select_mcu_timeout,
    stationary_imu_failure,
    stationary_imu_metrics,
    update_wheel_stall_timers,
)
from .result_store import publish_calibration_snapshot
from .safety import (
    MCU_FAULT_COMMAND_TIMEOUT,
    describe_mcu_fault_bits,
    effective_fatal_mcu_fault_bits,
    external_estop_active as _external_estop_active,
    fault_bit_snapshot as _fault_bit_snapshot,
    int32_delta as _int32_delta,
    yaw_from_quaternion as _yaw_from_quaternion,
)


CSV_FIELDS = [
    "timestamp_sec", "command_linear_mps", "command_angular_radps",
    "left_target_radps", "right_target_radps", "left_feedback_radps",
    "right_feedback_radps", "encoder_left_ticks", "encoder_right_ticks",
    "imu_yaw_rad", "phase_id", "segment", "trial_id",
    "request_timestamp_sec", "requested_linear_mps", "requested_angular_radps",
    "left_feedback_mps", "right_feedback_mps", "imu_angular_z_radps",
    "imu_bias_corrected_radps", "odom_linear_mps", "odom_angular_radps",
    "odom_yaw_rad", "mcu_sample_age_sec", "mcu_interarrival_sec",
    "move_sample_age_sec",
    "imu_sample_age_sec", "odom_sample_age_sec", "wifi_ok", "agent_ok",
    "imu_ok", "encoder_ok", "motor_driver_ok", "cmd_timeout_ok",
    "e_stop_active", "mcu_ok", "fault_bits", "jetson_fault_bits",
    "imu_bias_estimate_radps",
    "event_code", "event_message",
]


class SafetyAbort(RuntimeError):
    """运行时安全条件触发的可识别中止。"""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


class CalibrationNode(Node):
    """在独立命令通道上执行可中止、可审计的自动标定。"""

    def __init__(self):
        super().__init__("car_calibrate")
        self._declare_parameters()
        self._load_parameters()
        self._lock = threading.RLock()
        self._goal_reserved = False
        self._recording = False
        self._stopping = False
        self._rows: list[dict[str, object]] = []
        self._message_times = {name: [] for name in ("mcu", "imu", "move", "odom")}
        self._command_timeout_times = []
        self._run_started = 0.0
        self._measured_at = ""
        self._requested_linear = 0.0
        self._requested_angular = 0.0
        self._request_changed = 0.0
        self._phase = "idle"
        self._segment = "idle"
        self._trial_id = ""
        self._event_code = ""
        self._event_message = ""
        self._pending_events: list[tuple[str, str]] = []
        self._mcu = None
        self._status = None
        self._imu = None
        self._odom = None
        self._move = None
        self._last_mcu = 0.0
        self._last_status = 0.0
        self._last_imu = 0.0
        self._last_odom = 0.0
        self._last_move = 0.0
        self._previous_ticks = None
        self._data_error = ""
        self._imu_bias = 0.0
        self._imu_yaw = 0.0
        self._previous_imu_time = 0.0
        self._previous_corrected_gyro = 0.0
        self._imu_stale_active = False
        self._imu_dropout_count = 0
        self._imu_max_gap_sec = 0.0
        self._imu_hard_gap_sec = 0.0
        self._mismatch_started = 0.0
        self._stall_started = {"left": 0.0, "right": 0.0}
        self._command_timeout_samples = 0
        self._status_snapshot = CalibrationStatus()
        self._status_snapshot.state = "idle"
        self._status_snapshot.phase = "idle"
        # execute_callback 会同步等待话题更新和急停服务结果，因此所有相关
        # callback 必须允许由 MultiThreadedExecutor 并发调度。
        self._callback_group = ReentrantCallbackGroup()
        self._command_publisher = self.create_publisher(Twist, "/cmd_vel_calibration", 10)
        self._status_publisher = self.create_publisher(
            CalibrationStatus,
            "/car/calibration_status",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            CarMcuStatus, "/car/mcu_status", self._on_mcu,
            qos_profile_sensor_data, callback_group=self._callback_group,
        )
        self.create_subscription(
            CarStatus, "/car/status", self._on_status, 10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Imu, "/imu/data", self._on_imu, qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Odometry, "/odom", self._on_odom, 10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Twist, "/cmd_vel_move", self._on_move, qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self._estop_client = self.create_client(
            EmergencyStop, "/car/e_stop", callback_group=self._callback_group
        )
        self._action_server = ActionServer(
            self,
            RunCalibration,
            "/car/run_calibration",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.create_timer(
            1.0 / self.command_publish_hz, self._publish_command,
            callback_group=self._callback_group,
        )
        self.create_timer(
            0.5, self._publish_status, callback_group=self._callback_group
        )
        self._publish_status()

    def _declare_parameters(self):
        values = {
            "vehicle_id": "car01", "result_root": "", "wheel_radius_m": 0.032,
            "wheel_separation_m": 0.175, "imu_yaw_sign": 1.0,
            "command_publish_hz": 20.0, "countdown_sec": 5.0,
            "preflight_timeout_sec": 8.0, "a0_duration_sec": 10.0,
            "a1_duration_sec": 15.0, "baseline_duration_sec": 2.0,
            "motion_duration_sec": 3.0, "recovery_duration_sec": 3.0,
            "a0_max_attempts": 2, "a1_max_attempts": 2,
            "stage_retry_delay_sec": 2.0,
            "command_channel_stable_sec": 1.0,
            "command_channel_recovery_timeout_sec": 5.0,
            "repeats": 5, "linear_speeds_mps": [0.03, 0.05, 0.08, 0.10],
            "angular_speeds_radps": [0.523598776, 0.785398163, 1.047197551],
            "stationary_mcu_timeout_sec": 0.60,
            "motion_mcu_timeout_sec": 0.25, "imu_timeout_sec": 0.25,
            "imu_abort_timeout_sec": 0.50,
            "max_feedback_speed_mps": 0.20,
            "stopped_speed_mps": 0.01, "stopped_hold_sec": 0.5,
            "stall_timeout_sec": 1.5, "stall_startup_grace_sec": 0.20,
            "stall_response_speed_mps": 0.001,
            "command_mismatch_timeout_sec": 0.30,
            "command_linear_tolerance_mps": 0.005,
            "command_angular_tolerance_radps": 0.02, "max_tick_delta": 20000,
            "min_valid_trials_per_level": 4, "min_metric_samples": 8,
            "min_mcu_frequency_hz": 15.0, "min_imu_frequency_hz": 25.0,
            "min_move_frequency_hz": 12.0, "min_odom_frequency_hz": 12.0,
            "max_mcu_gap_sec": 0.15,
            "stationary_wheel_p99_max_mps": 0.015,
            "stationary_tick_drift_max": 2,
            "stationary_imu_std_max_radps": 0.05,
            "stationary_imu_bias_max_radps": 0.35,
            "stationary_imu_residual_p99_max_radps": 0.10,
            "stationary_imu_drift_max_radps": 0.02,
        }
        for name, value in values.items():
            self.declare_parameter(name, value)

    def _load_parameters(self):
        for name in (
            "vehicle_id", "result_root", "wheel_radius_m", "wheel_separation_m",
            "imu_yaw_sign", "command_publish_hz", "countdown_sec",
            "preflight_timeout_sec", "a0_duration_sec", "a1_duration_sec",
            "baseline_duration_sec", "motion_duration_sec", "recovery_duration_sec",
            "a0_max_attempts", "a1_max_attempts", "stage_retry_delay_sec",
            "command_channel_stable_sec", "command_channel_recovery_timeout_sec",
            "repeats", "linear_speeds_mps", "angular_speeds_radps",
            "stationary_mcu_timeout_sec", "motion_mcu_timeout_sec",
            "imu_timeout_sec", "imu_abort_timeout_sec",
            "max_feedback_speed_mps",
            "stopped_speed_mps", "stopped_hold_sec",
            "stall_timeout_sec", "stall_startup_grace_sec",
            "stall_response_speed_mps", "command_mismatch_timeout_sec",
            "command_linear_tolerance_mps", "command_angular_tolerance_radps",
            "max_tick_delta", "min_valid_trials_per_level", "min_metric_samples",
            "min_mcu_frequency_hz", "min_imu_frequency_hz",
            "min_move_frequency_hz", "min_odom_frequency_hz",
            "max_mcu_gap_sec",
            "stationary_wheel_p99_max_mps", "stationary_tick_drift_max",
            "stationary_imu_std_max_radps", "stationary_imu_bias_max_radps",
            "stationary_imu_residual_p99_max_radps",
            "stationary_imu_drift_max_radps",
        ):
            setattr(self, name, self.get_parameter(name).value)
        numeric_positive = (
            self.wheel_radius_m, self.wheel_separation_m, self.command_publish_hz,
            self.stationary_mcu_timeout_sec, self.motion_mcu_timeout_sec,
            self.imu_timeout_sec, self.imu_abort_timeout_sec,
            self.stopped_hold_sec,
            self.min_mcu_frequency_hz, self.min_imu_frequency_hz,
            self.min_move_frequency_hz, self.min_odom_frequency_hz,
            self.max_mcu_gap_sec, self.stage_retry_delay_sec,
            self.command_channel_stable_sec,
            self.command_channel_recovery_timeout_sec,
            self.stall_timeout_sec, self.stall_startup_grace_sec,
            self.stall_response_speed_mps,
        )
        if not all(
            math.isfinite(float(value)) and float(value) > 0.0
            for value in numeric_positive
        ):
            raise ValueError("标定配置中的尺寸、频率和超时必须为有限正数")
        if self.imu_yaw_sign not in (-1.0, 1.0):
            raise ValueError("imu_yaw_sign 只允许 -1 或 1")
        if self.imu_abort_timeout_sec <= self.imu_timeout_sec:
            raise ValueError("imu_abort_timeout_sec 必须大于 imu_timeout_sec")
        if int(self.a0_max_attempts) < 1 or int(self.a1_max_attempts) < 1:
            raise ValueError("A0/A1 最大尝试次数必须至少为 1")

    def _goal_callback(self, request):
        confirmations = (
            request.straight_path_clear,
            request.rotation_area_clear,
            request.physical_estop_ready,
            request.supervised,
        )
        with self._lock:
            if self._goal_reserved or not all(confirmations):
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def _publish_command(self):
        with self._lock:
            linear = self._requested_linear
            angular = self._requested_angular
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self._command_publisher.publish(message)

    def _publish_status(self):
        with self._lock:
            message = CalibrationStatus()
            for field in (
                "active", "state", "phase", "completed_trials", "total_trials",
                "progress", "imu_bias_radps",
                "imu_noise_radps", "imu_drift_radps", "mcu_frequency_hz",
                "mcu_max_gap_sec", "command_timeout_samples", "run_id",
                "result_directory", "message",
            ):
                setattr(message, field, getattr(self._status_snapshot, field))
        message.stamp = self.get_clock().now().to_msg()
        self._status_publisher.publish(message)

    def _set_status(
        self, *, active=None, state=None, phase=None, completed=None, total=None,
        progress=None, run_id=None, result_directory=None, message=None,
    ):
        with self._lock:
            updates = {
                "active": active, "state": state, "phase": phase,
                "completed_trials": completed, "total_trials": total,
                "progress": progress, "run_id": run_id,
                "result_directory": result_directory, "message": message,
            }
            for key, value in updates.items():
                if value is not None:
                    setattr(self._status_snapshot, key, value)
        self._publish_status()

    def _on_status(self, message):
        with self._lock:
            self._status = message
            self._last_status = time.monotonic()

    def _on_move(self, message):
        now = time.monotonic()
        with self._lock:
            self._move = message
            self._last_move = now
            if self._recording:
                self._message_times["move"].append(now - self._run_started)

    def _on_odom(self, message):
        now = time.monotonic()
        with self._lock:
            self._odom = message
            self._last_odom = now
            if self._recording:
                self._message_times["odom"].append(now - self._run_started)

    def _on_imu(self, message):
        now = time.monotonic()
        raw = float(message.angular_velocity.z)
        late_warning_message = ""
        recovery_message = ""
        hard_gap = False
        with self._lock:
            previous_last_imu = self._last_imu
            gap = max(0.0, now - previous_last_imu) if previous_last_imu else 0.0
            self._imu = message
            if not math.isfinite(raw):
                self._data_error = "IMU 包含非有限角速度"
            corrected = float(self.imu_yaw_sign) * (raw - self._imu_bias)
            if self._previous_imu_time and now - self._previous_imu_time <= self.imu_timeout_sec:
                dt = now - self._previous_imu_time
                self._imu_yaw += 0.5 * (self._previous_corrected_gyro + corrected) * dt
            self._previous_imu_time = now
            self._previous_corrected_gyro = corrected
            self._last_imu = now
            if self._recording:
                self._message_times["imu"].append(now - self._run_started)
                if previous_last_imu and gap > self.imu_timeout_sec:
                    if not self._imu_stale_active:
                        self._imu_dropout_count += 1
                        late_warning_message = (
                            f"检测到 IMU 数据空窗：{gap:.3f}s，"
                            f"告警门限 {self.imu_timeout_sec:.3f}s"
                        )
                        self._pending_events.append((
                            "imu_stale",
                            late_warning_message,
                        ))
                    self._imu_max_gap_sec = max(self._imu_max_gap_sec, gap)
                    hard_gap = gap > self.imu_abort_timeout_sec
                    if hard_gap:
                        self._imu_hard_gap_sec = max(self._imu_hard_gap_sec, gap)
                        recovery_message = (
                            f"IMU 空窗 {gap:.3f}s 后恢复，但已超过硬门限 "
                            f"{self.imu_abort_timeout_sec:.3f}s"
                        )
                    else:
                        recovery_message = (
                            f"IMU 短时空窗已恢复：{gap:.3f}s；标定继续，"
                            "结果仍由质量门禁判定"
                        )
                    self._pending_events.append((
                        "imu_recovered", recovery_message,
                    ))
                    self._imu_stale_active = False
        if late_warning_message:
            self.get_logger().warning(late_warning_message)
            self._set_status(message=late_warning_message)
        if recovery_message:
            if hard_gap:
                self.get_logger().error(recovery_message)
            else:
                self.get_logger().info(recovery_message)
                self._set_status(message=recovery_message)

    def _on_mcu(self, message):
        now = time.monotonic()
        with self._lock:
            interarrival = (
                max(0.0, now - self._last_mcu) if self._last_mcu else 0.0
            )
            numeric = (message.left_wheel_speed, message.right_wheel_speed)
            if not all(math.isfinite(float(value)) for value in numeric):
                self._data_error = "MCU 状态包含 NaN 或 Inf"
            ticks = (int(message.left_encoder_ticks), int(message.right_encoder_ticks))
            if self._previous_ticks is not None:
                deltas = (
                    _int32_delta(ticks[0], self._previous_ticks[0]),
                    _int32_delta(ticks[1], self._previous_ticks[1]),
                )
                if max(abs(deltas[0]), abs(deltas[1])) > int(self.max_tick_delta):
                    self._data_error = f"编码器单帧跳变超过 {self.max_tick_delta}"
            self._previous_ticks = ticks
            self._mcu = message
            self._last_mcu = now
            if not self._recording:
                return
            if bool(message.cmd_timeout_ok) or (
                int(message.fault_bits) & MCU_FAULT_COMMAND_TIMEOUT
            ):
                self._command_timeout_times.append(now - self._run_started)
                self._command_timeout_samples += 1
                self._status_snapshot.command_timeout_samples = (
                    self._command_timeout_samples
                )
            self._message_times["mcu"].append(now - self._run_started)
            event_code = self._event_code
            event_message = self._event_message
            if not event_code and self._pending_events:
                event_code, event_message = self._pending_events.pop(0)
            self._rows.append(self._snapshot_row(
                now, message, interarrival,
                event_code=event_code, event_message=event_message,
            ))

    def _snapshot_row(
        self, now, mcu, interarrival=0.0, *, event_code="", event_message="",
    ):
        move = self._move or Twist()
        status = self._status
        imu = self._imu or Imu()
        odom = self._odom or Odometry()
        v = float(move.linear.x)
        w = float(move.angular.z)
        left_target = (v - w * self.wheel_separation_m * 0.5) / self.wheel_radius_m
        right_target = (v + w * self.wheel_separation_m * 0.5) / self.wheel_radius_m
        left = float(mcu.left_wheel_speed)
        right = float(mcu.right_wheel_speed)
        raw_gyro = float(imu.angular_velocity.z)
        corrected = float(self.imu_yaw_sign) * (raw_gyro - self._imu_bias)
        mcu_fault_bits, jetson_fault_bits = _fault_bit_snapshot(mcu, status)
        return {
            "timestamp_sec": now - self._run_started,
            "command_linear_mps": v, "command_angular_radps": w,
            "left_target_radps": left_target, "right_target_radps": right_target,
            "left_feedback_radps": left / self.wheel_radius_m,
            "right_feedback_radps": right / self.wheel_radius_m,
            "encoder_left_ticks": int(mcu.left_encoder_ticks),
            "encoder_right_ticks": int(mcu.right_encoder_ticks),
            "imu_yaw_rad": self._imu_yaw, "phase_id": self._phase,
            "segment": self._segment, "trial_id": self._trial_id,
            "request_timestamp_sec": self._request_changed - self._run_started,
            "requested_linear_mps": self._requested_linear,
            "requested_angular_radps": self._requested_angular,
            "left_feedback_mps": left, "right_feedback_mps": right,
            "imu_angular_z_radps": raw_gyro,
            "imu_bias_corrected_radps": corrected,
            "odom_linear_mps": float(odom.twist.twist.linear.x),
            "odom_angular_radps": float(odom.twist.twist.angular.z),
            "odom_yaw_rad": _yaw_from_quaternion(odom.pose.pose.orientation),
            "mcu_sample_age_sec": 0.0,
            "mcu_interarrival_sec": interarrival,
            "move_sample_age_sec": max(0.0, now - self._last_move),
            "imu_sample_age_sec": max(0.0, now - self._last_imu),
            "odom_sample_age_sec": max(0.0, now - self._last_odom),
            "wifi_ok": int(bool(mcu.wifi_connect_ok)),
            "agent_ok": int(bool(mcu.agent_connect_ok)),
            "imu_ok": int(bool(mcu.imu_ok)),
            "encoder_ok": int(bool(mcu.encoder_ok)),
            "motor_driver_ok": int(bool(mcu.motor_driver_ok)),
            "cmd_timeout_ok": int(bool(mcu.cmd_timeout_ok)),
            "e_stop_active": int(bool(status.e_stop_ok)) if status else 0,
            "mcu_ok": int(bool(status.mcu_ok)) if status else 0,
            "fault_bits": mcu_fault_bits,
            "jetson_fault_bits": jetson_fault_bits,
            "imu_bias_estimate_radps": self._imu_bias,
            "event_code": event_code, "event_message": event_message,
        }

    def _check_imu_freshness(self, now):
        """短时 IMU 空窗只告警；持续空窗才进入安全中止链。"""

        warning_message = ""
        with self._lock:
            age = max(0.0, float(now) - self._last_imu)
            hard_gap = self._imu_hard_gap_sec
            state = classify_imu_freshness(
                max(age, hard_gap),
                self.imu_timeout_sec,
                self.imu_abort_timeout_sec,
            )
            self._imu_max_gap_sec = max(self._imu_max_gap_sec, age, hard_gap)
            if state == "stale" and not self._imu_stale_active:
                self._imu_stale_active = True
                self._imu_dropout_count += 1
                warning_message = (
                    f"IMU 暂时无新数据：{age:.3f}s；保持当前运动，"
                    f"超过 {self.imu_abort_timeout_sec:.3f}s 才安全终止"
                )
                self._pending_events.append(("imu_stale", warning_message))
        if state == "abort":
            observed = max(age, hard_gap)
            raise SafetyAbort(
                "imu_timeout",
                f"IMU 状态持续过期：{observed:.3f}s > 硬门限 "
                f"{self.imu_abort_timeout_sec:.3f}s",
            )
        if warning_message:
            self.get_logger().warning(warning_message)
            self._set_status(message=warning_message)

    def _raise_for_mcu_faults(self, mcu, *, moving=False):
        command_timeout = int(mcu.fault_bits) & MCU_FAULT_COMMAND_TIMEOUT
        effective_fatal = effective_fatal_mcu_fault_bits(
            mcu.fault_bits, moving=moving
        )
        fatal_without_timeout = effective_fatal & ~MCU_FAULT_COMMAND_TIMEOUT
        if fatal_without_timeout:
            raise SafetyAbort(
                "mcu_fault",
                f"MCU 致命故障：{describe_mcu_fault_bits(fatal_without_timeout)} "
                f"[fault_bits={fatal_without_timeout}]",
            )
        if command_timeout and moving:
            raise SafetyAbort(
                "command_timeout",
                "运动阶段 ESP32 命令超时停车，可能存在下行链路丢包",
            )

    def _set_command(self, linear, angular, phase, segment, trial_id=""):
        with self._lock:
            self._requested_linear = float(linear)
            self._requested_angular = float(angular)
            self._phase = phase
            self._segment = segment
            self._trial_id = trial_id
            self._request_changed = time.monotonic()
            self._mismatch_started = 0.0
            self._stall_started = {"left": 0.0, "right": 0.0}

    def _call_estop(self, stop: bool, reason: str):
        if not self._estop_client.wait_for_service(timeout_sec=2.0):
            raise SafetyAbort("estop_unavailable", "急停服务不可用")
        request = EmergencyStop.Request()
        request.stop = bool(stop)
        request.reason = reason
        request.source = "car_calibrate"
        future = self._estop_client.call_async(request)
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done() or future.result() is None or not future.result().success:
            raise SafetyAbort("estop_failed", "急停服务调用失败")

    def _wait_status_estop(self, expected: bool, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                status = self._status
            if status is not None and bool(status.e_stop_ok) == expected:
                return
            time.sleep(0.02)
        raise SafetyAbort("estop_state_timeout", "急停状态未在限定时间内确认")

    def _preflight(self):
        self._set_command(0.0, 0.0, "PREFLIGHT", "zero")
        with self._lock:
            initial_status = self._status
        if initial_status is not None and bool(initial_status.e_stop_ok):
            source = str(getattr(initial_status, "e_stop_source", "")) or "未知来源"
            raise SafetyAbort(
                "preexisting_estop",
                f"开始标定前已有安全锁止，来源={source}，请先人工确认并解除",
            )
        self._call_estop(True, "自动标定启动锁止")
        self._wait_status_estop(True)
        deadline = time.monotonic() + self.preflight_timeout_sec
        while time.monotonic() < deadline:
            now = time.monotonic()
            with self._lock:
                ready = all(
                    item is not None for item in
                    (self._mcu, self._status, self._imu, self._odom, self._move)
                )
                fresh = (
                    now - self._last_mcu <= self.stationary_mcu_timeout_sec
                    and now - self._last_imu <= self.imu_timeout_sec
                    and now - self._last_status <= self.motion_mcu_timeout_sec
                    and now - self._last_move <= self.motion_mcu_timeout_sec
                    and now - self._last_odom <= self.motion_mcu_timeout_sec
                )
            if ready and fresh:
                break
            time.sleep(0.05)
        else:
            raise SafetyAbort("preflight_topics", "必需标定话题未就绪")
        names = set(self.get_node_names())
        blocked = names.intersection({"slam_toolbox", "controller_server", "bt_navigator"})
        if blocked:
            raise SafetyAbort("stack_active", f"建图或导航节点仍在运行: {sorted(blocked)}")
        publishers = self.get_publishers_info_by_topic("/cmd_vel_calibration")
        if len(publishers) != 1:
            raise SafetyAbort("publisher_conflict", "检测到其它标定速度发布者")
        with self._lock:
            mcu = self._mcu
            status = self._status
        if not status.mcu_ok or not all(
            (mcu.wifi_connect_ok, mcu.agent_connect_ok, mcu.imu_ok,
             mcu.encoder_ok, mcu.motor_driver_ok)
        ):
            raise SafetyAbort("health_gate", "底盘硬件或通信健康门禁未通过")
        self._raise_for_mcu_faults(mcu, moving=False)
        if max(abs(mcu.left_wheel_speed), abs(mcu.right_wheel_speed)) >= self.stopped_speed_mps:
            raise SafetyAbort("wheels_moving", "预检时车轮尚未停止")

    def _check_safety(self, goal_handle):
        if goal_handle.is_cancel_requested:
            raise SafetyAbort("canceled", "标定已被取消")
        now = time.monotonic()
        with self._lock:
            mcu, status, move = self._mcu, self._status, self._move
            data_error = self._data_error
            requested = (self._requested_linear, self._requested_angular)
            request_changed = self._request_changed
            stall_started = dict(self._stall_started)
            last_mcu = self._last_mcu
            last_move = self._last_move
            last_status = self._last_status
            last_odom = self._last_odom
        if data_error:
            raise SafetyAbort("invalid_data", data_error)
        if mcu is None or status is None or move is None:
            raise SafetyAbort("missing_data", "运行时反馈缺失")
        mcu_timeout, timeout_phase = select_mcu_timeout(
            requested[0], requested[1],
            self.stationary_mcu_timeout_sec, self.motion_mcu_timeout_sec,
        )
        mcu_age = now - last_mcu
        if mcu_age > mcu_timeout:
            raise SafetyAbort(
                "mcu_timeout",
                f"MCU 状态过期：{mcu_age:.3f}s > "
                f"{timeout_phase}门限 {mcu_timeout:.3f}s",
            )
        self._check_imu_freshness(now)
        if now - last_move > self.motion_mcu_timeout_sec:
            raise SafetyAbort("move_timeout", "最终底盘速度命令状态过期")
        if now - last_status > self.motion_mcu_timeout_sec:
            raise SafetyAbort("status_timeout", "Jetson 底盘安全状态过期")
        if now - last_odom > self.motion_mcu_timeout_sec:
            raise SafetyAbort("odom_timeout", "里程计状态过期")
        if bool(status.e_stop_ok) and not self._stopping:
            raise SafetyAbort("external_estop", "检测到外部急停")
        moving_request = (
            abs(requested[0]) > 1.0e-9 or abs(requested[1]) > 1.0e-9
        )
        # car_move 的 MCU 新鲜度门限固定为 0.5 s，并会安全输出零速。
        # 静止采集允许 calibration 自己的 0.6 s 门限接管，避免一次短抖动
        # 直接终止；非零运动仍同时要求 car_move 与本节点都判定在线。
        if (moving_request and not status.mcu_ok) or not all(
            (mcu.wifi_connect_ok, mcu.agent_connect_ok, mcu.imu_ok,
             mcu.encoder_ok, mcu.motor_driver_ok)
        ):
            raise SafetyAbort("health_lost", "底盘健康状态在运行中失效")
        self._raise_for_mcu_faults(mcu, moving=moving_request)
        if (
            max(abs(mcu.left_wheel_speed), abs(mcu.right_wheel_speed))
            > self.max_feedback_speed_mps
        ):
            raise SafetyAbort("overspeed", "轮速超过标定安全上限")
        mismatch = (
            abs(float(move.linear.x) - requested[0]) > self.command_linear_tolerance_mps
            or abs(float(move.angular.z) - requested[1]) > self.command_angular_tolerance_radps
        )
        if mismatch and now - request_changed > 0.10:
            if not self._mismatch_started:
                self._mismatch_started = now
            elif now - self._mismatch_started >= self.command_mismatch_timeout_sec:
                raise SafetyAbort("command_mismatch", "仲裁输出持续偏离标定请求")
        else:
            self._mismatch_started = 0.0
        stall_started, failed_wheels = update_wheel_stall_timers(
            stall_started,
            now=now,
            request_changed=request_changed,
            linear=requested[0],
            angular=requested[1],
            wheel_separation=self.wheel_separation_m,
            left_feedback=mcu.left_wheel_speed,
            right_feedback=mcu.right_wheel_speed,
            startup_grace=self.stall_startup_grace_sec,
            response_speed=self.stall_response_speed_mps,
            stall_timeout=self.stall_timeout_sec,
        )
        with self._lock:
            self._stall_started = stall_started
        if failed_wheels:
            wheel_labels = {"left": "左轮", "right": "右轮"}
            failed_text = "、".join(
                wheel_labels[wheel] for wheel in failed_wheels
            )
            raise SafetyAbort(
                "stall",
                f"运动命令存在但{failed_text}持续未响应",
            )

    def _feedback(self, goal_handle, message, completed, total):
        feedback = RunCalibration.Feedback()
        feedback.state = "running"
        feedback.phase = self._phase
        feedback.completed_trials = int(completed)
        feedback.total_trials = int(total)
        feedback.progress = float(completed / total) if total else 0.0
        feedback.message = message
        goal_handle.publish_feedback(feedback)
        self._set_status(
            active=True, state="running", phase=self._phase, completed=completed,
            total=total, progress=feedback.progress, message=message,
        )

    def _hold(self, goal_handle, duration, completed, total, message):
        deadline = time.monotonic() + float(duration)
        last_feedback = 0.0
        while time.monotonic() < deadline:
            self._check_safety(goal_handle)
            if time.monotonic() - last_feedback >= 0.25:
                self._feedback(goal_handle, message, completed, total)
                last_feedback = time.monotonic()
            time.sleep(0.02)

    def _wait_stopped(self, goal_handle, timeout=5.0):
        stable_since = 0.0
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check_safety(goal_handle)
            with self._lock:
                mcu = self._mcu
            stopped = mcu is not None and max(
                abs(mcu.left_wheel_speed), abs(mcu.right_wheel_speed)
            ) < self.stopped_speed_mps
            if stopped:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= self.stopped_hold_sec:
                    return
            else:
                stable_since = 0.0
            time.sleep(0.02)
        raise SafetyAbort("stop_timeout", "车轮未能在限定时间内稳定停止")

    def _validate_a0(self, start, end):
        frequencies, maximum_gaps = frequency_window(
            self._message_times, start, end
        )
        command_timeouts = sum(
            start <= value <= end for value in self._command_timeout_times
        )
        mcu_gap = maximum_gaps.get("mcu", math.inf)
        with self._lock:
            self._status_snapshot.mcu_frequency_hz = float(
                frequencies.get("mcu", 0.0)
            )
            self._status_snapshot.mcu_max_gap_sec = float(mcu_gap)
        if command_timeouts:
            raise SafetyAbort(
                "a0_command_timeout",
                f"A0 检测到 {command_timeouts} 帧 ESP32 命令超时停车，"
                "下行控制链路不稳定",
            )
        thresholds = {
            "mcu": float(self.min_mcu_frequency_hz),
            "imu": float(self.min_imu_frequency_hz),
            "move": float(self.min_move_frequency_hz),
            "odom": float(self.min_odom_frequency_hz),
        }
        for name, minimum_hz in thresholds.items():
            frequency = frequencies.get(name, 0.0)
            if frequency < minimum_hz:
                raise SafetyAbort(
                    "a0_frequency",
                    f"{name} 实测 {frequency:.1f}Hz，低于配置门禁 "
                    f"{minimum_hz:.1f}Hz",
                )
        if mcu_gap > float(self.max_mcu_gap_sec):
            raise SafetyAbort(
                "a0_gap",
                f"MCU 最大相邻间隔 {mcu_gap:.3f}s，超过配置门禁 "
                f"{float(self.max_mcu_gap_sec):.3f}s",
            )
        return frequencies, maximum_gaps

    def _validate_a1(self, start, end):
        rows = [
            row for row in self._rows
            if row["phase_id"] == "A1"
            and start <= float(row["timestamp_sec"]) <= end
        ]
        if len(rows) < 20:
            raise SafetyAbort("a1_samples", "静止阶段样本不足")
        command_timeouts = sum(
            start <= value <= end for value in self._command_timeout_times
        )
        if command_timeouts:
            raise SafetyAbort(
                "a1_command_timeout",
                f"A1 检测到 {command_timeouts} 帧 ESP32 命令超时停车，"
                "本次静止采集无效",
            )
        left = np.asarray([float(row["left_feedback_mps"]) for row in rows])
        right = np.asarray([float(row["right_feedback_mps"]) for row in rows])
        gyro = np.asarray([float(row["imu_angular_z_radps"]) for row in rows])
        wheel_p99 = max(
            float(np.percentile(abs(left), 99)),
            float(np.percentile(abs(right), 99)),
        )
        if wheel_p99 >= self.stationary_wheel_p99_max_mps:
            raise SafetyAbort(
                "a1_wheel_noise",
                f"A1 静止轮速 p99={wheel_p99:.4f}m/s，超过门限 "
                f"{float(self.stationary_wheel_p99_max_mps):.4f}m/s",
            )
        for key in ("encoder_left_ticks", "encoder_right_ticks"):
            tick_drift = abs(
                _int32_delta(int(rows[-1][key]), int(rows[0][key]))
            )
            if tick_drift > int(self.stationary_tick_drift_max):
                raise SafetyAbort(
                    "a1_tick_drift",
                    f"A1 {key} 漂移 {tick_drift} tick，超过门限 "
                    f"{int(self.stationary_tick_drift_max)} tick",
                )
        metrics = stationary_imu_metrics(gyro)
        failed_metric = stationary_imu_failure(
            metrics,
            bias_limit=float(self.stationary_imu_bias_max_radps),
            noise_limit=float(self.stationary_imu_std_max_radps),
            residual_p99_limit=float(
                self.stationary_imu_residual_p99_max_radps
            ),
            drift_limit=float(self.stationary_imu_drift_max_radps),
        )
        with self._lock:
            self._status_snapshot.imu_bias_radps = float(metrics["bias"])
            self._status_snapshot.imu_noise_radps = float(metrics["noise"])
            self._status_snapshot.imu_drift_radps = float(metrics["drift"])
        if failed_metric:
            labels = {
                "bias": "零偏超过芯片允许范围",
                "noise": "随机噪声过大",
                "residual": "存在触碰或瞬时振动",
                "drift": "采集期间零点发生漂移",
            }
            raise SafetyAbort(
                "a1_imu_unstable",
                f"A1 IMU {labels[failed_metric]}：偏置={metrics['bias']:.4f}, "
                f"噪声={metrics['noise']:.4f}, "
                f"残差p99={metrics['residual_p99']:.4f}, "
                f"漂移={metrics['drift']:.4f} rad/s",
            )
        bias = float(metrics["bias"])
        with self._lock:
            self._imu_bias = bias
            self._imu_yaw = 0.0
            self._previous_imu_time = 0.0
            self._previous_corrected_gyro = 0.0
            for row in rows:
                raw = float(row["imu_angular_z_radps"])
                row["imu_bias_corrected_radps"] = (
                    float(self.imu_yaw_sign) * (raw - bias)
                )
                row["imu_bias_estimate_radps"] = bias
        return metrics

    def _retry_stationary_stage(self, goal_handle, phase, completed, total, message):
        self._set_command(0.0, 0.0, phase, "retry_wait")
        self._hold(
            goal_handle, self.stage_retry_delay_sec, completed, total, message
        )

    def _run_a0(self, goal_handle, total):
        last_error = None
        for attempt in range(1, int(self.a0_max_attempts) + 1):
            self._set_command(0.0, 0.0, "A0", "zero", f"A0-{attempt}")
            start = time.monotonic() - self._run_started
            self._hold(
                goal_handle, self.a0_duration_sec, 0, total,
                f"A0 接口质量检查（第 {attempt} 次）",
            )
            end = time.monotonic() - self._run_started
            try:
                frequencies, gaps = self._validate_a0(start, end)
                self._set_status(
                    message=(
                        f"A0 通过：MCU {frequencies.get('mcu', 0.0):.1f}Hz，"
                        f"最大间隔 {gaps.get('mcu', math.inf):.3f}s"
                    )
                )
                return
            except SafetyAbort as error:
                last_error = error
                if attempt >= int(self.a0_max_attempts):
                    raise
                self.get_logger().warning(
                    f"A0 第 {attempt} 次未通过，将保持零速重试：{error}"
                )
                self._retry_stationary_stage(
                    goal_handle, "A0", 0, total,
                    f"A0 未通过：{error}；保持零速后自动重试",
                )
        raise last_error

    def _run_a1(self, goal_handle, total):
        last_error = None
        for attempt in range(1, int(self.a1_max_attempts) + 1):
            self._set_command(
                0.0, 0.0, "A1", f"stationary_attempt_{attempt}",
                f"A1-{attempt}",
            )
            start = time.monotonic() - self._run_started
            self._hold(
                goal_handle, self.a1_duration_sec, 0, total,
                f"A1 静止 IMU 标定（第 {attempt} 次，请勿触碰小车）",
            )
            end = time.monotonic() - self._run_started
            try:
                metrics = self._validate_a1(start, end)
                self._set_status(
                    message=(
                        f"A1 通过并已修正 IMU 零偏 "
                        f"{metrics['bias']:.4f}rad/s；"
                        f"噪声 {metrics['noise']:.4f}rad/s"
                    )
                )
                return
            except SafetyAbort as error:
                last_error = error
                if attempt >= int(self.a1_max_attempts):
                    raise
                self.get_logger().warning(
                    f"A1 第 {attempt} 次未通过，将保持零速重试：{error}"
                )
                self._retry_stationary_stage(
                    goal_handle, "A1", 0, total,
                    f"A1 检测到触碰、漂移或链路问题：{error}；自动重试",
                )
        raise last_error

    def _wait_command_channel_ready(self, goal_handle):
        stable_since = 0.0
        deadline = time.monotonic() + self.command_channel_recovery_timeout_sec
        while time.monotonic() < deadline:
            self._check_safety(goal_handle)
            with self._lock:
                mcu = self._mcu
            timed_out = bool(
                mcu and (
                    mcu.cmd_timeout_ok
                    or int(mcu.fault_bits) & MCU_FAULT_COMMAND_TIMEOUT
                )
            )
            if not timed_out:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= self.command_channel_stable_sec:
                    return
            else:
                stable_since = 0.0
            time.sleep(0.02)
        raise SafetyAbort(
            "command_channel_unstable",
            "ESP32 命令通道未能在限定时间内恢复稳定，禁止开始运动",
        )

    def _run_trials(self, goal_handle, total):
        completed = 0
        for speed in self.linear_speeds_mps:
            for repeat in range(1, int(self.repeats) + 1):
                trial = f"A2-{float(speed):+.3f}-{repeat}"
                self._set_command(0.0, 0.0, "A2", "baseline", trial)
                self._wait_stopped(goal_handle)
                self._hold(
                    goal_handle, self.baseline_duration_sec, completed, total,
                    f"直线 {speed} 基线",
                )
                self._wait_command_channel_ready(goal_handle)
                self._set_command(speed, 0.0, "A2", "hold", trial)
                self._hold(
                    goal_handle, self.motion_duration_sec, completed, total,
                    f"直线 {speed} 第{repeat}次",
                )
                self._set_command(0.0, 0.0, "A2", "recovery", trial)
                self._hold(goal_handle, self.recovery_duration_sec, completed, total, "直线停车采集")
                self._wait_stopped(goal_handle)
                completed += 1
                self._feedback(
                    goal_handle, f"直线 {speed} 第{repeat}次完成",
                    completed, total,
                )
        for magnitude in self.angular_speeds_radps:
            for repeat in range(1, int(self.repeats) + 1):
                for angular in (-float(magnitude), float(magnitude)):
                    trial = f"A3-{angular:+.6f}-{repeat}"
                    self._set_command(0.0, 0.0, "A3", "baseline", trial)
                    self._wait_stopped(goal_handle)
                    self._hold(
                        goal_handle, self.baseline_duration_sec,
                        completed, total, f"旋转 {angular:+.3f} 基线",
                    )
                    self._wait_command_channel_ready(goal_handle)
                    self._set_command(0.0, angular, "A3", "hold", trial)
                    self._hold(
                        goal_handle, self.motion_duration_sec,
                        completed, total,
                        f"旋转 {angular:+.3f} 第{repeat}次",
                    )
                    self._set_command(0.0, 0.0, "A3", "recovery", trial)
                    self._hold(goal_handle, self.recovery_duration_sec, completed, total, "旋转停车采集")
                    self._wait_stopped(goal_handle)
                    completed += 1
                    self._feedback(
                        goal_handle, f"旋转 {angular:+.3f} 第{repeat}次完成",
                        completed, total,
                    )
        return completed

    def _safe_stop(self):
        with self._lock:
            self._stopping = True
        self._set_command(0.0, 0.0, "STOPPING", "zero")
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            time.sleep(0.02)
        estop_confirmed = False
        with self._lock:
            status = self._status
        if _external_estop_active(status):
            self.get_logger().warning(
                "检测到真实急停，保留其来源和原因，不用标定安全锁覆盖"
            )
            estop_confirmed = True
        else:
            try:
                self._call_estop(True, "自动标定结束锁止")
                self._wait_status_estop(True)
                estop_confirmed = True
            except SafetyAbort as error:
                self.get_logger().error(str(error))
        stable_since = 0.0
        wheels_stopped = False
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            with self._lock:
                mcu = self._mcu
            if (
                mcu
                and max(abs(mcu.left_wheel_speed), abs(mcu.right_wheel_speed))
                < self.stopped_speed_mps
            ):
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= self.stopped_hold_sec:
                    wheels_stopped = True
                    break
            else:
                stable_since = 0.0
            time.sleep(0.02)
        return estop_confirmed and wheels_stopped

    def _result_root(self):
        configured = str(self.result_root).strip()
        if configured:
            return Path(configured).expanduser().resolve()
        share = Path(get_package_share_directory("car_calibrate")).resolve()
        for parent in share.parents:
            if parent.name == "install":
                return parent.parent / "src" / "car_calibrate" / "result"
        return Path.cwd() / "src" / "car_calibrate" / "result"

    @staticmethod
    def _fsync_directory(path: Path):
        descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    def _write_csv(self, path: Path):
        with path.open("x", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            for row in self._rows:
                writer.writerow(row)
            stream.flush()
            os.fsync(stream.fileno())

    def _persist(self, run_id, success_requested, failure_code="safety"):
        root = self._result_root()
        root.mkdir(parents=True, exist_ok=True)
        staging = root / f".{run_id}.pending"
        if staging.exists() or (root / run_id).exists():
            raise FileExistsError(f"标定 run 已存在: {run_id}")
        staging.mkdir()
        csv_path = staging / "calibration_raw.csv"
        self._write_csv(csv_path)
        if not success_requested:
            safe_code = "".join(
                character if character.isalnum() or character in "_-" else "_"
                for character in str(failure_code)
            ).strip("_") or "safety"
            failed = root / f"{run_id}_failed_{safe_code}"
            os.replace(staging, failed)
            self._fsync_directory(root)
            return False, failed, None, failed / "calibration_raw.csv", "运行中止"
        digest = sha256_file(csv_path)
        try:
            result = estimate_rows(
                [{key: str(value) for key, value in row.items()} for row in self._rows],
                measured_at=self._measured_at,
                source_log_sha256=digest,
                min_valid_trials_per_level=int(self.min_valid_trials_per_level),
                min_metric_samples=int(self.min_metric_samples),
            )
        except CalibrationQualityError as error:
            failed = root / f"{run_id}_failed_quality"
            os.replace(staging, failed)
            self._fsync_directory(root)
            error.result_directory = failed
            error.csv_path = failed / "calibration_raw.csv"
            raise
        yaml_path = staging / "calibration.yaml"
        with yaml_path.open("x", encoding="utf-8") as stream:
            yaml.safe_dump(result.calibration, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        final = root / run_id
        os.replace(staging, final)
        self._fsync_directory(root)
        try:
            publish_calibration_snapshot(final, root / "calibration")
        except Exception as error:
            raise RuntimeError(
                f"历史结果已保存到 {final}，但最新 calibration 更新失败: "
                f"{error}"
            ) from error
        return True, final, final / "calibration.yaml", final / "calibration_raw.csv", "标定完成"

    def _execute(self, goal_handle):
        result = RunCalibration.Result()
        run_id = (
            datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%z")
            + f"_{self.vehicle_id}"
        )
        total = (
            len(self.linear_speeds_mps) * int(self.repeats)
            + 2 * len(self.angular_speeds_radps) * int(self.repeats)
        )
        completed = 0
        success_requested = False
        outcome = "failed"
        failure_code = "internal_error"
        message = "标定失败"
        final_directory = None
        yaml_path = None
        csv_path = None
        with self._lock:
            self._rows = []
            self._recording = False
            self._stopping = False
            self._event_code = ""
            self._event_message = ""
            self._pending_events = []
            self._data_error = ""
            self._previous_ticks = None
            self._command_timeout_samples = 0
            self._command_timeout_times = []
            self._imu_stale_active = False
            self._imu_dropout_count = 0
            self._imu_max_gap_sec = 0.0
            self._imu_hard_gap_sec = 0.0
            self._status_snapshot.imu_bias_radps = 0.0
            self._status_snapshot.imu_noise_radps = 0.0
            self._status_snapshot.imu_drift_radps = 0.0
            self._status_snapshot.mcu_frequency_hz = 0.0
            self._status_snapshot.mcu_max_gap_sec = 0.0
            self._status_snapshot.command_timeout_samples = 0
        self._set_status(
            active=True, state="preflight", phase="PREFLIGHT", completed=0,
            total=total, progress=0.0, run_id=run_id, result_directory="",
            message="正在执行安全预检",
        )
        try:
            self._preflight()
            for remaining in range(int(math.ceil(self.countdown_sec)), 0, -1):
                if goal_handle.is_cancel_requested:
                    raise SafetyAbort("canceled", "倒计时期间取消")
                self._set_status(
                    active=True, state="countdown", phase="COUNTDOWN",
                    completed=0, total=total, progress=0.0,
                    message=f"{remaining} 秒后开始自动移动",
                )
                time.sleep(min(1.0, self.countdown_sec))
            self._call_estop(False, "自动标定开始")
            self._wait_status_estop(False)
            with self._lock:
                self._rows = []
                self._message_times = {name: [] for name in ("mcu", "imu", "move", "odom")}
                self._command_timeout_times = []
                self._run_started = time.monotonic()
                self._measured_at = datetime.now().astimezone().isoformat(timespec="seconds")
                self._request_changed = self._run_started
                self._recording = True
                self._data_error = ""
                self._previous_ticks = None
                self._imu_bias = 0.0
                self._imu_yaw = 0.0
                self._previous_imu_time = 0.0
                self._imu_stale_active = False
                self._imu_dropout_count = 0
                self._imu_max_gap_sec = 0.0
                self._imu_hard_gap_sec = 0.0
            self._run_a0(goal_handle, total)
            self._run_a1(goal_handle, total)
            completed = self._run_trials(goal_handle, total)
            self._set_status(
                active=True, state="running", phase="A3",
                completed=completed, total=total, progress=1.0,
                message="全部运动试验完成，正在进入安全停车和分析",
            )
            success_requested = True
            outcome = "succeeded"
            message = "运动试验完成，正在安全停车和分析"
        except SafetyAbort as error:
            failure_code = error.code
            outcome = "canceled" if error.code == "canceled" else "aborted"
            message = str(error)
            with self._lock:
                self._event_code = error.code
                self._event_message = str(error)
            self.get_logger().error(f"标定中止 [{error.code}]: {error}")
        except Exception as error:  # 所有未知异常都必须进入同一停车链
            outcome = "failed"
            message = str(error)
            with self._lock:
                self._event_code = "internal_error"
                self._event_message = str(error)
            self.get_logger().error(f"标定异常: {error}")
        finally:
            stopping_message = (
                "正在零速并启用标定安全锁"
                if success_requested
                else f"正在安全停车；触发原因：{message}"
            )
            self._set_status(
                active=True, state="stopping", phase="STOPPING",
                message=stopping_message,
            )
            stop_confirmed = self._safe_stop()
            if not stop_confirmed:
                success_requested = False
                outcome = "failed_stop"
                failure_code = "stop_unconfirmed"
                message = "无法确认急停和车轮停止；系统保持标定闭锁，请检查底盘"
            with self._lock:
                self._recording = False
            if self._rows:
                analyzing_message = (
                    "正在校验并写入标定结果"
                    if success_requested
                    else f"正在保存失败数据；触发原因：{message}"
                )
                self._set_status(
                    active=True, state="analyzing", phase="ANALYZING",
                    message=analyzing_message,
                )
                try:
                    (
                        persisted,
                        final_directory,
                        yaml_path,
                        csv_path,
                        persist_message,
                    ) = self._persist(run_id, success_requested, failure_code)
                    if not persisted and outcome == "succeeded":
                        outcome = "failed"
                    message = persist_message if persisted else message
                    if persisted and outcome == "succeeded":
                        with self._lock:
                            dropout_count = self._imu_dropout_count
                            max_gap = self._imu_max_gap_sec
                        if dropout_count:
                            message = (
                                f"{message}（IMU 短时空窗 {dropout_count} 次，"
                                f"最长 {max_gap:.3f}s，质量门禁已通过）"
                            )
                except CalibrationQualityError as error:
                    outcome = "failed_quality"
                    message = str(error)
                    final_directory = getattr(error, "result_directory", None)
                    csv_path = getattr(error, "csv_path", None)
                except Exception as error:
                    outcome = "failed"
                    message = f"结果写入失败: {error}"
            with self._lock:
                self._requested_linear = 0.0
                self._requested_angular = 0.0
                self._stopping = False
                self._goal_reserved = not stop_confirmed
            success = outcome == "succeeded" and yaml_path is not None
            self._set_status(
                active=not stop_confirmed, state=outcome, phase="DONE", completed=completed,
                total=total, progress=1.0 if success else float(completed / total),
                result_directory=str(final_directory or ""), message=message,
            )
        result.success = bool(outcome == "succeeded" and yaml_path is not None)
        result.outcome = outcome
        result.run_id = run_id
        result.result_directory = str(final_directory or "")
        result.calibration_yaml = str(yaml_path or "")
        result.calibration_csv = str(csv_path or "")
        result.message = message
        if result.success:
            goal_handle.succeed()
        elif outcome == "canceled":
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def destroy_node(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
