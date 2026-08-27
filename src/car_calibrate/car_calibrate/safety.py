"""标定流程共享的故障位、急停来源和计数器安全规则。"""

import math


MCU_FAULT_COMMAND_TIMEOUT = 1 << 4
MCU_CALIBRATION_IGNORED_FAULT_MASK = (1 << 7) | (1 << 8)
JETSON_CALIBRATION_IGNORED_FAULT_MASK = (
    MCU_CALIBRATION_IGNORED_FAULT_MASK | (1 << 20)
)
MCU_FAULT_NAMES = {
    0: "Wi-Fi 未连接",
    1: "micro-ROS Agent 未连接",
    2: "IMU 无效",
    3: "编码器无效",
    4: "底盘命令超时停车",
    5: "电机驱动无效",
}


def external_estop_active(status):
    """判断当前锁止是否来自标定程序之外。"""

    return bool(
        status is not None
        and status.e_stop_ok
        and str(getattr(status, "e_stop_source", "")) != "car_calibrate"
    )


def yaw_from_quaternion(orientation) -> float:
    """从单位四元数取得平面偏航角。"""

    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def int32_delta(current: int, previous: int) -> int:
    """计算允许有符号 32 位计数器回绕的差值。"""

    return ((int(current) - int(previous) + 2**31) % 2**32) - 2**31


def describe_mcu_fault_bits(fault_bits: int) -> str:
    """把已知和未知 MCU 故障位转换为可操作的中文说明。"""

    bits = int(fault_bits) & 0xFFFFFFFF
    if not bits:
        return "无"
    return "、".join(
        MCU_FAULT_NAMES.get(bit, f"未知故障 bit{bit}")
        for bit in range(32)
        if bits & (1 << bit)
    )


def effective_fatal_mcu_fault_bits(fault_bits: int, *, moving: bool) -> int:
    """忽略超声波位；零速时另允许 ESP32 保持命令超时停车。"""

    fatal = int(fault_bits) & 0xFFFFFFFF
    fatal &= ~MCU_CALIBRATION_IGNORED_FAULT_MASK
    if not moving:
        fatal &= ~MCU_FAULT_COMMAND_TIMEOUT
    return fatal


def fault_bit_snapshot(mcu, status) -> tuple[int, int]:
    """从位图移除超声波相关位，再分别保存 MCU 与 Jetson 故障。"""

    mcu_bits = int(mcu.fault_bits) & ~MCU_CALIBRATION_IGNORED_FAULT_MASK
    jetson_bits = (
        int(status.fault_bits) & ~JETSON_CALIBRATION_IGNORED_FAULT_MASK
        if status else 0
    )
    return mcu_bits, jetson_bits
