#!/usr/bin/env bash

# 三个公开控制脚本共用的进程管理函数

JETSON_CAR_SCRIPT_DIR="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1
  pwd
)"
JETSON_CAR_ROOT="$(
  cd -- "${JETSON_CAR_SCRIPT_DIR}/.." >/dev/null 2>&1
  pwd
)"
JETSON_CAR_RUNTIME_DIR="${JETSON_CAR_RUNTIME_DIR:-/tmp/jetson-car-$(id -u)}"
JETSON_CAR_STATE_FILE="${JETSON_CAR_RUNTIME_DIR}/robot_bringup.state"
JETSON_CAR_LOCK_FILE="${JETSON_CAR_RUNTIME_DIR}/robot_bringup.lock"
JETSON_CAR_LOG_FILE="${JETSON_CAR_LOG_FILE:-${JETSON_CAR_ROOT}/log/robot_bringup.log}"
JETSON_CAR_HOTSPOT_CONFIG="${JETSON_CAR_HOTSPOT_CONFIG:-${JETSON_CAR_ROOT}/src/car_bringup/config/bringup.yaml}"
JETSON_CAR_PID=""
JETSON_CAR_START_TIME=""
JETSON_CAR_PROCESS_GROUP=""

jetson_car_acquire_lock()
{
  mkdir -p "${JETSON_CAR_RUNTIME_DIR}"
  exec 9>"${JETSON_CAR_LOCK_FILE}"
  flock -x 9
}

jetson_car_process_start_time()
{
  local pid="$1"
  local stat_line
  local stat_fields
  if [[ ! -r "/proc/${pid}/stat" ]]
  then
    return 1
  fi
  stat_line="$(<"/proc/${pid}/stat")"
  stat_fields="${stat_line#*) }"
  set -- ${stat_fields}
  if [[ "$#" -lt 20 ]]
  then
    return 1
  fi
  printf '%s\n' "${20}"
}

jetson_car_load_state()
{
  JETSON_CAR_PID=""
  JETSON_CAR_START_TIME=""
  JETSON_CAR_PROCESS_GROUP=""
  if [[ ! -r "${JETSON_CAR_STATE_FILE}" ]]
  then
    return 1
  fi
  read -r \
    JETSON_CAR_PID \
    JETSON_CAR_START_TIME \
    JETSON_CAR_PROCESS_GROUP < "${JETSON_CAR_STATE_FILE}" || return 1
  [[ "${JETSON_CAR_PID}" =~ ^[0-9]+$ ]] || return 1
  [[ "${JETSON_CAR_START_TIME}" =~ ^[0-9]+$ ]] || return 1
  [[ "${JETSON_CAR_PROCESS_GROUP}" =~ ^[0-9]+$ ]] || return 1
}

jetson_car_process_alive()
{
  local actual_start
  local actual_group
  local actual_status
  if ! kill -0 "${JETSON_CAR_PID}" 2>/dev/null
  then
    return 1
  fi
  actual_start="$(jetson_car_process_start_time "${JETSON_CAR_PID}")" ||
    return 1
  actual_group="$(
    ps -o pgid= -p "${JETSON_CAR_PID}" 2>/dev/null |
      tr -d '[:space:]'
  )"
  actual_status="$(
    ps -o stat= -p "${JETSON_CAR_PID}" 2>/dev/null |
      tr -d '[:space:]'
  )"
  [[ "${actual_start}" == "${JETSON_CAR_START_TIME}" ]] || return 1
  [[ "${actual_group}" == "${JETSON_CAR_PROCESS_GROUP}" ]] || return 1
  [[ "${actual_status}" != Z* ]] || return 1
  [[ "${JETSON_CAR_PROCESS_GROUP}" == "${JETSON_CAR_PID}" ]]
}

jetson_car_wait_stopped()
{
  local timeout_sec="$1"
  local checks=$((timeout_sec * 10))
  local index
  for ((index = 0; index < checks; ++index))
  do
    if ! jetson_car_process_alive
    then
      return 0
    fi
    sleep 0.1
  done
  ! jetson_car_process_alive
}

jetson_car_signal_group()
{
  local signal_name="$1"
  kill -s "${signal_name}" -- "-${JETSON_CAR_PROCESS_GROUP}" 2>/dev/null
}

jetson_car_prepare_environment()
{
  if [[ ! -r "/opt/ros/humble/setup.bash" ]]
  then
    echo "未找到ROS2 Humble环境" >&2
    return 1
  fi
  if [[ ! -r "${JETSON_CAR_ROOT}/install/setup.bash" ]]
  then
    echo "未找到Jetson工作区安装环境，请先完成colcon build" >&2
    return 1
  fi
  set +u
  source /opt/ros/humble/setup.bash
  source "${JETSON_CAR_ROOT}/install/setup.bash"
  set -u
}

jetson_car_hotspot_connection()
{
  local connection
  if [[ ! -r "${JETSON_CAR_HOTSPOT_CONFIG}" ]]
  then
    echo "未找到热点配置${JETSON_CAR_HOTSPOT_CONFIG}" >&2
    return 1
  fi
  connection="$(
    awk '$1 == "hotspot_connection:" {print $2; exit}' \
      "${JETSON_CAR_HOTSPOT_CONFIG}"
  )"
  if [[ -z "${connection}" ]]
  then
    echo "热点配置缺少hotspot_connection" >&2
    return 1
  fi
  printf '%s\n' "${connection}"
}

jetson_car_hotspot_active()
{
  local connection
  local nmcli_bin
  local connection_mode
  local active_connections

  connection="$(jetson_car_hotspot_connection)" || return 2

  nmcli_bin="${JETSON_CAR_NMCLI_BIN:-$(command -v nmcli || true)}"
  if [[ -z "${nmcli_bin}" || ! -x "${nmcli_bin}" ]]
  then
    echo "未找到可执行的nmcli命令" >&2
    return 2
  fi

  # 检查该连接配置是否确实为AP热点模式
  connection_mode="$(
    LC_ALL=C "${nmcli_bin}" \
      -g 802-11-wireless.mode \
      connection show id "${connection}" 2>/dev/null || true
  )"

  if [[ -z "${connection_mode}" ]]
  then
    echo "未找到热点连接配置：${connection}" >&2
    return 2
  fi

  if [[ "${connection_mode}" != "ap" ]]
  then
    echo "连接${connection}不是AP热点模式，mode=${connection_mode}" >&2
    return 2
  fi

  # --escape no 避免连接名称中的冒号被转义
  active_connections="$(
    LC_ALL=C "${nmcli_bin}" \
      -t --escape no -f NAME \
      connection show --active
  )" || {
    echo "读取NetworkManager活动连接失败" >&2
    return 2
  }

  grep -Fxq "${connection}" <<< "${active_connections}"
}

jetson_car_ensure_hotspot()
{
  local connection
  local nmcli_bin
  local check_result

  if jetson_car_hotspot_active
  then
    connection="$(jetson_car_hotspot_connection)" || return 1
    echo "小车热点已经开启，连接=${connection}"
    return 0
  else
    check_result="$?"
  fi

  # 返回2说明不是简单的“热点未启动”，而是配置或命令错误
  if [[ "${check_result}" -eq 2 ]]
  then
    return 1
  fi

  connection="$(jetson_car_hotspot_connection)" || return 1
  nmcli_bin="${JETSON_CAR_NMCLI_BIN:-$(command -v nmcli || true)}"

  echo "正在启动小车热点，连接=${connection}"

  if ! LC_ALL=C "${nmcli_bin}" --wait 15 \
    connection up id "${connection}" >/dev/null
  then
    echo "小车热点启动失败，连接=${connection}" >&2
    return 1
  fi

  if ! jetson_car_hotspot_active
  then
    echo "NetworkManager已执行连接启动，但热点状态检测失败" >&2
    return 1
  fi

  echo "小车热点启动完成，连接=${connection}"
}

jetson_car_stop_hotspot()
{
  local connection
  local nmcli_bin
  local active_connections
  connection="$(jetson_car_hotspot_connection)" || return 1
  nmcli_bin="${JETSON_CAR_NMCLI_BIN:-$(command -v nmcli || true)}"
  if [[ -z "${nmcli_bin}" || ! -x "${nmcli_bin}" ]]
  then
    echo "未找到可执行的nmcli命令，无法关闭小车热点" >&2
    return 1
  fi
  active_connections="$(
    "${nmcli_bin}" -t -f NAME connection show --active
  )" || {
    echo "读取NetworkManager活动连接失败" >&2
    return 1
  }
  if ! grep -Fxq "${connection}" <<< "${active_connections}"
  then
    echo "小车热点已经关闭"
    return 0
  fi
  if ! "${nmcli_bin}" --wait 10 connection down id "${connection}" \
    >/dev/null
  then
    echo "小车热点关闭失败，连接=${connection}" >&2
    return 1
  fi
  echo "小车热点已关闭"
}

jetson_car_start_locked()
{
  local ros2_bin
  local pid
  local start_time
  local process_group

  # 每次执行启动命令时都检查热点。
  # 即使ROS 2进程已经运行，热点被意外关闭后也可以重新启动。
  jetson_car_ensure_hotspot || return 1

  if jetson_car_load_state && jetson_car_process_alive
  then
    echo "Jetson小车已经启动，PID=${JETSON_CAR_PID}"
    return 0
  fi

  rm -f "${JETSON_CAR_STATE_FILE}"
  jetson_car_prepare_environment || return 1

  ros2_bin="${JETSON_CAR_ROS2_BIN:-$(command -v ros2 || true)}"
  if [[ -z "${ros2_bin}" || ! -x "${ros2_bin}" ]]
  then
    echo "未找到可执行的ros2命令" >&2
    return 1
  fi
  mkdir -p "$(dirname -- "${JETSON_CAR_LOG_FILE}")"
  printf '\n[%s] 启动Jetson小车\n' "$(date '+%F %T')" \
    >> "${JETSON_CAR_LOG_FILE}"
  (
    exec 9>&-
    cd "${JETSON_CAR_ROOT}"
    exec nohup setsid \
      "${ros2_bin}" launch car_bringup robot_bringup.launch.py
  ) >> "${JETSON_CAR_LOG_FILE}" 2>&1 &
  pid="$!"
  sleep 0.5
  if ! kill -0 "${pid}" 2>/dev/null
  then
    echo "Jetson小车启动失败，请查看${JETSON_CAR_LOG_FILE}" >&2
    return 1
  fi
  start_time="$(jetson_car_process_start_time "${pid}")" || return 1
  process_group="$(
    ps -o pgid= -p "${pid}" 2>/dev/null |
      tr -d '[:space:]'
  )"
  if [[ "${process_group}" != "${pid}" ]]
  then
    kill -s TERM "${pid}" 2>/dev/null || true
    echo "Jetson小车启动失败，无法建立独立进程组" >&2
    return 1
  fi
  printf '%s %s %s\n' "${pid}" "${start_time}" "${process_group}" \
    > "${JETSON_CAR_STATE_FILE}.tmp"
  mv -f "${JETSON_CAR_STATE_FILE}.tmp" "${JETSON_CAR_STATE_FILE}"
  JETSON_CAR_PID="${pid}"
  JETSON_CAR_START_TIME="${start_time}"
  JETSON_CAR_PROCESS_GROUP="${process_group}"
  echo "Jetson小车启动完成，PID=${pid}"
  echo "运行日志=${JETSON_CAR_LOG_FILE}"
}

jetson_car_stop_locked()
{
  if ! jetson_car_load_state || ! jetson_car_process_alive
  then
    rm -f "${JETSON_CAR_STATE_FILE}"
    echo "Jetson小车已经退出"
    return 0
  fi
  echo "正在正常退出Jetson小车，PID=${JETSON_CAR_PID}"
  jetson_car_signal_group INT || true
  if jetson_car_wait_stopped 20
  then
    rm -f "${JETSON_CAR_STATE_FILE}"
    echo "Jetson小车已退出"
    return 0
  fi
  echo "正常退出超时，发送终止信号"
  jetson_car_signal_group TERM || true
  if jetson_car_wait_stopped 5
  then
    rm -f "${JETSON_CAR_STATE_FILE}"
    echo "Jetson小车已退出"
    return 0
  fi
  echo "终止信号超时，强制结束进程"
  jetson_car_signal_group KILL || true
  if jetson_car_wait_stopped 2
  then
    rm -f "${JETSON_CAR_STATE_FILE}"
    echo "Jetson小车已强制退出"
    return 0
  fi
  echo "Jetson小车退出失败，请检查PID=${JETSON_CAR_PID}" >&2
  return 1
}
