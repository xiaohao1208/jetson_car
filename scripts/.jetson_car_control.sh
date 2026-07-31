#!/usr/bin/env bash

# 公共控制函数。
# 不要在这里再次设置 set -euo pipefail。

JETSON_CAR_SCRIPT_DIR="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1
  pwd
)"

JETSON_CAR_ROOT="$(
  cd -- "${JETSON_CAR_SCRIPT_DIR}/.." >/dev/null 2>&1
  pwd
)"

JETSON_CAR_RUNTIME_DIR="${
  JETSON_CAR_RUNTIME_DIR:-/tmp/jetson-car-$(id -u)
}"

JETSON_CAR_STATE_FILE="${
  JETSON_CAR_RUNTIME_DIR
}/robot_bringup.state"

JETSON_CAR_LOCK_FILE="${
  JETSON_CAR_RUNTIME_DIR
}/robot_bringup.lock"

JETSON_CAR_LOG_FILE="${
  JETSON_CAR_LOG_FILE:-${JETSON_CAR_ROOT}/log/robot_bringup.log
}"

JETSON_CAR_HOTSPOT_CONFIG="${
  JETSON_CAR_HOTSPOT_CONFIG:-
  ${JETSON_CAR_ROOT}/src/car_bringup/config/bringup.yaml
}"

JETSON_CAR_PID=""
JETSON_CAR_START_TIME=""
JETSON_CAR_PROCESS_GROUP=""


# 获取进程管理锁，避免多个脚本同时修改状态。
jetson_car_acquire_lock()
{
  mkdir -p "${JETSON_CAR_RUNTIME_DIR}"

  exec 9>"${JETSON_CAR_LOCK_FILE}"

  flock -x 9
}


# 获取 /proc/<pid>/stat 中的进程启动时间。
# 用于防止 PID 被系统重复使用后误杀其他进程。
jetson_car_process_start_time()
{
  local pid="$1"
  local stat_line
  local stat_fields_text
  local -a stat_fields

  if [[ ! -r "/proc/${pid}/stat" ]]
  then
    return 1
  fi

  stat_line="$(<"/proc/${pid}/stat")"

  # 去掉 PID 和括号中的进程名称。
  # 剩余部分从 /proc stat 的第 3 个字段开始。
  stat_fields_text="${stat_line#*) }"

  read -r -a stat_fields <<< "${stat_fields_text}"

  # 原始第 22 个字段 starttime，
  # 在去掉前两个字段后位于数组索引 19。
  if (( ${#stat_fields[@]} < 20 ))
  then
    return 1
  fi

  printf '%s\n' "${stat_fields[19]}"
}


# 从状态文件读取 ROS 2 启动进程信息。
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
    JETSON_CAR_PROCESS_GROUP \
    < "${JETSON_CAR_STATE_FILE}" || return 1

  [[ "${JETSON_CAR_PID}" =~ ^[0-9]+$ ]] || return 1
  [[ "${JETSON_CAR_START_TIME}" =~ ^[0-9]+$ ]] || return 1
  [[ "${JETSON_CAR_PROCESS_GROUP}" =~ ^[0-9]+$ ]] || return 1
}


# 判断状态文件记录的进程是否仍然是原来的 ROS 2 进程。
jetson_car_process_alive()
{
  local actual_start_time
  local actual_process_group
  local actual_status

  if ! kill -0 "${JETSON_CAR_PID}" 2>/dev/null
  then
    return 1
  fi

  actual_start_time="$(
    jetson_car_process_start_time "${JETSON_CAR_PID}"
  )" || return 1

  actual_process_group="$(
    ps -o pgid= -p "${JETSON_CAR_PID}" 2>/dev/null |
      tr -d '[:space:]'
  )"

  actual_status="$(
    ps -o stat= -p "${JETSON_CAR_PID}" 2>/dev/null |
      tr -d '[:space:]'
  )"

  [[ "${actual_start_time}" == "${JETSON_CAR_START_TIME}" ]] ||
    return 1

  [[ "${actual_process_group}" == "${JETSON_CAR_PROCESS_GROUP}" ]] ||
    return 1

  [[ "${actual_status}" != Z* ]] ||
    return 1

  # setsid 创建的进程组中，PID 应等于 PGID。
  [[ "${JETSON_CAR_PROCESS_GROUP}" == "${JETSON_CAR_PID}" ]]
}


# 等待 ROS 2 进程退出。
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


# 向整个 ROS 2 进程组发送信号。
jetson_car_signal_group()
{
  local signal_name="$1"

  kill \
    -s "${signal_name}" \
    -- "-${JETSON_CAR_PROCESS_GROUP}" \
    2>/dev/null
}


# 加载 ROS 2 Humble 和当前工作区环境。
jetson_car_prepare_environment()
{
  if [[ ! -r "/opt/ros/humble/setup.bash" ]]
  then
    echo "未找到 ROS 2 Humble 环境" >&2
    return 1
  fi

  if [[ ! -r "${JETSON_CAR_ROOT}/install/setup.bash" ]]
  then
    echo "未找到 Jetson 工作区安装环境" >&2
    echo "请先在 ${JETSON_CAR_ROOT} 中执行 colcon build" >&2
    return 1
  fi

  # ROS 2 setup.bash 中可能访问尚未定义的变量，
  # 因此加载时临时关闭 nounset。
  set +u

  source /opt/ros/humble/setup.bash
  source "${JETSON_CAR_ROOT}/install/setup.bash"

  set -u
}


# 从 bringup.yaml 读取 NetworkManager 热点连接名称。
jetson_car_hotspot_connection()
{
  local connection

  if [[ ! -r "${JETSON_CAR_HOTSPOT_CONFIG}" ]]
  then
    echo "未找到热点配置文件：" >&2
    echo "${JETSON_CAR_HOTSPOT_CONFIG}" >&2
    return 1
  fi

  connection="$(
    awk '
      /^[[:space:]]*hotspot_connection[[:space:]]*:/ {
        sub(
          /^[[:space:]]*hotspot_connection[[:space:]]*:[[:space:]]*/,
          ""
        )

        sub(/[[:space:]]+#.*$/, "")
        gsub(/^[[:space:]]+|[[:space:]]+$/, "")

        print
        exit
      }
    ' "${JETSON_CAR_HOTSPOT_CONFIG}"
  )"

  # 允许 YAML 值使用单引号或双引号。
  connection="${connection#\"}"
  connection="${connection%\"}"
  connection="${connection#\'}"
  connection="${connection%\'}"

  if [[ -z "${connection}" ]]
  then
    echo "热点配置缺少 hotspot_connection" >&2
    return 1
  fi

  printf '%s\n' "${connection}"
}


# 检测指定 NetworkManager 连接是否为活动热点。
#
# 返回值：
#   0：热点已经开启
#   1：热点配置存在，但当前未激活
#   2：配置错误或无法读取 NetworkManager 状态
jetson_car_hotspot_active()
{
  local connection
  local nmcli_bin
  local connection_mode
  local connection_uuid
  local active_uuids

  connection="$(jetson_car_hotspot_connection)" || return 2

  nmcli_bin="${
    JETSON_CAR_NMCLI_BIN:-$(command -v nmcli || true)
  }"

  if [[ -z "${nmcli_bin}" || ! -x "${nmcli_bin}" ]]
  then
    echo "未找到可执行的 nmcli 命令" >&2
    return 2
  fi

  if ! connection_mode="$(
    LC_ALL=C "${nmcli_bin}" \
      -g 802-11-wireless.mode \
      connection show id "${connection}" \
      2>/dev/null
  )"
  then
    echo "未找到热点连接配置：${connection}" >&2
    return 2
  fi

  if [[ "${connection_mode}" != "ap" ]]
  then
    echo "连接不是 AP 热点模式：" >&2
    echo "连接=${connection}，mode=${connection_mode}" >&2
    return 2
  fi

  if ! connection_uuid="$(
    LC_ALL=C "${nmcli_bin}" \
      -g connection.uuid \
      connection show id "${connection}" \
      2>/dev/null
  )"
  then
    echo "无法读取热点连接 UUID：${connection}" >&2
    return 2
  fi

  if [[ -z "${connection_uuid}" ]]
  then
    echo "热点连接 UUID 为空：${connection}" >&2
    return 2
  fi

  if ! active_uuids="$(
    LC_ALL=C "${nmcli_bin}" \
      -t \
      --escape no \
      -f UUID \
      connection show --active
  )"
  then
    echo "读取 NetworkManager 活动连接失败" >&2
    return 2
  fi

  grep -Fxq -- "${connection_uuid}" <<< "${active_uuids}"
}


# 检查热点是否已经由 NetworkManager 启动。
#
# 此函数只检测，不会执行 nmcli connection up，
# 避免通过无线 SSH 时切换网卡并中断连接。
jetson_car_require_hotspot()
{
  local connection
  local check_result

  connection="$(jetson_car_hotspot_connection)" || return 1

  if jetson_car_hotspot_active
  then
    echo "热点已开启，连接=${connection}"
    return 0
  else
    check_result="$?"
  fi

  if [[ "${check_result}" -eq 1 ]]
  then
    echo "热点未开启，连接=${connection}" >&2
    echo "请检查 NetworkManager 开机自动连接状态" >&2
  else
    echo "无法正确读取热点状态" >&2
  fi

  return 1
}


# 启动 ROS 2 小车 bringup。
jetson_car_start_locked()
{
  local ros2_bin
  local pid
  local start_time
  local process_group
  local state_temp_file

  # 只检测热点，不主动修改网络。
  jetson_car_require_hotspot || return 1

  if jetson_car_load_state && jetson_car_process_alive
  then
    echo "Jetson 小车已经启动，PID=${JETSON_CAR_PID}"
    return 0
  fi

  # 删除失效或损坏的旧状态文件。
  rm -f "${JETSON_CAR_STATE_FILE}"

  jetson_car_prepare_environment || return 1

  ros2_bin="${
    JETSON_CAR_ROS2_BIN:-$(command -v ros2 || true)
  }"

  if [[ -z "${ros2_bin}" || ! -x "${ros2_bin}" ]]
  then
    echo "未找到可执行的 ros2 命令" >&2
    return 1
  fi

  mkdir -p "$(dirname -- "${JETSON_CAR_LOG_FILE}")"

  printf '\n[%s] 启动 Jetson 小车\n' \
    "$(date '+%F %T')" \
    >> "${JETSON_CAR_LOG_FILE}"

  (
    # 子进程不能继承控制脚本使用的 flock 文件描述符。
    exec 9>&-

    cd "${JETSON_CAR_ROOT}"

    exec nohup setsid \
      "${ros2_bin}" \
      launch \
      car_bringup \
      robot_bringup.launch.py
  ) >> "${JETSON_CAR_LOG_FILE}" 2>&1 &

  pid="$!"

  # 给 ros2 launch 一点时间完成最初的启动。
  sleep 0.8

  if ! kill -0 "${pid}" 2>/dev/null
  then
    echo "Jetson 小车启动失败" >&2
    echo "请查看日志：${JETSON_CAR_LOG_FILE}" >&2
    return 1
  fi

  if ! start_time="$(
    jetson_car_process_start_time "${pid}"
  )"
  then
    kill -s TERM "${pid}" 2>/dev/null || true

    echo "Jetson 小车启动失败，无法读取进程启动时间" >&2
    return 1
  fi

  process_group="$(
    ps -o pgid= -p "${pid}" 2>/dev/null |
      tr -d '[:space:]'
  )"

  if [[ -z "${process_group}" ]]
  then
    kill -s TERM "${pid}" 2>/dev/null || true

    echo "Jetson 小车启动失败，无法读取进程组" >&2
    return 1
  fi

  if [[ "${process_group}" != "${pid}" ]]
  then
    kill -s TERM "${pid}" 2>/dev/null || true

    echo "Jetson 小车启动失败，无法建立独立进程组" >&2
    return 1
  fi

  state_temp_file="${JETSON_CAR_STATE_FILE}.tmp.$$"

  printf '%s %s %s\n' \
    "${pid}" \
    "${start_time}" \
    "${process_group}" \
    > "${state_temp_file}"

  mv -f \
    "${state_temp_file}" \
    "${JETSON_CAR_STATE_FILE}"

  JETSON_CAR_PID="${pid}"
  JETSON_CAR_START_TIME="${start_time}"
  JETSON_CAR_PROCESS_GROUP="${process_group}"

  echo "Jetson 小车启动完成，PID=${pid}"
  echo "运行日志=${JETSON_CAR_LOG_FILE}"
}


# 停止 ROS 2 小车 bringup。
#
# 此函数不会关闭热点，因此 SSH 连接不会因为 stop.sh 中断。
jetson_car_stop_locked()
{
  if ! jetson_car_load_state || ! jetson_car_process_alive
  then
    rm -f "${JETSON_CAR_STATE_FILE}"

    echo "Jetson 小车已经退出"
    return 0
  fi

  echo "正在正常退出 Jetson 小车，PID=${JETSON_CAR_PID}"

  jetson_car_signal_group INT || true

  if jetson_car_wait_stopped 20
  then
    rm -f "${JETSON_CAR_STATE_FILE}"

    echo "Jetson 小车已退出"
    return 0
  fi

  echo "正常退出超时，发送 TERM 终止信号"

  jetson_car_signal_group TERM || true

  if jetson_car_wait_stopped 5
  then
    rm -f "${JETSON_CAR_STATE_FILE}"

    echo "Jetson 小车已退出"
    return 0
  fi

  echo "TERM 信号超时，发送 KILL 强制结束进程"

  jetson_car_signal_group KILL || true

  if jetson_car_wait_stopped 2
  then
    rm -f "${JETSON_CAR_STATE_FILE}"

    echo "Jetson 小车已强制退出"
    return 0
  fi

  echo "Jetson 小车退出失败，PID=${JETSON_CAR_PID}" >&2
  return 1
}