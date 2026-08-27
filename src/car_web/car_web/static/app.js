const { api, post } = window.CarHttp;
const statusList = document.getElementById("status-list");
const connectionState = document.getElementById("connection-state");
const mapCanvas = document.getElementById("map-canvas");
const mapContext = mapCanvas.getContext("2d");
const mapStatus = document.getElementById("map-status");
const mapMessage = document.getElementById("map-message");
const robotPoseReadout = document.getElementById("robot-pose");
const linearSpeedValue = document.getElementById("linear-speed-value");
const angularSpeedValue = document.getElementById("angular-speed-value");
const mapActionButtons = Array.from(document.querySelectorAll("[data-map-action]"));
const controlButtons = Array.from(document.querySelectorAll("[data-drive]"));
const speedButtons = Array.from(document.querySelectorAll("[data-speed-kind]"));
const navModeButtons = Array.from(document.querySelectorAll("[data-nav-mode]"));
const allButtons = Array.from(document.querySelectorAll("button"));
const busyButtons = new Set();
const calibrationDialog = document.getElementById("calibration-dialog");
const calibrationForm = document.getElementById("calibration-form");
const calibrationChecks = Array.from(
  calibrationForm.querySelectorAll('input[type="checkbox"]'),
);
const confirmCalibration = document.getElementById("confirm-calibration");
const cancelCalibration = document.getElementById("cancel-calibration");
const observationDialog = document.getElementById("observation-dialog");
const observationForm = document.getElementById("observation-form");
const observationChecks = Array.from(
  observationForm.querySelectorAll('input[type="checkbox"]'),
);
const observationSafetyConfirmations = document.getElementById(
  "observation-safety-confirmations",
);
const observationUnavailableReason = document.getElementById(
  "observation-unavailable-reason",
);
const observationRequirementItems = Object.fromEntries(
  Array.from(
    observationForm.querySelectorAll("[data-observation-requirement]"),
  ).map((item) => [item.dataset.observationRequirement, item]),
);
const confirmObservation = document.getElementById("confirm-observation");
const cancelObservationDialog = document.getElementById("cancel-observation-dialog");

const buttons = {
  startMapping: document.getElementById("start-mapping"),
  stopMapping: document.getElementById("stop-mapping"),
  startNavigation: document.getElementById("start-navigation"),
  stopNavigation: document.getElementById("cancel-nav"),
  startGoals: document.getElementById("start-goal-navigation"),
  removeLastGoal: document.getElementById("remove-last-goal"),
  clearGoals: document.getElementById("clear-goals"),
  estop: document.getElementById("estop"),
  release: document.getElementById("release"),
  calibrate: document.getElementById("calibrate"),
  observation: document.getElementById("observation-collection"),
};

const COMMAND_REPEAT_PERIOD = 200;
const STATUS_POLL_PERIOD = 250;
const MAP_POLL_PERIOD = 2000;
const MIN_DIRECTION_DRAG = 8;
const speedLimits = {
  linear: { min: 5, max: 40, digits: 0, unit: "cm/s" },
  angular: { min: 30, max: 120, digits: 0, unit: "°/s" },
};

const statusLabels = {
  car_online: "小车",
  connection_phase: "连接阶段",
  lidar_tcp_connected: "雷达TCP",
  scan_ok: "/scan",
  odom_ok: "/odom",
  sensor_ready: "传感器",
  e_stop: "急停",
  obstacle: "障碍物",
  obstacle_distance: "障碍距离",
  mapping_state: "建图状态",
  rl_state: "RL模型",
  navigation_state: "导航任务",
  navigation_progress: "导航进度",
  obstacle_scan_state: "近障扫描",
  map_exists: "地图文件",
  fault_bits: "故障位",
  calibration_state: "自动标定",
  observation_state: "自动观测",
};

const state = {
  status: null,
  map: null,
  mapImage: null,
  transform: null,
  goals: [],
  selectedMode: "classic",
  mapAction: null,
  pointerStart: null,
  draftPose: null,
  activeDriveButton: null,
  driveTimer: null,
  speed: { linear: 10, angular: 60 },
};

async function runButtonAction(button, action) {
  if (busyButtons.has(button)) return null;
  const originalText = button.textContent;
  busyButtons.add(button);
  button.disabled = true;
  button.textContent = button.dataset.busyLabel || "处理中";
  try {
    const result = await action();
    mapMessage.textContent = result.message || "操作完成";
    return result;
  } catch (error) {
    mapMessage.textContent = error.message;
    return null;
  } finally {
    button.textContent = originalText;
    busyButtons.delete(button);
    updateButtonStates();
  }
}

function formatBoolean(value) {
  return value ? "是" : "否";
}

function formatFaultBits(value) {
  const bits = Number(value || 0) >>> 0;
  if (bits === 0) return "0x0（无）";
  const names = {
    0: "Wi-Fi未连接",
    1: "Agent未连接",
    2: "IMU无效",
    3: "编码器无效",
    4: "命令超时停车",
    5: "电机驱动无效",
    7: "超声波本次无回波（警告）",
    8: "前方障碍物已锁存",
    18: "真实急停",
    19: "MCU离线",
    20: "前方障碍物",
  };
  const active = [];
  for (let bit = 0; bit < 32; bit += 1) {
    if ((bits & (1 << bit)) !== 0) {
      active.push(names[bit] || `未知故障bit${bit}`);
    }
  }
  return `0x${bits.toString(16)}（${active.join("、")}）`;
}

function phaseText(value) {
  return {
    waiting_chassis: "等待小车",
    waiting_lidar: "等待雷达",
    ready: "已就绪",
  }[value] || value || "-";
}

function statusValue(key, payload) {
  if (key === "e_stop") {
    if (!payload.e_stop) return "未触发";
    const reason = payload.e_stop_reason
      ? `：${payload.e_stop_reason}`
      : "";
    if (payload.e_stop_kind === "calibration_lock") {
      return `标定安全锁${reason}`;
    }
    if (payload.e_stop_kind === "observation_lock") {
      return `观测采集安全锁${reason}`;
    }
    const source = payload.e_stop_source || "未知来源";
    return `真实急停（${source}）${reason}`;
  }
  if (key === "calibration_state") {
    const calibration = payload.calibration || {};
    if (!calibration.enabled) return "入口关闭";
    const stateText = {
      idle: "空闲",
      starting: "正在启动",
      preflight: "安全检查",
      countdown: "倒计时",
      running: "运行中",
      analyzing: "分析数据",
      stopping: "安全停车",
      succeeded: "已完成",
      aborted: "已中止",
      canceled: "已取消",
      failed_quality: "质量检查失败",
      failed_stop: "停车确认失败",
      failed: "失败",
    }[calibration.state] || calibration.state || "空闲";
    const total = Number(calibration.total_trials || 0);
    const completed = Number(calibration.completed_trials || 0);
    return total > 0 ? `${stateText} ${completed}/${total}` : stateText;
  }
  if (key === "observation_state") {
    const observation = payload.observation_collection || {};
    if (!observation.enabled) return "入口关闭";
    const stateText = {
      idle: "空闲",
      starting: "正在启动",
      preflight: "检查地图和路线",
      countdown: "倒计时",
      collecting: "采集中",
      balancing: "平衡选择",
      stopping: "安全停车",
      finalizing: "发布结果",
      succeeded: "已完成",
      aborted: "已中止",
      failed_quality: "覆盖质量不足",
      failed: "失败",
    }[observation.state] || observation.state || "空闲";
    const count = Number(observation.collected_samples || 0);
    const target = Number(observation.target_samples || 0);
    return target > 0 ? `${stateText} ${count}/${target}` : stateText;
  }
  if (key === "navigation_state") {
    const navigation = payload.navigation || {};
    return {
      idle: "未开始",
      sending: "正在发送",
      running: "导航中",
      pausing: "正在暂停",
      paused: "已暂停",
      canceling: "正在取消",
      succeeded: "已完成",
      partial: "部分完成",
      canceled: "已取消",
      failed: "失败",
      rejected: "目标被拒绝",
    }[navigation.state] || "-";
  }
  if (key === "navigation_progress") {
    const navigation = payload.navigation || {};
    const total = Math.max(
      Number(navigation.total_waypoints || 0),
      state.goals.length,
    );
    if (total <= 0) return "-";
    const completed = Number(navigation.completed_waypoints || 0);
    return `${Math.min(completed, total)}/${total}`;
  }
  if (key === "obstacle_scan_state") {
    const scan = payload.obstacle_scan || {};
    if (!scan.active) return "空闲";
    const target = Number(scan.target_offset_rad || 0) * 180 / Math.PI;
    const direction = target >= 0 ? "左" : "右";
    return `${direction}${Math.abs(target).toFixed(0)}°`;
  }
  if (key === "rl_state") {
    const rl = payload.rl || {};
    if (rl.bundle === false) return "未安装";
    if ("controller_available" in rl && rl.controller_available) {
      return "已就绪";
    }
    if ("controller_available" in rl) {
      return "未就绪";
    }
    if (rl.bundle && rl.backend && rl.controller_engine) {
      return "已就绪";
    }
    return "未就绪";
  }
  if (key === "connection_phase") return phaseText(payload[key]);
  if (key === "mapping_state") {
    return {
      idle: "空闲",
      starting: "正在启动",
      running: "建图中",
      saving: "正在保存",
      stopping: "正在停止",
      error: "异常",
    }[payload[key]] || payload[key] || "-";
  }
  if (key === "obstacle_distance") {
    return Number(payload[key]) > 0
      ? `${(Number(payload[key]) * 100).toFixed(1)} cm`
      : "-";
  }
  if (key === "fault_bits") {
    return formatFaultBits(payload[key]);
  }
  const value = payload[key];
  if (typeof value === "boolean") return formatBoolean(value);
  return value === undefined || value === null ? "-" : String(value);
}

function navigationModeAvailable(mode, payload = state.status || {}) {
  if (mode === "classic") return true;
  const rl = payload.rl || {};
  if (mode === "rl_controller" && "controller_available" in rl) {
    return Boolean(rl.controller_available);
  }
  if (!rl.bundle || !rl.backend || !rl.controller_engine) return false;
  return mode === "rl_controller";
}

function renderNavigationMode() {
  navModeButtons.forEach((button) => {
    const available = navigationModeAvailable(button.dataset.navMode);
    button.classList.toggle(
      "is-active",
      button.dataset.navMode === state.selectedMode,
    );
    button.classList.toggle("is-unavailable", !available);
    button.setAttribute("aria-disabled", String(!available));
  });
}

function clearMapAction(goalOnly = false) {
  if (goalOnly && state.mapAction !== "goal") return;
  state.mapAction = null;
  state.pointerStart = null;
  state.draftPose = null;
  mapActionButtons.forEach((button) => {
    button.classList.remove("is-active");
  });
}

function navigationCanEdit(payload = state.status || {}) {
  const navigation = payload.navigation || {};
  const blockedState = [
    "sending", "running", "pausing", "paused", "canceling",
  ].includes(navigation.state);
  return Boolean(
    payload.navigation_active
    && payload.navigation_phase === "ready"
    && payload.navigation_ready
    && !(payload.calibration && payload.calibration.active)
    && !(payload.observation_collection && payload.observation_collection.active)
    && !payload.e_stop
    && !blockedState
  );
}

function initialPoseCanEdit(payload = state.status || {}) {
  const navigation = payload.navigation || {};
  const navigationTransition = [
    "starting", "retrying", "stopping",
  ].includes(payload.navigation_phase);
  const blockedState = [
    "sending", "running", "pausing", "paused", "canceling",
  ].includes(navigation.state);
  if (
    payload.mapping_active
    || (payload.calibration && payload.calibration.active)
    || (payload.observation_collection && payload.observation_collection.active)
    || payload.e_stop
    || blockedState
    || navigationTransition
  ) return false;
  return (
    !payload.navigation_active
    || Boolean(payload.navigation_ready)
  );
}

function mapActionCanEdit(action, payload = state.status || {}) {
  return action === "pose"
    ? initialPoseCanEdit(payload)
    : navigationCanEdit(payload);
}

function navigationCanRun(payload = state.status || {}) {
  const navigation = payload.navigation || {};
  const completed = Math.max(
    0,
    Math.min(
      Number(navigation.completed_waypoints || 0),
      state.goals.length,
    ),
  );
  const blockedState = [
    "sending", "running", "pausing", "canceling",
  ].includes(navigation.state);
  return Boolean(
    payload.navigation_active
    && payload.navigation_phase === "ready"
    && payload.navigation_ready
    && !(payload.calibration && payload.calibration.active)
    && !(payload.observation_collection && payload.observation_collection.active)
    && !payload.e_stop
    && state.goals.length > completed
    && !blockedState
  );
}

function setObservationRequirement(name, ready, message) {
  const item = observationRequirementItems[name];
  if (!item) return;
  item.textContent = message;
  item.classList.toggle("is-ready", Boolean(ready));
  item.classList.toggle("is-blocked", !ready);
}

function renderObservationDialogState(payload = state.status || {}) {
  const observation = payload.observation_collection || {};
  const mapReady = Boolean(payload.map_exists);
  const classicNavigationReady = Boolean(
    payload.navigation_active
    && payload.navigation_phase === "ready"
    && payload.navigation_ready
    && payload.navigation_mode === "classic"
  );
  const localizationReady = Boolean(payload.localization_ready);
  const waypointCount = state.goals.length;
  const waypointsReady = waypointCount >= 3;

  setObservationRequirement(
    "map",
    mapReady,
    mapReady
      ? "已有可用地图"
      : "缺少地图：请先建图并保存，或加载已有地图",
  );
  let navigationMessage = "经典导航已启动并就绪";
  if (payload.navigation_active && payload.navigation_mode !== "classic") {
    navigationMessage = "当前不是经典导航：请结束导航后选择经典模式";
  } else if (!payload.navigation_active) {
    navigationMessage = "尚未启动导航：请选择经典模式并点击“开始导航”";
  } else if (!classicNavigationReady) {
    navigationMessage = "经典导航正在启动，请等待导航就绪";
  }
  setObservationRequirement(
    "navigation",
    classicNavigationReady,
    navigationMessage,
  );
  setObservationRequirement(
    "localization",
    localizationReady,
    localizationReady
      ? "地图定位已完成"
      : classicNavigationReady
        ? "定位未完成：请在地图上设置当前位置并等待 AMCL 确认"
        : "地图定位：请先启动经典导航",
  );
  setObservationRequirement(
    "waypoints",
    waypointsReady,
    waypointsReady
      ? `已设置 ${waypointCount} 个安全航点`
      : `安全航点不足：已设置 ${waypointCount}/3 个`,
  );

  const startReady = observation.start_ready === true;
  observationSafetyConfirmations.disabled = !startReady;
  observationUnavailableReason.textContent = startReady
    ? "前置条件已满足，请继续确认下面的安全事项。"
    : observation.unavailable_reason || "观测状态尚未就绪，请稍后重试。";
  observationUnavailableReason.classList.toggle("is-ready", startReady);
  confirmObservation.textContent = startReady
    ? "确认并倒计时开始"
    : "条件未满足";
  confirmObservation.disabled = (
    !startReady
    || !observationChecks.every((item) => item.checked)
  );
}

function updateButtonStates() {
  const payload = state.status || {};
  const online = Boolean(payload.car_online);
  const mapping = Boolean(payload.mapping_active);
  const navigationStack = Boolean(payload.navigation_active);
  const navigationTransition = [
    "starting", "retrying", "stopping",
  ].includes(payload.navigation_phase);
  const navigationBusy = (
    Boolean(payload.navigation_busy)
    || navigationStack
    || navigationTransition
  );
  const navigating = payload.navigation && [
    "sending", "running", "pausing", "paused", "canceling",
  ].includes(payload.navigation.state);
  const paused = payload.navigation
    && payload.navigation.state === "paused";
  const sensorReady = Boolean(payload.sensor_ready);
  const calibration = payload.calibration || {};
  const calibrationActive = Boolean(calibration.active);
  const observation = payload.observation_collection || {};
  const observationActive = Boolean(observation.active);

  buttons.calibrate.hidden = !Boolean(calibration.enabled);
  buttons.observation.hidden = !Boolean(observation.enabled) && !observationActive;
  buttons.observation.textContent = observationActive ? "取消观测" : "观测";

  controlButtons.forEach((button) => {
    const blocked = Boolean(payload.obstacle) && button.dataset.drive === "forward";
    button.disabled = !online || payload.e_stop || blocked || navigating || observationActive;
  });
  // 长任务的统一锁定会直接写入disabled；每次状态刷新都必须显式恢复
  // 速度按钮，否则观测结束和安全锁解除后它们会永久保持灰色。
  speedButtons.forEach((button) => {
    button.disabled = !online || payload.e_stop || calibrationActive || observationActive;
  });
  buttons.estop.disabled = false;
  buttons.release.disabled = calibrationActive || observationActive;
  buttons.startMapping.disabled = !sensorReady || mapping || navigationBusy;
  buttons.stopMapping.disabled = !mapping;
  buttons.startNavigation.disabled = (
    !sensorReady || !payload.map_exists || mapping || navigationBusy
  );
  buttons.stopNavigation.disabled = !navigationBusy && !navigating;
  buttons.startGoals.textContent = paused ? "继续导航" : "开始导航";
  buttons.startGoals.disabled = !navigationCanRun(payload);
  buttons.removeLastGoal.disabled = navigating || state.goals.length === 0;
  buttons.clearGoals.disabled = navigating || state.goals.length === 0;
  mapActionButtons.forEach((button) => {
    button.disabled = (
      !state.map
      || !mapActionCanEdit(button.dataset.mapAction, payload)
    );
  });
  navModeButtons.forEach((button) => {
    const available = navigationModeAvailable(
      button.dataset.navMode,
      payload,
    );
    button.disabled = mapping || navigationBusy || !available;
    button.classList.toggle("is-unavailable", !available);
    button.setAttribute("aria-disabled", String(!available));
  });
  buttons.calibrate.disabled = (
    !calibration.enabled
    || !calibration.available
    || !online
    || mapping
    || navigationBusy
    || calibrationActive
    || payload.e_stop
  );
  const observationUnavailableReason = (
    observation.unavailable_reason || "观测状态尚未就绪，请稍后重试"
  );
  buttons.observation.disabled = observationActive
    ? false
    : !observation.enabled || mapping || calibrationActive;
  buttons.observation.title = observationActive
    ? "取消正在运行的自动观测"
    : observation.start_ready
      ? "开始自动观测"
      : observationUnavailableReason;
  buttons.observation.setAttribute(
    "aria-disabled", String(buttons.observation.disabled),
  );
  renderObservationDialogState(payload);
  if (calibrationActive || observationActive) {
    clearDriveState();
    clearMapAction();
    if (calibrationDialog.open) calibrationDialog.close();
    if (observationDialog.open) observationDialog.close();
    allButtons.forEach((button) => {
      button.disabled = (
        button !== buttons.estop
        && !(observationActive && button === buttons.observation)
        && !(observationActive && button === buttons.stopNavigation)
      );
    });
    mapCanvas.classList.add("is-locked");
    mapCanvas.setAttribute("aria-disabled", "true");
  } else {
    mapCanvas.classList.remove("is-locked");
    mapCanvas.setAttribute("aria-disabled", "false");
  }
}

function renderStatus(payload) {
  state.status = payload;
  if (
    state.mapAction
    && !mapActionCanEdit(state.mapAction, payload)
  ) {
    clearMapAction();
  }
  if (payload.navigation_active && payload.navigation_mode) {
    state.selectedMode = payload.navigation_mode;
  } else if (!navigationModeAvailable(state.selectedMode, payload)) {
    state.selectedMode = "classic";
  }
  connectionState.textContent = phaseText(payload.connection_phase);
  connectionState.classList.toggle("is-online", Boolean(payload.car_online));
  statusList.innerHTML = "";
  Object.entries(statusLabels).forEach(([key, label]) => {
    const term = document.createElement("dt");
    const detail = document.createElement("dd");
    term.textContent = label;
    detail.textContent = statusValue(key, payload);
    statusList.appendChild(term);
    statusList.appendChild(detail);
  });
  renderMapStatus();
  renderNavigationMode();
  updateButtonStates();
  renderRobotPose(payload.robot_pose || null);
  if (state.map && state.mapImage) {
    state.map.robot_pose = payload.robot_pose || null;
    renderMap();
  }
}

function renderMapStatus() {
  const payload = state.status || {};
  const calibration = payload.calibration || {};
  const observation = payload.observation_collection || {};
  const imuBias = Number(calibration.imu_bias_radps || 0);
  const imuNoise = Number(calibration.imu_noise_radps || 0);
  const imuDrift = Number(calibration.imu_drift_radps || 0);
  const mcuFrequency = Number(calibration.mcu_frequency_hz || 0);
  const mcuMaxGap = Number(calibration.mcu_max_gap_sec || 0);
  const commandTimeouts = Number(calibration.command_timeout_samples || 0);
  const imuDiagnostic = imuBias !== 0 || imuNoise !== 0 || imuDrift !== 0
    ? `；IMU零偏 ${imuBias.toFixed(4)}rad/s，噪声 ${imuNoise.toFixed(4)}rad/s，漂移 ${imuDrift.toFixed(4)}rad/s`
    : "";
  const linkDiagnostic = mcuFrequency > 0
    ? `；MCU ${mcuFrequency.toFixed(1)}Hz，最大间隔 ${mcuMaxGap.toFixed(3)}s`
    : "";
  const timeoutDiagnostic = commandTimeouts > 0
    ? `；命令超时样本 ${commandTimeouts}`
    : "";
  const calibrationDiagnostics = (
    `${imuDiagnostic}${linkDiagnostic}${timeoutDiagnostic}`
  );
  const calibrationTerminalText = {
    succeeded: "标定已完成",
    aborted: "标定已中止",
    canceled: "标定已取消",
    failed_quality: "标定质量检查失败",
    failed_stop: "标定停车确认失败",
    failed: "标定失败",
  };
  if (calibration.active) {
    const progress = Math.round(Number(calibration.progress || 0) * 100);
    mapStatus.textContent = `标定中 ${progress}%`;
    if (calibration.message) {
      mapMessage.textContent = `${calibration.message}${calibrationDiagnostics}`;
    }
  } else if (calibrationTerminalText[calibration.state]) {
    mapStatus.textContent = calibrationTerminalText[calibration.state];
    mapMessage.textContent = `${calibration.message || mapStatus.textContent}${calibrationDiagnostics}`;
  } else if (observation.active) {
    const progress = Math.round(Number(observation.progress || 0) * 100);
    mapStatus.textContent = `观测中 ${progress}%`;
    const coverage = (
      `样本 ${Number(observation.collected_samples || 0)}/${Number(observation.target_samples || 0)}`
      + `，唯一 ${Number(observation.unique_samples || 0)}`
      + `，圈数 ${Number(observation.route_loops || 0)}`
      + `，航点 ${Number(observation.current_waypoint || 0) + 1}/${Number(observation.total_waypoints || 0)}`
    );
    mapMessage.textContent = `${observation.message || "正在采集"}；${coverage}`;
  } else if (["succeeded", "aborted", "failed_quality", "failed"].includes(observation.state)) {
    mapStatus.textContent = {
      succeeded: "观测采集已完成",
      aborted: "观测采集已中止",
      failed_quality: "观测覆盖质量不足",
      failed: "观测采集失败",
    }[observation.state];
    const warnings = (observation.warnings || []).length > 0
      ? `；警告：${observation.warnings.join("、")}` : "";
    mapMessage.textContent = `${observation.message || mapStatus.textContent}${warnings}`;
  } else if (observation.enabled && !payload.map_exists) {
    mapStatus.textContent = "观测需要地图";
    mapMessage.textContent = "没有可用地图，请先建图并保存或加载地图后再进行观测";
  } else if (payload.navigation && payload.navigation.state === "running") {
    const scan = payload.obstacle_scan || {};
    mapStatus.textContent = scan.active ? "近障扫描中" : "导航中";
    if (scan.active) {
      mapMessage.textContent = scan.message || "正在左右扫描并逐步扩大范围";
    }
  } else if (payload.navigation && payload.navigation.state === "sending") {
    mapStatus.textContent = "正在发送导航";
  } else if (payload.navigation && payload.navigation.state === "canceling") {
    mapStatus.textContent = "正在取消导航";
  } else if (payload.navigation && payload.navigation.state === "pausing") {
    mapStatus.textContent = "正在暂停导航";
  } else if (payload.navigation && payload.navigation.state === "paused") {
    mapStatus.textContent = "导航已暂停";
  } else if (payload.mapping_active) {
    mapStatus.textContent = {
      starting: "正在启动建图",
      saving: "正在保存地图",
      stopping: "正在停止建图",
      running: "建图中",
    }[payload.mapping_state] || "建图中";
  } else if (payload.navigation_phase === "retrying") {
    mapStatus.textContent = "正在重新启动导航";
  } else if (payload.navigation_phase === "starting") {
    mapStatus.textContent = "正在启动导航";
  } else if (payload.navigation_phase === "stopping") {
    mapStatus.textContent = "正在停止导航";
  } else if (payload.navigation_phase === "error") {
    mapStatus.textContent = (
      payload.navigation_error || "导航启动失败"
    );
  } else if (payload.navigation_active) {
    mapStatus.textContent = payload.navigation_ready
      ? "导航就绪"
      : "正在启动导航";
  } else if (state.map) {
    mapStatus.textContent = "地图在线";
  } else {
    mapStatus.textContent = "等待地图数据";
  }
}

function renderRobotPose(pose) {
  if (!pose) {
    robotPoseReadout.textContent = "位置: -";
    return;
  }
  robotPoseReadout.textContent = (
    `位置: x=${(pose.x * 100).toFixed(1)} cm, `
    + `y=${(pose.y * 100).toFixed(1)} cm, `
    + `yaw=${pose.yaw.toFixed(3)} rad`
  );
}

function mapToCanvas(pose) {
  const map = state.map;
  const transform = state.transform;
  const gridX = (pose.x - map.origin.x) / map.resolution;
  const gridY = (pose.y - map.origin.y) / map.resolution;
  return {
    x: transform.offsetX + gridX * transform.scale,
    y: transform.offsetY + (map.height - gridY) * transform.scale,
  };
}

function eventToMap(event) {
  const rect = mapCanvas.getBoundingClientRect();
  const canvasX = (event.clientX - rect.left) * mapCanvas.width / rect.width;
  const canvasY = (event.clientY - rect.top) * mapCanvas.height / rect.height;
  const gridX = (canvasX - state.transform.offsetX) / state.transform.scale;
  const gridY = state.map.height
    - (canvasY - state.transform.offsetY) / state.transform.scale;
  return {
    x: state.map.origin.x + gridX * state.map.resolution,
    y: state.map.origin.y + gridY * state.map.resolution,
    canvasX,
    canvasY,
  };
}

function drawPose(pose, color, alpha = 1) {
  const point = mapToCanvas(pose);
  mapContext.save();
  mapContext.translate(point.x, point.y);
  mapContext.rotate(-pose.yaw);
  mapContext.globalAlpha = alpha;
  mapContext.fillStyle = color;
  mapContext.beginPath();
  mapContext.moveTo(15, 0);
  mapContext.lineTo(-10, 8);
  mapContext.lineTo(-10, -8);
  mapContext.closePath();
  mapContext.fill();
  mapContext.restore();
}

function drawGoals() {
  if (state.goals.length === 0) return;
  mapContext.save();
  mapContext.lineWidth = 2;
  mapContext.strokeStyle = "#9ca3af";
  mapContext.font = "14px Arial";
  mapContext.textAlign = "center";
  mapContext.textBaseline = "middle";
  let previous = state.map.robot_pose;
  const completed = Number(
    state.status
    && state.status.navigation
    && state.status.navigation.completed_waypoints
    || 0
  );
  const missed = new Set(
    (
      state.status
      && state.status.navigation
      && state.status.navigation.missed_waypoints
      || []
    ).map(Number)
  );
  state.goals.forEach((goal, index) => {
    const point = mapToCanvas(goal);
    if (previous) {
      const previousPoint = mapToCanvas(previous);
      mapContext.beginPath();
      mapContext.moveTo(previousPoint.x, previousPoint.y);
      mapContext.lineTo(point.x, point.y);
      mapContext.stroke();
    }
    mapContext.fillStyle = missed.has(index)
      ? "#dc2626"
      : index < completed
        ? "#16a34a"
        : "#2563eb";
    mapContext.beginPath();
    mapContext.arc(point.x, point.y, 12, 0, Math.PI * 2);
    mapContext.fill();
    mapContext.fillStyle = "#ffffff";
    mapContext.fillText(String(index + 1), point.x, point.y);
    previous = goal;
  });
  mapContext.restore();
}

function renderMap() {
  mapContext.fillStyle = "#e5e7eb";
  mapContext.fillRect(0, 0, mapCanvas.width, mapCanvas.height);
  if (!state.map || !state.mapImage) {
    mapContext.fillStyle = "#374151";
    mapContext.font = "22px Arial";
    mapContext.textAlign = "center";
    mapContext.fillText("等待地图数据", mapCanvas.width / 2, mapCanvas.height / 2);
    state.transform = null;
    renderMapStatus();
    updateButtonStates();
    return;
  }
  const scale = Math.min(
    mapCanvas.width / state.map.width,
    mapCanvas.height / state.map.height,
  );
  const drawWidth = state.map.width * scale;
  const drawHeight = state.map.height * scale;
  state.transform = {
    scale,
    offsetX: (mapCanvas.width - drawWidth) * 0.5,
    offsetY: (mapCanvas.height - drawHeight) * 0.5,
  };
  mapContext.drawImage(
    state.mapImage,
    state.transform.offsetX,
    state.transform.offsetY,
    drawWidth,
    drawHeight,
  );
  if (state.map.robot_pose) drawPose(state.map.robot_pose, "#2563eb");
  drawGoals();
  if (state.draftPose) {
    drawPose(
      state.draftPose,
      state.mapAction === "goal" ? "#f97316" : "#0ea5e9",
      0.6,
    );
  }
  renderMapStatus();
  updateButtonStates();
}

async function refreshStatus() {
  const payload = await api("/api/status");
  renderStatus(payload);
}

async function refreshMap() {
  const payload = await api("/api/map");
  state.goals = payload.goals || [];
  renderRobotPose(payload.robot_pose || null);
  if (!payload.available) {
    clearMapAction();
    state.map = null;
    state.mapImage = null;
    if (payload.reason) mapMessage.textContent = payload.reason;
    renderMap();
    return;
  }
  const image = new Image();
  image.onload = () => {
    state.map = payload;
    state.mapImage = image;
    renderMap();
  };
  image.src = payload.image;
}

function renderSpeedControls() {
  linearSpeedValue.textContent = (
    `${state.speed.linear.toFixed(speedLimits.linear.digits)} `
    + speedLimits.linear.unit
  );
  angularSpeedValue.textContent = (
    `${state.speed.angular.toFixed(speedLimits.angular.digits)} `
    + speedLimits.angular.unit
  );
}

function adjustSpeed(kind, delta) {
  const limits = speedLimits[kind];
  if (!limits) return;
  const value = Math.min(
    limits.max,
    Math.max(limits.min, state.speed[kind] + delta),
  );
  state.speed[kind] = Number(value.toFixed(limits.digits));
  renderSpeedControls();
}

function driveCommand(button) {
  const linear = state.speed.linear / 100;
  const angular = state.speed.angular * Math.PI / 180;
  return {
    forward: { linear, angular: 0 },
    reverse: { linear: -linear, angular: 0 },
    left: { linear: 0, angular },
    right: { linear: 0, angular: -angular },
  }[button.dataset.drive];
}

function sendDrive(command) {
  return post("/api/move", command).catch((error) => {
    mapMessage.textContent = error.message;
  });
}

function clearDriveState() {
  if (state.driveTimer !== null) {
    window.clearInterval(state.driveTimer);
    state.driveTimer = null;
  }
  if (state.activeDriveButton) {
    state.activeDriveButton.classList.remove("is-active");
    state.activeDriveButton = null;
  }
}

function stopDrive() {
  clearDriveState();
  sendDrive({ linear: 0, angular: 0 });
}

function startDrive(button) {
  clearDriveState();
  state.activeDriveButton = button;
  button.classList.add("is-active");
  const send = () => sendDrive(driveCommand(button));
  send();
  state.driveTimer = window.setInterval(send, COMMAND_REPEAT_PERIOD);
}

document.querySelectorAll("[data-speed-kind]").forEach((button) => {
  button.addEventListener("click", () => {
    if (button.disabled) return;
    adjustSpeed(button.dataset.speedKind, Number(button.dataset.speedDelta));
  });
});

controlButtons.forEach((button) => {
  button.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    if (button.disabled) return;
    button.setPointerCapture(event.pointerId);
    startDrive(button);
  });
  button.addEventListener("pointerup", stopDrive);
  button.addEventListener("pointercancel", stopDrive);
  button.addEventListener("pointerleave", () => {
    if (state.activeDriveButton === button) stopDrive();
  });
});

window.addEventListener("blur", stopDrive);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) stopDrive();
});

navModeButtons.forEach((button) => {
  button.addEventListener("click", () => {
    if (button.disabled) return;
    if (!navigationModeAvailable(button.dataset.navMode)) {
      const rl = state.status && state.status.rl || {};
      mapMessage.textContent = (
        rl.reason || "强化学习模型或推理环境尚未就绪"
      );
      return;
    }
    state.selectedMode = button.dataset.navMode;
    renderNavigationMode();
  });
});

mapActionButtons.forEach((button) => {
  button.addEventListener("click", () => {
    if (button.disabled) return;
    state.mapAction = button.dataset.mapAction;
    state.draftPose = null;
    mapActionButtons.forEach((item) => {
      item.classList.toggle("is-active", item === button);
    });
    mapMessage.textContent = state.mapAction === "pose"
      ? "在地图上按住当前位置并拖动车头方向"
      : "在地图上按住目标点并拖动车头方向";
  });
});

mapCanvas.addEventListener("pointerdown", (event) => {
  if (
    state.status
    && (
      (state.status.calibration && state.status.calibration.active)
      || (state.status.observation_collection && state.status.observation_collection.active)
    )
  ) return;
  if (!state.transform || !state.mapAction) return;
  if (!mapActionCanEdit(state.mapAction)) {
    clearMapAction();
    renderMap();
    return;
  }
  event.preventDefault();
  state.pointerStart = eventToMap(event);
  state.draftPose = {
    x: state.pointerStart.x,
    y: state.pointerStart.y,
    yaw: 0,
  };
  mapCanvas.setPointerCapture(event.pointerId);
  renderMap();
});

mapCanvas.addEventListener("pointermove", (event) => {
  if (!state.pointerStart || !state.transform) return;
  event.preventDefault();
  const point = eventToMap(event);
  const distance = Math.hypot(
    point.canvasX - state.pointerStart.canvasX,
    point.canvasY - state.pointerStart.canvasY,
  );
  const fallback = state.map.robot_pose ? state.map.robot_pose.yaw : 0;
  state.draftPose.yaw = distance >= MIN_DIRECTION_DRAG
    ? Math.atan2(point.y - state.pointerStart.y, point.x - state.pointerStart.x)
    : fallback;
  renderMap();
});

mapCanvas.addEventListener("pointerup", async (event) => {
  if (!state.pointerStart || !state.draftPose) return;
  event.preventDefault();
  const pose = {
    x: state.draftPose.x,
    y: state.draftPose.y,
    yaw: state.draftPose.yaw,
  };
  try {
    if (state.mapAction === "pose") {
      if (!initialPoseCanEdit()) {
        throw new Error("当前不能设置当前位置");
      }
      await post("/api/navigation/initial-pose", pose);
      mapMessage.textContent = "当前位置已发送";
    } else {
      if (!navigationCanEdit()) {
        throw new Error("后台导航尚未准备就绪");
      }
      const result = await post("/api/navigation/goals", pose);
      state.goals = result.goals || state.goals;
      mapMessage.textContent = `已添加第 ${state.goals.length} 个目标点`;
    }
  } catch (error) {
    mapMessage.textContent = error.message;
  }
  state.pointerStart = null;
  state.draftPose = null;
  renderMap();
});

mapCanvas.addEventListener("pointercancel", () => {
  state.pointerStart = null;
  state.draftPose = null;
  renderMap();
});

buttons.estop.addEventListener("click", () => runButtonAction(
  buttons.estop,
  () => post("/api/emergency-stop", { stop: true, reason: "Web急停" }),
));
buttons.release.addEventListener("click", () => runButtonAction(
  buttons.release,
  () => post("/api/emergency-stop", { stop: false, reason: "Web解除急停" }),
));
buttons.calibrate.addEventListener("click", () => {
  if (buttons.calibrate.disabled) return;
  calibrationForm.reset();
  confirmCalibration.disabled = true;
  calibrationDialog.showModal();
});
cancelCalibration.addEventListener("click", () => calibrationDialog.close());
calibrationForm.addEventListener("change", () => {
  confirmCalibration.disabled = !calibrationChecks.every((item) => item.checked);
});
calibrationForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  if (!calibrationChecks.every((item) => item.checked)) return;
  const confirmations = Object.fromEntries(
    calibrationChecks.map((item) => [item.name, item.checked]),
  );
  calibrationDialog.close();
  state.status = state.status || {};
  state.status.calibration = {
    ...(state.status.calibration || {}),
    active: true,
    state: "starting",
    message: "正在提交标定请求",
  };
  updateButtonStates();
  const result = await runButtonAction(
    buttons.calibrate,
    () => post("/api/calibration/start", confirmations),
  );
  if (!result) {
    state.status.calibration.active = false;
    await refreshStatus().catch(() => {});
  }
});
buttons.observation.addEventListener("click", async () => {
  const observation = state.status && state.status.observation_collection || {};
  if (observation.active) {
    await runButtonAction(
      buttons.observation,
      () => post("/api/observation-collection/cancel"),
    );
    return;
  }
  if (buttons.observation.disabled) return;
  observationForm.reset();
  renderObservationDialogState();
  observationDialog.showModal();
});
cancelObservationDialog.addEventListener("click", () => observationDialog.close());
observationForm.addEventListener("change", () => {
  renderObservationDialogState();
});
observationForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  const observation = state.status && state.status.observation_collection || {};
  if (
    observation.start_ready !== true
    || !observationChecks.every((item) => item.checked)
  ) return;
  const confirmations = Object.fromEntries(
    observationChecks.map((item) => [item.name, item.checked]),
  );
  observationDialog.close();
  state.status = state.status || {};
  state.status.observation_collection = {
    ...(state.status.observation_collection || {}),
    active: true,
    state: "starting",
    message: "正在提交自动观测请求",
  };
  updateButtonStates();
  const result = await runButtonAction(
    buttons.observation,
    () => post("/api/observation-collection/start", confirmations),
  );
  if (!result) {
    state.status.observation_collection.active = false;
    await refreshStatus().catch(() => {});
  }
});
buttons.startMapping.addEventListener("click", async () => {
  clearMapAction();
  await runButtonAction(buttons.startMapping, () => post("/api/mapping/start"));
  await refreshStatus().catch(() => {});
});
buttons.stopMapping.addEventListener("click", async () => {
  await runButtonAction(buttons.stopMapping, () => post("/api/mapping/save-stop"));
  await refreshStatus().catch(() => {});
  await refreshMap().catch(() => {});
});
buttons.startNavigation.addEventListener("click", async () => {
  stopDrive();
  await runButtonAction(
    buttons.startNavigation,
    () => post("/api/navigation/start", { mode: state.selectedMode }),
  );
  await refreshStatus().catch(() => {});
});
buttons.stopNavigation.addEventListener("click", async () => {
  await runButtonAction(
    buttons.stopNavigation,
    () => post("/api/navigation/stop"),
  );
  clearMapAction();
  state.goals = [];
  renderMap();
  await refreshStatus().catch(() => {});
});
buttons.startGoals.addEventListener("click", async () => {
  stopDrive();
  if (!navigationCanRun()) {
    mapMessage.textContent = "后台导航尚未准备就绪";
    updateButtonStates();
    return;
  }
  const paused = state.status
    && state.status.navigation
    && state.status.navigation.state === "paused";
  await runButtonAction(
    buttons.startGoals,
    () => post(
      paused ? "/api/navigation/resume" : "/api/navigation/run"
    ),
  );
  await refreshStatus().catch(() => {});
});
buttons.removeLastGoal.addEventListener("click", async () => {
  const result = await runButtonAction(
    buttons.removeLastGoal,
    () => api("/api/navigation/goals/last", { method: "DELETE" }),
  );
  if (!result) return;
  state.goals = result.goals || [];
  renderMap();
});
buttons.clearGoals.addEventListener("click", async () => {
  await runButtonAction(
    buttons.clearGoals,
    () => api("/api/navigation/goals", { method: "DELETE" }),
  );
  state.goals = [];
  renderMap();
});

renderSpeedControls();
renderNavigationMode();
updateButtonStates();
refreshStatus().catch(() => {
  connectionState.textContent = "服务断开";
});
refreshMap().catch((error) => {
  mapMessage.textContent = error.message;
});
window.setInterval(() => {
  refreshStatus().catch(() => {
    connectionState.textContent = "服务断开";
    connectionState.classList.remove("is-online");
  });
}, STATUS_POLL_PERIOD);
window.setInterval(() => refreshMap().catch(() => {}), MAP_POLL_PERIOD);
