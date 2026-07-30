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
const navModeButtons = Array.from(document.querySelectorAll("[data-nav-mode]"));
const busyButtons = new Set();

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
  map_exists: "地图文件",
  fault_bits: "故障位",
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

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const responseText = await response.text();
  let payload = {};
  if (responseText) {
    try {
      payload = JSON.parse(responseText);
    } catch (_) {
      if (!response.ok) {
        throw new Error(`服务请求失败，HTTP ${response.status}`);
      }
      throw new Error("服务返回数据格式错误");
    }
  }
  if (!response.ok || payload.ok === false) {
    throw new Error(payload.detail || payload.message || `HTTP ${response.status}`);
  }
  return payload;
}

async function post(path, body = {}) {
  return api(path, {
    method: "POST",
    body: JSON.stringify(body),
  });
}

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

function phaseText(value) {
  return {
    waiting_chassis: "等待小车",
    waiting_lidar: "等待雷达",
    ready: "已就绪",
  }[value] || value || "-";
}

function statusValue(key, payload) {
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
    return `0x${Number(payload[key] || 0).toString(16)}`;
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
    && !payload.e_stop
    && state.goals.length > completed
    && !blockedState
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

  controlButtons.forEach((button) => {
    const blocked = Boolean(payload.obstacle) && button.dataset.drive === "forward";
    button.disabled = !online || payload.e_stop || blocked || navigating;
  });
  buttons.estop.disabled = false;
  buttons.release.disabled = false;
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
  if (payload.navigation && payload.navigation.state === "running") {
    mapStatus.textContent = "导航中";
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
