import {
  AXIS_STATE, CONTROL_MODE, LineParser, MODE, command, decodeFaults, encodeCommand,
  faultSummary, parseFastTelemetry, parseTelemetry,
} from './protocol.js?v=abz-low-speed-v2';

const state = {
  transport: 'disconnected',
  mode: MODE.SPI,
  modePending: null,
  controlMode: CONTROL_MODE.POSITION,
  stateCode: 1,
  axisState: AXIS_STATE[1],
  fault: 0,
  velocity: 0,
  velocitySetpoint: 0,
  rawVelocity: 0,
  windowVelocity: 0,
  current: 0,
  position: 0,
  positionSetpoint: 0,
  positionError: 0,
  positionTarget: Number.NaN,
  busVoltage: 12,
  temperature: 25,
  phaseAVoltage: 0,
  phaseBVoltage: 0,
  phaseCVoltage: 0,
  idMeasured: 0,
  iqSetpoint: 0,
  idSetpoint: 0,
  velocityIntegratorTorque: 0,
  lowSpeedTorque: 0,
  lowSpeedState: 0,
  observerLocked: false,
  angleError: 0,
  motorError: 0,
  encoderError: 0,
  controllerError: 0,
  sensorlessError: 0,
  armedState: 0,
  pwmArmed: false,
  encoderReady: false,
  motorCalibrated: false,
  anticoggingValid: false,
  anticoggingCalibrationActive: false,
  anticoggingIndex: 0,
  anticoggingCoverage: 0,
  direction: 0,
  fetThermistorError: 0,
  motorThermistorError: 0,
  history: [],
  faultHistory: [],
  view: 'console',
};

const dom = Object.fromEntries([
  'connectionBadge', 'portSelect', 'refreshPortsButton', 'connectButton', 'simulateButton',
  'estopButton', 'stateValue', 'faultValue', 'velocityValue', 'rpmValue', 'busVoltageValue',
  'currentValue', 'temperatureValue', 'modeHint', 'modeState', 'commandModeLabel',
  'velocityInput', 'positionInput', 'torqueInput', 'velocityButton', 'positionButton',
  'torqueButton', 'calibrateButton', 'clearFaultButton', 'saveButton', 'encoderStatus',
  'observerStatus', 'angleError', 'telemetryRate', 'lastMessage', 'toast', 'telemetryCanvas',
  'scopeCanvas', 'scopeVelocity', 'scopeCurrent', 'scopeSamples', 'terminalLog', 'terminalInput',
  'terminalSendButton', 'terminalClearButton', 'terminalTelemetryToggle', 'terminalAutoScroll',
  'guideStartButton', 'coggingCalibrationButton', 'guideModeText',
  'stepSafety', 'stepMode', 'stepCalibration', 'stepSave', 'calibrationStatus',
  'calibrationOutput', 'readConfigButton', 'applyConfigButton', 'configSaveButton',
  'configStatus', 'faultTableBody', 'clearFaultHistoryButton', 'pageEyebrow', 'pageTitle',
  'mobileViewSelect', 'pwmStatus', 'watchdogStatus', 'currentStatus', 'controlReadyStatus',
  'currentModeBadge', 'safeProfileButton', 'scopeRunButton', 'scopeClearButton',
  'scopeAutoScaleButton', 'scopeWindowSelect', 'scopeCaptureState', 'scopeSampleRate',
  'scopeWindowLabel', 'scopeWindowEndLabel', 'scopeRunIndicator', 'scopeAxisMin',
  'scopeAxisMax', 'scopeAxisApplyButton', 'scopeAxisResetButton', 'scopeMeasurements',
  'scopeSelectionBox', 'scopeCursorTooltip', 'consoleChartLegend',
  'scopeYZoomSlider', 'scopeYZoomValue', 'scopeXWindowSlider', 'scopeXWindowValue',
  'brakeOffPresetButton', 'brake5PresetButton', 'brake2PresetButton', 'brakePowerPreview',
].map((id) => [id, document.getElementById(id)]));

const modeButtons = [...document.querySelectorAll('[data-mode]')];
const navButtons = [...document.querySelectorAll('[data-view]')];
const views = [...document.querySelectorAll('.view')];
const configInputs = [...document.querySelectorAll('[data-config]')];
const writableConfigInputs = configInputs.filter((input) => !input.readOnly);
const foc3505Tuning = Object.freeze({
  'axis0.controller.config.pos_gain': '0.8',
  'axis0.controller.config.vel_gain': '0.0015',
  'axis0.controller.config.vel_integrator_gain': '0.005',
  'axis0.controller.config.vel_ramp_rate': '0.3',
  'axis0.motor.config.current_control_bandwidth': '500',
  'axis0.encoder.config.bandwidth': '100',
});
writableConfigInputs.forEach((input) => {
  if (foc3505Tuning[input.dataset.config] !== undefined) {
    input.value = foc3505Tuning[input.dataset.config];
    input.dataset.safeValue = foc3505Tuning[input.dataset.config];
  }
});
const parser = new LineParser();
const desktop = window.focDesktop?.isDesktop === true;
let serialPort = null;
let serialReader = null;
let serialWriter = null;
let serialWriteQueue = Promise.resolve();
let authorizedPorts = [];
let mockTimer = null;
let mockPositionTarget = 0;
let mockCoggingStartedAt = null;
let lastFrameAt = 0;
let configReadQueue = [];
let polling = false;
let nextSlowPollAt = 0;
let statusVersion = 0;
const scopeChannels = [...document.querySelectorAll('[data-scope-key]')];
const scopeValueOutputs = [...document.querySelectorAll('[data-scope-value]')];
let scopeRunning = true;
let scopeAutoScale = true;
let scopePausedAt = null;
let scopeWindowSeconds = 20;
let scopeTimeOffsetSeconds = 0;
let scopeSharedRange = { min: -2, max: 2 };
let scopeSharedZoom = 100;
let scopeSelectionStart = null;
let scopeCursorSample = null;
let lastMeasurementRenderAt = 0;

const viewMeta = {
  console: ['实时控制', '电机控制台'],
  calibration: ['CALIBRATION', '校准向导'],
  scope: ['OSCILLOSCOPE', '实时示波器'],
  config: ['DEVICE CONFIG', '参数配置'],
  faults: ['FAULT HISTORY', '故障记录'],
};

function showToast(message) {
  dom.toast.textContent = message;
  dom.toast.classList.add('visible');
  window.clearTimeout(showToast.timer);
  showToast.timer = window.setTimeout(() => dom.toast.classList.remove('visible'), 2600);
}

function appendTerminal(line, direction = '') {
  if ((line.startsWith('@ ') || line.startsWith('! ')) && !dom.terminalTelemetryToggle.checked) return;
  const prefix = direction === 'out' ? '> ' : direction === 'in' ? '< ' : '';
  const stamp = new Date().toLocaleTimeString();
  dom.terminalLog.textContent += `[${stamp}] ${prefix}${line}\n`;
  const lines = dom.terminalLog.textContent.split('\n');
  if (lines.length > 300) dom.terminalLog.textContent = `${lines.slice(-300).join('\n')}`;
  if (dom.terminalAutoScroll.checked) dom.terminalLog.scrollTop = dom.terminalLog.scrollHeight;
}

function setTransport(kind, label) {
  state.transport = kind;
  dom.connectionBadge.textContent = label;
  dom.connectionBadge.className = `status-badge ${kind === 'connected' ? 'connected' : kind === 'simulated' ? 'simulated' : 'neutral'}`;
  dom.connectButton.textContent = kind === 'connected' ? '断开 USB' : '连接 USB';
  dom.simulateButton.textContent = kind === 'simulated' ? '停止模拟' : '模拟设备';
  dom.simulateButton.disabled = kind === 'connected';
  dom.portSelect.disabled = kind === 'connected' || !desktop;
  dom.refreshPortsButton.disabled = kind === 'connected' || !desktop;
}

async function sendCommand(text, { log = true } = {}) {
  if (log) appendTerminal(text, 'out');
  if (state.transport === 'connected') {
    const writer = serialWriter;
    if (!writer) return;
    const write = serialWriteQueue.then(() => {
      if (serialWriter !== writer) throw new Error('串口已断开');
      return writer.write(encodeCommand(text));
    });
    serialWriteQueue = write.catch(() => {});
    await write;
  } else if (state.transport === 'simulated') {
    simulateCommand(text);
  }
}

function isSensorless() {
  return state.mode >= MODE.SENSORLESS;
}

function setView(view) {
  state.view = view;
  const [eyebrow, title] = viewMeta[view];
  dom.pageEyebrow.textContent = eyebrow;
  dom.pageTitle.textContent = title;
  navButtons.forEach((button) => button.classList.toggle('active', button.dataset.view === view));
  views.forEach((section) => section.classList.toggle('active', section.id === `view-${view}`));
  dom.mobileViewSelect.value = view;
  if (view === 'scope') drawCharts();
  if (view === 'config' && state.transport === 'connected') readConfig();
}

async function setMode(mode) {
  if (state.stateCode !== 1) {
    showToast('请先停止电机，再切换反馈模式');
    return;
  }
  if (state.modePending !== null) return;
  state.modePending = mode;
  updateModeUI();
  try {
    await sendCommand(command.mode(mode));
  } catch (error) {
    state.modePending = null;
    updateModeUI();
    showToast(`模式命令发送失败：${error.message}`);
  }
}

function updateModeUI() {
  modeButtons.forEach((button) => {
    button.classList.toggle('active', Number(button.dataset.mode) === state.mode);
    button.classList.toggle('pending', Number(button.dataset.mode) === state.modePending);
    button.disabled = state.stateCode !== 1 || state.modePending !== null;
  });
  const sensorless = isSensorless();
  dom.modeHint.textContent = state.modePending === null
    ? (sensorless ? '无感启动需 5～10 turn/s，位置控制不可用' : '位置控制可用')
    : '正在等待设备确认…';
  dom.modeState.textContent = state.modePending === null ? '模式切换仅在 Idle 执行' : `正在切换到模式 ${state.modePending}`;
  dom.positionButton.disabled = sensorless || ![1, 8].includes(state.stateCode);
  dom.positionInput.disabled = sensorless;
  dom.commandModeLabel.textContent = sensorless ? '速度 5～10 / 锁定后扭矩' : '速度 / 位置 / 扭矩';
  if (state.mode === MODE.SPI || state.mode === MODE.SENSORLESS_SPI_MONITOR) {
    dom.encoderStatus.textContent = 'AS5047P · 16384 CPR';
  } else if (state.mode === MODE.ABZ || state.mode === MODE.SENSORLESS_ABZ_MONITOR) {
    dom.encoderStatus.textContent = 'ABZ · 4000 CPR';
  } else {
    dom.encoderStatus.textContent = '仅无感估算';
  }
  dom.observerStatus.textContent = sensorless ? (state.observerLocked ? '已锁定' : '等待锁定') : '监视未启用';
  dom.guideModeText.textContent = modeLabel(state.mode);
}

function modeLabel(mode) {
  return [
    'SPI 绝对 · AS5047P · 16384 CPR',
    'ABZ 增量 · 4000 CPR',
    '无感 FOC · 不使用编码器闭环',
    '无感 + SPI · 角度监视',
    '无感 + ABZ · 角度监视',
  ][mode] ?? '未知模式';
}

function modeShortLabel(mode) {
  return ['SPI 绝对', 'ABZ 增量', '无感 FOC', '无感 + SPI', '无感 + ABZ'][mode] ?? '未知反馈';
}

function controlModeLabel(mode) {
  return {
    [CONTROL_MODE.TORQUE]: '扭矩控制',
    [CONTROL_MODE.VELOCITY]: '速度控制',
    [CONTROL_MODE.POSITION]: '位置控制',
  }[mode] ?? '控制模式未知';
}

function seedSensorlessTarget() {
  if (isSensorless() && Math.abs(Number(dom.velocityInput.value) || 0) < 5) {
    dom.velocityInput.value = '6';
    dom.torqueInput.value = '0';
  }
}

function faultSignature(status) {
  return [status.fault, status.motorError, status.encoderError, status.controllerError,
    status.sensorlessError, status.fetThermistorError, status.motorThermistorError].join(':');
}

function currentFaultText(status = state) {
  const details = decodeFaults(status);
  if (!details.length) return '无故障';
  return details.map(({ title, advice }) => `${title}。${advice}`).join('\n');
}

function recordFault(status) {
  if (!status.fault || faultSignature(status) === faultSignature(state)) return;
  const details = decodeFaults(status);
  state.faultHistory.unshift({
    time: new Date().toLocaleTimeString(),
    axisState: status.axisState,
    fault: status.fault,
    velocity: status.velocity,
    current: status.current,
    summary: details.map(({ title }) => title).join('；') || '未知故障',
    advice: [...new Set(details.map(({ advice }) => advice))].join('；') || '读取各子模块故障码后再处理。',
  });
  state.faultHistory = state.faultHistory.slice(0, 100);
  renderFaults();
}

function setAxisStatus(status) {
  recordFault(status);
  const previousMode = state.mode;
  Object.assign(state, Object.fromEntries(Object.entries(status).filter(([, value]) => value !== undefined)));
  statusVersion += 1;
  if (status.mode !== undefined && state.modePending === null) state.mode = status.mode;
  if (state.mode !== previousMode) seedSensorlessTarget();
  renderStatus();
  updateModeUI();
  updateCalibrationState();
}

function setFastAxisStatus(status) {
  Object.assign(state, status);
  statusVersion += 1;
  renderStatus();
}

function renderStatus() {
  const faultLabel = state.fault ? `${faultSummary(state)} · 0x${state.fault.toString(16).padStart(8, '0')}` : '无故障';
  dom.stateValue.textContent = state.axisState;
  dom.faultValue.textContent = faultLabel;
  dom.faultValue.style.color = state.fault ? 'var(--red)' : 'var(--green)';
  dom.velocityValue.textContent = state.velocity.toFixed(1);
  dom.rpmValue.textContent = `${Math.round(state.velocity * 60)} RPM`;
  dom.busVoltageValue.textContent = state.busVoltage.toFixed(1);
  dom.currentValue.textContent = state.current.toFixed(2);
  dom.temperatureValue.textContent = state.temperature.toFixed(1);
  dom.observerStatus.textContent = isSensorless()
    ? (state.observerLocked ? '已锁定' : '等待锁定') : '监视未启用';
  dom.angleError.textContent = `${state.angleError.toFixed(4)} turn`;
  dom.scopeVelocity.textContent = state.velocity.toFixed(2);
  dom.scopeCurrent.textContent = state.current.toFixed(2);
  scopeValueOutputs.forEach((output) => {
    const value = Number(state[output.dataset.scopeValue]) || 0;
    output.value = Math.abs(value) < 0.01 ? value.toFixed(4) : value.toFixed(2);
  });
  dom.currentModeBadge.textContent = `${modeShortLabel(state.mode)} · ${controlModeLabel(state.controlMode)}${state.stateCode === 1 ? ' · Idle' : ''}`;
  dom.pwmStatus.textContent = state.pwmArmed ? '已使能' : '已关闭';
  dom.pwmStatus.style.color = state.pwmArmed ? 'var(--red)' : 'var(--green)';
  dom.watchdogStatus.textContent = state.stateCode === 1 ? 'Idle 不计时' : '1.0 s 运行保护';
  dom.currentStatus.textContent = state.stateCode === 1 || !state.pwmArmed ? '功率级关闭，显示 0 A' : '实时 q 轴电流';
  const ready = state.motorCalibrated && state.direction !== 0 && (isSensorless() || state.encoderReady);
  dom.controlReadyStatus.textContent = ready ? '允许进入控制' : '需要校准/方向确认';
  dom.controlReadyStatus.style.color = ready ? 'var(--green)' : 'var(--orange)';
}

function ingestLine(line) {
  const fastTelemetry = parseFastTelemetry(line);
  if (fastTelemetry) {
    appendTerminal(line, 'in');
    lastFrameAt = performance.now();
    setFastAxisStatus(fastTelemetry);
    captureTelemetrySample();
    dom.lastMessage.textContent = `最后响应 ${new Date().toLocaleTimeString()}`;
    return;
  }
  const telemetry = parseTelemetry(line);
  if (telemetry) {
    appendTerminal(line, 'in');
    lastFrameAt = performance.now();
    setAxisStatus(telemetry);
    captureTelemetrySample();
    dom.lastMessage.textContent = `最后响应 ${new Date().toLocaleTimeString()}`;
    return;
  }
  appendTerminal(line, 'in');
  if (line.startsWith('ok mode')) {
    const mode = Number(line.split(/\s+/)[2]);
    if (Number.isInteger(mode)) state.mode = mode;
    state.modePending = null;
    seedSensorlessTarget();
    updateModeUI();
    showToast(`反馈模式已切换：${modeLabel(state.mode)}`);
    return;
  }
  if (line.startsWith('err ')) {
    state.modePending = null;
    updateModeUI();
    showToast(`设备拒绝命令：${line.slice(4)}`);
    return;
  }
  if (line === 'ok clear') showToast('故障已清除');
  if (line === 'ok stopped') showToast('电机已停止');
  if (line === 'ok calibrating') {
    dom.calibrationStatus.textContent = '校准进行中';
    dom.stepCalibration.textContent = '进行中';
    showToast('校准已开始');
  }
  if (line === 'ok cogging-calibrating') {
    state.anticoggingCalibrationActive = true;
    state.anticoggingIndex = 0;
    dom.calibrationStatus.textContent = 'ABZ 齿槽补偿标定中';
    dom.stepCalibration.textContent = '齿槽标定中 0%';
    showToast('齿槽标定已开始：+2 / -2 turn/s，正反各采集 6 圈，预计 15～25 秒');
  }
  if (configReadQueue.length && !line.startsWith('ok ')) {
    const input = configReadQueue.shift();
    if (input) {
      input.value = line;
      input.dataset.deviceValue = line;
    }
  }
}

function makeMockStatus() {
  let coggingTarget = null;
  if (mockCoggingStartedAt !== null) {
    const elapsed = (performance.now() - mockCoggingStartedAt) / 1000;
    if (elapsed < 2.67) {
      coggingTarget = 2 * elapsed / 2.67;
      state.anticoggingIndex = 0;
    } else if (elapsed < 5.67) {
      coggingTarget = 2;
      state.anticoggingIndex = Math.round((elapsed - 2.67) / 3 * 1800);
    } else if (elapsed < 11) {
      coggingTarget = 2 - 4 * (elapsed - 5.67) / 5.33;
      state.anticoggingIndex = 1800;
    } else if (elapsed < 14) {
      coggingTarget = -2;
      state.anticoggingIndex = 1800 + Math.round((elapsed - 11) / 3 * 1800);
    } else if (elapsed < 16.67) {
      coggingTarget = -2 + 2 * (elapsed - 14) / 2.67;
      state.anticoggingIndex = 3600;
    } else {
      mockCoggingStartedAt = null;
      state.anticoggingCalibrationActive = false;
      state.anticoggingValid = true;
      state.anticoggingIndex = 0;
      state.stateCode = 1;
      state.axisState = AXIS_STATE[1];
    }
  }
  const running = [5, 8].includes(state.stateCode);
  const target = coggingTarget ?? (state.controlMode === CONTROL_MODE.POSITION
    ? Math.max(-0.3, Math.min(0.3, (mockPositionTarget - state.position) * 5))
    : Number(dom.velocityInput.value) || 0);
  const delta = target - state.velocity;
  const velocity = running ? state.velocity + Math.max(-1.2, Math.min(1.2, delta * 0.12)) : state.velocity * 0.94;
  const torqueCurrent = (Number(dom.torqueInput.value) || 0) / (8.27 / 650);
  const iqSetpoint = running
    ? (state.controlMode === CONTROL_MODE.TORQUE ? torqueCurrent : Math.sign(delta || 1) * Math.min(2, Math.abs(delta) * 0.35 + 0.08)) : 0;
  const current = running ? iqSetpoint + Math.sin(performance.now() / 47) * 0.025 : 0;
  const nextPosition = state.position + velocity * 0.02;
  const electricalPhase = nextPosition * Math.PI * 2 * 10;
  const voltageAmplitude = running ? Math.min(6, 0.4 + Math.abs(current) * 1.8 + Math.abs(velocity) * 0.08) : 0;
  return {
    axisState: AXIS_STATE[state.stateCode], stateCode: state.stateCode, fault: state.fault,
    velocity, current, position: nextPosition,
    busVoltage: 12 + current * 0.005, temperature: 25 + current * 0.18,
    phaseAVoltage: voltageAmplitude * Math.sin(electricalPhase),
    phaseBVoltage: voltageAmplitude * Math.sin(electricalPhase - Math.PI * 2 / 3),
    phaseCVoltage: voltageAmplitude * Math.sin(electricalPhase + Math.PI * 2 / 3),
    idMeasured: running ? Math.sin(performance.now() / 89) * 0.02 : 0,
    iqSetpoint, idSetpoint: 0,
    velocitySetpoint: target, rawVelocity: velocity,
    windowVelocity: velocity, velocityIntegratorTorque: 0,
    lowSpeedTorque: 0, positionSetpoint: mockPositionTarget,
    positionError: mockPositionTarget - nextPosition, lowSpeedState: 1,
    observerLocked: isSensorless() && Math.abs(velocity) > 5,
    angleError: isSensorless() ? Math.sin(performance.now() / 800) * 0.003 : 0,
    mode: state.mode, motorError: state.motorError, encoderError: state.encoderError,
    controllerError: state.controllerError, sensorlessError: state.sensorlessError,
    armedState: running ? 3 : 0, pwmArmed: running,
    encoderReady: state.encoderReady, motorCalibrated: state.motorCalibrated,
    direction: state.direction, controlMode: state.controlMode,
  };
}

const simulatedConfig = new Map([
  ['axis0.motor.config.current_lim', '2'], ['axis0.motor.config.torque_lim', '0.2'],
  ['axis0.controller.config.vel_limit', '20'], ['axis0.motor.config.direction', '0'],
  ['axis0.motor.config.phase_resistance', '0.1'], ['axis0.motor.config.phase_inductance', '0.0000423'],
  ['axis0.motor.config.pole_pairs', '10'], ['axis0.encoder.config.cpr', '16384'],
  ['axis0.motor.config.pre_calibrated', '0'], ['axis0.encoder.config.pre_calibrated', '0'],
  ['axis0.controller.config.pos_gain', '0.8'], ['axis0.controller.config.vel_gain', '0.0015'],
  ['axis0.controller.config.vel_integrator_gain', '0.005'],
  ['axis0.controller.config.vel_ramp_rate', '0.3'],
  ['axis0.motor.config.current_control_bandwidth', '500'],
  ['axis0.encoder.config.bandwidth', '100'],
  ['axis0.motor.config.current_lim_margin', '1'],
  ['config.brake_resistance', '2'], ['config.max_regen_current', '0'],
  ['config.dc_max_negative_current', '-0.000001'],
  ['config.enable_dc_bus_overvoltage_ramp', '0'],
  ['config.dc_bus_overvoltage_ramp_start', '14'],
  ['config.dc_bus_overvoltage_ramp_end', '15.5'],
]);

function simulateCommand(text) {
  const fields = text.split(/\s+/);
  const verb = fields[0];
  if (verb === 'm') window.setTimeout(() => ingestLine(`ok mode ${fields[2]}`), 80);
  if (verb === 'v') { state.controlMode = CONTROL_MODE.VELOCITY; dom.velocityInput.value = Number(fields[2]).toFixed(1); }
  if (verb === 'p' || verb === 't') { state.controlMode = CONTROL_MODE.POSITION; mockPositionTarget = Number(fields[2]); }
  if (verb === 'q') { state.controlMode = CONTROL_MODE.POSITION; mockPositionTarget = Number(fields[2]); }
  if (verb === 'c') { state.controlMode = CONTROL_MODE.TORQUE; dom.torqueInput.value = Number(fields[2]).toFixed(3); }
  if (verb === 'a') {
    state.stateCode = isSensorless() ? 4 : 3;
    updateCalibrationState();
    window.setTimeout(() => {
      state.stateCode = 1;
      state.motorCalibrated = true;
      state.encoderReady = !isSensorless();
      state.direction = 1;
      setAxisStatus(makeMockStatus());
    }, 900);
  }
  if (verb === 'b') {
    mockCoggingStartedAt = performance.now();
    state.stateCode = 8;
    state.axisState = AXIS_STATE[8];
    state.controlMode = CONTROL_MODE.VELOCITY;
    window.setTimeout(() => ingestLine('ok cogging-calibrating'), 50);
  }
  if (verb === 'x') {
    mockCoggingStartedAt = null;
    state.anticoggingCalibrationActive = false;
    state.anticoggingIndex = 0;
    state.stateCode = 1; state.axisState = AXIS_STATE[1]; state.current = 0; state.armedState = 0; state.pwmArmed = false;
  }
  if (verb === 'k') {
    state.fault = 0; state.motorError = 0; state.encoderError = 0;
    state.controllerError = 0; state.sensorlessError = 0;
    window.setTimeout(() => ingestLine('ok clear'), 50);
  }
  if (verb === 'w' && fields[1] === 'axis0.requested_state') { state.stateCode = Number(fields[2]); state.axisState = AXIS_STATE[state.stateCode]; }
  if (verb === 'r') window.setTimeout(() => ingestLine(simulatedConfig.get(fields[1]) ?? '0'), 60);
  if (verb === 'w' && fields[1]) simulatedConfig.set(fields[1], fields[2]);
  if (text === 'ss') window.setTimeout(() => ingestLine('ok saved'), 50);
  setAxisStatus(makeMockStatus());
}

function startSimulation() {
  if (mockTimer) {
    window.clearInterval(mockTimer);
    mockTimer = null;
    setTransport('disconnected', '未连接');
    showToast('已停止模拟设备');
    return;
  }
  setTransport('simulated', '模拟设备');
  appendTerminal('模拟设备已连接', 'in');
  showToast('模拟设备已连接');
  mockTimer = window.setInterval(() => {
    setAxisStatus(makeMockStatus());
    captureTelemetrySample();
  }, 50);
}

async function refreshPorts() {
  if (!('serial' in navigator)) {
    showToast('当前环境不支持 Web Serial');
    return;
  }
  try {
    authorizedPorts = await navigator.serial.getPorts();
    dom.portSelect.innerHTML = '<option value="">选择 COM 口</option>';
    authorizedPorts.forEach((port, index) => {
      const info = port.getInfo();
      const option = document.createElement('option');
      option.value = String(index);
      option.textContent = `USB 串口 ${index + 1}${info.usbVendorId ? ` · VID ${info.usbVendorId.toString(16).padStart(4, '0')}` : ''}`;
      dom.portSelect.append(option);
    });
    if (!authorizedPorts.length) showToast('没有已授权的串口，请点击连接 USB 选择设备');
  } catch (error) {
    showToast(`读取串口列表失败：${error.message}`);
  }
}

async function connectSerial() {
  if (state.transport === 'connected') {
    await disconnectSerial();
    return;
  }
  try {
    if (!('serial' in navigator)) throw new Error('当前环境不支持 Web Serial');
    const selectedIndex = dom.portSelect.value;
    serialPort = selectedIndex === '' ? await navigator.serial.requestPort() : authorizedPorts[Number(selectedIndex)];
    if (!serialPort) throw new Error('没有可用的 USB 串口');
    await serialPort.open({ baudRate: 115200 });
    serialWriter = serialPort.writable.getWriter();
    serialWriteQueue = Promise.resolve();
    serialReader = serialPort.readable.getReader();
    void readSerialLoop();
    setTransport('connected', desktop ? '桌面 USB 已连接' : 'USB 已连接');
    showToast('USB CDC 已连接');
    await sendCommand(command.telemetry());
    nextSlowPollAt = performance.now() + 200;
  } catch (error) {
    showToast(`USB 连接失败：${error.message}`);
    await disconnectSerial(false);
  }
}

async function disconnectSerial(showMessage = true) {
  const reader = serialReader;
  serialReader = null;
  try { await reader?.cancel(); } catch { /* Device may be removed. */ }
  try { reader?.releaseLock(); } catch { /* Already released. */ }
  try { serialWriter?.releaseLock(); } catch { /* Already released. */ }
  serialWriter = null;
  serialWriteQueue = Promise.resolve();
  try { await serialPort?.close(); } catch { /* Device may be removed. */ }
  serialPort = null;
  setTransport('disconnected', '未连接');
  dom.telemetryRate.textContent = '0 Hz';
  if (showMessage) showToast('USB 已断开，看门狗将停止电机');
}

async function readSerialLoop() {
  const reader = serialReader;
  try {
    while (serialReader === reader) {
      const { value, done } = await reader.read();
      if (done) break;
      for (const line of parser.push(value)) ingestLine(line);
    }
  } catch (error) {
    if (serialReader === reader) showToast(`USB 读取停止：${error.message}`);
  } finally {
    try { reader?.releaseLock(); } catch { /* Already released. */ }
    if (serialReader === reader) serialReader = null;
  }
}

function waitForState(targetState, timeoutMs = 1800) {
  const initialVersion = statusVersion;
  return new Promise((resolve) => {
    const started = performance.now();
    const timer = window.setInterval(() => {
      if (statusVersion > initialVersion && (state.stateCode === targetState || state.fault)) {
        window.clearInterval(timer);
        resolve(state.stateCode === targetState && !state.fault);
      } else if (performance.now() - started >= timeoutMs) {
        window.clearInterval(timer);
        resolve(false);
      }
    }, 25);
  });
}

function controlPreflight(kind) {
  if (state.transport === 'disconnected') return '请先连接 USB 或启动模拟设备';
  if (kind === 'position' && isSensorless()) return '无感模式不支持位置控制';
  if (state.fault) return `${faultSummary(state)}。请先处理并清除故障`;
  if (!state.motorCalibrated) return '电机尚未校准，请先执行校准向导';
  if (state.direction === 0) return '电机方向尚未确定。编码器模式请执行完整校准；无感模式请在参数配置中设置方向为 1 或 -1';
  if (!isSensorless() && !state.encoderReady) return '编码器尚未就绪，请检查接线并完成编码器校准';
  if (isSensorless() && kind === 'position') return '无感模式只能使用速度或扭矩控制';
  if (isSensorless() && kind === 'torque' && (!state.observerLocked || Math.abs(state.velocity) < 3)) {
    return '无感扭矩不能从静止启动。请先用速度模式运行到 6 turn/s，显示“无感已锁定”后再切换扭矩';
  }
  return '';
}

function requestedControlMode(kind) {
  return kind === 'velocity' ? CONTROL_MODE.VELOCITY
    : kind === 'position' ? CONTROL_MODE.POSITION : CONTROL_MODE.TORQUE;
}

function zeroControlCommand(kind) {
  if (kind === 'velocity') return command.velocity(0);
  if (kind === 'position') return command.position(state.position);
  return command.torque(0);
}

function validateControlTarget(kind, rawValue) {
  const value = Number(rawValue);
  if (!Number.isFinite(value)) return '目标值必须是有效数字';
  if (kind === 'velocity' && isSensorless() && (Math.abs(value) < 5 || Math.abs(value) > 10)) {
    return '纯无感调试速度必须在 ±5～±10 turn/s；0.1 turn/s 的反电动势不足以估算转子角度';
  }
  if (kind === 'position' && Math.abs(value) > 0.25) return '单次相对位移限制为 ±0.25 turn';
  if (kind === 'torque' && Math.abs(value) > 0.008) return '空载台架目标扭矩限制为 ±0.008 Nm';
  return '';
}

async function sendControl(text, kind, rawValue) {
  const blocked = controlPreflight(kind);
  if (blocked) { showToast(blocked); return; }
  const invalidTarget = validateControlTarget(kind, rawValue);
  if (invalidTarget) { showToast(invalidTarget); return; }
  const targetState = isSensorless() ? 5 : 8;
  const nextControlMode = requestedControlMode(kind);
  if (state.stateCode !== targetState || state.controlMode !== nextControlMode) {
    // Select the new controller with a zero target first. This clears stale
    // velocity/torque feed-forward before PWM is enabled or the mode changes.
    await sendCommand(zeroControlCommand(kind));
  }
  if (state.stateCode !== targetState) {
    if (state.stateCode !== 1) { showToast('轴正在切换状态，请先急停回到 Idle'); return; }
    await sendCommand(command.state(targetState));
    const active = await waitForState(targetState);
    if (!active) {
      showToast(state.fault ? currentFaultText(state) : `未能进入${AXIS_STATE[targetState]}，请检查校准和反馈状态`);
      return;
    }
  }
  const resolvedCommand = typeof text === 'function' ? text() : text;
  await sendCommand(resolvedCommand);
  showToast(kind === 'position'
    ? `已从当前位置相对移动 ${Number(rawValue).toFixed(3)} turn`
    : `${kind === 'velocity' ? '速度' : '扭矩'}目标已发送`);
}

async function readConfig() {
  if (state.transport === 'disconnected') { showToast('请先连接设备'); return; }
  configReadQueue = [];
  dom.configStatus.textContent = '正在读取设备参数…';
  for (const input of configInputs) {
    configReadQueue.push(input);
    await sendCommand(command.read(input.dataset.config));
    await new Promise((resolve) => window.setTimeout(resolve, 70));
  }
  dom.configStatus.textContent = '参数读取完成';
}

function configNumber(path) {
  const input = configInputs.find((candidate) => candidate.dataset.config === path);
  return Number(input?.value);
}

function validateConfigValues() {
  const tuningRanges = [
    ['axis0.encoder.config.bandwidth', 20, 100, '编码器带宽应在 20～100 rad/s'],
    ['axis0.controller.config.vel_gain', 0.0002, 0.01, '速度增益应在 0.0002～0.01'],
    ['axis0.controller.config.vel_integrator_gain', 0, 0.03, '速度积分增益应在 0～0.03'],
    ['axis0.controller.config.vel_ramp_rate', 0.05, 2, '速度斜坡应在 0.05～2 turn/s²'],
    ['axis0.motor.config.current_control_bandwidth', 100, 1000, '电流环带宽应在 100～1000 rad/s'],
  ];
  for (const [path, min, max, message] of tuningRanges) {
    const value = configNumber(path);
    if (!Number.isFinite(value) || value < min || value > max) return message;
  }
  const resistance = configNumber('config.brake_resistance');
  if (!Number.isFinite(resistance) || resistance < 0 || (resistance > 0 && resistance < 0.1) || resistance > 20) {
    return '制动电阻应为 0（禁用）或 0.1～20 Ω，并且必须与实际外接电阻一致';
  }
  const maxRegen = configNumber('config.max_regen_current');
  if (!Number.isFinite(maxRegen) || maxRegen < 0 || maxRegen > 10) return '允许回馈电流必须在 0～10 A';
  const negativeCurrent = configNumber('config.dc_max_negative_current');
  if (!Number.isFinite(negativeCurrent) || negativeCurrent > 0 || negativeCurrent < -10) {
    return '电源吸收电流限制必须在 -10～0 A；普通不可回馈电源建议保持接近 0';
  }
  const rampEnabled = configNumber('config.enable_dc_bus_overvoltage_ramp') === 1;
  const rampStart = configNumber('config.dc_bus_overvoltage_ramp_start');
  const rampEnd = configNumber('config.dc_bus_overvoltage_ramp_end');
  if (rampEnabled && resistance <= 0) return '启用过压制动斜坡前必须配置并接好制动电阻';
  if (rampEnabled && (!Number.isFinite(rampStart) || !Number.isFinite(rampEnd)
      || rampStart < 13 || rampEnd <= rampStart || rampEnd > 15.8)) {
    return '12 V 档过压斜坡建议起点 13～15 V，终点必须更高且不超过 15.8 V';
  }
  return '';
}

async function applyConfig() {
  if (state.transport === 'disconnected') { showToast('请先连接设备'); return; }
  if (state.stateCode !== 1) { showToast('请在 Idle 状态写入参数'); return; }
  const unread = writableConfigInputs.filter((input) => input.dataset.deviceValue === undefined);
  if (unread.length) {
    dom.configStatus.textContent = '写入被阻止：必须先读取设备参数，防止页面默认值覆盖校准方向和反馈配置。';
    showToast('请先点击“读取设备参数”，再修改和写入');
    return;
  }
  const configError = validateConfigValues();
  if (configError) { showToast(configError); return; }
  const changed = writableConfigInputs.filter((input) => input.value.trim() !== input.dataset.deviceValue.trim());
  if (!changed.length) { dom.configStatus.textContent = '没有需要写入的参数'; return; }
  for (const input of changed) {
    await sendCommand(command.write(input.dataset.config, input.value));
    input.dataset.deviceValue = input.value.trim();
    await new Promise((resolve) => window.setTimeout(resolve, 15));
  }
  dom.configStatus.textContent = `已写入 ${changed.length} 个可编辑参数；校准结果和预校准标志未改动。点击“保存到 Flash”才能持久化。`;
}

function loadSafeProfile() {
  writableConfigInputs.forEach((input) => {
    if (foc3505Tuning[input.dataset.config] !== undefined) input.value = foc3505Tuning[input.dataset.config];
  });
  dom.configStatus.textContent = '已填入已验证的基础参数：编码器带宽 100、速度增益 0.0015、积分 0.005、速度斜坡 0.3、位置增益 0.8、电流环 500 rad/s。请先读取设备，再写入；速度反馈低通只在速度模式临时启用。';
}

function updateCalibrationState() {
  const standardActive = [3, 4, 6, 7, 10].includes(state.stateCode);
  const coggingActive = state.anticoggingCalibrationActive;
  const active = standardActive || coggingActive;
  const coggingPercent = Math.min(100, Math.max(0, state.anticoggingIndex / 36));
  dom.stepSafety.textContent = state.transport === 'disconnected' ? '待连接' : '已连接';
  dom.stepSafety.className = `step-state ${state.transport === 'disconnected' ? '' : 'done'}`;
  dom.stepMode.textContent = modeLabel(state.mode);
  dom.stepCalibration.textContent = coggingActive
    ? `齿槽标定中 ${coggingPercent.toFixed(1)}%`
    : standardActive ? '进行中'
      : state.anticoggingValid ? '齿槽补偿已就绪'
        : state.stateCode === 1 && dom.stepCalibration.textContent === '进行中' ? '已完成' : dom.stepCalibration.textContent;
  dom.calibrationStatus.textContent = coggingActive
    ? `ABZ 齿槽标定 ${coggingPercent.toFixed(1)}%`
    : active ? '校准进行中' : (state.stateCode === 1 ? '设备空闲' : state.axisState);
  const readiness = `电机校准：${state.motorCalibrated ? '完成' : '未完成'} · 编码器：${state.encoderReady ? '就绪' : '未就绪'} · 方向：${state.direction || '未确定'}`;
  const coverage = Math.min(3600, Math.max(0, state.anticoggingCoverage || 0));
  const cogging = `齿槽补偿：${coggingActive ? `${coggingPercent.toFixed(1)}%` : state.anticoggingValid ? `本次上电已就绪 · 有效覆盖 ${coverage}/3600` : `未标定 · 有效覆盖 ${coverage}/3600`}`;
  dom.calibrationOutput.textContent = `${new Date().toLocaleTimeString()}  状态：${state.axisState}\n反馈：${modeLabel(state.mode)}\n${readiness}\n${cogging}\n${state.fault ? currentFaultText(state) : '无故障'}`;
  dom.coggingCalibrationButton.disabled = state.transport === 'disconnected' ||
    state.mode !== MODE.ABZ || state.stateCode !== 1 || !state.motorCalibrated ||
    !state.encoderReady || coggingActive;
}

function renderFaults() {
  if (!state.faultHistory.length) {
    dom.faultTableBody.innerHTML = '<tr><td colspan="7" class="empty-row">当前会话暂无故障</td></tr>';
    return;
  }
  dom.faultTableBody.innerHTML = state.faultHistory.map((item) => `<tr><td>${item.time}</td><td>${item.axisState}</td><td class="fault-code">0x${item.fault.toString(16).padStart(8, '0')}</td><td class="fault-detail">${item.summary}</td><td class="fault-detail">${item.advice}</td><td>${item.velocity.toFixed(2)}</td><td>${item.current.toFixed(2)} A</td></tr>`).join('');
}

const chartSeries = Object.freeze({
  velocity: { label: '速度', color: '#1479ff', floor: 0.1, unit: 'turn/s' },
  velocitySetpoint: { label: '速度给定', color: '#2f9e75', floor: 0.1, unit: 'turn/s', dashed: true },
  rawVelocity: { label: 'PLL 原始速度', color: '#d9485f', floor: 0.1, unit: 'turn/s' },
  windowVelocity: { label: 'ABZ 窗口速度', color: '#008b8b', floor: 0.1, unit: 'turn/s' },
  position: { label: '位置', color: '#7c5ce7', floor: 0.1, unit: 'turn' },
  positionSetpoint: { label: '轨迹位置', color: '#d07a00', floor: 0.1, unit: 'turn', dashed: true },
  positionTarget: { label: '目标位置', color: '#2f9e75', floor: 0.1, unit: 'turn', dashed: true },
  positionError: { label: '位置误差', color: '#c2418c', floor: 0.001, unit: 'turn' },
  velocityIntegratorTorque: { label: '速度积分转矩', color: '#7a5af8', floor: 0.001, unit: 'Nm' },
  lowSpeedTorque: { label: '低速补偿转矩', color: '#b45309', floor: 0.001, unit: 'Nm' },
  current: { label: 'Iq 测量', color: '#d07a00', floor: 0.1, unit: 'A' },
  iqSetpoint: { label: 'Iq 给定', color: '#efb118', floor: 0.1, unit: 'A' },
  idMeasured: { label: 'Id 测量', color: '#06a67a', floor: 0.05, unit: 'A' },
  idSetpoint: { label: 'Id 给定', color: '#6bbf59', floor: 0.05, unit: 'A' },
  phaseAVoltage: { label: 'Va 指令', color: '#e5474d', floor: 0.2, unit: 'V' },
  phaseBVoltage: { label: 'Vb 指令', color: '#a855c7', floor: 0.2, unit: 'V' },
  phaseCVoltage: { label: 'Vc 指令', color: '#3f66d4', floor: 0.2, unit: 'V' },
  busVoltage: { label: '母线电压', color: '#079ca1', floor: 0.5, unit: 'V' },
  temperature: { label: 'MOS 温度', color: '#db4b77', floor: 2, unit: '°C' },
  angleError: { label: '角度误差', color: '#64748b', floor: 0.001, unit: 'turn' },
});

function captureTelemetrySample() {
  state.history.push({
    time: performance.now(),
    velocity: state.velocity,
    velocitySetpoint: state.velocitySetpoint,
    rawVelocity: state.rawVelocity,
    windowVelocity: state.windowVelocity,
    position: state.position,
    positionSetpoint: state.positionSetpoint,
    positionError: state.positionError,
    positionTarget: state.positionTarget,
    velocityIntegratorTorque: state.velocityIntegratorTorque,
    lowSpeedTorque: state.lowSpeedTorque,
    current: state.current,
    iqSetpoint: state.iqSetpoint,
    idMeasured: state.idMeasured,
    idSetpoint: state.idSetpoint,
    phaseAVoltage: state.phaseAVoltage,
    phaseBVoltage: state.phaseBVoltage,
    phaseCVoltage: state.phaseCVoltage,
    busVoltage: state.busVoltage,
    temperature: state.temperature,
    angleError: state.angleError,
  });
  if (state.history.length > 3600) state.history.shift();
  drawCharts();
}

function historyWindow(windowSeconds, cutoff = performance.now()) {
  const start = cutoff - windowSeconds * 1000;
  return state.history.filter((sample) => sample.time >= start && sample.time <= cutoff);
}

function selectedScopeKeys() {
  return scopeChannels.filter((input) => input.checked).map((input) => input.dataset.scopeKey);
}

function scopeKeysWithSetpoints(keys) {
  const related = [...keys];
  if (keys.includes('velocity')) related.push('velocitySetpoint');
  if (keys.includes('position')) related.push('positionSetpoint', 'positionTarget');
  return [...new Set(related)];
}

function sharedAutoRange(keys, samples) {
  const rangeKeys = scopeKeysWithSetpoints(keys);
  if (!keys.length || !samples.length) return { ...scopeSharedRange };
  let min = Infinity;
  let max = -Infinity;
  rangeKeys.forEach((key) => samples.forEach((sample) => {
    const value = Number(sample[key]);
    if (!Number.isFinite(value)) return;
    min = Math.min(min, value);
    max = Math.max(max, value);
  }));
  if (!Number.isFinite(min) || !Number.isFinite(max)) return { ...scopeSharedRange };
  const floor = Math.max(...rangeKeys.map((key) => chartSeries[key]?.floor ?? 0.1));
  const center = (min + max) / 2;
  const span = Math.max(max - min, floor * 2);
  return { min: center - span * 0.58, max: center + span * 0.58 };
}

function formatAxisTick(value) {
  const magnitude = Math.abs(value);
  if (magnitude >= 100) return value.toFixed(2);
  if (magnitude >= 10) return value.toFixed(1);
  if (magnitude >= 1) return value.toFixed(2);
  return value.toFixed(4);
}

function formatAxisInput(value) {
  if (!Number.isFinite(value)) return '0';
  return String(Number(value.toPrecision(7)));
}

function formatMeasurement(value) {
  if (!Number.isFinite(value)) return '--';
  const magnitude = Math.abs(value);
  if (magnitude >= 100) return value.toFixed(1);
  if (magnitude >= 10) return value.toFixed(2);
  if (magnitude >= 1) return value.toFixed(3);
  return value.toFixed(4);
}

function drawOneChart(canvas, keys = ['velocity', 'current'], options = {}) {
  if (!canvas) return;
  const windowSeconds = options.windowSeconds ?? 20;
  const cutoff = options.cutoff ?? performance.now();
  const samples = options.samples ?? historyWindow(windowSeconds, cutoff);
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(320, Math.floor(rect.width * dpr));
  const height = Math.max(220, Math.floor(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, width, height);
  const left = 62 * dpr;
  const right = 15 * dpr;
  const top = 16 * dpr;
  const bottom = 13 * dpr;
  const innerW = width - left - right;
  const innerH = height - top - bottom;
  ctx.strokeStyle = '#e4eaf0';
  ctx.lineWidth = dpr;
  for (let index = 0; index <= 8; index += 1) {
    const x = left + innerW * index / 8;
    ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, top + innerH); ctx.stroke();
  }
  for (let index = 0; index <= 8; index += 1) {
    const y = top + innerH * index / 8;
    ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(left + innerW, y); ctx.stroke();
  }
  const axisRange = options.range ?? { min: -2, max: 2 };
  ctx.fillStyle = '#596978';
  ctx.font = `${10 * dpr}px Consolas, monospace`;
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  for (let index = 0; index <= 4; index += 1) {
    const y = top + innerH * index / 4;
    const value = axisRange.max - (axisRange.max - axisRange.min) * index / 4;
    ctx.fillText(formatAxisTick(value), left - 7 * dpr, y);
  }
  if (axisRange.min <= 0 && axisRange.max >= 0) {
    const zeroY = top + innerH * axisRange.max / (axisRange.max - axisRange.min);
    ctx.strokeStyle = '#aebbc6';
    ctx.beginPath(); ctx.moveTo(left, zeroY); ctx.lineTo(left + innerW, zeroY); ctx.stroke();
  }
  ctx.save();
  ctx.beginPath(); ctx.rect(left, top, innerW, innerH); ctx.clip();
  const plot = (key) => {
    if (samples.length < 2) return;
    const series = chartSeries[key];
    ctx.strokeStyle = series.color; ctx.lineWidth = 1.6 * dpr; ctx.lineJoin = 'round'; ctx.beginPath();
    if (series.dashed) ctx.setLineDash([6 * dpr, 4 * dpr]);
    let started = false;
    samples.forEach((sample, index) => {
      const value = Number(sample[key]);
      if (!Number.isFinite(value)) { started = false; return; }
      const ageRatio = (cutoff - sample.time) / (windowSeconds * 1000);
      const x = left + innerW * (1 - ageRatio);
      const normalized = Math.max(0, Math.min(1,
        (value - axisRange.min) / (axisRange.max - axisRange.min)));
      const y = top + innerH * (1 - normalized);
      if (!started) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      started = true;
    });
    ctx.stroke();
    ctx.setLineDash([]);
  };
  scopeKeysWithSetpoints(keys).filter((key) => chartSeries[key]).forEach(plot);
  ctx.restore();
  if (options.cursorSample) {
    const ageRatio = (cutoff - options.cursorSample.time) / (windowSeconds * 1000);
    const x = left + innerW * (1 - ageRatio);
    if (x >= left && x <= left + innerW) {
      ctx.save();
      ctx.strokeStyle = '#1d2730';
      ctx.lineWidth = dpr;
      ctx.setLineDash([4 * dpr, 4 * dpr]);
      ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, top + innerH); ctx.stroke();
      ctx.restore();
    }
  }
}

function drawCharts() {
  const selected = selectedScopeKeys();
  const baseTime = scopePausedAt ?? performance.now();
  const cutoff = baseTime - scopeTimeOffsetSeconds * 1000;
  const samples = historyWindow(scopeWindowSeconds, cutoff);
  const range = scopeAutoScale ? sharedAutoRange(selected, samples) : scopeSharedRange;
  drawOneChart(dom.telemetryCanvas, selected, { windowSeconds: scopeWindowSeconds, cutoff, samples, range });
  drawOneChart(dom.scopeCanvas, selected, { windowSeconds: scopeWindowSeconds, cutoff, samples, range, cursorSample: scopeCursorSample });
  const legendKeys = scopeKeysWithSetpoints(selected);
  dom.consoleChartLegend.innerHTML = legendKeys.map((key) => `<span><i style="background:${chartSeries[key].color}"></i>${chartSeries[key].label}</span>`).join('');
  if (!selected.length) dom.consoleChartLegend.textContent = '未选择通道';
  if (scopeAutoScale) {
    dom.scopeAxisMin.value = formatAxisInput(range.min);
    dom.scopeAxisMax.value = formatAxisInput(range.max);
  }
  renderScopeMeasurements(selected, samples, range);
  const now = performance.now();
  const rate = state.history.filter((sample) => sample.time >= now - 1000).length;
  dom.telemetryRate.textContent = state.transport === 'disconnected' ? '0 Hz' : `${rate} Hz`;
  dom.scopeSampleRate.textContent = `${rate} Hz`;
  dom.scopeSamples.textContent = `${samples.length} samples`;
}

function renderScopeMeasurements(keys, samples, range) {
  const now = performance.now();
  if (now - lastMeasurementRenderAt < 100) return;
  lastMeasurementRenderAt = now;
  if (!keys.length) {
    dom.scopeMeasurements.innerHTML = '<span class="scope-measurement-empty">请在左侧勾选需要测量的通道</span>';
    return;
  }
  const cursor = scopeCursorSample && samples.includes(scopeCursorSample) ? scopeCursorSample : samples.at(-1);
  const cards = keys.map((key) => {
    const values = samples.map((sample) => Number(sample[key])).filter(Number.isFinite);
    const min = values.length ? Math.min(...values) : NaN;
    const max = values.length ? Math.max(...values) : NaN;
    const avg = values.length ? values.reduce((sum, value) => sum + value, 0) / values.length : NaN;
    const current = cursor ? Number(cursor[key]) : NaN;
    return `<article style="--measure-color:${chartSeries[key].color}"><header><i></i><strong>${chartSeries[key].label}</strong><b>${formatMeasurement(current)} <small>${chartSeries[key].unit}</small></b></header><dl><div><dt>最小</dt><dd>${formatMeasurement(min)}</dd></div><div><dt>最大</dt><dd>${formatMeasurement(max)}</dd></div><div><dt>平均</dt><dd>${formatMeasurement(avg)}</dd></div><div><dt>峰峰值</dt><dd>${formatMeasurement(max - min)}</dd></div></dl></article>`;
  }).join('');
  dom.scopeMeasurements.innerHTML = `<div class="scope-measurement-range"><b>可见窗口测量</b><span>共享 Y：${formatAxisTick(range.min)} ～ ${formatAxisTick(range.max)}</span></div>${cards}`;
}

function updateScopeControls() {
  dom.scopeRunButton.textContent = scopeRunning ? '暂停采集' : '继续采集';
  dom.scopeRunButton.classList.toggle('primary', scopeRunning);
  dom.scopeRunButton.setAttribute('aria-pressed', String(scopeRunning));
  dom.scopeCaptureState.textContent = scopeRunning ? '正在采集' : '波形已暂停';
  dom.scopeCaptureState.parentElement.classList.toggle('paused', !scopeRunning);
  dom.scopeAutoScaleButton.classList.toggle('active', scopeAutoScale);
  dom.scopeAutoScaleButton.setAttribute('aria-pressed', String(scopeAutoScale));
  dom.scopeWindowLabel.textContent = `-${formatMeasurement(scopeWindowSeconds + scopeTimeOffsetSeconds)} s`;
  dom.scopeWindowEndLabel.textContent = scopeTimeOffsetSeconds > 0.001 ? `-${formatMeasurement(scopeTimeOffsetSeconds)} s` : '0 s';
  dom.scopeXWindowSlider.value = String(scopeWindowSeconds);
  dom.scopeXWindowValue.value = `${scopeWindowSeconds.toFixed(scopeWindowSeconds % 1 ? 1 : 0)} s`;
  updateScopeAxisControls();
  drawCharts();
}

function updateScopeAxisControls() {
  if (!scopeAutoScale) {
    dom.scopeAxisMin.value = formatAxisInput(scopeSharedRange.min);
    dom.scopeAxisMax.value = formatAxisInput(scopeSharedRange.max);
  }
  dom.scopeYZoomSlider.value = String(scopeSharedZoom);
  dom.scopeYZoomValue.value = `${Math.round(scopeSharedZoom)}%`;
}

function applyScopeAxisRange() {
  const min = Number(dom.scopeAxisMin.value);
  const max = Number(dom.scopeAxisMax.value);
  if (!Number.isFinite(min) || !Number.isFinite(max) || max <= min) {
    showToast('Y 轴范围无效：最大值必须大于最小值');
    return;
  }
  scopeSharedRange = { min, max };
  scopeSharedZoom = 100;
  scopeAutoScale = false;
  updateScopeControls();
  showToast(`已对全部通道应用共享 Y 范围 ${min} ～ ${max}`);
}

function setScopeWindow(value) {
  scopeWindowSeconds = Math.max(0.2, Math.min(60, Number(value) || 20));
  scopeTimeOffsetSeconds = clampScopeOffset(scopeTimeOffsetSeconds);
  const exactOption = [...dom.scopeWindowSelect.options].find((option) => Number(option.value) === scopeWindowSeconds);
  let customOption = dom.scopeWindowSelect.querySelector('[data-custom-window]');
  if (!exactOption) {
    if (!customOption) {
      customOption = document.createElement('option');
      customOption.dataset.customWindow = 'true';
      dom.scopeWindowSelect.append(customOption);
    }
    customOption.value = String(scopeWindowSeconds);
    customOption.textContent = `${scopeWindowSeconds.toFixed(scopeWindowSeconds < 1 ? 1 : 0)} s`;
  }
  dom.scopeWindowSelect.value = String(scopeWindowSeconds);
  updateScopeControls();
}

function prepareManualScopeAxis() {
  if (scopeAutoScale) {
    const cutoff = (scopePausedAt ?? performance.now()) - scopeTimeOffsetSeconds * 1000;
    scopeSharedRange = sharedAutoRange(selectedScopeKeys(), historyWindow(scopeWindowSeconds, cutoff));
    scopeSharedZoom = 100;
    scopeAutoScale = false;
  }
}

function applyScopeAxisZoom(percent, anchorRatio = 0.5) {
  prepareManualScopeAxis();
  const oldPercent = scopeSharedZoom;
  const nextPercent = Math.max(25, Math.min(400, Math.round(percent / 5) * 5));
  const range = scopeSharedRange;
  const span = range.max - range.min;
  const nextSpan = span * oldPercent / nextPercent;
  const anchor = range.max - anchorRatio * span;
  scopeSharedRange = {
    min: anchor - (1 - anchorRatio) * nextSpan,
    max: anchor + anchorRatio * nextSpan,
  };
  scopeSharedZoom = nextPercent;
  updateScopeControls();
}

function clampScopeOffset(value) {
  const baseTime = scopePausedAt ?? performance.now();
  const oldest = state.history[0]?.time ?? baseTime;
  const maxOffset = Math.max(0, (baseTime - oldest) / 1000 - scopeWindowSeconds);
  return Math.max(0, Math.min(maxOffset, value));
}

function panScopeTime(deltaSeconds) {
  scopeTimeOffsetSeconds = clampScopeOffset(scopeTimeOffsetSeconds + deltaSeconds);
  scopeCursorSample = null;
  dom.scopeCursorTooltip.hidden = true;
  updateScopeControls();
}

function handleScopeWheel(event) {
  event.preventDefault();
  if (event.ctrlKey) {
    setScopeWindow(scopeWindowSeconds * (event.deltaY > 0 ? 1.15 : 1 / 1.15));
    return;
  }
  if (event.shiftKey || Math.abs(event.deltaX) > Math.abs(event.deltaY)) {
    const delta = Math.abs(event.deltaX) > Math.abs(event.deltaY) ? event.deltaX : event.deltaY;
    panScopeTime(delta / 500 * scopeWindowSeconds);
    return;
  }
  const rect = dom.scopeCanvas.getBoundingClientRect();
  const anchorRatio = Math.max(0, Math.min(1, (event.clientY - rect.top) / rect.height));
  applyScopeAxisZoom(scopeSharedZoom * (event.deltaY < 0 ? 1.12 : 1 / 1.12), anchorRatio);
}

function resetScopeView() {
  scopeAutoScale = true;
  scopeSharedRange = { min: -2, max: 2 };
  scopeSharedZoom = 100;
  scopeTimeOffsetSeconds = 0;
  scopeWindowSeconds = 20;
  scopeCursorSample = null;
  dom.scopeCursorTooltip.hidden = true;
  setScopeWindow(20);
}

function scopePlotPoint(event) {
  const rect = dom.scopeCanvas.getBoundingClientRect();
  return {
    rect,
    x: Math.max(62, Math.min(rect.width - 15, event.clientX - rect.left)),
    y: Math.max(16, Math.min(rect.height - 13, event.clientY - rect.top)),
  };
}

function updateScopeCursor(event) {
  const { rect, x } = scopePlotPoint(event);
  const cutoff = (scopePausedAt ?? performance.now()) - scopeTimeOffsetSeconds * 1000;
  const ratio = (x - 62) / Math.max(rect.width - 77, 1);
  const targetTime = cutoff - (1 - ratio) * scopeWindowSeconds * 1000;
  const samples = historyWindow(scopeWindowSeconds, cutoff);
  scopeCursorSample = samples.reduce((best, sample) => (!best || Math.abs(sample.time - targetTime) < Math.abs(best.time - targetTime) ? sample : best), null);
  if (!scopeCursorSample) { dom.scopeCursorTooltip.hidden = true; return; }
  const keys = selectedScopeKeys();
  dom.scopeCursorTooltip.innerHTML = `<b>${((scopeCursorSample.time - cutoff) / 1000).toFixed(3)} s</b>${keys.map((key) => `<span><i style="background:${chartSeries[key].color}"></i>${chartSeries[key].label}<strong>${formatMeasurement(Number(scopeCursorSample[key]))} ${chartSeries[key].unit}</strong></span>`).join('')}`;
  dom.scopeCursorTooltip.style.left = `${Math.min(x + 12, rect.width - 190)}px`;
  dom.scopeCursorTooltip.style.top = '25px';
  dom.scopeCursorTooltip.hidden = false;
  drawCharts();
}

function finishScopeSelection(event) {
  if (!scopeSelectionStart) return;
  const end = scopePlotPoint(event);
  const start = scopeSelectionStart;
  const left = Math.min(start.x, end.x);
  const right = Math.max(start.x, end.x);
  const top = Math.min(start.y, end.y);
  const bottom = Math.max(start.y, end.y);
  scopeSelectionStart = null;
  dom.scopeSelectionBox.hidden = true;
  dom.scopeCanvas.classList.remove('selecting');
  if (right - left < 10 || bottom - top < 10) return;
  prepareManualScopeAxis();
  const plotWidth = Math.max(end.rect.width - 77, 1);
  const plotHeight = Math.max(end.rect.height - 29, 1);
  const x1 = (left - 62) / plotWidth;
  const x2 = (right - 62) / plotWidth;
  const viewCutoff = (scopePausedAt ?? performance.now()) - scopeTimeOffsetSeconds * 1000;
  const viewStart = viewCutoff - scopeWindowSeconds * 1000;
  const selectedEnd = viewStart + x2 * scopeWindowSeconds * 1000;
  const baseTime = scopePausedAt ?? performance.now();
  const oldRange = { ...scopeSharedRange };
  const oldSpan = oldRange.max - oldRange.min;
  scopeWindowSeconds = Math.max(0.2, (x2 - x1) * scopeWindowSeconds);
  scopeTimeOffsetSeconds = Math.max(0, (baseTime - selectedEnd) / 1000);
  const y1 = (top - 16) / plotHeight;
  const y2 = (bottom - 16) / plotHeight;
  scopeSharedRange = {
    min: oldRange.max - y2 * oldSpan,
    max: oldRange.max - y1 * oldSpan,
  };
  scopeSharedZoom = 100;
  scopeAutoScale = false;
  setScopeWindow(scopeWindowSeconds);
}

function setConfigValue(path, value) {
  const input = configInputs.find((candidate) => candidate.dataset.config === path);
  if (input) input.value = String(value);
}

function updateBrakePowerPreview() {
  const resistance = configNumber('config.brake_resistance');
  if (!(resistance > 0)) {
    dom.brakePowerPreview.textContent = '未接制动电阻：保持 0 Ω 并关闭过压斜坡。';
    return;
  }
  const rampEnd = configNumber('config.dc_bus_overvoltage_ramp_end');
  const voltage = Number.isFinite(rampEnd) ? rampEnd : 15.5;
  const rampPower = voltage * voltage / resistance;
  const tripPower = 16 * 16 / resistance;
  dom.brakePowerPreview.textContent = `${resistance} Ω：${voltage.toFixed(1)} V 满占空约 ${rampPower.toFixed(1)} W，16 V 约 ${tripPower.toFixed(1)} W。${resistance <= 2.1 ? '50 W 电阻只能短时脉冲使用，必须留出冷却时间。' : '5 Ω / 50 W 与本项目的 12 V 母线更匹配，仍需通风散热。'}`;
}

function loadBrakePreset(resistance) {
  setConfigValue('config.brake_resistance', resistance);
  setConfigValue('config.max_regen_current', 0);
  setConfigValue('config.dc_max_negative_current', -0.000001);
  setConfigValue('config.enable_dc_bus_overvoltage_ramp', resistance > 0 ? 1 : 0);
  setConfigValue('config.dc_bus_overvoltage_ramp_start', 14);
  setConfigValue('config.dc_bus_overvoltage_ramp_end', 15.5);
  updateBrakePowerPreview();
  dom.configStatus.textContent = resistance > 0
    ? `已填入 ${resistance} Ω 制动预设，确认实物已跨接 DC+ 与 AUX 后再写入设备。`
    : '已填入“未接制动电阻”安全预设；写入后制动电阻保持禁用。';
}

function bindActions() {
  navButtons.forEach((button) => button.addEventListener('click', () => setView(button.dataset.view)));
  dom.mobileViewSelect.addEventListener('change', () => setView(dom.mobileViewSelect.value));
  dom.connectButton.addEventListener('click', connectSerial);
  dom.refreshPortsButton.addEventListener('click', refreshPorts);
  dom.simulateButton.addEventListener('click', startSimulation);
  modeButtons.forEach((button) => button.addEventListener('click', () => setMode(Number(button.dataset.mode))));
  dom.velocityButton.addEventListener('click', () => {
    state.positionTarget = Number.NaN;
    sendControl(command.velocity(dom.velocityInput.value), 'velocity', dom.velocityInput.value);
  });
  dom.positionButton.addEventListener('click', () => sendControl(
    () => {
      state.positionTarget = Number(
        (state.position + Number(dom.positionInput.value)).toFixed(6),
      );
      return command.trajectoryPosition(state.positionTarget);
    },
    'position', dom.positionInput.value,
  ));
  dom.torqueButton.addEventListener('click', () => {
    state.positionTarget = Number.NaN;
    sendControl(command.torque(dom.torqueInput.value), 'torque', dom.torqueInput.value);
  });
  dom.calibrateButton.addEventListener('click', () => { setView('calibration'); sendCommand(command.calibrate()); });
  dom.guideStartButton.addEventListener('click', () => sendCommand(command.calibrate()));
  dom.coggingCalibrationButton.addEventListener('click', () => {
    if (state.mode !== MODE.ABZ) { showToast('齿槽补偿标定只支持 ABZ 增量反馈'); return; }
    if (state.stateCode !== 1) { showToast('请先急停并等待设备进入 Idle'); return; }
    if (!state.motorCalibrated || !state.encoderReady) { showToast('请先完成电机和编码器校准'); return; }
    sendCommand(command.calibrateCogging());
  });
  dom.clearFaultButton.addEventListener('click', () => sendCommand(command.clearFault()));
  dom.saveButton.addEventListener('click', async () => {
    if (state.stateCode !== 1) { showToast('请先急停并确认 Idle，再保存到 Flash'); return; }
    await sendCommand(command.save());
    showToast('已发送保存参数命令');
  });
  dom.estopButton.addEventListener('click', async () => { await sendCommand(command.stop()); showToast('急停命令已发送，等待设备确认 PWM 关闭'); });
  dom.terminalSendButton.addEventListener('click', () => { const text = dom.terminalInput.value.trim(); if (text) { dom.terminalInput.value = ''; sendCommand(text); } });
  dom.terminalInput.addEventListener('keydown', (event) => { if (event.key === 'Enter') dom.terminalSendButton.click(); });
  dom.terminalClearButton.addEventListener('click', () => { dom.terminalLog.textContent = ''; });
  dom.readConfigButton.addEventListener('click', readConfig);
  dom.applyConfigButton.addEventListener('click', applyConfig);
  dom.safeProfileButton.addEventListener('click', loadSafeProfile);
  dom.brakeOffPresetButton.addEventListener('click', () => loadBrakePreset(0));
  dom.brake5PresetButton.addEventListener('click', () => loadBrakePreset(5));
  dom.brake2PresetButton.addEventListener('click', () => loadBrakePreset(2));
  configInputs.filter((input) => input.dataset.config.startsWith('config.')).forEach((input) => input.addEventListener('input', updateBrakePowerPreview));
  dom.configSaveButton.addEventListener('click', async () => {
    if (state.stateCode !== 1) {
      dom.configStatus.textContent = '保存被阻止：请先急停并确认设备处于 Idle。';
      showToast('请先急停并确认 Idle，再保存到 Flash');
      return;
    }
    await sendCommand(command.save());
    dom.configStatus.textContent = '已发送 Flash 保存命令';
  });
  dom.clearFaultHistoryButton.addEventListener('click', () => { state.faultHistory = []; renderFaults(); });
  scopeChannels.forEach((input) => input.addEventListener('change', () => {
    scopeCursorSample = null;
    dom.scopeCursorTooltip.hidden = true;
    drawCharts();
  }));
  dom.scopeRunButton.addEventListener('click', () => {
    scopeRunning = !scopeRunning;
    scopePausedAt = scopeRunning ? null : performance.now();
    updateScopeControls();
  });
  dom.scopeClearButton.addEventListener('click', () => {
    state.history = [];
    scopeTimeOffsetSeconds = 0;
    scopeCursorSample = null;
    if (!scopeRunning) scopePausedAt = performance.now();
    drawCharts();
    showToast('示波器缓存已清空');
  });
  dom.scopeAutoScaleButton.addEventListener('click', () => {
    if (scopeAutoScale) prepareManualScopeAxis();
    else scopeAutoScale = true;
    updateScopeControls();
  });
  dom.scopeWindowSelect.addEventListener('change', () => {
    setScopeWindow(dom.scopeWindowSelect.value);
  });
  dom.scopeAxisApplyButton.addEventListener('click', applyScopeAxisRange);
  dom.scopeAxisResetButton.addEventListener('click', resetScopeView);
  [dom.scopeAxisMin, dom.scopeAxisMax].forEach((input) => input.addEventListener('keydown', (event) => {
    if (event.key === 'Enter') applyScopeAxisRange();
  }));
  dom.scopeYZoomSlider.addEventListener('input', () => applyScopeAxisZoom(Number(dom.scopeYZoomSlider.value)));
  dom.scopeXWindowSlider.addEventListener('input', () => setScopeWindow(dom.scopeXWindowSlider.value));
  dom.scopeCanvas.addEventListener('wheel', handleScopeWheel, { passive: false });
  dom.scopeCanvas.addEventListener('pointerdown', (event) => {
    if (event.button !== 0) return;
    prepareManualScopeAxis();
    scopeSelectionStart = scopePlotPoint(event);
    dom.scopeSelectionBox.hidden = false;
    dom.scopeSelectionBox.style.left = `${scopeSelectionStart.x}px`;
    dom.scopeSelectionBox.style.top = `${scopeSelectionStart.y}px`;
    dom.scopeSelectionBox.style.width = '0px';
    dom.scopeSelectionBox.style.height = '0px';
    dom.scopeCursorTooltip.hidden = true;
    dom.scopeCanvas.classList.add('selecting');
    dom.scopeCanvas.setPointerCapture(event.pointerId);
  });
  dom.scopeCanvas.addEventListener('pointermove', (event) => {
    if (!scopeSelectionStart) { updateScopeCursor(event); return; }
    const point = scopePlotPoint(event);
    dom.scopeSelectionBox.style.left = `${Math.min(scopeSelectionStart.x, point.x)}px`;
    dom.scopeSelectionBox.style.top = `${Math.min(scopeSelectionStart.y, point.y)}px`;
    dom.scopeSelectionBox.style.width = `${Math.abs(point.x - scopeSelectionStart.x)}px`;
    dom.scopeSelectionBox.style.height = `${Math.abs(point.y - scopeSelectionStart.y)}px`;
  });
  dom.scopeCanvas.addEventListener('pointerup', finishScopeSelection);
  dom.scopeCanvas.addEventListener('pointercancel', () => {
    scopeSelectionStart = null;
    dom.scopeSelectionBox.hidden = true;
    dom.scopeCanvas.classList.remove('selecting');
  });
  dom.scopeCanvas.addEventListener('pointerleave', () => {
    if (scopeSelectionStart) return;
    scopeCursorSample = null;
    dom.scopeCursorTooltip.hidden = true;
    drawCharts();
  });
  dom.scopeCanvas.addEventListener('dblclick', resetScopeView);
  window.addEventListener('resize', drawCharts);
}

bindActions();
setTransport('disconnected', '未连接');
setView('console');
updateModeUI();
renderStatus();
renderFaults();
updateCalibrationState();
updateScopeControls();
updateScopeAxisControls();
updateBrakePowerPreview();
if (desktop) refreshPorts();

window.setInterval(() => {
  if (state.transport !== 'connected' || polling) return;
  polling = true;
  const now = performance.now();
  sendCommand(command.fastTelemetry(), { log: false })
    .then(() => {
      if (now < nextSlowPollAt) return undefined;
      nextSlowPollAt = now + 200;
      return sendCommand(command.telemetry(), { log: false });
    })
    .catch(() => disconnectSerial(false))
    .finally(() => { polling = false; });
  if (performance.now() - lastFrameAt > 800) dom.lastMessage.textContent = '设备响应超时';
}, 20);
