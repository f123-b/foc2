export const MODE = Object.freeze({
  SPI: 0,
  ABZ: 1,
  SENSORLESS: 2,
  SENSORLESS_SPI_MONITOR: 3,
  SENSORLESS_ABZ_MONITOR: 4,
});

export const CONTROL_MODE = Object.freeze({
  TORQUE: 1,
  VELOCITY: 2,
  POSITION: 3,
});

export const AXIS_STATE = Object.freeze({
  0: '未定义',
  1: '空闲',
  2: '启动中',
  3: '完整校准',
  4: '电机校准',
  5: '无感运行',
  6: '索引搜索',
  7: '编码器校准',
  8: '闭环运行',
  9: '锁定旋转',
  10: '方向识别',
  11: '回零',
});

const AXIS_FAULTS = Object.freeze([
  [0x00000001, '状态不允许', '请先完成所需校准，并确认电机与编码器已就绪。'],
  [0x00000002, '母线欠压', '检查 12 V 电源、接线和电源带载能力。'],
  [0x00000004, '母线过压', '降低减速度或检查制动电阻与过压阈值。'],
  [0x00000008, '电流采样超时', '检查 PWM/ADC 时钟和固件实时性。'],
  [0x00000010, '制动电阻未使能', '检查制动电阻配置和功率级状态。'],
  [0x00000020, '电机意外掉使能', '检查驱动器故障和 PWM 安全链路。'],
  [0x00000040, '电机模块故障', '查看电机故障明细，重点检查 DRV8301 和三相接线。'],
  [0x00000080, '无感估算器故障', '检查磁链参数、方向、启动电流和无感锁定状态。'],
  [0x00000100, '编码器模块故障', '检查 AS5047P/ABZ 接线、供电、CPR 和校准状态。'],
  [0x00000200, '控制器模块故障', '检查控制模式、速度限制和反馈是否有效。'],
  [0x00000400, '无感模式禁止位置控制', '无感模式只能使用速度或扭矩控制。'],
  [0x00000800, '通信看门狗超时', '运行中持续发送控制/看门狗帧；Idle 状态不会触发该故障。'],
  [0x00001000, '触发最小限位', '检查机械限位和限位输入配置。'],
  [0x00002000, '触发最大限位', '检查机械限位和限位输入配置。'],
  [0x00004000, '急停请求', '确认急停原因，清除故障前确保电机处于安全状态。'],
  [0x00020000, '无回零限位', '启用并检查最小限位后再执行回零。'],
  [0x00040000, '温度过高', '检查 MOS/电机温度和散热，降低电流限制。'],
]);

const MOTOR_FAULTS = Object.freeze([
  [0x00000001, '相电阻超范围', '检查三相接线，重新执行电阻校准。'],
  [0x00000002, '相电感超范围', '检查三相接线，重新执行电感校准。'],
  [0x00000004, 'ADC 采样故障', '检查电流采样和功率级。'],
  [0x00000008, 'DRV8301 驱动器故障', '读取 gate_driver.drv_fault，检查过流、欠压和温度。'],
  [0x00000010, '控制周期未生成 PWM', '若同时有控制器超速，这是超速退出后的次生故障；先处理位置/速度峰值。单独出现时再检查固件实时性。'],
  [0x00000020, '不支持的电机类型', '确认电机类型为高电流 PMSM。'],
  [0x00000040, '制动电流超范围', '检查制动电阻和母线配置。'],
  [0x00000080, '调制幅值不足', '检查母线电压与电机参数，降低目标电流。'],
  [0x00000400, '电流采样饱和', '降低电流限制，检查采样硬件。'],
  [0x00001000, '电流限制被违反', '降低目标扭矩/速度并检查控制参数。'],
  [0x00004000, '回馈过流', '降低减速度，检查电源是否能吸收回馈能量。'],
  [0x00008000, '母线过流', '降低电流限制并检查供电。'],
]);

const ENCODER_FAULTS = Object.freeze([
  [0x00000001, '编码器 PLL 增益不稳定', '降低编码器带宽。'],
  [0x00000002, 'CPR 与极对数不匹配', '核对 CPR=4000（ABZ）或 16384（SPI）和极对数 10。'],
  [0x00000004, '编码器无响应', '检查编码器供电、信号线和转子是否可动。'],
  [0x00000008, '不支持的编码器模式', '检查 SPI/ABZ 模式配置。'],
  [0x00000020, '尚未找到索引', '执行索引搜索或关闭索引要求。'],
  [0x00000040, 'SPI 超时', '检查 AS5047P 的 CS、SCK、MISO、MOSI 和共地。'],
  [0x00000080, 'SPI 通信错误', '检查 SPI 信号完整性和 3.3 V 电平。'],
  [0x00000100, 'SPI 编码器未就绪', '确认 AS5047P 上电并完成校准。'],
]);

const CONTROLLER_FAULTS = Object.freeze([
  [0x00000001, '超速', '检查控制模式、速度限制和反馈估算；降低目标值，不要仅提高超速阈值。'],
  [0x00000002, '输入模式无效', '检查控制器输入模式。'],
  [0x00000004, '控制器增益不稳定', '降低控制器带宽或重新配置增益。'],
  [0x00000010, '负载编码器无效', '检查加载的编码器轴。'],
  [0x00000020, '反馈估算无效', '检查编码器或无感估算器是否 ready。'],
]);

const SENSORLESS_FAULTS = Object.freeze([
  [0x00000001, '无感增益不稳定', '降低无感 PLL/observer 增益。'],
]);

function decodeFlags(mask, table, prefix) {
  const value = Number(mask) >>> 0;
  return table.filter(([bit]) => (value & bit) !== 0).map(([bit, title, advice]) => ({ bit, title: `${prefix}${title}`, advice }));
}

export function decodeFaults(status) {
  return [
    ...decodeFlags(status.fault, AXIS_FAULTS, '轴：'),
    ...decodeFlags(status.motorError, MOTOR_FAULTS, '电机：'),
    ...decodeFlags(status.encoderError, ENCODER_FAULTS, '编码器：'),
    ...decodeFlags(status.controllerError, CONTROLLER_FAULTS, '控制器：'),
    ...decodeFlags(status.sensorlessError, SENSORLESS_FAULTS, '无感：'),
  ];
}

export function faultSummary(status) {
  const faults = decodeFaults(status);
  return faults.length ? faults.map(({ title }) => title).join('；') : '无故障';
}

export const command = Object.freeze({
  telemetry: (axis = 0) => `j ${axis}`,
  fastTelemetry: (axis = 0) => `g ${axis}`,
  watchdog: (axis = 0) => `u ${axis}`,
  mode: (mode, axis = 0) => `m ${axis} ${mode}`,
  stop: (axis = 0) => `x ${axis}`,
  calibrate: (axis = 0) => `a ${axis}`,
  calibrateCogging: (axis = 0) => `b ${axis}`,
  clearFault: (axis = 0) => `k ${axis}`,
  save: () => 'ss',
  state: (value, axis = 0) => `w axis${axis}.requested_state ${value}`,
  velocity: (value, axis = 0) => `v ${axis} ${finite(value)}`,
  position: (value, axis = 0) => `p ${axis} ${finite(value)}`,
  safePosition: (value, axis = 0) => `p ${axis} ${finite(value)} 0 0`,
  trajectoryPosition: (value, axis = 0) => `t ${axis} ${finite(value)}`,
  torque: (value, axis = 0) => `c ${axis} ${finite(value)}`,
  read: (path) => `r ${path}`,
  write: (path, value) => `w ${path} ${finite(value)}`,
});

function finite(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number : 0;
}

export function encodeCommand(text) {
  return new TextEncoder().encode(`${text}\n`);
}

export function parseTelemetry(line) {
  const fields = line.trim().split(/\s+/);
  if (fields.length < 11 || fields[0] !== '@') return null;
  const values = fields.slice(1).map(Number);
  if (values.some((value) => !Number.isFinite(value))) return null;
  const [state, fault, velocity, current, position, busVoltage, temperature,
    observerLocked, angleError, mode, motorError = 0, encoderError = 0,
    controllerError = 0, sensorlessError = 0, armedState = 0,
    encoderReady = 0, motorCalibrated = 0, direction = 0,
    fetThermistorError = 0, motorThermistorError = 0,
    controlMode = CONTROL_MODE.POSITION, phaseAVoltage = 0,
    phaseBVoltage = 0, phaseCVoltage = 0, idMeasured = 0,
    iqSetpoint = 0, idSetpoint = 0, anticoggingValid = 0,
    anticoggingCalibrationActive = 0, anticoggingIndex = 0,
    anticoggingCoverage = 0] = values;
  return {
    axisState: AXIS_STATE[state] ?? `状态 ${state}`,
    stateCode: state,
    fault,
    velocity,
    current,
    position,
    busVoltage,
    temperature,
    observerLocked: Boolean(observerLocked),
    angleError,
    mode,
    motorError,
    encoderError,
    controllerError,
    sensorlessError,
    armedState,
    pwmArmed: armedState === 3,
    encoderReady: Boolean(encoderReady),
    motorCalibrated: Boolean(motorCalibrated),
    direction,
    fetThermistorError,
    motorThermistorError,
    controlMode,
    phaseAVoltage,
    phaseBVoltage,
    phaseCVoltage,
    idMeasured,
    iqSetpoint,
    idSetpoint,
    anticoggingValid: Boolean(anticoggingValid),
    anticoggingCalibrationActive: Boolean(anticoggingCalibrationActive),
    anticoggingIndex,
    anticoggingCoverage,
  };
}

export function parseFastTelemetry(line) {
  const fields = line.trim().split(/\s+/);
  if (fields.length < 12 || fields[0] !== '!') return null;
  const values = fields.slice(1).map(Number);
  if (values.some((value) => !Number.isFinite(value))) return null;
  const [state, velocity, current, position, busVoltage, phaseAVoltage,
    phaseBVoltage, phaseCVoltage, idMeasured, iqSetpoint, idSetpoint,
    velocitySetpoint = 0, rawVelocity = velocity, windowVelocity = velocity,
    velocityIntegratorTorque = 0, lowSpeedTorque = 0,
    positionSetpoint = position, positionError = 0, lowSpeedState = 0,
    velocityProportionalTorque = 0, anticoggingTorque = 0,
    finalTorque = 0, maxAvailableTorque = 0,
    mTVelocity = velocity, velocityError = 0,
    torqueUnsaturated = 0, motorTorqueSaturated = 0, encoderEdgeAge = 0,
    controlObserverVelocity = velocity, encoderDeltaCount = 0,
    encoderShadowCount = 0,
    abzVelocityTorqueBeforeLimit = 0, abzVelocityTorqueAfterLimit = 0,
    abzVelocityTorqueSaturated = 0,
    abzVelGain = 0, abzVelIntegratorGain = 0,
    controlVelocityObserverBandwidth = 0, abzVelocityTorqueLimit = 0,
    abzCoulombFrictionTorque = 0, abzBreakawayTorque = 0,
    enableLowSpeedCompensation = 0,
    frictionTargetTorque = 0, frictionSpeedRatio = 0, frictionAssistBlend = 0,
    frictionNoProgressTime = 0, frictionRecoveryTimer = 0] = values;
  return {
    axisState: AXIS_STATE[state] ?? `状态 ${state}`,
    stateCode: state,
    velocity,
    current,
    position,
    busVoltage,
    phaseAVoltage,
    phaseBVoltage,
    phaseCVoltage,
    idMeasured,
    iqSetpoint,
    idSetpoint,
    velocitySetpoint,
    rawVelocity,
    windowVelocity,
    velocityIntegratorTorque,
    lowSpeedTorque,
    frictionTorque: lowSpeedTorque,
    positionSetpoint,
    positionError,
    lowSpeedState,
    frictionState: lowSpeedState,
    velocityProportionalTorque,
    anticoggingTorque,
    finalTorque,
    maxAvailableTorque,
    mTVelocity,
    velocityError,
    torqueUnsaturated,
    motorTorqueSaturated,
    encoderEdgeAge,
    controlObserverVelocity,
    encoderDeltaCount,
    encoderShadowCount,
    abzVelocityTorqueBeforeLimit,
    abzVelocityTorqueAfterLimit,
    abzVelocityTorqueSaturated,
    abzVelGain,
    abzVelIntegratorGain,
    controlVelocityObserverBandwidth,
    abzVelocityTorqueLimit,
    abzCoulombFrictionTorque,
    abzBreakawayTorque,
    enableLowSpeedCompensation,
    frictionTargetTorque,
    frictionSpeedRatio,
    frictionAssistBlend,
    frictionNoProgressTime,
    frictionRecoveryTimer,
  };
}

export class LineParser {
  constructor() {
    this.decoder = new TextDecoder();
    this.buffer = '';
  }

  push(input) {
    this.buffer += typeof input === 'string' ? input : this.decoder.decode(input, { stream: true });
    const parts = this.buffer.split(/\r?\n/);
    this.buffer = parts.pop() ?? '';
    return parts.map((line) => line.trim()).filter(Boolean);
  }
}
