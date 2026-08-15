import assert from 'node:assert/strict';
import {
  AXIS_STATE, CONTROL_MODE, LineParser, MODE, command, decodeFaults, encodeCommand,
  faultSummary, parseFastTelemetry, parseTelemetry,
} from './protocol.js';

const parser = new LineParser();
assert.deepEqual(parser.push('@ 1 0 1.5'), []);
assert.deepEqual(parser.push(' 0.2 3 12.1 31 1 0.01 3\r\nok mode 3\n'), [
  '@ 1 0 1.5 0.2 3 12.1 31 1 0.01 3',
  'ok mode 3',
]);

assert.deepEqual(parseTelemetry('@ 8 0 12.5 -1.25 3.5 12.2 35 1 -0.004 4'), {
  axisState: AXIS_STATE[8], stateCode: 8, fault: 0, velocity: 12.5,
  current: -1.25, position: 3.5, busVoltage: 12.2, temperature: 35,
  observerLocked: true, angleError: -0.004, mode: 4, motorError: 0,
  encoderError: 0, controllerError: 0, sensorlessError: 0, armedState: 0,
  pwmArmed: false, encoderReady: false, motorCalibrated: false, direction: 0,
  fetThermistorError: 0, motorThermistorError: 0,
  controlMode: CONTROL_MODE.POSITION,
  phaseAVoltage: 0, phaseBVoltage: 0, phaseCVoltage: 0,
  idMeasured: 0, iqSetpoint: 0, idSetpoint: 0,
  anticoggingValid: false, anticoggingCalibrationActive: false, anticoggingIndex: 0,
  anticoggingCoverage: 0,
  anticoggingMapMean: 0, anticoggingMapRms: 0, anticoggingMapPeakToPeak: 0,
  anticoggingMapMaxJump: 0, anticoggingMapWrapJump: 0,
  anticoggingForwardValidBins: 0, anticoggingReverseValidBins: 0,
  anticoggingRejectedVelocitySamples: 0, anticoggingRejectedReverseSamples: 0,
  anticoggingRejectedStateSamples: 0, anticoggingRejectedSaturationSamples: 0,
});
const detailed = parseTelemetry('@ 1 320 0 0 3.5 12.2 35 0 0 0 8 128 32 0 0 0 1 -1 0 0');
assert.equal(detailed.motorError, 8);
assert.equal(detailed.encoderError, 128);
assert.equal(detailed.controllerError, 32);
assert.equal(detailed.motorCalibrated, true);
assert.equal(detailed.direction, -1);
assert.equal(detailed.controlMode, CONTROL_MODE.POSITION);
assert.equal(parseTelemetry('@ 8 0 0 0 0 12 30 0 0 0 0 0 0 0 3 1 1 1 0 0 2').controlMode, CONTROL_MODE.VELOCITY);
const scopeTelemetry = parseTelemetry('@ 8 0 0.2 0.7 0.1 12.1 30 0 0 1 0 0 0 0 3 1 1 1 0 0 1 1.2 -0.4 -0.8 0.03 0.75 0');
assert.equal(scopeTelemetry.phaseAVoltage, 1.2);
assert.equal(scopeTelemetry.phaseBVoltage, -0.4);
assert.equal(scopeTelemetry.phaseCVoltage, -0.8);
assert.equal(scopeTelemetry.idMeasured, 0.03);
assert.equal(scopeTelemetry.iqSetpoint, 0.75);
const coggingTelemetry = parseTelemetry('@ 8 0 0.2 0.7 0.1 12.1 30 0 0 1 0 0 0 0 3 1 1 1 0 0 2 0 0 0 0 0.2 0 1 1 1800 3120');
assert.equal(coggingTelemetry.anticoggingValid, true);
assert.equal(coggingTelemetry.anticoggingCalibrationActive, true);
assert.equal(coggingTelemetry.anticoggingIndex, 1800);
assert.equal(coggingTelemetry.anticoggingCoverage, 3120);
assert.deepEqual(parseFastTelemetry('! 8 1.25 -0.3 2.5 12.2 1 -0.2 -0.8 0.01 -0.25 0'), {
  axisState: AXIS_STATE[8], stateCode: 8, velocity: 1.25, current: -0.3,
  position: 2.5, busVoltage: 12.2, phaseAVoltage: 1, phaseBVoltage: -0.2,
  phaseCVoltage: -0.8, idMeasured: 0.01, iqSetpoint: -0.25, idSetpoint: 0,
  velocitySetpoint: 0, rawVelocity: 1.25, windowVelocity: 1.25,
  velocityIntegratorTorque: 0, lowSpeedTorque: 0, frictionTorque: 0,
  positionSetpoint: 2.5, positionError: 0, lowSpeedState: 0, frictionState: 0,
  velocityProportionalTorque: 0, anticoggingTorque: 0, finalTorque: 0,
  maxAvailableTorque: 0, mTVelocity: 1.25, velocityError: 0,
  torqueUnsaturated: 0, motorTorqueSaturated: 0, encoderEdgeAge: 0,
  controlObserverVelocity: 1.25, encoderDeltaCount: 0, encoderShadowCount: 0,
  abzVelocityTorqueBeforeLimit: 0, abzVelocityTorqueAfterLimit: 0,
  abzVelocityTorqueSaturated: 0, abzVelGain: 0, abzVelIntegratorGain: 0,
  controlVelocityObserverBandwidth: 0, abzVelocityTorqueLimit: 0,
  abzCoulombFrictionTorque: 0, abzBreakawayTorque: 0, enableLowSpeedCompensation: 0,
  frictionTargetTorque: 0, frictionSpeedRatio: 0, frictionAssistBlend: 0,
  frictionNoProgressTime: 0, frictionRecoveryTimer: 0,
  frictionForwardVelocity: 0, frictionReverseDetected: 0,
  anticoggingCalibrationPhase: 0, anticoggingProgressPercent: 0,
  anticoggingScanVelocity: 0, anticoggingScanVelocityError: 0,
});
const controlTelemetry = parseFastTelemetry('! 8 0.2 0.3 2.5 12.2 1 -0.2 -0.8 0.01 0.25 0 0.2 0.35 0.175 0.0015 0.003 2.55 0.05 2 0.0004 -0.0008 0.0041 0.0254');
assert.equal(controlTelemetry.velocitySetpoint, 0.2);
assert.equal(controlTelemetry.rawVelocity, 0.35);
assert.equal(controlTelemetry.windowVelocity, 0.175);
assert.equal(controlTelemetry.velocityIntegratorTorque, 0.0015);
assert.equal(controlTelemetry.lowSpeedTorque, 0.003);
assert.equal(controlTelemetry.positionSetpoint, 2.55);
assert.equal(controlTelemetry.positionError, 0.05);
assert.equal(controlTelemetry.lowSpeedState, 2);
assert.equal(controlTelemetry.velocityProportionalTorque, 0.0004);
assert.equal(controlTelemetry.anticoggingTorque, -0.0008);
assert.equal(controlTelemetry.finalTorque, 0.0041);
assert.equal(controlTelemetry.maxAvailableTorque, 0.0254);
assert.equal(parseFastTelemetry('@ 8 0 1 2 3 4 5 6 7 8 9'), null);
assert.ok(decodeFaults(detailed).some(({ title }) => title.includes('DRV8301')));
assert.ok(faultSummary(detailed).includes('编码器'));
assert.equal(parseTelemetry('ok mode 1'), null);
assert.equal(command.mode(MODE.ABZ), 'm 0 1');
assert.equal(command.velocity(8.5), 'v 0 8.5');
assert.equal(command.fastTelemetry(), 'g 0');
assert.equal(command.calibrateCogging(), 'b 0');
assert.equal(command.safePosition(0.1), 'p 0 0.1 0 0');
assert.equal(command.trajectoryPosition(0.1), 't 0 0.1');
assert.equal(command.read('axis0.error'), 'r axis0.error');
assert.equal(command.write('axis0.motor.config.current_lim', 2), 'w axis0.motor.config.current_lim 2');
assert.equal(new TextDecoder().decode(encodeCommand(command.stop())), 'x 0\n');

console.log('ASCII protocol tests passed');
