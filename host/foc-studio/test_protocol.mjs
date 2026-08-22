import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
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
  anticoggingMapMaxAbs: 0, anticoggingCalibrationFailed: 0,
  anticoggingCalibrationAbortReason: 0, anticoggingStatsIndex: 0,
  anticoggingPostprocessIndex: 0, anticoggingRejectedEstimatorSamples: 0,
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
  controlVelocity: 1.25, encoderPllVelocity: 1.25, velocityWindow50ms: 1.25,
  velocityWindow100ms: 0, mtVelocity: 1.25, observerVelocity: 1.25,
  observerBandwidth: 0, velocityEstimatorDisagreement: 0, abzCountGlitchCount: 0,
   frictionContinuousTorque: 0, frictionBreakawayExtraTorque: 0,
   frictionRunningAssistBlend: 0, frictionBreakawayExitTimer: 0,
   effectiveAbzVelGain: 0, effectiveAbzVelIntegratorGain: 0, abzLowSpeedGainBlend: 0,
   anticoggingEffectiveScale: 0,
  velocitySetpoint: 0, rawVelocity: 1.25, windowVelocity: 1.25,
  mTVelocity: 1.25, controlObserverVelocity: 1.25,
  velocityIntegratorTorque: 0, lowSpeedTorque: 0, frictionTorque: 0,
  positionSetpoint: 2.5, positionError: 0, lowSpeedState: 0, frictionState: 0,
  velocityProportionalTorque: 0, anticoggingTorque: 0, finalTorque: 0,
  maxAvailableTorque: 0, velocityError: 0,
  torqueUnsaturated: 0, motorTorqueSaturated: 0, encoderEdgeAge: 0,
  encoderDeltaCount: 0, encoderShadowCount: 0,
  abzVelocityTorqueBeforeLimit: 0, abzVelocityTorqueAfterLimit: 0,
  abzVelocityTorqueSaturated: 0, abzVelGain: 0, abzVelIntegratorGain: 0,
  abzObserverMinBandwidth: 0, controlVelocityObserverBandwidth: 0,
  abzVelocityTorqueLimit: 0,
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
// Full 64-field fast-telemetry record: the appended low-speed diagnostics must
// land on their exact positions (firmware formatter and JS parser aligned).
const fullFastFields = new Array(64).fill(0);
fullFastFields[0] = 8;        // axis state
fullFastFields[1] = 0.2;      // velocity / controlVelocity
fullFastFields[12] = 0.35;    // rawVelocity / encoderPllVelocity
fullFastFields[13] = 0.175;   // windowVelocity / velocityWindow50ms
fullFastFields[23] = 0.05;    // mTVelocity / mtVelocity
fullFastFields[28] = 2;       // controlObserverVelocity / observerVelocity
fullFastFields[36] = 15;      // abzObserverMinBandwidth
fullFastFields[52] = 0.18;    // velocityWindow100ms
fullFastFields[53] = 24.5;    // observerBandwidth (effective)
fullFastFields[54] = -0.02;   // velocityEstimatorDisagreement
fullFastFields[55] = 7;       // abzCountGlitchCount
fullFastFields[56] = 0.0029;  // frictionContinuousTorque
fullFastFields[57] = 0.0018;  // frictionBreakawayExtraTorque
fullFastFields[58] = 0.5;     // frictionRunningAssistBlend
fullFastFields[59] = 0.04;    // frictionBreakawayExitTimer
fullFastFields[60] = 0.004;   // effectiveAbzVelGain
fullFastFields[61] = 0.0025;  // effectiveAbzVelIntegratorGain
fullFastFields[62] = 0.75;    // abzLowSpeedGainBlend
fullFastFields[63] = -2.5;    // anticoggingEffectiveScale
assert.equal(fullFastFields.length, 64, 'firmware fast telemetry carries 64 fields');
const parsedFull = parseFastTelemetry(`! ${fullFastFields.join(' ')}`);
assert.equal(parsedFull.velocityWindow100ms, 0.18);
assert.equal(parsedFull.observerBandwidth, 24.5);
assert.equal(parsedFull.velocityEstimatorDisagreement, -0.02);
assert.equal(parsedFull.abzCountGlitchCount, 7);
assert.equal(parsedFull.frictionContinuousTorque, 0.0029);
assert.equal(parsedFull.frictionBreakawayExtraTorque, 0.0018);
assert.equal(parsedFull.frictionRunningAssistBlend, 0.5);
assert.equal(parsedFull.frictionBreakawayExitTimer, 0.04);
assert.equal(parsedFull.effectiveAbzVelGain, 0.004);
assert.equal(parsedFull.effectiveAbzVelIntegratorGain, 0.0025);
assert.equal(parsedFull.abzLowSpeedGainBlend, 0.75);
assert.equal(parsedFull.anticoggingEffectiveScale, -2.5);
assert.equal(parsedFull.abzObserverMinBandwidth, 15);
assert.equal(parsedFull.controlVelocity, 0.2);
assert.equal(parsedFull.encoderPllVelocity, 0.35);
assert.equal(parsedFull.velocityWindow50ms, 0.175);
assert.equal(parsedFull.mtVelocity, 0.05);
assert.equal(parsedFull.observerVelocity, 2);
// The legacy controlVelocityObserverBandwidth alias must track the EFFECTIVE
// dynamic bandwidth (observerBandwidth), never the fixed min-bandwidth config.
assert.equal(parsedFull.controlVelocityObserverBandwidth, 24.5);
assert.notEqual(parsedFull.controlVelocityObserverBandwidth, parsedFull.abzObserverMinBandwidth);
// Worst-case fast-telemetry line length, computed from the firmware's own
// format string in ascii_protocol.cpp (this must stay under the firmware
// respond() buffer so a record is never truncated; the firmware additionally
// drops a frame if snprintf ever reports it would not fit).
const firmwarePath = fileURLToPath(new URL('../../firmware-target/Firmware/communication/ascii_protocol.cpp', import.meta.url));
const asciiSource = readFileSync(firmwarePath, 'utf8');
const formatMatch = asciiSource.match(/"(![^"]*)"/);
assert.ok(formatMatch, 'fast telemetry format string found in ascii_protocol.cpp');
const format = formatMatch[1];
const specifiers = [...format.matchAll(/%(?:u|ld|lu|\.6g)/g)].map((m) => m[0]);
const literalLength = format.replace(/%(?:u|ld|lu|\.6g)/g, '').length;
const worstValueWidth = { '%u': 10, '%ld': 11, '%lu': 10, '%.6g': 13 }; // chars
const worstCaseLength = literalLength + specifiers.reduce((sum, s) => sum + worstValueWidth[s], 0);
assert.equal(specifiers.length, 64, 'firmware fast telemetry format has 64 specifiers');
assert.equal(literalLength, 65, 'g-format literal length (separators + "! ")');
assert.equal(worstCaseLength, 65 + 54 * 13 + 7 * 10 + 2 * 11 + 1 * 10,
  'worst-case formula matches the specifier mix (54x%.6g, 7x%u, 2x%ld, 1x%lu)');
assert.ok(worstCaseLength < 1024,
  `fast telemetry worst case ${worstCaseLength} chars must fit the 1024-byte respond() buffer`);
// A line with more fields than the parser knows must still parse (the parser
// maps by position and ignores trailing unknown fields gracefully).
const longerLine = parseFastTelemetry(`! ${[...fullFastFields, 99, 98].join(' ')}`);
assert.equal(longerLine.velocityWindow100ms, 0.18);
// The appended estimator-agreement counter on the aggregate 'j' record.
const aggregate = parseTelemetry('@ 8 0 0.2 0.7 0.1 12.1 30 0 0 1 0 0 0 0 3 1 1 1 0 0 2 0 0 0 0 0.2 0 1 1 1800 3120 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1');
assert.equal(aggregate.anticoggingRejectedEstimatorSamples, 1);
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
