# 控制架构

## 实际闭环路径

```text
FOC Studio / other host
  -> USB CDC (`communication/interface_usb.*`)
  -> ASCII parser (`communication/ascii_protocol.cpp`)
  -> Axis state and Controller input
  -> Controller::update()
  -> torque setpoint
  -> Motor::update()
  -> Clarke/Park, Id/Iq PI, inverse Park, SVPWM
  -> modulation timings / PWM / DRV8301 / MOSFET / motor
  -> encoder sampling + Encoder::update() or SensorlessEstimator::update()
  -> Controller feedback source
```

`Axis::run_closed_loop_control_loop()` in `MotorControl/axis.cpp` executes `Controller::update()` then passes torque, encoder phase and phase velocity to `Motor::update()`. The latter owns current sampling and modulation. This ABZ work must not change that path. Sensorless has its own `Axis::run_sensorless_control_loop()` and `SensorlessEstimator`; it is outside the ABZ optimization boundary.

## ABZ speed feedback path

For incremental encoders, `Encoder::update()` maintains the normal PLL (`vel_estimate_`) and, only for diagnostics/cascaded ABZ control, a rolling 400-sample count sum. At the nominal 8 kHz current loop this is a 50 ms window:

```text
timer count -> delta_enc -> shadow_count
                      +-> encoder PLL -> raw_velocity
                      +-> 400-tick rolling sum / (4000 CPR * elapsed) -> window_velocity
raw + window -> Controller::velocity_feedback_for_control()
             -> command-speed blend: window below 2.50 turn/s, PLL above 4 turn/s
             -> VelocityFeedbackFilter: 6 Hz at <=1 turn/s, linear to 12 Hz at 2 turn/s
             -> velocity error -> scheduled P + bounded I + optional low-speed compensation
             -> motor torque limit -> Motor::update()
```

The command, not measured speed, selects the 2.50–4.00 turn/s blend. That prevents estimator chatter caused by count noise during a steady command. It also means acceleration, deceleration, reversal and position control can select an estimator region that does not match instantaneous rotor speed; this is an intentional but unvalidated trade-off.

## ABZ-specific stages and their consequences

| Stage | Purpose | Cost/risk |
|---|---|---|
| 50 ms rolling window | Avoid PLL zero-speed deadband and update every control tick. | Quantized at 4000 CPR: one count per 50 ms equals 0.005 turn/s; moving window adds roughly half-window observation delay but greatly reduces correlated steps. |
| Command-speed blend | Avoid estimator source toggling. | Window feedback remains dominant below 2.5 turn/s and blends to PLL by 4 turn/s; command-based selection avoids estimator chatter. |
| 6–12 Hz one-pole LPF | Prevent P/I chasing edge impulses. | A first-order lag has material phase delay near its bandwidth, compounded with window delay; bandwidth changes continuously in value but has slope break at 1/2 turn/s. |
| Gain schedule | Lowers low-speed P/I so breakaway torque is supplied by one bounded source. | Continuous at 1.00/1.75 in value, but effective gains differ from host-visible configured values. |
| I clamp | Limits stored energy to 0–0.0045 Nm, blended in from 1.0 to 1.75 turn/s. | Bounds windup and prevents a tooth-crossing release from becoming a speed impulse. |
| LowSpeedCompensator | Supplies a 0.004–0.018 Nm feed-forward/breakaway ramp, adds bounded positive-speed-error assist and a low-speed hold term, and holds worsening I. | It may use the extended ceiling below 0.5 turn/s; after breakaway, the running hold is 0.014 Nm through 1.0 turn/s and tapers to 0.004 Nm by 2.0 turn/s. The controller keeps the complete helper through 2.0 turn/s, then fades it only across the denser 2.0–2.5 turn/s transition. Recovery requires both forward encoder progress and 12 ms at ≥55% of the commanded speed, so low-speed count dither cannot unload the torque prematurely. |

The present code has no abrupt value step in these linear blends, but it does have several slope and state transitions. A boundary test must measure final torque, not only individual gains.

## Position mode

`Controller::update()` produces `vel_des = vel_setpoint + position_gain * position_error` before the ABZ stages. Incremental position control clamps `pos_gain` to 1.0–1.2. The low-speed compensator can activate in position mode after a 4-count error and deactivate at 2 counts; it injects a minimum 0.02 turn/s virtual command. Therefore position control can enter a low-speed estimator/compensation region even with zero explicit velocity feed-forward. This is appropriate to audit separately from direct velocity mode.

## Telemetry and current observability

`g` emits closed-loop feedback as `velocity`, raw PLL velocity, window velocity, I torque and low-speed torque. It does not yet emit filtered feedback as a separate field, P torque, total pre-limit torque, final limited torque, saturation, delta count, blend, bandwidth, no-progress time or effective gains. `j` is slower aggregate status. USB is requested at 50 Hz by the host; it must not perform high-rate logging inside the current ISR.

## Emergency stop path

`x axis` in `ascii_protocol.cpp` clears controller inputs, calls `Controller::reset()`, clears current-control state, calls `safety_critical_disarm_motor_pwm()`, and requests Idle. `Motor::arm()` also resets controller and current-controller state before PWM arm. This covers the main transient states, but requires an integration test that demonstrates no trajectory, compensation or feed-forward survives re-arm.
