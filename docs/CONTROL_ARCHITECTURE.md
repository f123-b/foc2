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

For incremental encoders, `Encoder::update()` maintains the normal ODrive PLL (`vel_estimate_`) and, only for diagnostics/cascaded ABZ control, a rolling 80-sample count sum. At the nominal 8 kHz current loop this is a 10 ms window:

```text
timer count -> delta_enc -> shadow_count
                      +-> encoder PLL -> raw_velocity
                      +-> 80-tick rolling sum / (4000 CPR * elapsed) -> window_velocity
raw + window -> Controller::velocity_feedback_for_control()
             -> command-speed blend: window below 1.50 turn/s, PLL above 1.75 turn/s
             -> VelocityFeedbackFilter: 8 Hz at <=1 turn/s, linear to 15 Hz at 2 turn/s
             -> velocity error -> scheduled P + bounded I + optional low-speed compensation
             -> motor torque limit -> Motor::update()
```

The command, not measured speed, selects the 1.50–1.75 turn/s blend. That prevents estimator chatter caused by count noise during a steady command. It also means acceleration, deceleration, reversal and position control can select an estimator region that does not match instantaneous rotor speed; this is an intentional but unvalidated trade-off.

## ABZ-specific stages and their consequences

| Stage | Purpose | Cost/risk |
|---|---|---|
| 10 ms rolling window | Avoid PLL zero-speed deadband and update every control tick. | Quantized at 4000 CPR: one count per 10 ms equals 0.025 turn/s; moving window adds roughly half-window observation delay and correlated steps. |
| Command-speed blend | Avoid estimator source toggling. | Linear value continuity but derivative discontinuity at 1.50/1.75; no hysteresis or acceleration-aware policy. |
| 8–15 Hz one-pole LPF | Prevent P/I chasing edge impulses. | A first-order lag has material phase delay near its bandwidth, compounded with window delay; bandwidth changes continuously in value but has slope break at 1/2 turn/s. |
| Gain schedule | Raises low-speed P while lowering I. | Continuous at 0.75/1.50 in value, but effective gains differ from host-visible configured values. |
| I clamp | Limits stored energy to 0.003–0.008 Nm. | Bounds windup but release slope changes at 0.50 and 2.00 turn/s. |
| LowSpeedCompensator | Supplies feed-forward/breakaway torque while holding worsening I. | It overlaps with P and I as a motion-producing mechanism; breakaway/recovery state thresholds need HIL proof. |

The present code has no abrupt value step in these linear blends, but it does have several slope and state transitions. A boundary test must measure final torque, not only individual gains.

## Position mode

`Controller::update()` produces `vel_des = vel_setpoint + position_gain * position_error` before the ABZ stages. Incremental position control clamps `pos_gain` to 1.0–1.2. The low-speed compensator can activate in position mode after a 4-count error and deactivate at 2 counts; it injects a minimum 0.02 turn/s virtual command. Therefore position control can enter a low-speed estimator/compensation region even with zero explicit velocity feed-forward. This is appropriate to audit separately from direct velocity mode.

## Telemetry and current observability

`g` emits closed-loop feedback as `velocity`, raw PLL velocity, window velocity, I torque and low-speed torque. It does not yet emit filtered feedback as a separate field, P torque, total pre-limit torque, final limited torque, saturation, delta count, blend, bandwidth, no-progress time or effective gains. `j` is slower aggregate status. USB is requested at 50 Hz by the host; it must not perform high-rate logging inside the current ISR.

## Emergency stop path

`x axis` in `ascii_protocol.cpp` clears controller inputs, calls `Controller::reset()`, clears current-control state, calls `safety_critical_disarm_motor_pwm()`, and requests Idle. `Motor::arm()` also resets controller and current-controller state before PWM arm. This covers the main transient states, but requires an integration test that demonstrates no trajectory, compensation or feed-forward survives re-arm.
