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

For incremental encoders, `Encoder::update()` maintains the normal PLL
(`vel_estimate_`, drives commutation / phase interpolation; also the emergency
overspeed layer, see below), plus
mechanical diagnostics that never enter the electrical angle path:

```text
timer count -> delta_enc -> shadow_count_ (int32, commutation fast path)
                       +-> mechanical_count_ (int64, diagnostics)
                       +-> 50/100 ms true sliding window (shared ring buffer)
                       +-> M/T (count-time) estimator (edge diagnostic)
                       +-> encoder PLL -> raw_velocity (commutation / emergency overspeed layer)

Controller::update() (ABZ velocity/position mode):
   delta_position = last_delta_count_ / CPR
   -> AbzVelocityObserver (adaptive bandwidth, delta-driven, local frame)
   -> control_observer_velocity_   <-- SINGLE ABZ velocity PI feedback
   -> velocity error -> abz_vel_gain P + bounded I (anti-windup)
   -> + low-speed friction/breakaway FF (same observer feedback)
   -> + anticogging FF (observer-gated samples)
   -> ABZ velocity torque limit -> motor torque limit -> Motor::update()
```

The observer bandwidth follows the commanded speed with a smooth schedule
(~15 Hz at standstill, ~30 Hz at 1 turn/s, ~50 Hz at 4+ turn/s, clamped by
`abz_velocity_observer_min/max_bandwidth`); gains are recomputed only when the
bandwidth changes by more than 0.5 Hz. The M/T estimator, the 50 ms and the
100 ms windows are diagnostics only — none of them is ever switched in as the
loop feedback, so there is no estimator hand-over discontinuity. The old
command-speed PLL/window blend and the VelocityFeedbackFilter LPF have been
removed. See [ABZ 机械测速架构](ABZ_VELOCITY_ESTIMATION.md) for the full
estimator-by-estimator description.

## Overspeed safety (dual-layer qualified detection)

The velocity PI feedback stays exclusively the ABZ control observer, but the
overspeed protection is a separate dual-layer qualified detector
(`AbzOverspeedQualifier`, 16-cycle consecutive qualification shared by both
layers):

- Layer A (normal): the ABZ control feedback (observer) exceeds
  `vel_limit * vel_limit_tolerance`;
- Layer B (emergency): the raw encoder PLL **or** the 50 ms count window
  exceeds `2 * normal_limit`, so a real runaway is not entirely dependent on
  the low-bandwidth observer.

A single raw PLL spike never drops PWM; a sustained violation (either layer)
latches ERROR_OVERSPEED. Non-ABZ modes keep the raw-PLL-vs-normal-limit
qualification. This is the exact overspeed architecture the code implements —
the encoder PLL drives commutation / phase interpolation, not the overspeed
decision by itself.

## ABZ-specific stages and their consequences

| Stage | Purpose | Cost/risk |
|---|---|---|
| AbzVelocityObserver | Single ABZ velocity feedback; adaptive bandwidth; incremental delta input with local-frame rebase (no int32 overflow / float precision loss). | Low-passed vs raw PLL: adds phase lag that grows at low bandwidth; 15 Hz at standstill is a deliberate noise/response trade. |
| 50 ms sliding window | Fast mechanical diagnostic reference + observer seed + anticogging sanity gate. | Quantized at 4000 CPR to ~0.005 turn/s; never closes the loop. |
| 100 ms sliding window | Steady-state mechanical reference (steadiest). | ~0.0025 turn/s quantization; too slow for loop feedback. |
| M/T estimator | Low-speed edge diagnostic; hold-while-expected / decay-to-zero idle logic. | Slew-limited output; not a control source. |
| I clamp | Bounds stored integrator energy to the ABZ velocity integrator limit. | Prevents windup release impulses. |
| FrictionCompensator | Coulomb + static breakaway FF on the observer feedback. | Must not use the 100 ms window or raw PLL (too slow / too noisy). |
| ABZ count glitch counter | Diagnostic only; counts ticks whose |delta| exceeds 3x the expected counts per tick. | Never faults; feeds `abzCountGlitchCount` telemetry. |

## Position mode

`Controller::update()` produces `vel_des = vel_setpoint + position_gain * position_error` before the ABZ velocity stages. Incremental position control clamps `pos_gain` to 1.0–1.2. The low-speed compensator can activate in position mode after a 4-count error and deactivate at 2 counts; it injects a minimum 0.02 turn/s virtual command. Therefore position control can enter a low-speed estimator/compensation region even with zero explicit velocity feed-forward. This is appropriate to audit separately from direct velocity mode.

## Telemetry and current observability

`g` (fast telemetry, 56 fields) emits closed-loop feedback as `velocity`, raw PLL velocity, 50 ms window velocity, 100 ms window velocity, M/T velocity, control observer velocity, effective observer bandwidth, estimator disagreement, I torque, P torque, low-speed torque, pre/post ABZ torque limit, saturation, delta count, glitch count and the friction/anticogging state. `j` is slower aggregate status. USB is requested at 50 Hz by the host; it must not perform high-rate logging inside the current ISR.

## Emergency stop path

`x axis` in `ascii_protocol.cpp` clears controller inputs, calls `Controller::reset()`, clears current-control state, calls `safety_critical_disarm_motor_pwm()`, and requests Idle. `Motor::arm()` also resets controller and current-controller state before PWM arm. This covers the main transient states, but requires an integration test that demonstrates no trajectory, compensation or feed-forward survives re-arm.
