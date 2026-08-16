#ifndef __ABZ_VELOCITY_OBSERVER_HPP
#define __ABZ_VELOCITY_OBSERVER_HPP

#include <algorithm>
#include <cmath>

// ABZ mechanical velocity observer.  It is the SINGLE velocity feedback source
// of the ABZ velocity PI: the encoder PLL keeps driving commutation, phase
// interpolation and the safety checks; this observer only closes the
// mechanical speed loop.
//
//   per-tick delta position (turn) -> [ kp, ki ] -> velocity (turn/s)
//
// Design notes
// ------------
// * Incremental input: the observer consumes `delta_count / CPR` every control
//   tick and integrates it into a local frame.  Unlike the previous
//   implementation (which was fed `shadow_count_ / CPR`), it is immune to
//   int32 shadow-count overflow and never lets a growing absolute float
//   position eat the count-level resolution: the local frame is rebased back
//   to zero every few turns so the state always stays small.
//
// * Adaptive bandwidth: a single fixed bandwidth (previously 40 Hz everywhere)
//   was a compromise between low-speed noise rejection and high-speed
//   tracking.  The bandwidth now follows the commanded speed with a smooth
//   (piecewise linear, no steps) schedule:
//
//       |commanded velocity| (turn/s)   observer bandwidth (Hz)
//       ------------------------------------------------
//       0                              min (default 15)
//       0.3                            ~20
//       1.0                            ~30
//       2.0                            ~40
//       4.0+                           max (default 50)
//
//   The schedule is clamped to [min_bandwidth_hz, max_bandwidth_hz].
//
// * Gain-recompute throttling: the gains are a function of the bandwidth
//   (critically damped pair).  They are only recomputed when the bandwidth
//   actually changes by more than a small epsilon, so the 8 kHz control tick
//   does not redo the 2*pi / multiply / square work every cycle.
//
// The observer is a critically damped position PLL:
//
//   omega_n = 2*pi*bandwidth
//   kp      = 2*omega_n
//   ki      = 0.25*kp^2   (critically damped)
class AbzVelocityObserver {
public:
    // Default adaptive schedule (Hz) as a function of |commanded velocity| in
    // turn/s.  These are the recommended starting points from the ABZ
    // estimation spec; the endpoints are clamped by the runtime min/max
    // configuration.
    static constexpr float kScheduleVel[5]  = {0.0f, 0.3f, 1.0f, 2.0f, 4.0f};
    static constexpr float kScheduleBw[5]   = {15.0f, 20.0f, 30.0f, 40.0f, 50.0f};
    static constexpr float kBandwidthEpsilonHz = 0.5f;  // gain-recompute deadband
    static constexpr float kRebaseThresholdTurn = 8.0f; // local frame rebase point
    static constexpr float kMinBandwidthHz = 5.0f;      // hard safety floor
    static constexpr float kMaxBandwidthHz = 200.0f;    // hard safety ceiling

    AbzVelocityObserver() {
        recompute_gains(kScheduleBw[0]);
    }

    // Set the runtime bandwidth bounds.  Only the bounds are stored here; the
    // effective bandwidth is applied per tick through set_bandwidth().  This
    // is called on the 8 kHz control hot path, so it is a cached no-op unless
    // a bound actually changed: the normal tick only does bandwidth_for() +
    // set_bandwidth() + update().
    void configure(float min_bandwidth_hz, float max_bandwidth_hz) {
        const float min_bw = std::isfinite(min_bandwidth_hz)
                ? std::clamp(min_bandwidth_hz, kMinBandwidthHz, kMaxBandwidthHz)
                : kScheduleBw[0];
        const float max_bw = std::isfinite(max_bandwidth_hz)
                ? std::clamp(max_bandwidth_hz, min_bw, kMaxBandwidthHz)
                : std::max(min_bw, kScheduleBw[4]);
        if (min_bw == min_bw_ && max_bw == max_bw_)
            return;  // cached: bounds unchanged, nothing to recompute
        min_bw_ = min_bw;
        max_bw_ = max_bw;
    }

    // Map |commanded velocity| (turn/s) to a bandwidth (Hz) using the smooth
    // piecewise-linear schedule above, clamped to the configured bounds.
    // No divisions: the segment slopes are compile-time constants.
    float bandwidth_for(float abs_command_velocity) const {
        const float v = std::fabs(abs_command_velocity);
        float bw;
        if (v <= kScheduleVel[1]) {
            bw = kScheduleBw[0] + v * ((kScheduleBw[1] - kScheduleBw[0]) / kScheduleVel[1]);
        } else if (v <= kScheduleVel[2]) {
            bw = kScheduleBw[1] + (v - kScheduleVel[1]) *
                    ((kScheduleBw[2] - kScheduleBw[1]) / (kScheduleVel[2] - kScheduleVel[1]));
        } else if (v <= kScheduleVel[3]) {
            bw = kScheduleBw[2] + (v - kScheduleVel[2]) *
                    ((kScheduleBw[3] - kScheduleBw[2]) / (kScheduleVel[3] - kScheduleVel[2]));
        } else if (v <= kScheduleVel[4]) {
            bw = kScheduleBw[3] + (v - kScheduleVel[3]) *
                    ((kScheduleBw[4] - kScheduleBw[3]) / (kScheduleVel[4] - kScheduleVel[3]));
        } else {
            bw = kScheduleBw[4];
        }
        return std::clamp(bw, min_bw_, max_bw_);
    }

    // Apply the bandwidth for the current command.  Gains are only recomputed
    // when the bandwidth moves by more than kBandwidthEpsilonHz, so this is
    // cheap on a steady command.
    void set_bandwidth(float bandwidth_hz) {
        const float bw = std::isfinite(bandwidth_hz)
                ? std::clamp(bandwidth_hz, min_bw_, max_bw_) : min_bw_;
        if (std::fabs(bw - bandwidth_) > kBandwidthEpsilonHz)
            recompute_gains(bw);
    }

    // Initialize (or re-initialize) the observer while the rotor may be
    // moving.  `position_turn` is the current mechanical position in the
    // caller's frame (used only as the local-frame origin and immediately
    // rebased); `initial_velocity` seeds vel_hat_ so switching into velocity
    // mode while spinning does not start the loop from zero and create a
    // torque impulse.
    void reset(float position_turn, float initial_velocity) {
        const float pos = std::isfinite(position_turn) ? position_turn : 0.0f;
        pos_meas_ = pos;
        pos_hat_ = pos;
        vel_hat_ = std::isfinite(initial_velocity) ? initial_velocity : 0.0f;
        rebase_if_needed();
        initialized_ = true;
    }

    // One control-tick update.  `delta_position_turn` is the per-tick
    // incremental mechanical position (delta_count / CPR); `dt` the control
    // period.  Returns the smoothed velocity in turn/s.
    float update(float delta_position_turn, float dt) {
        if (!(dt > 0.0f) || !std::isfinite(delta_position_turn)) {
            reset(0.0f, 0.0f);
            return 0.0f;
        }
        if (!initialized_) {
            reset(0.0f, 0.0f);
            return 0.0f;
        }
        // Integrate the incremental measurement into the local frame.
        pos_meas_ += delta_position_turn;
        rebase_if_needed();
        // Predict position ahead by the current velocity estimate.
        pos_hat_ += dt * vel_hat_;
        // Correct from the measured position.
        const float pos_err = pos_meas_ - pos_hat_;
        pos_hat_ += dt * kp_ * pos_err;
        vel_hat_ += dt * ki_ * pos_err;
        return vel_hat_;
    }

    float velocity() const { return vel_hat_; }
    float position() const { return pos_hat_; }
    float bandwidth() const { return bandwidth_; }
    bool initialized() const { return initialized_; }

private:
    void recompute_gains(float bandwidth_hz) {
        bandwidth_ = bandwidth_hz;
        constexpr float two_pi = 6.28318530717958647692f;
        const float omega = two_pi * std::max(0.1f, bandwidth_hz);
        kp_ = 2.0f * omega;        // [1/s]
        ki_ = 0.25f * kp_ * kp_;   // [1/s^2]
    }

    // Keep the local frame bounded so float32 never loses count resolution:
    // shift both measured and estimated position by the same amount (the
    // innovation, and therefore the velocity, is unchanged).
    void rebase_if_needed() {
        if (std::fabs(pos_meas_) <= kRebaseThresholdTurn)
            return;
        const float shift = pos_meas_;
        pos_meas_ -= shift;
        pos_hat_ -= shift;
    }

    float kp_ = 0.0f;            // [1/s]
    float ki_ = 0.0f;            // [1/s^2]
    float bandwidth_ = kScheduleBw[0];
    float min_bw_ = kScheduleBw[0];
    float max_bw_ = kScheduleBw[4];
    float pos_meas_ = 0.0f;      // local-frame measured position [turn]
    float pos_hat_ = 0.0f;       // local-frame estimated position [turn]
    float vel_hat_ = 0.0f;       // [turn/s]
    bool initialized_ = false;
};

#endif // __ABZ_VELOCITY_OBSERVER_HPP
