#ifndef __FRICTION_COMPENSATOR_HPP
#define __FRICTION_COMPENSATOR_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

struct FrictionCompensationResult {
    float friction_torque = 0.0f;   // post-slew output
    float target_torque = 0.0f;     // pre-slew target
    float speed_ratio = 0.0f;       // directional speed ratio [0,1]
    float assist_blend = 0.0f;      // (1 - speed_ratio)^2
    float no_progress_time = 0.0f;  // [s] since last confirmed forward progress
    float recovery_timer = 0.0f;    // [s] sustained directional ratio >= threshold
    float forward_velocity = 0.0f;  // measured_velocity * direction [turn/s]
    bool reverse_detected = false;  // forward_velocity < -reverse threshold
    uint8_t state = 0;
};

// Minimal Coulomb-friction feed-forward plus a static breakaway assist for
// the ABZ velocity loop.  Exactly ONE integrator exists in the whole loop (the
// velocity PI with anti-windup); this class only produces a bounded,
// slew-limited feed-forward.
//
//   RUNNING     -> output = sign(command) * coulomb (faded near 0)
//   BREAKAWAY   -> output ramps up toward sign(command) * breakaway while the
//                  rotor is stationary with persistent positive error
//   RECOVERING  -> output smoothly blends from breakaway toward coulomb as the
//                  directional speed approaches the command (no torque step)
//
// RECOVERING is held until the DIRECTIONAL speed (measured projected onto the
// command direction, never abs(speed)) has stayed above a fraction of the
// command for a dwell time. A re-stall or reverse motion raises the assist
// smoothly instead of cycling RUNNING -> BREAKAWAY -> RUNNING.
class FrictionCompensator {
public:
    enum State : uint8_t {
        STATE_IDLE = 0,
        STATE_RUNNING,
        STATE_BREAKAWAY,
        STATE_RECOVERING,
    };

    // Command magnitude below which the feed-forward fades to zero [turn/s].
    static constexpr float command_threshold = 0.02f;
    // Initial breakaway ramp-up (slow, avoids a torque step into the detent).
    static constexpr float breakaway_rise_rate = 0.020f;      // Nm/s
    // Re-engage assist quickly when speed drops during RECOVERING.
    static constexpr float assist_reengage_rate = 0.080f;     // Nm/s
    // Smooth release toward coulomb as speed approaches the command.
    static constexpr float recovery_release_rate = 0.020f;    // Nm/s
    // Fast ramp-to-zero only for command=0 / disable / closed-loop exit.
    static constexpr float disable_fall_rate = 0.60f;         // Nm/s
    // Persistence required before engaging breakaway from RUNNING.
    static constexpr float stall_confirm_time = 0.060f;
    // Confirmed encoder counts (in the command direction) before leaving
    // BREAKAWAY for RECOVERING.
    static constexpr int32_t recovery_progress_counts = 3;
    // Directional speed ratio above which RECOVERING may finish.
    static constexpr float recovery_enter_running_ratio = 0.85f;
    // Dwell time at/above that ratio before RECOVERING -> RUNNING.
    static constexpr float recovery_hold_time = 0.090f;
    // Denominator floor for the speed ratio (avoids division by zero).
    static constexpr float minimum_speed = 0.05f;
    // Forward velocity below which the rotor is considered "moving reverse"
    // (kept away from zero to reject encoder noise).
    static constexpr float reverse_velocity_threshold = 0.02f;

    // Runtime torque amplitudes, set from the controller config each control
    // cycle. breakaway >= coulomb is enforced here so illegal parameters
    // degrade safely (never produce NaN or a negative feed-forward).
    void configure(float coulomb_torque, float breakaway_torque) {
        coulomb_torque_ = std::max(0.0f,
                std::isfinite(coulomb_torque) ? coulomb_torque : 0.0f);
        breakaway_torque_ = std::max(coulomb_torque_,
                std::isfinite(breakaway_torque) ? breakaway_torque : coulomb_torque_);
    }

    float coulomb_torque() const { return coulomb_torque_; }
    float breakaway_torque() const { return breakaway_torque_; }

    void clear() {
        state_ = STATE_IDLE;
        direction_ = 0.0f;
        no_progress_time_ = 0.0f;
        recovery_hold_timer_ = 0.0f;
        output_ = 0.0f;
        initialized_ = false;
        progress_count_ = 0;
        breakaway_start_count_ = 0;
    }

    // Graceful ramp-to-zero for a normal disable (config toggle / zero
    // command). Hard clear() is reserved for fault / estop / closed-loop exit.
    float disable(float period) {
        const FrictionCompensationResult r =
                update(false, 0.0f, 0.0f, 0.0f, 0, period);
        return r.friction_torque;
    }

    FrictionCompensationResult update(bool active,
                                      float command_velocity,
                                      float measured_velocity,
                                      float velocity_error,
                                      int32_t encoder_count,
                                      float period) {
        FrictionCompensationResult result;
        const bool bad_input = !std::isfinite(command_velocity) ||
                !std::isfinite(measured_velocity) ||
                !std::isfinite(velocity_error);
        if (bad_input || !(period > 0.0f)) {
            // No valid time step to slew with: only a hard reset is possible.
            clear();
            result.friction_torque = 0.0f;
            result.state = state_;
            return result;
        }
        const float direction = command_velocity > 0.0f ? 1.0f :
                command_velocity < 0.0f ? -1.0f : 0.0f;
        if (!active || direction == 0.0f) {
            // Fast graceful ramp-to-zero (command zero / disabled / exit).
            output_ = slew(output_, 0.0f, disable_fall_rate, period);
            if (std::abs(output_) < 0.0001f) {
                clear();
            } else {
                state_ = STATE_IDLE;
                direction_ = 0.0f;
                no_progress_time_ = 0.0f;
                recovery_hold_timer_ = 0.0f;
                initialized_ = false;
            }
            result.friction_torque = output_;
            result.state = state_;
            return result;
        }

        const bool direction_changed =
                direction_ != 0.0f && direction != direction_;
        if (direction_changed) {
            clear();
            direction_ = direction;
            initialized_ = true;
            progress_count_ = encoder_count;
        } else if (!initialized_) {
            initialized_ = true;
            direction_ = direction;
            progress_count_ = encoder_count;
        }

        // Confirmed forward progress in the command direction.  Isolated
        // single-count dither must not reset the stall timer.
        bool forward_progress = false;
        const int64_t delta = static_cast<int64_t>(encoder_count) -
                static_cast<int64_t>(progress_count_);
        if (static_cast<float>(delta) * direction >=
                static_cast<float>(recovery_progress_counts)) {
            progress_count_ = encoder_count;
            forward_progress = true;
        }
        no_progress_time_ = forward_progress ? 0.0f : no_progress_time_ + period;

        const float command_mag = std::abs(command_velocity);
        const float coulomb_blend = std::clamp(
                command_mag / command_threshold, 0.0f, 1.0f);
        const float coulomb_target = direction * coulomb_torque_ * coulomb_blend;

        // Directional speed ratio: the measured speed PROJECTED onto the
        // command direction. Reverse motion yields 0 (never abs(speed)).
        const float forward_velocity = measured_velocity * direction;
        const float speed_ratio = std::clamp(
                forward_velocity / std::max(command_mag, minimum_speed),
                0.0f, 1.0f);
        const bool moving_reverse =
                forward_velocity < -reverse_velocity_threshold;
        const float assist_blend = (1.0f - speed_ratio) * (1.0f - speed_ratio);

        if (state_ == STATE_IDLE)
            state_ = STATE_RUNNING;

        const float forward_error = velocity_error * direction;
        const bool persistent_stall = no_progress_time_ >= stall_confirm_time;

        float target = coulomb_target;
        if (state_ == STATE_RUNNING) {
            if (forward_error > 0.0f && persistent_stall) {
                state_ = STATE_BREAKAWAY;
                breakaway_start_count_ = progress_count_;
                recovery_hold_timer_ = 0.0f;
            }
        } else if (state_ == STATE_BREAKAWAY) {
            target = direction * breakaway_torque_;
            const int64_t breakaway_progress =
                    static_cast<int64_t>(progress_count_) -
                    static_cast<int64_t>(breakaway_start_count_);
            const bool progressed = static_cast<float>(breakaway_progress) *
                    direction >= static_cast<float>(recovery_progress_counts);
            if (progressed) {
                state_ = STATE_RECOVERING;
                recovery_hold_timer_ = 0.0f;
            }
        } else if (state_ == STATE_RECOVERING) {
            // Continuous blend from breakaway (speed 0 or reverse) to coulomb
            // (directional speed reaches command). A re-stall or reverse
            // motion raises the assist smoothly with no stall_confirm wait.
            target = direction * (coulomb_torque_ +
                    assist_blend * (breakaway_torque_ - coulomb_torque_));
            // Only forward speed accumulates the RUNNING dwell; reverse motion
            // must reset it (directional_speed_ratio is already 0 there).
            if (speed_ratio >= recovery_enter_running_ratio) {
                recovery_hold_timer_ += period;
            } else {
                recovery_hold_timer_ = 0.0f;
            }
            if (recovery_hold_timer_ >= recovery_hold_time) {
                state_ = STATE_RUNNING;
            }
        }

        // Slew-rate selection.
        float rate = breakaway_rise_rate;
        if (state_ == STATE_BREAKAWAY) {
            rate = breakaway_rise_rate;
        } else if (state_ == STATE_RECOVERING) {
            const bool increasing = std::abs(target) >= std::abs(output_);
            rate = increasing ? assist_reengage_rate : recovery_release_rate;
        } else {  // RUNNING
            const bool increasing = std::abs(target) >= std::abs(output_);
            rate = increasing ? breakaway_rise_rate : recovery_release_rate;
        }

        output_ = slew(output_, target, rate, period);

        result.friction_torque = output_;
        result.target_torque = target;
        result.speed_ratio = speed_ratio;
        result.assist_blend = assist_blend;
        result.no_progress_time = no_progress_time_;
        result.recovery_timer = recovery_hold_timer_;
        result.forward_velocity = forward_velocity;
        result.reverse_detected = moving_reverse;
        result.state = static_cast<uint8_t>(state_);
        return result;
    }

    float friction_torque() const { return output_; }
    State state() const { return state_; }

private:
    float slew(float value, float target, float rate, float period) {
        const float step = rate * period;
        return value + std::clamp(target - value, -step, step);
    }

    State state_ = STATE_IDLE;
    float direction_ = 0.0f;
    float no_progress_time_ = 0.0f;
    float recovery_hold_timer_ = 0.0f;
    float output_ = 0.0f;
    bool initialized_ = false;
    int32_t progress_count_ = 0;
    int32_t breakaway_start_count_ = 0;
    float coulomb_torque_ = 0.0015f;
    float breakaway_torque_ = 0.0055f;
};

#endif // __FRICTION_COMPENSATOR_HPP
