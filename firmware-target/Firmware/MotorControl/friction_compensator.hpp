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
// the ABZ velocity loop.  All tuning values are runtime-configurable via
// configure(); there is exactly ONE integrator in the loop (the velocity PI
// with anti-windup).
class FrictionCompensator {
public:
    enum State : uint8_t {
        STATE_IDLE = 0,
        STATE_RUNNING,
        STATE_BREAKAWAY,
        STATE_RECOVERING,
    };

    // Confirmed encoder counts (in the command direction) before leaving
    // BREAKAWAY for RECOVERING. Kept compile-time (small, ties to count logic).
    static constexpr int32_t recovery_progress_counts = 3;

    // Runtime configuration with safe clamping (no NaN, no negative).
    void configure(float coulomb_torque, float breakaway_torque,
                   float command_threshold, float stall_confirm_time,
                   float recovery_speed_ratio, float recovery_confirm_time,
                   float stall_velocity_threshold,
                   float reverse_velocity_threshold,
                   float breakaway_rise_rate, float assist_reengage_rate,
                   float recovery_release_rate, float disable_fall_rate) {
        coulomb_torque_ = clamp_nonneg(coulomb_torque, 0.0f);
        breakaway_torque_ = std::max(coulomb_torque_, clamp_nonneg(breakaway_torque, coulomb_torque_));
        command_threshold_ = clamp_pos(command_threshold, 0.02f);
        stall_confirm_time_ = clamp_nonneg(stall_confirm_time, 0.0f);
        recovery_speed_ratio_ = std::clamp(recovery_speed_ratio, 0.5f, 1.0f);
        recovery_confirm_time_ = clamp_nonneg(recovery_confirm_time, 0.0f);
        stall_velocity_threshold_ = clamp_pos(stall_velocity_threshold, 0.01f);
        reverse_velocity_threshold_ = clamp_nonneg(reverse_velocity_threshold, 0.0f);
        breakaway_rise_rate_ = clamp_nonneg(breakaway_rise_rate, 0.0f);
        assist_reengage_rate_ = clamp_nonneg(assist_reengage_rate, 0.0f);
        recovery_release_rate_ = clamp_nonneg(recovery_release_rate, 0.0f);
        disable_fall_rate_ = clamp_nonneg(disable_fall_rate, 0.0f);
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
            clear();
            result.friction_torque = 0.0f;
            result.state = state_;
            return result;
        }
        const float direction = command_velocity > 0.0f ? 1.0f :
                command_velocity < 0.0f ? -1.0f : 0.0f;
        if (!active || direction == 0.0f) {
            output_ = slew(output_, 0.0f, disable_fall_rate_, period);
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
                command_mag / command_threshold_, 0.0f, 1.0f);
        const float coulomb_target = direction * coulomb_torque_ * coulomb_blend;

        const float forward_velocity = measured_velocity * direction;
        const float speed_ratio = std::clamp(
                forward_velocity / std::max(command_mag, stall_velocity_threshold_),
                0.0f, 1.0f);
        const bool moving_reverse =
                forward_velocity < -reverse_velocity_threshold_;
        const float assist_blend = (1.0f - speed_ratio) * (1.0f - speed_ratio);

        if (state_ == STATE_IDLE)
            state_ = STATE_RUNNING;

        const float forward_error = velocity_error * direction;
        const bool persistent_stall = no_progress_time_ >= stall_confirm_time_;

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
            target = direction * (coulomb_torque_ +
                    assist_blend * (breakaway_torque_ - coulomb_torque_));
            if (speed_ratio >= recovery_speed_ratio_) {
                recovery_hold_timer_ += period;
            } else {
                recovery_hold_timer_ = 0.0f;
            }
            if (recovery_hold_timer_ >= recovery_confirm_time_) {
                state_ = STATE_RUNNING;
            }
        }

        float rate = breakaway_rise_rate_;
        if (state_ == STATE_BREAKAWAY) {
            rate = breakaway_rise_rate_;
        } else if (state_ == STATE_RECOVERING) {
            const bool increasing = std::abs(target) >= std::abs(output_);
            rate = increasing ? assist_reengage_rate_ : recovery_release_rate_;
        } else {
            const bool increasing = std::abs(target) >= std::abs(output_);
            rate = increasing ? breakaway_rise_rate_ : recovery_release_rate_;
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
    static float clamp_nonneg(float v, float fallback) {
        return std::isfinite(v) ? std::max(0.0f, v) : fallback;
    }
    static float clamp_pos(float v, float fallback) {
        return (std::isfinite(v) && v > 0.0f) ? v : fallback;
    }
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
    // Runtime configuration (defaults match the previous hardcoded values).
    float coulomb_torque_ = 0.0015f;
    float breakaway_torque_ = 0.0055f;
    float command_threshold_ = 0.02f;
    float stall_confirm_time_ = 0.06f;
    float recovery_speed_ratio_ = 0.85f;
    float recovery_confirm_time_ = 0.09f;
    float stall_velocity_threshold_ = 0.05f;
    float reverse_velocity_threshold_ = 0.02f;
    float breakaway_rise_rate_ = 0.020f;
    float assist_reengage_rate_ = 0.080f;
    float recovery_release_rate_ = 0.020f;
    float disable_fall_rate_ = 0.60f;
};

#endif // __FRICTION_COMPENSATOR_HPP
