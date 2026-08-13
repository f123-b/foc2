#ifndef __LOW_SPEED_COMPENSATOR_HPP
#define __LOW_SPEED_COMPENSATOR_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

struct LowSpeedCompensationResult {
    float friction_torque = 0.0f;
    bool hold_integrator = false;
    uint8_t state = 0;
};

// Runtime-only breakaway compensation for cascaded ABZ control. The caller
// owns mode selection, so torque control, commutation and motor state are not
// visible to this class.
class LowSpeedCompensator {
public:
    enum State : uint8_t {
        STATE_IDLE = 0,
        STATE_RUNNING,
        STATE_BREAKAWAY,
        STATE_RECOVERING,
    };

    static constexpr float command_threshold = 0.02f;
    static constexpr float position_velocity_floor = 0.02f;
    static constexpr float integrator_hold_delay = 0.012f;
    static constexpr float stall_confirm_time = 0.08f;
    static constexpr float soft_breakaway_ramp_time = 0.20f;
    static constexpr float hard_breakaway_ramp_time = 0.40f;
    static constexpr float recovery_min_time = 0.04f;
    static constexpr int32_t recovery_progress_counts = 3;
    // Use a bounded, slew-limited breakaway ramp rather than a torque step;
    // the velocity loop supplies the remaining acceleration torque.
    static constexpr float running_torque = 0.0012f;
    static constexpr float soft_breakaway_torque = 0.0060f;
    static constexpr float breakaway_torque = 0.0120f;
    static constexpr float torque_rise_rate = 0.0180f;
    static constexpr float torque_fall_rate = 0.2500f;
    static constexpr float overspeed_fade_band = 0.08f;

    void clear() {
        state_ = STATE_IDLE;
        direction_ = 0.0f;
        no_progress_time_ = 0.0f;
        breakaway_time_ = 0.0f;
        recovery_time_ = 0.0f;
        compensation_magnitude_ = 0.0f;
        encoder_count_initialized_ = false;
        progress_count_ = 0;
        breakaway_start_count_ = 0;
    }

    LowSpeedCompensationResult update(bool active,
                                      float requested_direction,
                                      float command_velocity,
                                      float measured_velocity,
                                      float velocity_error,
                                      float integrator_torque,
                                      int32_t encoder_count,
                                      float period) {
        LowSpeedCompensationResult result;
        if (!active || !(period > 0.0f) ||
                !std::isfinite(requested_direction) ||
                !std::isfinite(command_velocity) ||
                !std::isfinite(measured_velocity) ||
                !std::isfinite(velocity_error)) {
            clear();
            return result;
        }

        const float direction = requested_direction > 0.0f ? 1.0f :
                requested_direction < 0.0f ? -1.0f : 0.0f;
        if (direction == 0.0f) {
            clear();
            return result;
        }
        if (direction_ != 0.0f && direction != direction_)
            clear();
        direction_ = direction;

        bool forward_progress = false;
        if (encoder_count_initialized_) {
            const int64_t delta = static_cast<int64_t>(encoder_count) -
                    static_cast<int64_t>(progress_count_);
            if (static_cast<float>(delta) * direction > 0.0f) {
                progress_count_ = encoder_count;
                forward_progress = true;
            }
        } else {
            encoder_count_initialized_ = true;
            progress_count_ = encoder_count;
        }

        if (forward_progress) {
            no_progress_time_ = 0.0f;
        } else {
            no_progress_time_ += period;
        }

        if (state_ == STATE_IDLE)
            state_ = STATE_RUNNING;
        if (state_ == STATE_BREAKAWAY) {
            breakaway_time_ += period;
            const int64_t breakaway_progress =
                    static_cast<int64_t>(progress_count_) -
                    static_cast<int64_t>(breakaway_start_count_);
            if (static_cast<float>(breakaway_progress) * direction >=
                    static_cast<float>(recovery_progress_counts)) {
                state_ = STATE_RECOVERING;
                recovery_time_ = 0.0f;
            }
        } else if ((state_ == STATE_RUNNING || state_ == STATE_RECOVERING) &&
                no_progress_time_ >= stall_confirm_time) {
            state_ = STATE_BREAKAWAY;
            breakaway_time_ = 0.0f;
            breakaway_start_count_ = progress_count_;
        }

        float target_magnitude = running_torque;
        if (state_ == STATE_BREAKAWAY) {
            const float soft_blend = std::clamp(
                    breakaway_time_ / soft_breakaway_ramp_time, 0.0f, 1.0f);
            const float hard_blend = std::clamp(
                    (breakaway_time_ - soft_breakaway_ramp_time) /
                            hard_breakaway_ramp_time,
                    0.0f, 1.0f);
            target_magnitude = running_torque + soft_blend *
                    (soft_breakaway_torque - running_torque) + hard_blend *
                    (breakaway_torque - soft_breakaway_torque);
        } else if (state_ == STATE_RECOVERING) {
            recovery_time_ += period;
        }

        // Remove feed-forward continuously once the rotor passes the command.
        const float direction_command = std::abs(command_velocity);
        const float forward_velocity = measured_velocity * direction;
        const float overspeed = std::max(
                0.0f, forward_velocity - direction_command);
        const float overspeed_ratio = std::clamp(
                overspeed / overspeed_fade_band, 0.0f, 1.0f);
        const float smooth_ratio = overspeed_ratio * overspeed_ratio *
                (3.0f - 2.0f * overspeed_ratio);
        target_magnitude *= 1.0f - smooth_ratio;

        compensation_magnitude_ = slew(
                compensation_magnitude_, target_magnitude,
                torque_rise_rate, torque_fall_rate, period);
        if (state_ == STATE_RECOVERING &&
                recovery_time_ >= recovery_min_time &&
                compensation_magnitude_ <= running_torque + 0.0001f) {
            state_ = STATE_RUNNING;
        }

        const float forward_error = velocity_error * direction;
        result.friction_torque = direction * compensation_magnitude_;
        result.hold_integrator =
                (state_ == STATE_BREAKAWAY || state_ == STATE_RECOVERING ||
                 no_progress_time_ >= integrator_hold_delay) &&
                integrator_torque * direction >= 0.0f && forward_error > 0.0f;
        result.state = static_cast<uint8_t>(state_);
        return result;
    }

    float compensation_magnitude() const { return compensation_magnitude_; }
    float no_progress_time() const { return no_progress_time_; }
    State state() const { return state_; }

private:
    static float slew(float value, float target, float rise_rate,
                      float fall_rate, float period) {
        const float rate = target >= value ? rise_rate : fall_rate;
        const float step = rate * period;
        return value + std::clamp(target - value, -step, step);
    }

    State state_ = STATE_IDLE;
    float direction_ = 0.0f;
    float no_progress_time_ = 0.0f;
    float breakaway_time_ = 0.0f;
    float recovery_time_ = 0.0f;
    float compensation_magnitude_ = 0.0f;
    bool encoder_count_initialized_ = false;
    int32_t progress_count_ = 0;
    int32_t breakaway_start_count_ = 0;
};

#endif // __LOW_SPEED_COMPENSATOR_HPP
