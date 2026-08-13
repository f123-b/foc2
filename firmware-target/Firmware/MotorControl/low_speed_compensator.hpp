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
    static constexpr float integrator_hold_delay = 0.008f;
    static constexpr float stall_confirm_time = 0.060f;
    // Ramp through static friction without a hard torque step.  The larger
    // final value is needed to start this motor below 1 turn/s.
    static constexpr float soft_breakaway_ramp_time = 0.12f;
    static constexpr float hard_breakaway_ramp_time = 0.24f;
    static constexpr float recovery_min_time = 0.025f;
    static constexpr int32_t recovery_progress_counts = 3;
    // Encoder edges alone are not enough to declare breakaway complete:
    // low-speed ABZ feedback can advance a few counts while the rotor is
    // still below the commanded velocity.  Require a short dwell at a
    // meaningful fraction of the command before unloading the assist.
    static constexpr float recovery_speed_ratio = 0.55f;
    static constexpr float recovery_speed_confirm_time = 0.012f;
    // Use a bounded, slew-limited breakaway ramp rather than a torque step;
    // the velocity loop supplies the remaining acceleration torque.
    // Keep enough torque after breakaway to stay above static friction at
    // 0.2--1.0 turn/s; otherwise the rotor stops again as soon as the ramp
    // enters recovery.
    static constexpr float running_torque = 0.0040f;
    static constexpr float soft_breakaway_torque = 0.0085f;
    // The nominal ceiling is retained at and above 1 turn/s.  Below that
    // point static friction dominates, so allow a little more bounded torque
    // to cross a tooth without raising the current limit for the whole speed
    // range.  The controller multiplies this by its low-speed fade below 2
    // turn/s, so the additional torque is confined to the actual breakaway
    // region.
    static constexpr float breakaway_torque = 0.0180f;
    static constexpr float nominal_breakaway_torque = 0.0140f;
    static constexpr float torque_rise_rate = 0.0180f;
    static constexpr float torque_fall_rate = 0.60f;
    static constexpr float overspeed_fade_band = 0.12f;
    static constexpr float error_assist_gain = 0.020f;
    static constexpr float overspeed_confirm_time = 0.003f;
    // A bounded low-speed hold term replaces the normal velocity integrator,
    // which is intentionally clamped in the sparse-count region.  It builds
    // slowly from persistent positive error and releases quickly on overspeed.
    static constexpr float speed_hold_gain = 0.12f;
    static constexpr float speed_hold_decay = 0.60f;
    static constexpr float speed_hold_limit = 0.0090f;

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
        recovery_ready_time_ = 0.0f;
        overspeed_time_ = 0.0f;
        speed_hold_torque_ = 0.0f;
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
        const bool direction_changed = direction_ != 0.0f && direction != direction_;
        if (direction_changed)
            clear();
        direction_ = direction;
        // Do not carry the previous direction's compensation through a
        // reversal.  The next control tick starts from the new running torque.
        if (direction_changed)
            return result;

        bool forward_progress = false;
        if (encoder_count_initialized_) {
            const int64_t delta = static_cast<int64_t>(encoder_count) -
                    static_cast<int64_t>(progress_count_);
            // Ignore isolated ABZ edges while the rotor is stuck.  They are
            // commonly contact bounce/electrical dither and must not reset
            // the stall timer before the breakaway ramp can build torque.
            if (static_cast<float>(delta) * direction >=
                    static_cast<float>(recovery_progress_counts)) {
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
        const float direction_command = std::abs(command_velocity);
        const float forward_velocity = measured_velocity * direction;
        const bool recovery_speed_ready = direction_command <= command_threshold ||
                forward_velocity >= recovery_speed_ratio * direction_command;
        if (state_ == STATE_BREAKAWAY) {
            breakaway_time_ += period;
            const int64_t breakaway_progress =
                    static_cast<int64_t>(progress_count_) -
                    static_cast<int64_t>(breakaway_start_count_);
            if (static_cast<float>(breakaway_progress) * direction >=
                    static_cast<float>(recovery_progress_counts) &&
                    recovery_speed_ready) {
                recovery_ready_time_ = std::min(
                        recovery_ready_time_ + period,
                        recovery_speed_confirm_time);
            } else {
                recovery_ready_time_ = 0.0f;
            }
            if (recovery_ready_time_ >= recovery_speed_confirm_time) {
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

        const float forward_error = velocity_error * direction;
        // A few encoder counts can be real motion that is still below the
        // requested speed.  Keep building torque from the positive velocity
        // error instead of treating those counts as proof that breakaway is
        // complete.  The assist is bounded by the same breakaway ceiling.
        if (state_ == STATE_RUNNING || state_ == STATE_RECOVERING) {
            if (forward_error > 0.01f) {
                speed_hold_torque_ += speed_hold_gain * forward_error * period;
            } else if (forward_error < -0.02f) {
                speed_hold_torque_ -= speed_hold_decay * period;
            }
            speed_hold_torque_ = std::clamp(
                    speed_hold_torque_, 0.0f, speed_hold_limit);
            const float positive_error = std::max(0.0f, forward_error);
            target_magnitude += std::min(
                    breakaway_torque - running_torque,
                    error_assist_gain * positive_error);
            target_magnitude += speed_hold_torque_;
        }
        const float low_speed_ceiling = nominal_breakaway_torque +
                std::clamp((1.0f - direction_command) / 0.5f, 0.0f, 1.0f) *
                (breakaway_torque - nominal_breakaway_torque);
        target_magnitude = std::min(target_magnitude, low_speed_ceiling);

        // Remove feed-forward continuously once the rotor passes the command.
        const float overspeed = std::max(
                0.0f, forward_velocity - direction_command);
        if (overspeed > overspeed_fade_band) {
            overspeed_time_ = std::min(
                    overspeed_time_ + period, overspeed_confirm_time);
        } else {
            overspeed_time_ = std::max(0.0f, overspeed_time_ - 4.0f * period);
        }
        const float confirmed_overspeed = overspeed_time_ >=
                overspeed_confirm_time ? overspeed : 0.0f;
        const float overspeed_ratio = std::clamp(
                confirmed_overspeed / overspeed_fade_band, 0.0f, 1.0f);
        const float smooth_ratio = overspeed_ratio * overspeed_ratio *
                (3.0f - 2.0f * overspeed_ratio);
        target_magnitude *= 1.0f - smooth_ratio;

        compensation_magnitude_ = slew(
                compensation_magnitude_, target_magnitude,
                torque_rise_rate, torque_fall_rate, period);
        const float running_target = running_torque + speed_hold_torque_;
        if (state_ == STATE_RECOVERING &&
                recovery_time_ >= recovery_min_time &&
                compensation_magnitude_ <= running_target + 0.0001f) {
                state_ = STATE_RUNNING;
        }

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
    float recovery_ready_time_ = 0.0f;
    float overspeed_time_ = 0.0f;
    float speed_hold_torque_ = 0.0f;
};

#endif // __LOW_SPEED_COMPENSATOR_HPP
