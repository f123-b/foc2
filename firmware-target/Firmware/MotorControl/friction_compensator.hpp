#ifndef __FRICTION_COMPENSATOR_HPP
#define __FRICTION_COMPENSATOR_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

struct FrictionCompensationResult {
    float friction_torque = 0.0f;
    uint8_t state = 0;
};

// Minimal Coulomb-friction feed-forward plus a static breakaway assist for
// the ABZ velocity loop.  It replaces the old LowSpeedCompensator state
// machine: there is exactly ONE integrator in the whole loop (the velocity PI
// with anti-windup), and this class only produces a bounded, slew-limited
// feed-forward.
//
//   RUNNING     -> output = sign(command) * coulomb_torque (faded near 0)
//   BREAKAWAY   -> output ramps toward sign(command) * breakaway_torque when
//                  the rotor is stationary with persistent positive error
//   RECOVERING  -> output ramps back to coulomb_torque after confirmed progress
//
// No speed-hold integrator, no error-assist term, no overspeed torque fade.
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
    // Slew rates [Nm/s].  Slow rise avoids a torque step into the detent;
    // fast fall unloads promptly once the rotor is moving.
    static constexpr float torque_rise_rate = 0.020f;
    static constexpr float torque_fall_rate = 0.60f;
    // Persistence required before engaging / disengaging breakaway.
    static constexpr float stall_confirm_time = 0.060f;
    static constexpr float recovery_confirm_time = 0.012f;
    // Confirmed encoder counts (in the command direction) before unloading.
    static constexpr int32_t recovery_progress_counts = 3;
    // Fraction of the commanded speed that counts as "confirmed moving".
    static constexpr float recovery_speed_ratio = 0.55f;

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
        recovery_confirm_timer_ = 0.0f;
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
            // Graceful ramp-to-zero: never step the output to zero in a single
            // control cycle. Hard clear() is reserved for fault / estop / exit
            // from closed loop.
            output_ = slew(output_, 0.0f, period);
            if (std::abs(output_) < 0.0001f) {
                clear();
            } else {
                state_ = STATE_IDLE;
                direction_ = 0.0f;
                no_progress_time_ = 0.0f;
                recovery_confirm_timer_ = 0.0f;
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

        // Coulomb FF, faded smoothly to zero as the command approaches zero.
        const float command_mag = std::abs(command_velocity);
        const float coulomb_blend = std::clamp(
                command_mag / command_threshold, 0.0f, 1.0f);
        float target = direction * coulomb_torque_ * coulomb_blend;

        if (state_ == STATE_IDLE)
            state_ = STATE_RUNNING;

        const float forward_error = velocity_error * direction;
        const bool persistent_stall = no_progress_time_ >= stall_confirm_time;
        const bool want_breakaway =
                forward_error > 0.0f && persistent_stall;

        if (state_ == STATE_RUNNING || state_ == STATE_RECOVERING) {
            if (want_breakaway) {
                state_ = STATE_BREAKAWAY;
                breakaway_start_count_ = progress_count_;
                recovery_confirm_timer_ = 0.0f;
            }
        } else if (state_ == STATE_BREAKAWAY) {
            if (want_breakaway) {
                // stay in breakaway, refresh the progress reference
                breakaway_start_count_ = progress_count_;
            }
            const int64_t breakaway_progress =
                    static_cast<int64_t>(progress_count_) -
                    static_cast<int64_t>(breakaway_start_count_);
            const bool progressed = static_cast<float>(breakaway_progress) *
                    direction >= static_cast<float>(recovery_progress_counts);
            const bool speed_confirmed =
                    measured_velocity * direction >=
                            recovery_speed_ratio * command_mag;
            recovery_confirm_timer_ = (progressed && speed_confirmed)
                    ? recovery_confirm_timer_ + period : 0.0f;
            if (recovery_confirm_timer_ >= recovery_confirm_time) {
                state_ = STATE_RECOVERING;
                recovery_confirm_timer_ = 0.0f;
            }
        }

        if (state_ == STATE_BREAKAWAY)
            target = direction * breakaway_torque_;

        output_ = slew(output_, target, period);

        // Once the output has slewed back to the Coulomb level, go RUNNING.
        if (state_ == STATE_RECOVERING &&
                std::abs(output_ - direction * coulomb_torque_ * coulomb_blend) <= 0.0001f) {
            state_ = STATE_RUNNING;
        }

        result.friction_torque = output_;
        result.state = static_cast<uint8_t>(state_);
        return result;
    }

    float friction_torque() const { return output_; }
    State state() const { return state_; }

private:
    float slew(float value, float target, float period) {
        // Rise/fall is selected by torque MAGNITUDE so +direction and
        // -direction behave symmetrically (the old target>=value comparison
        // used the fall rate for every negative-direction move).
        const float rate = std::abs(target) >= std::abs(value)
                ? torque_rise_rate : torque_fall_rate;
        const float step = rate * period;
        return value + std::clamp(target - value, -step, step);
    }

    State state_ = STATE_IDLE;
    float direction_ = 0.0f;
    float no_progress_time_ = 0.0f;
    float recovery_confirm_timer_ = 0.0f;
    float output_ = 0.0f;
    bool initialized_ = false;
    int32_t progress_count_ = 0;
    int32_t breakaway_start_count_ = 0;
    float coulomb_torque_ = 0.0015f;
    float breakaway_torque_ = 0.0055f;
};

#endif // __FRICTION_COMPENSATOR_HPP
