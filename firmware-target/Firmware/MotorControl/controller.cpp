
#include "odrive_main.h"
#include <algorithm>

// FOC Studio uses one characterised ABZ velocity profile for both normal
// speed commands and the cogging-map scan.  Keeping these values together is
// important: a map measured at a different speed contains the velocity-loop
// transient and cannot be reused as a feed-forward at the requested speed.
static constexpr float FOC_STUDIO_ABZ_SPEED = 2.0f;       // turn/s
static constexpr float FOC_STUDIO_ABZ_RAMP_RATE = 0.3f;   // turn/s^2

Controller::Controller(Config_t& config) :
    config_(config)
{
    update_filter_gains();
}

void Controller::reset(bool abort_anticogging) {
    if (abort_anticogging && config_.anticogging.calib_anticogging) {
        config_.anticogging.calib_anticogging = false;
        config_.anticogging.index = 0;
        anticogging_sample_sum_ = 0.0f;
        anticogging_sample_count_ = 0;
        anticogging_map_sum_ = 0.0f;
        anticogging_valid_bin_count_ = 0;
        anticogging_sample_position_valid_ = false;
        anticogging_scan_phase_ = ANTICOGGING_SCAN_IDLE;
    }
    pos_setpoint_ = 0.0f;
    vel_setpoint_ = 0.0f;
    vel_integrator_torque_ = 0.0f;
    torque_setpoint_ = 0.0f;
    abz_velocity_feedback_filter_.clear();
    velocity_control_feedback_ = 0.0f;
    velocity_control_feedback_valid_ = false;
    overspeed_violation_count_ = 0;
    velocity_loop_torque_ = 0.0f;
    low_speed_compensator_.clear();
    low_speed_friction_torque_ = 0.0f;
    low_speed_compensator_state_ = LowSpeedCompensator::STATE_IDLE;
    position_error_ = 0.0f;
    position_low_speed_active_ = false;
}

void Controller::set_error(Error error) {
    error_ |= error;
    axis_->error_ |= Axis::ERROR_CONTROLLER_FAILED;
}

float Controller::velocity_feedback_for_control(float raw_velocity,
                                                float commanded_velocity) const {
    const bool cascaded_abz_mode =
            (config_.control_mode == CONTROL_MODE_VELOCITY_CONTROL ||
             config_.control_mode == CONTROL_MODE_POSITION_CONTROL) &&
            axis_ && axis_->encoder_.mode_ == Encoder::MODE_INCREMENTAL;
    if (!cascaded_abz_mode ||
            !axis_->encoder_.incremental_window_velocity_valid_) {
        return raw_velocity;
    }

    // At low speed the count window avoids the PLL's zero-speed dead band. At
    // higher speed its short horizon exposes position-synchronous ripple and
    // makes the velocity P term chase it. Blend by commanded speed so the
    // estimator selection cannot switch back and forth with feedback noise.
    // The ABZ count-window is less sensitive to single PLL phase impulses in
    // the 0.5--3 turn/s range used by the product. Keep it dominant there and
    // only blend toward the PLL once the count stream is dense enough.
    constexpr float pll_blend_start = 2.50f;
    constexpr float pll_blend_end = 4.00f;
    const float pll_weight = std::clamp(
            (std::abs(commanded_velocity) - pll_blend_start) /
                    (pll_blend_end - pll_blend_start),
            0.0f, 1.0f);
    const float window_velocity = axis_->encoder_.incremental_window_velocity_;
    return window_velocity + pll_weight * (raw_velocity - window_velocity);
}

float Controller::velocity_feedback_for_telemetry(float raw_velocity) const {
    if ((config_.control_mode == CONTROL_MODE_VELOCITY_CONTROL ||
         config_.control_mode == CONTROL_MODE_POSITION_CONTROL) &&
            velocity_control_feedback_valid_) {
        return velocity_control_feedback_;
    }
    return raw_velocity;
}

//--------------------------------
// Command Handling
//--------------------------------


bool Controller::select_encoder(size_t encoder_num) {
    if (encoder_num < AXIS_COUNT) {
        Axis* ax = axes[encoder_num];
        pos_estimate_circular_src_ = &ax->encoder_.pos_circular_;
        pos_wrap_src_ = &config_.circular_setpoint_range;
        pos_estimate_linear_src_ = &ax->encoder_.pos_estimate_;
        pos_estimate_valid_src_ = &ax->encoder_.pos_estimate_valid_;
        vel_estimate_src_ = &ax->encoder_.vel_estimate_;
        vel_estimate_valid_src_ = &ax->encoder_.vel_estimate_valid_;
        return true;
    } else {
        return set_error(Controller::ERROR_INVALID_LOAD_ENCODER), false;
    }
}

void Controller::move_to_pos(float goal_point) {
    axis_->trap_traj_.planTrapezoidal(goal_point, pos_setpoint_, vel_setpoint_,
                                 axis_->trap_traj_.config_.vel_limit,
                                 axis_->trap_traj_.config_.accel_limit,
                                 axis_->trap_traj_.config_.decel_limit);
    axis_->trap_traj_.t_ = 0.0f;
    trajectory_done_ = false;
}

void Controller::move_incremental(float displacement, bool from_input_pos = true){
    if(from_input_pos){
        input_pos_ += displacement;
    } else{
        input_pos_ = pos_setpoint_ + displacement;
    }

    input_pos_updated();
}

void Controller::start_anticogging_calibration() {
    start_anticogging_calibration(false);
}

void Controller::start_anticogging_calibration(bool velocity_only) {
    // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
    if (axis_->error_ == Axis::ERROR_NONE &&
            axis_->motor_.is_calibrated_ && axis_->encoder_.is_ready_) {
        config_.anticogging.index = 0;
        config_.anticogging.pre_calibrated = false;
        config_.anticogging.calib_anticogging = true;
        anticogging_valid_ = false;
        anticogging_velocity_only_ = velocity_only;
        anticogging_calibration_base_pos_ = std::round(axis_->encoder_.pos_estimate_);
        anticogging_sample_sum_ = 0.0f;
        anticogging_sample_count_ = 0;
        anticogging_map_sum_ = 0.0f;
        anticogging_map_mean_ = 0.0f;
        anticogging_valid_bin_count_ = 0;
        anticogging_sample_position_valid_ = false;
        // The ABZ workflow uses a slow, bidirectional velocity scan.  It
        // avoids 3600 discrete position jumps, which can trip overspeed on a
        // light rotor and also lets friction cancel when the two directions
        // are averaged.
        config_.control_mode = velocity_only ? CONTROL_MODE_VELOCITY_CONTROL
                                             : CONTROL_MODE_POSITION_CONTROL;
        config_.input_mode = velocity_only ? INPUT_MODE_VEL_RAMP : INPUT_MODE_PASSTHROUGH;
        if (velocity_only) {
            // Use exactly the same speed/ramp profile as FOC Studio's normal
            // ABZ speed mode.  Sampling at a separate 1.2/1.5 turn/s profile
            // produces a map that changes the loop gain and speed when it is
            // later applied at the 2 turn/s operating point.
            config_.vel_limit = std::min(2.2f, config_.vel_limit);
            config_.vel_ramp_rate = FOC_STUDIO_ABZ_RAMP_RATE;
            anticogging_scan_phase_ = ANTICOGGING_SCAN_RAMP_FORWARD;
            anticogging_scan_start_pos_ = anticogging_calibration_base_pos_;
            anticogging_reverse_start_pos_ = anticogging_calibration_base_pos_;
            anticogging_finalize_index_ = 0;
            std::fill(anticogging_forward_map_, anticogging_forward_map_ + 3600, 0);
            std::fill(anticogging_forward_count_, anticogging_forward_count_ + 3600, 0);
            std::fill(anticogging_reverse_count_, anticogging_reverse_count_ + 3600, 0);
            input_vel_ = FOC_STUDIO_ABZ_SPEED;
        } else {
            input_pos_ = anticogging_calibration_base_pos_;
            input_vel_ = 0.0f;
        }
        input_torque_ = 0.0f;
        pos_setpoint_ = axis_->encoder_.pos_estimate_;
        vel_setpoint_ = 0.0f;
        vel_integrator_torque_ = 0.0f;
        velocity_loop_torque_ = 0.0f;
        input_pos_updated();
    }
}


/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 * 
 * This holding current is added as a feedforward term in the control loop.
 */
bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) {
    if (anticogging_velocity_only_) {
        constexpr float scan_speed = FOC_STUDIO_ABZ_SPEED;
        // The ABZ count-window contains position-synchronous ripple at the
        // scan speed.  A symmetric gate keeps the map coverage high without
        // accepting a stalled or runaway interval.
        constexpr float scan_tolerance = 0.45f;
        constexpr float scan_turns = 6.0f;
        constexpr float torque_scale = 1000000.0f;

        const auto sample_map = [&](bool reverse) {
            // Sample once on entry to each unwrapped map bin. This retains the
            // position phase of every control-tick scan without repeatedly
            // running fmodf and averaging the same bin while the rotor is
            // between encoder edges.
            const float scaled_position = pos_estimate * 3600.0f;
            int32_t unwrapped_bin = static_cast<int32_t>(scaled_position);
            if (scaled_position < static_cast<float>(unwrapped_bin))
                --unwrapped_bin;
            if (anticogging_sample_position_valid_ &&
                    unwrapped_bin == anticogging_last_sample_position_bin_)
                return;
            anticogging_last_sample_position_bin_ = unwrapped_bin;
            anticogging_sample_position_valid_ = true;
            int32_t wrapped_bin = unwrapped_bin % 3600;
            if (wrapped_bin < 0)
                wrapped_bin += 3600;
            const uint32_t index = static_cast<uint32_t>(wrapped_bin);
            // The proportional term carries most position-synchronous torque
            // during a constant-speed scan. Recording only the integrator
            // produces an almost flat map and leaves cogging uncompensated.
            const float sample = std::clamp(velocity_loop_torque_, -0.012f, 0.012f);
            if (!reverse) {
                uint8_t& count = anticogging_forward_count_[index];
                const int32_t old_mean = anticogging_forward_map_[index];
                const int32_t value = static_cast<int32_t>(std::lrintf(sample * torque_scale));
                const uint32_t next_count = std::min<uint32_t>(255, static_cast<uint32_t>(count) + 1);
                anticogging_forward_map_[index] = static_cast<int16_t>(old_mean + (value - old_mean) / static_cast<int32_t>(next_count));
                count = static_cast<uint8_t>(next_count);
            } else {
                uint8_t& count = anticogging_reverse_count_[index];
                const uint32_t next_count = std::min<uint32_t>(255, static_cast<uint32_t>(count) + 1);
                if (count == 0) {
                    config_.anticogging.cogging_map[index] = sample;
                } else {
                    config_.anticogging.cogging_map[index] +=
                            (sample - config_.anticogging.cogging_map[index]) / static_cast<float>(next_count);
                }
                count = static_cast<uint8_t>(next_count);
            }
        };
        switch (anticogging_scan_phase_) {
            case ANTICOGGING_SCAN_RAMP_FORWARD:
                input_vel_ = scan_speed;
                if (vel_setpoint_ >= scan_speed - 0.01f &&
                        std::abs(vel_estimate - scan_speed) <= scan_tolerance) {
                    anticogging_scan_start_pos_ = pos_estimate;
                    anticogging_sample_position_valid_ = false;
                    anticogging_scan_phase_ = ANTICOGGING_SCAN_FORWARD;
                }
                break;
            case ANTICOGGING_SCAN_FORWARD:
                input_vel_ = scan_speed;
                if (std::abs(vel_estimate - scan_speed) <= scan_tolerance) sample_map(false);
                config_.anticogging.index = static_cast<uint32_t>(std::clamp(
                        (pos_estimate - anticogging_scan_start_pos_) / scan_turns, 0.0f, 1.0f) * 1800.0f);
                if (pos_estimate - anticogging_scan_start_pos_ >= scan_turns) {
                    input_vel_ = -scan_speed;
                    anticogging_scan_phase_ = ANTICOGGING_SCAN_RAMP_REVERSE;
                }
                break;
            case ANTICOGGING_SCAN_RAMP_REVERSE:
                input_vel_ = -scan_speed;
                if (vel_setpoint_ <= -scan_speed + 0.01f &&
                        std::abs(vel_estimate + scan_speed) <= scan_tolerance) {
                    anticogging_reverse_start_pos_ = pos_estimate;
                    anticogging_sample_position_valid_ = false;
                    anticogging_scan_phase_ = ANTICOGGING_SCAN_REVERSE;
                }
                break;
            case ANTICOGGING_SCAN_REVERSE:
                input_vel_ = -scan_speed;
                if (std::abs(vel_estimate + scan_speed) <= scan_tolerance) sample_map(true);
                config_.anticogging.index = 1800u + static_cast<uint32_t>(std::clamp(
                        (anticogging_reverse_start_pos_ - pos_estimate) / scan_turns, 0.0f, 1.0f) * 1800.0f);
                if (anticogging_reverse_start_pos_ - pos_estimate >= scan_turns) {
                    input_vel_ = 0.0f;
                    anticogging_finalize_index_ = 0;
                    anticogging_scan_phase_ = ANTICOGGING_SCAN_FINALIZE;
                }
                break;
            case ANTICOGGING_SCAN_FINALIZE: {
                input_vel_ = 0.0f;
                // Spread the 3600-bin finalize work over control cycles so a
                // single pass cannot starve PWM generation.
                constexpr uint16_t bins_per_cycle = 2;
                for (uint16_t n = 0; n < bins_per_cycle && anticogging_finalize_index_ < 3600; ++n) {
                    const uint16_t index = anticogging_finalize_index_++;
                    if (anticogging_forward_count_[index] && anticogging_reverse_count_[index]) {
                        const float forward = static_cast<float>(anticogging_forward_map_[index]) / torque_scale;
                        const float reverse = config_.anticogging.cogging_map[index];
                        config_.anticogging.cogging_map[index] = std::clamp(0.5f * (forward + reverse), -0.008f, 0.008f);
                        ++anticogging_valid_bin_count_;
                    } else {
                        config_.anticogging.cogging_map[index] = 0.0f;
                    }
                    anticogging_forward_map_[index] = static_cast<int16_t>(std::lrintf(
                            config_.anticogging.cogging_map[index] * torque_scale));
                }
                config_.anticogging.index = 3600;
                if (anticogging_finalize_index_ >= 3600) {
                    anticogging_finalize_index_ = 0;
                    anticogging_map_sum_ = 0.0f;
                    anticogging_scan_phase_ = ANTICOGGING_SCAN_SMOOTH;
                }
                break;
            }
            case ANTICOGGING_SCAN_SMOOTH: {
                input_vel_ = 0.0f;
                constexpr uint16_t bins_per_cycle = 2;
                for (uint16_t n = 0; n < bins_per_cycle && anticogging_finalize_index_ < 3600; ++n) {
                    const uint16_t index = anticogging_finalize_index_++;
                    const uint16_t im2 = (index + 3598) % 3600;
                    const uint16_t im1 = (index + 3599) % 3600;
                    const uint16_t ip1 = (index + 1) % 3600;
                    const uint16_t ip2 = (index + 2) % 3600;
                    const int32_t weighted = anticogging_forward_map_[im2] +
                            2 * anticogging_forward_map_[im1] +
                            3 * anticogging_forward_map_[index] +
                            2 * anticogging_forward_map_[ip1] +
                            anticogging_forward_map_[ip2];
                    const float smoothed = std::clamp(
                            static_cast<float>(weighted) / (9.0f * torque_scale),
                            -0.008f, 0.008f);
                    config_.anticogging.cogging_map[index] = smoothed;
                    anticogging_map_sum_ += smoothed;
                }
                config_.anticogging.index = 3600;
                if (anticogging_finalize_index_ >= 3600) {
                    anticogging_map_mean_ = anticogging_map_sum_ / 3600.0f;
                }
                if (anticogging_finalize_index_ >= 3600 && std::abs(vel_estimate) <= 0.05f) {
                    config_.anticogging.index = 0;
                    config_.anticogging.calib_anticogging = false;
                    // Do not report a successful calibration if unstable scan
                    // speed left most mechanical positions without a valid
                    // sample in either direction.
                    anticogging_valid_ = anticogging_valid_bin_count_ >= 2880;
                    anticogging_scan_phase_ = ANTICOGGING_SCAN_IDLE;
                    axis_->requested_state_ = Axis::AXIS_STATE_IDLE;
                    return true;
                }
                break;
            }
            default:
                break;
        }
        return false;
    }

    float pos_err = input_pos_ - pos_estimate;
    const float position_threshold_counts = anticogging_velocity_only_
            ? 2.0f : config_.anticogging.calib_pos_threshold;
    const float velocity_threshold_counts = anticogging_velocity_only_
            ? 4.0f : config_.anticogging.calib_vel_threshold;
    if (std::abs(pos_err) <= position_threshold_counts / (float)axis_->encoder_.config_.cpr &&
        std::abs(vel_estimate) < velocity_threshold_counts / (float)axis_->encoder_.config_.cpr) {
        anticogging_sample_sum_ += vel_integrator_torque_;
        ++anticogging_sample_count_;
        if (anticogging_sample_count_ >= 16) {
            const uint32_t index = std::clamp<uint32_t>(config_.anticogging.index, 0, 3599);
            const float sample = std::clamp(
                    anticogging_sample_sum_ / (float)anticogging_sample_count_,
                    -0.012f, 0.012f);
            config_.anticogging.cogging_map[index] = sample;
            anticogging_map_sum_ += sample;
            ++config_.anticogging.index;
            anticogging_sample_sum_ = 0.0f;
            anticogging_sample_count_ = 0;
        }
    } else {
        // Require consecutive stable samples so a pass through the target does
        // not write acceleration torque into the cogging map.
        anticogging_sample_sum_ = 0.0f;
        anticogging_sample_count_ = 0;
    }
    if (config_.anticogging.index < 3600) {
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        config_.input_mode = INPUT_MODE_PASSTHROUGH;
        input_pos_ = anticogging_calibration_base_pos_ +
                config_.anticogging.index * axis_->encoder_.getCoggingRatio();
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        return false;
    } else {
        config_.anticogging.index = 0;
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        config_.input_mode = INPUT_MODE_PASSTHROUGH;
        input_pos_ = anticogging_calibration_base_pos_;  // Return to the session reference.
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        anticogging_map_mean_ = anticogging_velocity_only_
                ? anticogging_map_sum_ / 3600.0f : 0.0f;
        anticogging_valid_ = true;
        config_.anticogging.calib_anticogging = false;
        return true;
    }
}

void Controller::update_filter_gains() {
    float bandwidth = std::min(config_.input_filter_bandwidth, 0.25f * current_meas_hz);
    input_filter_ki_ = 2.0f * bandwidth;  // basic conversion to discrete time
    input_filter_kp_ = 0.25f * (input_filter_ki_ * input_filter_ki_); // Critically damped
}

static float limitVel(const float vel_limit, const float vel_estimate, const float vel_gain, const float torque) {
    float Tmax = (vel_limit - vel_estimate) * vel_gain;
    float Tmin = (-vel_limit - vel_estimate) * vel_gain;
    return std::clamp(torque, Tmin, Tmax);
}

bool Controller::update(float* torque_setpoint_output) {
    float* pos_estimate_linear = (pos_estimate_valid_src_ && *pos_estimate_valid_src_)
            ? pos_estimate_linear_src_ : nullptr;
    float* pos_estimate_circular = (pos_estimate_valid_src_ && *pos_estimate_valid_src_)
            ? pos_estimate_circular_src_ : nullptr;
    float* vel_estimate_src = (vel_estimate_valid_src_ && *vel_estimate_valid_src_)
            ? vel_estimate_src_ : nullptr;

    // Calib_anticogging is only true when calibration is occurring, so we can't block anticogging_pos
    float anticogging_pos = axis_->encoder_.pos_estimate_ / axis_->encoder_.getCoggingRatio();
    if (config_.anticogging.calib_anticogging) {
        if (!axis_->encoder_.pos_estimate_valid_ || !axis_->encoder_.vel_estimate_valid_) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        // A calibration scan is intentionally autonomous. Keep the axis
        // watchdog alive while the controller moves through the map.
        axis_->watchdog_feed();
        const float calibration_velocity = velocity_feedback_for_telemetry(
                axis_->encoder_.vel_estimate_);
        // non-blocking
        anticogging_calibration(axis_->encoder_.pos_estimate_, calibration_velocity);
    }

    // TODO also enable circular deltas for 2nd order filter, etc.
    if (config_.circular_setpoints) {
        // Keep pos setpoint from drifting
        input_pos_ = fmodf_pos(input_pos_, config_.circular_setpoint_range);
    }

    // Update inputs
    switch (config_.input_mode) {
        case INPUT_MODE_INACTIVE: {
            // do nothing
        } break;
        case INPUT_MODE_PASSTHROUGH: {
            pos_setpoint_ = input_pos_;
            vel_setpoint_ = input_vel_;
            torque_setpoint_ = input_torque_; 
        } break;
        case INPUT_MODE_VEL_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.vel_ramp_rate);
            float full_step = input_vel_ - vel_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            vel_setpoint_ += step;
            torque_setpoint_ = (step / current_meas_period) * config_.inertia;
        } break;
        case INPUT_MODE_TORQUE_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.torque_ramp_rate);
            float full_step = input_torque_ - torque_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            torque_setpoint_ += step;
        } break;
        case INPUT_MODE_POS_FILTER: {
            // 2nd order pos tracking filter
            float delta_pos = input_pos_ - pos_setpoint_; // Pos error
            float delta_vel = input_vel_ - vel_setpoint_; // Vel error
            float accel = input_filter_kp_*delta_pos + input_filter_ki_*delta_vel; // Feedback
            torque_setpoint_ = accel * config_.inertia; // Accel
            vel_setpoint_ += current_meas_period * accel; // delta vel
            pos_setpoint_ += current_meas_period * vel_setpoint_; // Delta pos
        } break;
        case INPUT_MODE_MIRROR: {
            if (config_.axis_to_mirror < AXIS_COUNT) {
                pos_setpoint_ = axes[config_.axis_to_mirror]->encoder_.pos_estimate_ * config_.mirror_ratio;
                vel_setpoint_ = axes[config_.axis_to_mirror]->encoder_.vel_estimate_ * config_.mirror_ratio;
            } else {
                set_error(ERROR_INVALID_MIRROR_AXIS);
                return false;
            }
        } break;
        // case INPUT_MODE_MIX_CHANNELS: {
        //     // NOT YET IMPLEMENTED
        // } break;
        case INPUT_MODE_TRAP_TRAJ: {
            if(input_pos_updated_){
                move_to_pos(input_pos_);
                input_pos_updated_ = false;
            }
            // Avoid updating uninitialized trajectory
            if (trajectory_done_)
                break;
            
            if (axis_->trap_traj_.t_ > axis_->trap_traj_.Tf_) {
                // Drop into position control mode when done to avoid problems on loop counter delta overflow
                config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
                pos_setpoint_ = input_pos_;
                vel_setpoint_ = 0.0f;
                torque_setpoint_ = 0.0f;
                trajectory_done_ = true;
            } else {
                TrapezoidalTrajectory::Step_t traj_step = axis_->trap_traj_.eval(axis_->trap_traj_.t_);
                pos_setpoint_ = traj_step.Y;
                vel_setpoint_ = traj_step.Yd;
                torque_setpoint_ = traj_step.Ydd * config_.inertia;
                axis_->trap_traj_.t_ += current_meas_period;
            }
            anticogging_pos = pos_setpoint_; // FF the position setpoint instead of the pos_estimate
        } break;
        default: {
            set_error(ERROR_INVALID_INPUT_MODE);
            return false;
        }
        
    }

    // Position control
    // TODO Decide if we want to use encoder or pll position here
    float gain_scheduling_multiplier = 1.0f;
    float vel_des = vel_setpoint_;
    float pos_err = 0.0f;
    if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL) {
        if (config_.circular_setpoints) {
            if(!pos_estimate_circular) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            // Keep pos setpoint from drifting
            pos_setpoint_ = fmodf_pos(pos_setpoint_, *pos_wrap_src_);
            // Circular delta
            pos_err = pos_setpoint_ - *pos_estimate_circular;
            pos_err = wrap_pm(pos_err, 0.5f * *pos_wrap_src_);
        } else {
            if(!pos_estimate_linear) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            pos_err = pos_setpoint_ - *pos_estimate_linear;
        }

        const float position_gain =
                config_.control_mode == CONTROL_MODE_POSITION_CONTROL &&
                        axis_->encoder_.mode_ == Encoder::MODE_INCREMENTAL
                ? std::clamp(config_.pos_gain, 1.0f, 1.2f)
                : config_.pos_gain;
        vel_des += position_gain * pos_err;
        position_error_ = pos_err;
        // V-shaped gain shedule based on position error
        float abs_pos_err = std::abs(pos_err);
        if (config_.enable_gain_scheduling && abs_pos_err <= config_.gain_scheduling_width) {
            gain_scheduling_multiplier = abs_pos_err / config_.gain_scheduling_width;
        }
    } else {
        position_error_ = 0.0f;
        position_low_speed_active_ = false;
    }

    // Velocity limiting
    float vel_lim = config_.vel_limit;
    if (config_.enable_vel_limit) {
        vel_des = std::clamp(vel_des, -vel_lim, vel_lim);
    }

    // Check for overspeed fault (done in this module (controller) for cohesion with vel_lim).
    // Use the conditioned control feedback for ABZ and qualify the violation;
    // raw encoder PLL impulses are not a reliable reason to drop PWM.
    if (config_.enable_overspeed_error) {  // 0.0f to disable
        if (!vel_estimate_src) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        float overspeed_velocity = *vel_estimate_src;
        const bool abz_velocity_control =
                (config_.control_mode == CONTROL_MODE_VELOCITY_CONTROL ||
                 config_.control_mode == CONTROL_MODE_POSITION_CONTROL) &&
                axis_->encoder_.mode_ == Encoder::MODE_INCREMENTAL;
        if (abz_velocity_control) {
            if (config_.anticogging.calib_anticogging && anticogging_velocity_only_ &&
                    axis_->encoder_.incremental_window_velocity_valid_) {
                overspeed_velocity = axis_->encoder_.incremental_window_velocity_;
            } else if (velocity_control_feedback_valid_) {
                overspeed_velocity = velocity_control_feedback_;
            } else {
                overspeed_velocity = velocity_feedback_for_control(*vel_estimate_src, vel_des);
            }
        }
        const float overspeed_limit = std::abs(config_.vel_limit_tolerance * vel_lim);
        if (std::isfinite(overspeed_limit) && overspeed_limit > 0.0f &&
                std::abs(overspeed_velocity) > overspeed_limit) {
            // 16 control cycles is short enough to catch a real runaway while
            // rejecting the one-to-three-cycle impulses visible in ABZ plots.
            if (++overspeed_violation_count_ >= 16) {
                set_error(ERROR_OVERSPEED);
                return false;
            }
        } else {
            overspeed_violation_count_ = 0;
        }
    }

    // TODO: Change to controller working in torque units
    // Torque per amp gain scheduling (ACIM)
    float vel_gain = config_.vel_gain;
    float vel_integrator_gain = config_.vel_integrator_gain;

    // ABZ at low mechanical speed has sparse count updates. A large integral
    // term then accumulates while the rotor is held by static friction and is
    // released as one burst when a tooth is crossed. Static-friction
    // breakaway is supplied by the bounded helper below, so keep the P term
    // quiet in the low-speed region to avoid stick-slip.
    // This branch is deliberately limited to cascaded incremental-encoder
    // control; torque control and SPI/sensorless paths are unchanged.
    const bool cascaded_abz_mode =
            (config_.control_mode == CONTROL_MODE_VELOCITY_CONTROL ||
             config_.control_mode == CONTROL_MODE_POSITION_CONTROL) &&
            axis_->encoder_.mode_ == Encoder::MODE_INCREMENTAL;
    if (cascaded_abz_mode) {
        const float command_speed = std::abs(vel_des);
        const float high_speed_blend = std::clamp(
                (command_speed - 1.00f) / 0.75f, 0.0f, 1.0f);
        constexpr float low_speed_vel_gain = 0.0035f;
        constexpr float low_speed_integrator_gain = 0.0010f;
        vel_gain = low_speed_vel_gain + high_speed_blend *
                (vel_gain - low_speed_vel_gain);
        vel_integrator_gain = low_speed_integrator_gain + high_speed_blend *
                (vel_integrator_gain - low_speed_integrator_gain);
    }

    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_ACIM) {
        float effective_flux = axis_->motor_.current_control_.acim_rotor_flux;
        float minflux = axis_->motor_.config_.acim_gain_min_flux;
        if (fabsf(effective_flux) < minflux)
            effective_flux = std::copysignf(minflux, effective_flux);
        vel_gain /= effective_flux;
        vel_integrator_gain /= effective_flux;
        // TODO: also scale the integral value which is also changing units.
        // (or again just do control in torque units)
    }

    // Velocity control
    float torque = torque_setpoint_;

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging_valid_ && config_.anticogging.anticogging_enabled) {
        float anticogging_scale = 1.0f;
        const bool abz_velocity_mode =
                config_.control_mode == CONTROL_MODE_VELOCITY_CONTROL &&
                axis_->encoder_.mode_ == Encoder::MODE_INCREMENTAL;
        if (anticogging_velocity_only_ && abz_velocity_mode) {
            // Keep the scan map out of the sparse-count region.  The map is a
            // feed-forward correction, not breakaway torque; applying it while
            // the low-speed helper is still active adds a second torque source
            // and was the main cause of the 1.0-1.5 turn/s stick-slip bursts.
            const float command_speed = std::abs(vel_des);
            const float speed_blend = std::clamp(
                    (command_speed - 2.00f) / 0.50f, 0.0f, 1.0f);
            // Start below unity to avoid a current step on the first run; the
            // map itself is bounded to +/-8 mNm during finalisation.
            anticogging_scale = 0.35f * speed_blend;
        } else if (anticogging_velocity_only_) {
            // A map produced by the ABZ velocity scan is not valid for the
            // other controller modes. In particular, do not change torque
            // mode behavior.
            anticogging_scale = 0.0f;
        }
        if (anticogging_scale > 0.0f) {
            const float map_position = fmodf_pos(anticogging_pos, 3600.0f);
            const uint32_t index0 = (uint32_t)map_position;
            const uint32_t index1 = (index0 + 1) % 3600;
            const float fraction = map_position - (float)index0;
            const float map_torque = (1.0f - fraction) * config_.anticogging.cogging_map[index0] +
                    fraction * config_.anticogging.cogging_map[index1] - anticogging_map_mean_;
            torque += anticogging_scale * std::clamp(map_torque, -0.008f, 0.008f);
        }
    }

    float v_err = 0.0f;
    float integral_v_err = 0.0f;
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL) {
        abz_velocity_feedback_filter_.clear();
        velocity_control_feedback_valid_ = false;
        low_speed_compensator_.clear();
        low_speed_friction_torque_ = 0.0f;
        low_speed_compensator_state_ = LowSpeedCompensator::STATE_IDLE;
        position_low_speed_active_ = false;
    }
    if (config_.control_mode >= CONTROL_MODE_VELOCITY_CONTROL) {
        if (!vel_estimate_src) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }

        const float selected_velocity_feedback = velocity_feedback_for_control(
                *vel_estimate_src, vel_des);
        float velocity_feedback = selected_velocity_feedback;
        if (cascaded_abz_mode) {
            // The 15 ms count window and the PLL both contain count-edge
            // impulses. Condition their already speed-blended result before
            // either P or I sees it. The bandwidth rises toward 2 turn/s to
            // retain braking response while removing the 1 turn/s ripple.
            const float bandwidth_blend = std::clamp(
                    (std::abs(vel_des) - 1.0f) / 1.0f, 0.0f, 1.0f);
            const float feedback_bandwidth_hz =
                    6.0f + bandwidth_blend * 6.0f;
            velocity_feedback = abz_velocity_feedback_filter_.update(
                    selected_velocity_feedback, current_meas_period,
                    feedback_bandwidth_hz);
        } else {
            abz_velocity_feedback_filter_.clear();
        }
        velocity_control_feedback_ = velocity_feedback;
        velocity_control_feedback_valid_ = true;
        // Keep the filtered value for normal acceleration, but use the
        // instantaneous count-window value when it clearly reports an
        // overspeed.  This asymmetric path removes low-pass braking delay
        // without making isolated low-speed count edges drive the P term.
        float velocity_error_feedback = velocity_feedback;
        if (cascaded_abz_mode && std::abs(vel_des) >= 0.02f) {
            const float command_sign = vel_des > 0.0f ? 1.0f : -1.0f;
            if (selected_velocity_feedback * command_sign >
                    velocity_feedback * command_sign + 0.02f) {
                velocity_error_feedback = selected_velocity_feedback;
            }
        }
        v_err = vel_des - velocity_error_feedback;
        integral_v_err = v_err;
        const float proportional_torque =
                (vel_gain * gain_scheduling_multiplier) * v_err;
        torque += proportional_torque;

        if (cascaded_abz_mode) {
            // Keep the helper active through the sparse-count range. It is
            // full-strength up to 1 turn/s and fades out by 2 turn/s;
            // this prevents the 1.0--1.5 turn/s commands from falling into
            // the gap between static-friction compensation and the dense
            // velocity loop.
            const float low_speed_scale = std::clamp(
                    (2.00f - std::abs(vel_des)) / 1.00f, 0.0f, 1.0f);
            if (low_speed_scale > 0.0f) {
                const bool abz_position_mode =
                        config_.control_mode == CONTROL_MODE_POSITION_CONTROL;
                if (abz_position_mode) {
                    const float encoder_count_turn = 1.0f /
                            std::max<int32_t>(1, axis_->encoder_.config_.cpr);
                    const float activate_error = 4.0f * encoder_count_turn;
                    const float settle_error = 2.0f * encoder_count_turn;
                    if (std::abs(pos_err) >= activate_error) {
                        position_low_speed_active_ = true;
                    } else if (std::abs(pos_err) <= settle_error) {
                        position_low_speed_active_ = false;
                    }
                } else {
                    position_low_speed_active_ = false;
                }

                const bool velocity_request_active =
                        std::abs(vel_des) >= LowSpeedCompensator::command_threshold;
                const bool compensation_active = velocity_request_active ||
                        position_low_speed_active_;
                float compensation_direction = 0.0f;
                if (std::abs(vel_des) >= 0.002f) {
                    compensation_direction = vel_des;
                } else if (position_low_speed_active_) {
                    compensation_direction = pos_err;
                }
                float compensation_command = vel_des;
                if (position_low_speed_active_ &&
                        std::abs(compensation_command) <
                                LowSpeedCompensator::position_velocity_floor) {
                    compensation_command = std::copysignf(
                            LowSpeedCompensator::position_velocity_floor,
                            compensation_direction);
                }
                // Use the conditioned feedback for torque fade and error
                // gating.  Feeding the raw count-window impulses here makes
                // a single edge look like overspeed and removes the very
                // torque needed to keep a 0.2--0.5 turn/s command moving.
                const float compensation_error =
                        compensation_command - velocity_error_feedback;
                const LowSpeedCompensationResult compensation =
                        low_speed_compensator_.update(
                                compensation_active, compensation_direction,
                                compensation_command, velocity_error_feedback,
                                compensation_error, vel_integrator_torque_,
                                axis_->encoder_.shadow_count_,
                                current_meas_period);
                low_speed_friction_torque_ =
                        compensation.friction_torque * low_speed_scale;
                low_speed_compensator_state_ = compensation.state;
                torque += low_speed_friction_torque_;
                if (compensation.hold_integrator)
                    integral_v_err = 0.0f;
            } else {
                low_speed_compensator_.clear();
                low_speed_friction_torque_ = 0.0f;
                low_speed_compensator_state_ = LowSpeedCompensator::STATE_IDLE;
                position_low_speed_active_ = false;
            }
        } else {
            low_speed_compensator_.clear();
            low_speed_friction_torque_ = 0.0f;
            low_speed_compensator_state_ = LowSpeedCompensator::STATE_IDLE;
            position_low_speed_active_ = false;
        }

        // Do not let the integral store energy while the encoder is in the
        // sparse-count/static-friction region.  A previous high-speed I term
        // must also be removed before it can be added to the low-speed torque.
        // Blend it back in only from 1.0 to 1.75 turn/s, where the helper has
        // already faded and the velocity feedback is dense enough.
        if (cascaded_abz_mode) {
            const float integrator_blend = std::clamp(
                    (std::abs(vel_des) - 1.0f) / 0.75f, 0.0f, 1.0f);
            const float integrator_limit = 0.0045f * integrator_blend;
            vel_integrator_torque_ = std::clamp(
                    vel_integrator_torque_, -integrator_limit, integrator_limit);
            integral_v_err *= integrator_blend;
        }

        // Velocity integral action before limiting
        torque += vel_integrator_torque_;
        velocity_loop_torque_ = proportional_torque + vel_integrator_torque_;
    } else {
        velocity_loop_torque_ = 0.0f;
    }

    // Velocity limiting in current mode
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL && config_.enable_current_mode_vel_limit) {
        if (!vel_estimate_src) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        torque = limitVel(config_.vel_limit, *vel_estimate_src, vel_gain, torque);
    }

    // Torque limiting
    bool limited = false;
    float Tlim = axis_->motor_.max_available_torque();
    if (torque > Tlim) {
        limited = true;
        torque = Tlim;
    }
    if (torque < -Tlim) {
        limited = true;
        torque = -Tlim;
    }

    // Velocity integrator (behaviour dependent on limiting)
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL) {
        // reset integral if not in use
        vel_integrator_torque_ = 0.0f;
    } else {
        if (limited) {
            // TODO make decayfactor configurable
            vel_integrator_torque_ *= 0.99f;
        } else {
            vel_integrator_torque_ += ((vel_integrator_gain * gain_scheduling_multiplier) * current_meas_period) * integral_v_err;
        }
        if (cascaded_abz_mode) {
            // Keep the incremental-encoder integrator bounded and blend it in
            // only after the sparse-count/static-friction region.
            const float integral_blend = std::clamp(
                    (std::abs(vel_des) - 1.0f) / 0.75f, 0.0f, 1.0f);
            const float integral_limit = 0.0045f * integral_blend;
            vel_integrator_torque_ = std::clamp(
                    vel_integrator_torque_, -integral_limit, integral_limit);
        }
    }

    if (torque_setpoint_output) *torque_setpoint_output = torque;
    return true;
}
