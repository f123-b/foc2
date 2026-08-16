
#include "odrive_main.h"
#include <algorithm>

// The cogging-map scan runs at 2 turn/s, where the ABZ velocity loop has been
// verified to rotate continuously. Sampling is only allowed when the control
// velocity is within a tight tolerance of the requested scan velocity and the
// mechanical state is clean (no breakaway/recovery/reverse/saturation).
static constexpr float FOC_STUDIO_ABZ_SCAN_SPEED = 2.0f;       // turn/s
static constexpr float FOC_STUDIO_ABZ_SCAN_RAMP_RATE = 2.0f;  // turn/s^2
// Tight velocity gate: a 0 turn/s sample must not pass. The scan velocity is
// held within +/-0.5 turn/s of the request before any sample is accepted.
static constexpr float FOC_STUDIO_ABZ_SCAN_VEL_TOLERANCE = 0.5f;   // turn/s
// Dwell at the requested velocity before sampling starts, so ramp/acceleration
// torque is not written into the map.
static constexpr float FOC_STUDIO_ABZ_SCAN_DWELL_TIME = 0.15f;     // s
// Full mechanical turns scanned per direction.
static constexpr float FOC_STUDIO_ABZ_SCAN_TURNS = 6.0f;           // turns

// The position-hold cogging scan steps over the map at a coarse resolution so
// each step is large enough for the low-speed breakaway to move the rotor (the
// detent needs a few encoder counts to cross). 10 bins = 11 counts at 4000 CPR.
// The intermediate bins are filled by linear interpolation afterwards.
static constexpr uint32_t ANTICOGGING_POSITION_SCAN_STEP_BINS = 10;
static constexpr uint32_t ANTICOGGING_POSITION_SCAN_POINTS =
        3600 / ANTICOGGING_POSITION_SCAN_STEP_BINS;  // 360

// Post-processing (finalize/smooth/stats) is spread over control cycles so a
// single control tick never walks the full 3600-bin map and misses the PWM
// deadline. The bins-per-cycle count is runtime configurable and clamped to a
// safe fixed set (1/2/4/8).

// Validate a finished cogging map from pre-computed statistics (O(1), no map
// walk). Thresholds are runtime-configurable.
static bool anticogging_map_quality_ok(uint32_t valid_bins, uint32_t total_bins,
                                       float peak_to_peak, float max_abs,
                                       float max_jump, float wrap_jump,
                                       float min_coverage, float max_abs_limit,
                                       float max_peak_to_peak,
                                       float max_adjacent_jump,
                                       float max_wrap_jump) {
    if (total_bins == 0)
        return false;
    if (valid_bins < (uint32_t)((float)total_bins * std::clamp(min_coverage, 0.0f, 1.0f)))
        return false;
    if (peak_to_peak < 0.001f)
        return false;
    if (max_abs > max_abs_limit)
        return false;
    if (peak_to_peak > max_peak_to_peak)
        return false;
    if (max_jump > max_adjacent_jump)
        return false;
    if (wrap_jump > max_wrap_jump)
        return false;
    return true;
}

Controller::Controller(Config_t& config) :
    config_(config)
{
    update_filter_gains();
    if (config_.anticogging.pre_calibrated) {
        float map_sum = 0.0f;
        for (float sample : config_.anticogging.cogging_map)
            map_sum += sample;
        anticogging_map_mean_ = map_sum / 3600.0f;
        anticogging_valid_ = true;
        anticogging_valid_bin_count_ = 3600;
    }
}

void Controller::reset(bool abort_anticogging) {
    if (abort_anticogging && config_.anticogging.calib_anticogging) {
        config_.anticogging.calib_anticogging = false;
        anticogging_velocity_only_ = false;
        config_.anticogging.index = 0;
        anticogging_sample_sum_ = 0.0f;
        anticogging_sample_count_ = 0;
        anticogging_map_sum_ = 0.0f;
        anticogging_valid_bin_count_ = 0;
        anticogging_sample_position_valid_ = false;
        anticogging_scan_phase_ = ANTICOGGING_SCAN_IDLE;
        anticogging_dwell_time_ = 0.0f;
        anticogging_progress_percent_ = 0.0f;
        anticogging_rejected_velocity_samples_ = 0;
        anticogging_rejected_reverse_samples_ = 0;
        anticogging_rejected_state_samples_ = 0;
        anticogging_rejected_saturation_samples_ = 0;
        anticogging_stats_index_ = 0;
        anticogging_calibration_failed_ = false;
        anticogging_calibration_abort_reason_ = 0;
    }
    pos_setpoint_ = 0.0f;
    vel_setpoint_ = 0.0f;
    vel_integrator_torque_ = 0.0f;
    torque_setpoint_ = 0.0f;
    velocity_control_feedback_ = 0.0f;
    velocity_control_feedback_valid_ = false;
    overspeed_violation_count_ = 0;
    velocity_loop_torque_ = 0.0f;
    velocity_proportional_torque_ = 0.0f;
    anticogging_torque_ = 0.0f;
    final_torque_ = 0.0f;
    velocity_error_ = 0.0f;
    torque_unsaturated_ = 0.0f;
    motor_torque_saturated_ = false;
    abz_velocity_torque_before_limit_ = 0.0f;
    abz_velocity_torque_after_limit_ = 0.0f;
    abz_velocity_torque_saturated_ = false;
    friction_compensator_.clear();
    low_speed_friction_torque_ = 0.0f;
    low_speed_compensator_state_ = FrictionCompensator::STATE_IDLE;
    friction_target_torque_ = 0.0f;
    friction_speed_ratio_ = 0.0f;
    friction_assist_blend_ = 0.0f;
    friction_no_progress_time_ = 0.0f;
    friction_recovery_timer_ = 0.0f;
    friction_forward_velocity_ = 0.0f;
    friction_reverse_detected_ = false;
    control_observer_velocity_ = 0.0f;
    control_observer_valid_ = false;
    observer_bandwidth_ = 0.0f;
    velocity_estimator_disagreement_ = 0.0f;
    abz_count_glitch_count_ = 0;
    position_error_ = 0.0f;
    position_low_speed_active_ = false;
}

void Controller::set_error(Error error) {
    error_ |= error;
    axis_->error_ |= Axis::ERROR_CONTROLLER_FAILED;
}

float Controller::velocity_feedback_for_control(float raw_velocity,
                                                float commanded_velocity) const {
    (void)commanded_velocity;
    // ABZ velocity/position control closes on the ABZ mechanical velocity
    // observer; every other feedback source (torque, sensorless, SPI) uses the
    // encoder PLL velocity directly.  This is the ONLY ABZ velocity feedback.
    if (cascaded_abz_control() && control_observer_valid_) {
        return control_observer_velocity_;
    }
    return raw_velocity;
}

float Controller::velocity_feedback_for_telemetry(float raw_velocity) const {
    // ABZ telemetry reports the observer velocity that closed the loop, so the
    // UI shows the mechanical speed the controller actually saw.
    if (cascaded_abz_control() && control_observer_valid_) {
        return control_observer_velocity_;
    }
    return raw_velocity;
}

bool Controller::cascaded_abz_control() const {
    return axis_ &&
            (config_.control_mode == CONTROL_MODE_VELOCITY_CONTROL ||
             config_.control_mode == CONTROL_MODE_POSITION_CONTROL) &&
            axis_->encoder_.mode_ == Encoder::MODE_INCREMENTAL &&
            vel_estimate_src_ == &axis_->encoder_.vel_estimate_;
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
        anticogging_dwell_time_ = 0.0f;
        anticogging_progress_percent_ = 0.0f;
        anticogging_scan_velocity_ = 0.0f;
        anticogging_scan_velocity_error_ = 0.0f;
        anticogging_current_bin_ = 0;
        anticogging_forward_valid_bins_ = 0;
        anticogging_reverse_valid_bins_ = 0;
        anticogging_rejected_velocity_samples_ = 0;
        anticogging_rejected_reverse_samples_ = 0;
        anticogging_rejected_state_samples_ = 0;
        anticogging_rejected_saturation_samples_ = 0;
        anticogging_map_rms_ = 0.0f;
        anticogging_map_peak_to_peak_ = 0.0f;
        anticogging_map_max_jump_ = 0.0f;
        anticogging_map_wrap_jump_ = 0.0f;
        anticogging_map_max_abs_ = 0.0f;
        anticogging_map_min_ = 0.0f;
        anticogging_map_max_ = 0.0f;
        anticogging_map_sum_sq_ = 0.0f;
        anticogging_map_first_ = 0.0f;
        anticogging_map_last_ = 0.0f;
        anticogging_stats_index_ = 0;
        anticogging_calibration_failed_ = false;
        anticogging_calibration_abort_reason_ = 0;
        // The ABZ workflow uses a slow, bidirectional velocity scan.  It
        // avoids 3600 discrete position jumps, which can trip overspeed on a
        // light rotor and also lets friction cancel when the two directions
        // are averaged.
        config_.control_mode = velocity_only ? CONTROL_MODE_VELOCITY_CONTROL
                                             : CONTROL_MODE_POSITION_CONTROL;
        config_.input_mode = velocity_only ? INPUT_MODE_VEL_RAMP : INPUT_MODE_PASSTHROUGH;
        if (velocity_only) {
            // Keep the scan below the normal 2 turn/s operating point. This
            // is deliberate: the ABZ count feedback and the breakaway assist
            // need speed/current headroom during startup and reversal.
            config_.vel_limit = std::min(3.0f, config_.vel_limit);
            config_.vel_ramp_rate = config_.anticogging_calibration_accel;
            anticogging_scan_phase_ = ANTICOGGING_SCAN_RAMP_FORWARD;
            anticogging_scan_start_pos_ = anticogging_calibration_base_pos_;
            anticogging_reverse_start_pos_ = anticogging_calibration_base_pos_;
            anticogging_finalize_index_ = 0;
            std::fill(anticogging_forward_map_, anticogging_forward_map_ + 3600, 0);
            std::fill(anticogging_forward_count_, anticogging_forward_count_ + 3600, 0);
            std::fill(anticogging_reverse_count_, anticogging_reverse_count_ + 3600, 0);
            input_vel_ = FOC_STUDIO_ABZ_SCAN_SPEED;
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

// Abort the calibration atomically: a half-finished map must never be applied.
// reason: 0 = none, 1 = fault (motor/axis/controller/overspeed/deadline),
// 2 = map quality gate failed.
void Controller::abort_anticogging_calibration(uint8_t reason) {
    config_.anticogging.calib_anticogging = false;
    anticogging_valid_ = false;
    config_.anticogging.pre_calibrated = false;
    anticogging_torque_ = 0.0f;
    anticogging_calibration_failed_ = true;
    anticogging_calibration_abort_reason_ = reason;
    anticogging_scan_phase_ = ANTICOGGING_SCAN_FAILED;
    axis_->requested_state_ = Axis::AXIS_STATE_IDLE;
}

bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) {
    // Abort on any fault so a half-finished map is never applied.
    if (axis_->error_ != Axis::ERROR_NONE) {
        abort_anticogging_calibration(1);
        return false;
    }
    if (anticogging_velocity_only_) {
        const float scan_speed = config_.anticogging_scan_speed;
        const float tolerance = config_.anticogging_scan_velocity_tolerance;
        const float dwell_time = config_.anticogging_scan_dwell_time;
        const float scan_turns = config_.anticogging_scan_turns;
        // Clamp postprocess bins to a safe fixed set (1/2/4/8) so a bad runtime
        // value cannot blow the control deadline.
        const uint16_t postprocess_bins = config_.anticogging_postprocess_bins_per_cycle >= 8 ? 8 :
                config_.anticogging_postprocess_bins_per_cycle >= 4 ? 4 :
                config_.anticogging_postprocess_bins_per_cycle >= 2 ? 2 : 1;
        constexpr float torque_scale = 1000000.0f;

        // Telemetry: control velocity and its error vs the requested scan
        // velocity.
        anticogging_scan_velocity_ = vel_estimate;
        const bool reverse_phase =
                anticogging_scan_phase_ == ANTICOGGING_SCAN_RAMP_REVERSE ||
                anticogging_scan_phase_ == ANTICOGGING_SCAN_REVERSE;
        const float requested = reverse_phase ? -scan_speed : scan_speed;
        anticogging_scan_velocity_error_ = vel_estimate - requested;

        // A sample is valid only when the mechanical state is clean: velocity
        // within the tight tolerance in the scan direction, friction RUNNING,
        // no reverse motion, and no torque saturation. Transient torque from
        // breakaway/recovery/saturation must not enter the cogging map.
        const auto sample_valid = [&](bool reverse) -> bool {
            const bool velocity_ok = reverse
                    ? (vel_estimate < 0.0f &&
                            std::abs(vel_estimate + scan_speed) <= tolerance)
                    : (vel_estimate > 0.0f &&
                            std::abs(vel_estimate - scan_speed) <= tolerance);
            if (!velocity_ok) {
                ++anticogging_rejected_velocity_samples_;
                return false;
            }
            // Secondary sanity check: reject the sample while the control
            // observer and the 50 ms mechanical window disagree by more than
            // the scan tolerance. A bad estimator must not be written into the
            // cogging map; this never faults, it only rejects samples.
            if (axis_->encoder_.velocity_window_50ms_valid_ &&
                    std::abs(control_observer_velocity_ -
                            axis_->encoder_.velocity_window_50ms_) > tolerance) {
                ++anticogging_rejected_estimator_samples_;
                return false;
            }
            if (friction_reverse_detected_) {
                ++anticogging_rejected_reverse_samples_;
                return false;
            }
            if (low_speed_compensator_state_ != FrictionCompensator::STATE_RUNNING) {
                ++anticogging_rejected_state_samples_;
                return false;
            }
            if (abz_velocity_torque_saturated_ || motor_torque_saturated_) {
                ++anticogging_rejected_saturation_samples_;
                return false;
            }
            return true;
        };

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
            // Record the complete non-anticogging torque that maintains the
            // constant scan speed (P + I + friction FF). The anticogging FF is
            // forced to zero during calibration, so this is the true total
            // mechanical torque. Averaging forward and reverse cancels the
            // direction-only Coulomb friction.
            const float sample = std::clamp(
                    velocity_loop_torque_ + low_speed_friction_torque_,
                    -0.012f, 0.012f);
            if (!reverse) {
                uint8_t& count = anticogging_forward_count_[index];
                if (count == 0)
                    ++anticogging_forward_valid_bins_;
                const int32_t old_mean = anticogging_forward_map_[index];
                const int32_t value = static_cast<int32_t>(std::lrintf(sample * torque_scale));
                const uint32_t next_count = std::min<uint32_t>(255, static_cast<uint32_t>(count) + 1);
                anticogging_forward_map_[index] = static_cast<int16_t>(old_mean + (value - old_mean) / static_cast<int32_t>(next_count));
                count = static_cast<uint8_t>(next_count);
            } else {
                uint8_t& count = anticogging_reverse_count_[index];
                if (count == 0)
                    ++anticogging_reverse_valid_bins_;
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

        // Dwell: only leave the ramp phase once the velocity has stayed in
        // range for dwell_time, so ramp/acceleration torque is not recorded.
        const auto dwell_ok = [&](bool reverse) -> bool {
            const bool velocity_ok = reverse
                    ? (vel_estimate < 0.0f &&
                            std::abs(vel_estimate + scan_speed) <= tolerance)
                    : (vel_estimate > 0.0f &&
                            std::abs(vel_estimate - scan_speed) <= tolerance);
            anticogging_dwell_time_ = velocity_ok
                    ? anticogging_dwell_time_ + current_meas_period
                    : 0.0f;
            return anticogging_dwell_time_ >= dwell_time;
        };

        // Current mechanical bin for telemetry.
        int32_t current_bin = static_cast<int32_t>(pos_estimate * 3600.0f);
        current_bin = ((current_bin % 3600) + 3600) % 3600;
        anticogging_current_bin_ = static_cast<uint32_t>(current_bin);

        switch (anticogging_scan_phase_) {
            case ANTICOGGING_SCAN_RAMP_FORWARD:
                input_vel_ = scan_speed;
                anticogging_progress_percent_ = 0.0f;
                if (vel_setpoint_ >= scan_speed - 0.01f && dwell_ok(false)) {
                    anticogging_scan_start_pos_ = pos_estimate;
                    anticogging_sample_position_valid_ = false;
                    anticogging_dwell_time_ = 0.0f;
                    anticogging_scan_phase_ = ANTICOGGING_SCAN_FORWARD;
                }
                break;
            case ANTICOGGING_SCAN_FORWARD:
                input_vel_ = scan_speed;
                if (sample_valid(false))
                    sample_map(false);
                {
                    const float progress = std::clamp(
                            (pos_estimate - anticogging_scan_start_pos_) / scan_turns,
                            0.0f, 1.0f);
                    anticogging_progress_percent_ = 45.0f * progress;
                    config_.anticogging.index = static_cast<uint32_t>(progress * 1620.0f);
                }
                if (pos_estimate - anticogging_scan_start_pos_ >= scan_turns) {
                    input_vel_ = -scan_speed;
                    anticogging_dwell_time_ = 0.0f;
                    anticogging_scan_phase_ = ANTICOGGING_SCAN_RAMP_REVERSE;
                }
                break;
            case ANTICOGGING_SCAN_RAMP_REVERSE:
                input_vel_ = -scan_speed;
                if (vel_setpoint_ <= -scan_speed + 0.01f && dwell_ok(true)) {
                    anticogging_reverse_start_pos_ = pos_estimate;
                    anticogging_sample_position_valid_ = false;
                    anticogging_dwell_time_ = 0.0f;
                    anticogging_scan_phase_ = ANTICOGGING_SCAN_REVERSE;
                }
                break;
            case ANTICOGGING_SCAN_REVERSE:
                input_vel_ = -scan_speed;
                if (sample_valid(true))
                    sample_map(true);
                {
                    const float progress = std::clamp(
                            (anticogging_reverse_start_pos_ - pos_estimate) / scan_turns,
                            0.0f, 1.0f);
                    anticogging_progress_percent_ = 45.0f + 45.0f * progress;
                    config_.anticogging.index = 1620u + static_cast<uint32_t>(progress * 1620.0f);
                }
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
                for (uint16_t n = 0; n < postprocess_bins && anticogging_finalize_index_ < 3600; ++n) {
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
                config_.anticogging.index = 3240;
                anticogging_progress_percent_ = 90.0f + 3.0f *
                        ((float)anticogging_finalize_index_ / 3600.0f);
                if (anticogging_finalize_index_ >= 3600) {
                    anticogging_finalize_index_ = 0;
                    anticogging_map_sum_ = 0.0f;
                    anticogging_scan_phase_ = ANTICOGGING_SCAN_SMOOTH;
                }
                break;
            }
            case ANTICOGGING_SCAN_SMOOTH: {
                input_vel_ = 0.0f;
                for (uint16_t n = 0; n < postprocess_bins &&
                        anticogging_finalize_index_ < 3600; ++n) {
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
                anticogging_progress_percent_ = 93.0f + 3.0f *
                        ((float)anticogging_finalize_index_ / 3600.0f);
                if (anticogging_finalize_index_ >= 3600) {
                    anticogging_map_mean_ = anticogging_map_sum_ / 3600.0f;
                    anticogging_stats_index_ = 0;
                    anticogging_map_sum_sq_ = 0.0f;
                    anticogging_map_min_ = 1e9f;
                    anticogging_map_max_ = -1e9f;
                    anticogging_map_max_jump_ = 0.0f;
                    anticogging_map_first_ = config_.anticogging.cogging_map[0];
                    anticogging_map_last_ = config_.anticogging.cogging_map[3599];
                    anticogging_scan_phase_ = ANTICOGGING_SCAN_STATS;
                }
                break;
            }
            case ANTICOGGING_SCAN_STATS: {
                input_vel_ = 0.0f;
                // Incremental statistics: only a few bins per control cycle so
                // the 8 kHz PWM deadline is never missed.
                const float* map = config_.anticogging.cogging_map;
                for (uint16_t n = 0; n < postprocess_bins &&
                        anticogging_stats_index_ < 3600; ++n) {
                    const uint16_t i = anticogging_stats_index_++;
                    const float v = map[i];
                    anticogging_map_sum_sq_ += v * v;
                    anticogging_map_min_ = std::min(anticogging_map_min_, v);
                    anticogging_map_max_ = std::max(anticogging_map_max_, v);
                    if (i > 0) {
                        anticogging_map_max_jump_ = std::max(
                                anticogging_map_max_jump_, std::abs(v - map[i - 1]));
                    }
                }
                anticogging_progress_percent_ = 96.0f + 3.0f *
                        ((float)anticogging_stats_index_ / 3600.0f);
                if (anticogging_stats_index_ >= 3600) {
                    anticogging_map_rms_ = std::sqrt(anticogging_map_sum_sq_ / 3600.0f);
                    anticogging_map_peak_to_peak_ =
                            anticogging_map_max_ - anticogging_map_min_;
                    anticogging_map_max_abs_ = std::max(
                            std::abs(anticogging_map_min_), std::abs(anticogging_map_max_));
                    anticogging_map_wrap_jump_ = std::abs(
                            anticogging_map_last_ - anticogging_map_first_);
                    anticogging_scan_phase_ = ANTICOGGING_SCAN_VALIDATE;
                }
                break;
            }
            case ANTICOGGING_SCAN_VALIDATE: {
                input_vel_ = 0.0f;
                anticogging_progress_percent_ = 99.0f;
                // O(1) quality gate from pre-computed statistics; no map walk.
                const bool quality_ok = anticogging_map_quality_ok(
                        anticogging_valid_bin_count_, 3600,
                        anticogging_map_peak_to_peak_, anticogging_map_max_abs_,
                        anticogging_map_max_jump_, anticogging_map_wrap_jump_,
                        config_.anticogging_quality_min_coverage,
                        config_.anticogging_quality_max_abs,
                        config_.anticogging_quality_max_peak_to_peak,
                        config_.anticogging_quality_max_adjacent_jump,
                        config_.anticogging_quality_max_wrap_jump);
                if (quality_ok) {
                    anticogging_valid_ = true;
                    config_.anticogging.pre_calibrated = true;
                    config_.anticogging.calib_anticogging = false;
                    anticogging_calibration_failed_ = false;
                    anticogging_scan_phase_ = ANTICOGGING_SCAN_COMPLETE;
                } else {
                    abort_anticogging_calibration(2);  // quality gate failed
                }
                break;
            }
            case ANTICOGGING_SCAN_COMPLETE: {
                input_vel_ = 0.0f;
                anticogging_progress_percent_ = 100.0f;
                config_.anticogging.index = 0;
                anticogging_scan_phase_ = ANTICOGGING_SCAN_IDLE;
                axis_->requested_state_ = Axis::AXIS_STATE_IDLE;
                return true;
            }
            case ANTICOGGING_SCAN_FAILED:
            default:
                input_vel_ = 0.0f;
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
            const uint32_t point = std::clamp<uint32_t>(
                    config_.anticogging.index, 0,
                    ANTICOGGING_POSITION_SCAN_POINTS - 1);
            const float sample = std::clamp(
                    anticogging_sample_sum_ / (float)anticogging_sample_count_,
                    -0.012f, 0.012f);
            config_.anticogging.cogging_map[
                    point * ANTICOGGING_POSITION_SCAN_STEP_BINS] = sample;
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
    if (config_.anticogging.index < ANTICOGGING_POSITION_SCAN_POINTS) {
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        config_.input_mode = INPUT_MODE_PASSTHROUGH;
        input_pos_ = anticogging_calibration_base_pos_ +
                (float)(config_.anticogging.index *
                        ANTICOGGING_POSITION_SCAN_STEP_BINS) *
                axis_->encoder_.getCoggingRatio();
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        return false;
    } else {
        anticogging_valid_bin_count_ = config_.anticogging.index;
        config_.anticogging.index = 0;
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        config_.input_mode = INPUT_MODE_PASSTHROUGH;
        input_pos_ = anticogging_calibration_base_pos_;  // Return to the session reference.
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        // Remove the DC mean (friction) via the normal feed-forward subtraction
        // and fill the bins between the coarse samples by linear interpolation.
        anticogging_map_mean_ = anticogging_map_sum_ /
                (float)ANTICOGGING_POSITION_SCAN_POINTS;
        for (uint32_t p = 0; p < ANTICOGGING_POSITION_SCAN_POINTS; ++p) {
            const uint32_t bin0 = p * ANTICOGGING_POSITION_SCAN_STEP_BINS;
            const uint32_t next_p = (p + 1) % ANTICOGGING_POSITION_SCAN_POINTS;
            const uint32_t bin1 = next_p * ANTICOGGING_POSITION_SCAN_STEP_BINS;
            const float v0 = config_.anticogging.cogging_map[bin0];
            const float v1 = config_.anticogging.cogging_map[bin1];
            for (uint32_t j = 1; j < ANTICOGGING_POSITION_SCAN_STEP_BINS; ++j) {
                const float t = (float)j /
                        (float)ANTICOGGING_POSITION_SCAN_STEP_BINS;
                config_.anticogging.cogging_map[bin0 + j] = v0 + (v1 - v0) * t;
            }
        }
        // Circular smoothing across the 0/360 wrap so the coarse-sample
        // boundaries do not leave sharp steps in the interpolated map. The
        // forward map buffer is reused as the smoothing source (it is otherwise
        // idle during the position-hold scan).
        constexpr float torque_scale = 1000000.0f;
        for (uint32_t i = 0; i < 3600; ++i) {
            anticogging_forward_map_[i] = static_cast<int16_t>(std::lrintf(
                    config_.anticogging.cogging_map[i] * torque_scale));
        }
        for (uint32_t i = 0; i < 3600; ++i) {
            const uint16_t im2 = (i + 3598) % 3600;
            const uint16_t im1 = (i + 3599) % 3600;
            const uint16_t ip1 = (i + 1) % 3600;
            const uint16_t ip2 = (i + 2) % 3600;
            const int32_t weighted = anticogging_forward_map_[im2] +
                    2 * anticogging_forward_map_[im1] +
                    3 * anticogging_forward_map_[i] +
                    2 * anticogging_forward_map_[ip1] +
                    anticogging_forward_map_[ip2];
            config_.anticogging.cogging_map[i] = std::clamp(
                    static_cast<float>(weighted) / (9.0f * torque_scale),
                    -0.012f, 0.012f);
        }
        // Compute map statistics for the quality gate (the FOC Studio velocity
        // scan uses the incremental STATS phase; this CAN position-hold path
        // walks the map once here).
        {
            const float* map = config_.anticogging.cogging_map;
            float lo = 1e9f, hi = -1e9f, sum_sq = 0.0f;
            float prev = map[3599];
            float max_jump = 0.0f;
            for (uint32_t i = 0; i < 3600; ++i) {
                const float v = map[i];
                lo = std::min(lo, v);
                hi = std::max(hi, v);
                sum_sq += v * v;
                max_jump = std::max(max_jump, std::abs(v - prev));
                prev = v;
            }
            anticogging_map_rms_ = std::sqrt(sum_sq / 3600.0f);
            anticogging_map_peak_to_peak_ = hi - lo;
            anticogging_map_max_abs_ = std::max(std::abs(lo), std::abs(hi));
            anticogging_map_max_jump_ = max_jump;
            anticogging_map_wrap_jump_ = std::abs(map[3599] - map[0]);
        }
        anticogging_valid_ = anticogging_map_quality_ok(
                anticogging_valid_bin_count_, ANTICOGGING_POSITION_SCAN_POINTS,
                anticogging_map_peak_to_peak_, anticogging_map_max_abs_,
                anticogging_map_max_jump_, anticogging_map_wrap_jump_,
                config_.anticogging_quality_min_coverage,
                config_.anticogging_quality_max_abs,
                config_.anticogging_quality_max_peak_to_peak,
                config_.anticogging_quality_max_adjacent_jump,
                config_.anticogging_quality_max_wrap_jump);
        config_.anticogging.pre_calibrated = anticogging_valid_;
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
        const bool abz_velocity_control = cascaded_abz_control();
        if (abz_velocity_control) {
            // The rolling count window is useful for the velocity loop, but it
            // is not a safety signal during a scan: one position-synchronous
            // burst can report 4+ turn/s while the filtered plant speed is
            // still near the 2 turn/s target. Use the same conditioned value
            // that drives P/I for the qualified overspeed check.
            if (velocity_control_feedback_valid_) {
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

    const bool cascaded_abz_mode = cascaded_abz_control();

    // ABZ uses its own PI gains: the generic ODrive defaults are far too large
    // for a 4000 CPR incremental encoder, so keep them tunable and independent.
    if (cascaded_abz_mode) {
        vel_gain = std::max(0.0f, config_.abz_vel_gain);
        vel_integrator_gain = std::max(0.0f, config_.abz_vel_integrator_gain);
    }

    // ABZ mechanical velocity observer: the SINGLE velocity feedback source of
    // the ABZ velocity PI.  It is fed the per-tick delta position (delta_count
    // / CPR) so it is immune to int32 shadow-count overflow and never loses
    // float precision over long runs.  Its bandwidth follows the commanded
    // speed (adaptive, smooth interpolation) with gains recomputed only when
    // the bandwidth actually changes.  The M/T estimator and the 50/100 ms
    // windows stay diagnostics; the encoder PLL stays untouched for
    // commutation / phase interpolation / safety.
    if (cascaded_abz_mode) {
        const int32_t cpr = std::max<int32_t>(1, axis_->encoder_.config_.cpr);
        abz_velocity_observer_.configure(
                config_.abz_velocity_observer_min_bandwidth,
                config_.abz_velocity_observer_max_bandwidth);
        abz_velocity_observer_.set_bandwidth(
                abz_velocity_observer_.bandwidth_for(std::fabs(vel_des)));
        observer_bandwidth_ = abz_velocity_observer_.bandwidth();
        if (!control_observer_valid_) {
            // Initialize while the rotor may be moving: seed the velocity from
            // the best available estimator so switching into velocity mode
            // does not start the loop from zero and produce a torque impulse.
            float initial_velocity = 0.0f;
            if (axis_->encoder_.velocity_window_50ms_valid_)
                initial_velocity = axis_->encoder_.velocity_window_50ms_;
            else if (axis_->encoder_.mt_velocity_estimator_.valid())
                initial_velocity = axis_->encoder_.mt_velocity_estimate_;
            abz_velocity_observer_.reset(
                    (float)axis_->encoder_.shadow_count_ / (float)cpr,
                    initial_velocity);
        }
        const float delta_position =
                (float)axis_->encoder_.last_delta_count_ / (float)cpr;
        control_observer_velocity_ = abz_velocity_observer_.update(
                delta_position, current_meas_period);
        control_observer_valid_ = true;

        // Estimator agreement diagnostic: control observer vs 50 ms window.
        // A large disagreement means the estimators disagree (see
        // docs/ABZ_VELOCITY_ESTIMATION.md); it never affects control.
        velocity_estimator_disagreement_ =
                axis_->encoder_.velocity_window_50ms_valid_
                ? control_observer_velocity_ - axis_->encoder_.velocity_window_50ms_
                : 0.0f;

        // Abnormal count detection (diagnostic only, never faults): a single
        // control tick whose |delta| exceeds 3x the physically expected counts
        // at the commanded speed is a likely ABZ signal-integrity glitch.
        const float expected_counts_per_tick =
                std::max(std::fabs(vel_des), 1.0f) *
                (float)cpr * current_meas_period;
        const int32_t glitch_threshold = std::max(
                (int32_t)2, (int32_t)(expected_counts_per_tick * 3.0f + 0.999f));
        if (std::abs(axis_->encoder_.last_delta_count_) > glitch_threshold)
            ++abz_count_glitch_count_;
    } else {
        control_observer_velocity_ = 0.0f;
        control_observer_valid_ = false;
        observer_bandwidth_ = 0.0f;
        velocity_estimator_disagreement_ = 0.0f;
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
    anticogging_torque_ = 0.0f;
    anticogging_effective_scale_ = 0.0f;
    if (anticogging_valid_ && config_.anticogging.anticogging_enabled) {
        float anticogging_scale = 1.0f;
        const bool abz_velocity_mode =
                cascaded_abz_control() &&
                config_.control_mode == CONTROL_MODE_VELOCITY_CONTROL;
        if (anticogging_velocity_only_ && abz_velocity_mode) {
            // The map is useful precisely where cogging causes the low-speed
            // stick-slip. Blend it in gradually so enabling a completed map
            // cannot create a torque step at the first command.
            const float command_speed = std::abs(vel_des);
            const float speed_blend = std::clamp(
                    command_speed / 0.08f, 0.0f, 1.0f);
            anticogging_scale = speed_blend;
        } else if (anticogging_velocity_only_) {
            // A map produced by the ABZ velocity scan is not valid for the
            // other controller modes. In particular, do not change torque
            // mode behavior.
            anticogging_scale = 0.0f;
        }
        if (anticogging_scale > 0.0f) {
            const float map_position = fmodf_pos(
                    anticogging_pos + config_.anticogging_phase_offset_bins,
                    3600.0f);
            const uint32_t index0 = (uint32_t)map_position;
            const uint32_t index1 = (index0 + 1) % 3600;
            const float fraction = map_position - (float)index0;
            const float map_torque = (1.0f - fraction) * config_.anticogging.cogging_map[index0] +
                    fraction * config_.anticogging.cogging_map[index1] - anticogging_map_mean_;
            // cogging_ratio is the tunable feed-forward scale; the map torque is
            // clamped to the configurable anticogging_torque_limit.
            const float clamp_limit = std::isfinite(config_.anticogging_torque_limit) &&
                    config_.anticogging_torque_limit > 0.0f
                    ? config_.anticogging_torque_limit : 0.005f;
            anticogging_torque_ = anticogging_scale * config_.anticogging.cogging_ratio *
                    std::clamp(map_torque, -clamp_limit, clamp_limit);
            anticogging_effective_scale_ = anticogging_scale * config_.anticogging.cogging_ratio;
            torque += anticogging_torque_;
        }
    }

    float v_err = 0.0f;
    float torque_unsaturated = torque;
    if (config_.control_mode >= CONTROL_MODE_VELOCITY_CONTROL) {
        if (!vel_estimate_src) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }

        // Single feedback source: the ABZ mechanical velocity observer for ABZ
        // velocity/position control, the encoder PLL everywhere else.  No
        // second filter, no raw PLL overspeed lead, no gain schedule.
        const float velocity_feedback = velocity_feedback_for_control(
                *vel_estimate_src, vel_des);
        velocity_control_feedback_ = velocity_feedback;
        velocity_control_feedback_valid_ = true;

        v_err = vel_des - velocity_feedback;
        velocity_error_ = v_err;
        const float proportional_torque =
                (vel_gain * gain_scheduling_multiplier) * v_err;
        velocity_proportional_torque_ = proportional_torque;
        torque_unsaturated += proportional_torque + vel_integrator_torque_;
        velocity_loop_torque_ = proportional_torque + vel_integrator_torque_;

        // Coulomb-friction + static breakaway feed-forward. Only for ABZ
        // incremental encoder in velocity control (never torque / SPI /
        // sensorless / position hold). The breakaway torque must exit after
        // the rotor starts moving, leaving only the Coulomb term.
        low_speed_friction_torque_ = 0.0f;
        low_speed_compensator_state_ = FrictionCompensator::STATE_IDLE;
        friction_target_torque_ = 0.0f;
        friction_speed_ratio_ = 0.0f;
        friction_assist_blend_ = 0.0f;
        friction_no_progress_time_ = 0.0f;
        friction_recovery_timer_ = 0.0f;
        friction_forward_velocity_ = 0.0f;
        friction_reverse_detected_ = false;
        const bool abz_velocity_mode = cascaded_abz_mode &&
                config_.control_mode == CONTROL_MODE_VELOCITY_CONTROL;
        const bool friction_enabled =
                abz_velocity_mode && config_.enable_low_speed_compensation;
        friction_compensator_.configure(
                config_.abz_coulomb_friction_torque,
                config_.abz_breakaway_torque,
                config_.friction_command_threshold,
                config_.friction_stall_confirm_time,
                config_.friction_recovery_speed_ratio,
                config_.friction_recovery_confirm_time,
                config_.friction_stall_velocity_threshold,
                config_.friction_reverse_velocity_threshold,
                config_.friction_breakaway_rise_rate,
                config_.friction_assist_reengage_rate,
                config_.friction_recovery_release_rate,
                config_.friction_disable_fall_rate);
        const bool ff_active = friction_enabled &&
                std::abs(vel_des) >= config_.friction_command_threshold;
        if (ff_active) {
            const FrictionCompensationResult compensation =
                    friction_compensator_.update(
                            true, vel_des, velocity_feedback, v_err,
                            axis_->encoder_.shadow_count_,
                            current_meas_period);
            low_speed_friction_torque_ = compensation.friction_torque;
            low_speed_compensator_state_ = compensation.state;
            friction_target_torque_ = compensation.target_torque;
            friction_speed_ratio_ = compensation.speed_ratio;
            friction_assist_blend_ = compensation.assist_blend;
            friction_no_progress_time_ = compensation.no_progress_time;
            friction_recovery_timer_ = compensation.recovery_timer;
            friction_forward_velocity_ = compensation.forward_velocity;
            friction_reverse_detected_ = compensation.reverse_detected;
        } else {
            // Graceful ramp-to-zero instead of a one-cycle torque step.
            low_speed_friction_torque_ =
                    friction_compensator_.disable(current_meas_period);
            low_speed_compensator_state_ = friction_compensator_.state();
            position_low_speed_active_ = false;
        }
        torque_unsaturated += low_speed_friction_torque_;
    } else {
        velocity_control_feedback_valid_ = false;
        velocity_error_ = 0.0f;
        velocity_loop_torque_ = 0.0f;
        velocity_proportional_torque_ = 0.0f;
        friction_compensator_.clear();
        low_speed_friction_torque_ = 0.0f;
        low_speed_compensator_state_ = FrictionCompensator::STATE_IDLE;
        position_low_speed_active_ = false;
    }

    // ABZ velocity-loop torque limit (separate from the motor global limit).
    // P + I + FF is recorded, then clamped symmetrically before the global
    // motor torque limit applies.
    float torque_pre_abz_limit = torque_unsaturated;
    if (cascaded_abz_mode) {
        abz_velocity_torque_before_limit_ = torque_unsaturated;
        const float abz_lim = config_.abz_velocity_torque_limit;
        if (std::isfinite(abz_lim) && abz_lim > 0.0f) {
            const float clamped = std::clamp(torque_unsaturated, -abz_lim, abz_lim);
            abz_velocity_torque_after_limit_ = clamped;
            abz_velocity_torque_saturated_ = (clamped != torque_unsaturated);
            torque_unsaturated = clamped;
        } else {
            abz_velocity_torque_after_limit_ = torque_unsaturated;
            abz_velocity_torque_saturated_ = false;
        }
    } else {
        abz_velocity_torque_before_limit_ = 0.0f;
        abz_velocity_torque_after_limit_ = 0.0f;
        abz_velocity_torque_saturated_ = false;
    }

    torque = torque_unsaturated;

    // Velocity limiting in current mode
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL && config_.enable_current_mode_vel_limit) {
        if (!vel_estimate_src) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        torque = limitVel(config_.vel_limit, *vel_estimate_src, vel_gain, torque);
        torque_unsaturated = torque;
    }

    // Torque limiting (records saturation for the anti-windup below).
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
    torque_unsaturated_ = torque_unsaturated;
    motor_torque_saturated_ = limited;

    // Velocity integrator: conditional anti-windup. The integrator only
    // advances when the torque is not already saturated in the direction the
    // velocity error is pushing it, and it releases as soon as the error
    // reverses sign.
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL) {
        // reset integral if not in use
        vel_integrator_torque_ = 0.0f;
    } else {
        // Tightest active limit: the ABZ velocity torque limit and the motor
        // global torque limit both count toward windup.
        float effective_limit = Tlim;
        if (cascaded_abz_mode) {
            const float abz_lim = config_.abz_velocity_torque_limit;
            if (std::isfinite(abz_lim) && abz_lim > 0.0f)
                effective_limit = std::min(abz_lim, Tlim);
        }
        const bool saturated_high = torque_pre_abz_limit > effective_limit;
        const bool saturated_low = torque_pre_abz_limit < -effective_limit;
        const bool windup_forward = saturated_high && v_err > 0.0f;
        const bool windup_reverse = saturated_low && v_err < 0.0f;
        if (!windup_forward && !windup_reverse) {
            vel_integrator_torque_ +=
                    (vel_integrator_gain * gain_scheduling_multiplier) *
                    v_err * current_meas_period;
        }
        // Bound the integrator to the tightest active limit so a saturated
        // transient cannot store a large torque to release later. The ABZ
        // integrator limit is itself bounded by the ABZ velocity torque limit.
        float integrator_limit = effective_limit;
        if (cascaded_abz_mode) {
            const float il = config_.abz_velocity_integrator_limit;
            if (std::isfinite(il) && il > 0.0f)
                integrator_limit = std::min(il, effective_limit);
        }
        vel_integrator_torque_ = std::clamp(
                vel_integrator_torque_, -integrator_limit, integrator_limit);
    }

    final_torque_ = torque;
    if (torque_setpoint_output) *torque_setpoint_output = torque;
    return true;
}
