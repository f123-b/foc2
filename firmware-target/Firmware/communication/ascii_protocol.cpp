/*
* The ASCII protocol is a simpler, human readable alternative to the main native
* protocol.
* In the future this protocol might be extended to support selected GCode commands.
* For a list of supported commands see doc/ascii-protocol.md
*/

/* Includes ------------------------------------------------------------------*/

#include "odrive_main.h"
#include "communication.h"
#include "ascii_protocol.hpp"
#include <utils.hpp>
#include <fibre/cpp_utils.hpp>

#include "autogen/type_info.hpp"
#include "communication/interface_can.hpp"
#include "low_level.h"

#include <cmath>

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/

#define MAX_LINE_LENGTH 256
#define TO_STR_INNER(s) #s
#define TO_STR(s) TO_STR_INNER(s)

/* Private variables ---------------------------------------------------------*/

static Introspectable root_obj = ODriveTypeInfo<ODrive>::make_introspectable(odrv);

/* Private function prototypes -----------------------------------------------*/
/* Function implementations --------------------------------------------------*/

// @brief Sends a line on the specified output.
template<typename ... TArgs>
void respond(StreamSink& output, bool include_checksum, const char * fmt, TArgs&& ... args) {
    char response[384]; // Large enough for one FOC Studio diagnostic telemetry record.
    size_t len = snprintf(response, sizeof(response), fmt, std::forward<TArgs>(args)...);
    len = std::min(len, sizeof(response));
    output.process_bytes((uint8_t*)response, len, nullptr); // TODO: use process_all instead
    if (include_checksum) {
        uint8_t checksum = 0;
        for (size_t i = 0; i < len; ++i)
            checksum ^= response[i];
        len = snprintf(response, sizeof(response), "*%u", checksum);
        len = std::min(len, sizeof(response));
        output.process_bytes((uint8_t*)response, len, nullptr);
    }
    output.process_bytes((const uint8_t*)"\r\n", 2, nullptr);
}

enum FocFeedbackMode : uint8_t {
    FOC_MODE_SPI = 0,
    FOC_MODE_ABZ = 1,
    FOC_MODE_SENSORLESS = 2,
    FOC_MODE_SENSORLESS_SPI_MONITOR = 3,
    FOC_MODE_SENSORLESS_ABZ_MONITOR = 4,
};

static uint8_t foc_feedback_modes[AXIS_COUNT] = {0xff, 0xff};

struct FocVelocityTuningState {
    bool active;
    float vel_gain;
    float vel_integrator_gain;
    float vel_ramp_rate;
    Controller::InputMode input_mode;
};

static FocVelocityTuningState foc_velocity_tuning[AXIS_COUNT] = {};

struct FocPositionLimitState {
    bool active;
    bool cogging_calibration;
    float vel_limit;
    float torque_limit;
    float pos_gain;
    float vel_limit_tolerance;
};

struct FocTorqueSafetyState {
    bool active;
    float vel_limit;
    float vel_limit_tolerance;
    bool enable_current_mode_vel_limit;
    bool enable_overspeed_error;
};

static FocPositionLimitState foc_position_limits[AXIS_COUNT] = {};
static FocTorqueSafetyState foc_torque_safety[AXIS_COUNT] = {};

static uint8_t get_foc_feedback_mode(unsigned axis_num) {
    if (foc_feedback_modes[axis_num] == 0xff) {
        foc_feedback_modes[axis_num] = (axes[axis_num]->encoder_.config_.mode == Encoder::MODE_SPI_ABS_AMS)
                ? FOC_MODE_SPI : FOC_MODE_ABZ;
    }
    return foc_feedback_modes[axis_num];
}

static bool foc_mode_is_sensorless(uint8_t mode) {
    return mode >= FOC_MODE_SENSORLESS;
}

static void restore_foc_velocity_tuning(Axis* axis) {
    FocVelocityTuningState& tuning = foc_velocity_tuning[axis->axis_num_];
    if (!tuning.active)
        return;
    Controller::Config_t& controller = axis->controller_.config_;
    controller.vel_gain = tuning.vel_gain;
    controller.vel_integrator_gain = tuning.vel_integrator_gain;
    controller.vel_ramp_rate = tuning.vel_ramp_rate;
    controller.input_mode = tuning.input_mode;
    axis->encoder_.reset_incremental_velocity_window();
    tuning.active = false;
}

static void restore_foc_position_limits(Axis* axis) {
    FocPositionLimitState& limits = foc_position_limits[axis->axis_num_];
    if (!limits.active)
        return;
    axis->controller_.config_.vel_limit = limits.vel_limit;
    axis->motor_.config_.torque_lim = limits.torque_limit;
    axis->controller_.config_.pos_gain = limits.pos_gain;
    axis->controller_.config_.vel_limit_tolerance = limits.vel_limit_tolerance;
    limits.active = false;
    limits.cogging_calibration = false;
}

static void apply_foc_position_limits(Axis* axis, float vel_limit, float torque_limit) {
    FocPositionLimitState& limits = foc_position_limits[axis->axis_num_];
    if (!limits.active) {
        limits.vel_limit = axis->controller_.config_.vel_limit;
        limits.torque_limit = axis->motor_.config_.torque_lim;
        limits.pos_gain = axis->controller_.config_.pos_gain;
        limits.vel_limit_tolerance = axis->controller_.config_.vel_limit_tolerance;
        limits.active = true;
    }
    axis->controller_.config_.vel_limit = std::min(std::abs(vel_limit), limits.vel_limit);
    axis->motor_.config_.torque_lim = std::min(std::abs(torque_limit), limits.torque_limit);
    axis->controller_.config_.pos_gain = std::min(1.0f, limits.pos_gain);
    axis->controller_.config_.vel_limit_tolerance = std::max(2.0f, limits.vel_limit_tolerance);
}

static void restore_completed_cogging_limits(Axis* axis) {
    FocPositionLimitState& limits = foc_position_limits[axis->axis_num_];
    if (limits.active && limits.cogging_calibration &&
            !axis->controller_.config_.anticogging.calib_anticogging) {
        restore_foc_position_limits(axis);
    }
}

static void restore_foc_torque_safety(Axis* axis) {
    FocTorqueSafetyState& safety = foc_torque_safety[axis->axis_num_];
    if (!safety.active)
        return;
    axis->controller_.config_.vel_limit = safety.vel_limit;
    axis->controller_.config_.vel_limit_tolerance = safety.vel_limit_tolerance;
    axis->controller_.config_.enable_current_mode_vel_limit =
            safety.enable_current_mode_vel_limit;
    axis->controller_.config_.enable_overspeed_error = safety.enable_overspeed_error;
    safety.active = false;
}

// A sustained torque command on an unloaded rotor has no software speed
// boundary of its own and would otherwise accelerate until the overspeed
// fault trips. Cap the software velocity boundary at the proven low-speed
// operating point; the configured limit remains the tighter bound when it is
// already lower. The independent overspeed fault, motor current limit and
// thermal protection remain enabled as a second line of defence.
static constexpr float FOC_TORQUE_MODE_VEL_LIMIT = 2.0f;

static void apply_foc_torque_safety(Axis* axis) {
    FocTorqueSafetyState& safety = foc_torque_safety[axis->axis_num_];
    if (!safety.active) {
        safety.vel_limit = axis->controller_.config_.vel_limit;
        safety.vel_limit_tolerance = axis->controller_.config_.vel_limit_tolerance;
        safety.enable_current_mode_vel_limit =
                axis->controller_.config_.enable_current_mode_vel_limit;
        safety.enable_overspeed_error = axis->controller_.config_.enable_overspeed_error;
        safety.active = true;
    }
    axis->controller_.config_.vel_limit = std::min(
            std::abs(axis->controller_.config_.vel_limit),
            FOC_TORQUE_MODE_VEL_LIMIT);
    axis->controller_.config_.enable_current_mode_vel_limit = true;
    axis->controller_.config_.enable_overspeed_error = true;
}

static void apply_foc_velocity_tuning(Axis* axis) {
    FocVelocityTuningState& tuning = foc_velocity_tuning[axis->axis_num_];
    Controller::Config_t& controller = axis->controller_.config_;
    if (!tuning.active) {
        tuning.vel_gain = controller.vel_gain;
        tuning.vel_integrator_gain = controller.vel_integrator_gain;
        tuning.vel_ramp_rate = controller.vel_ramp_rate;
        tuning.input_mode = controller.input_mode;
        axis->encoder_.reset_incremental_velocity_window();
        tuning.active = true;
    }
    // The ABZ low-speed plant needs more proportional torque to cross
    // static friction. Controller-side scheduling tapers this back as the
    // commanded speed approaches the proven 2 turn/s operating region.
    controller.vel_gain = 0.0025f;
    controller.vel_integrator_gain = 0.01f;
    // Normal commands should reach the requested speed before the low-speed
    // breakaway assist builds a large torque. The cogging scan overrides this
    // with its deliberately slow 0.45 turn/s^2 ramp when it starts.
    controller.vel_ramp_rate = 1.0f;
    controller.input_mode = Controller::INPUT_MODE_VEL_RAMP;
}

static bool set_foc_control_mode(Axis* axis, Controller::ControlMode mode) {
    // FOC Studio selects a controller while the axis is still Idle and arms it
    // in the following command. Re-arm the runtime profile here so telemetry
    // between those commands cannot leave a stale or calibration-modified
    // profile active.
    if (mode == Controller::CONTROL_MODE_VELOCITY_CONTROL &&
            axis->current_state_ == Axis::AXIS_STATE_IDLE &&
            foc_velocity_tuning[axis->axis_num_].active) {
        restore_foc_velocity_tuning(axis);
    }
    if (mode != Controller::CONTROL_MODE_POSITION_CONTROL)
        restore_foc_position_limits(axis);
    if (mode != Controller::CONTROL_MODE_TORQUE_CONTROL)
        restore_foc_torque_safety(axis);
    if (mode != Controller::CONTROL_MODE_VELOCITY_CONTROL)
        restore_foc_velocity_tuning(axis);
    bool changed = axis->controller_.config_.control_mode != mode;
    axis->controller_.config_.control_mode = mode;
    if (mode == Controller::CONTROL_MODE_VELOCITY_CONTROL) {
        apply_foc_velocity_tuning(axis);
        if (changed) {
            // Start the velocity ramp at the measured speed. This avoids a
            // stale setpoint/integrator producing a torque impulse when a
            // running axis changes from torque or position control.
            const uint8_t feedback_mode = get_foc_feedback_mode(axis->axis_num_);
            const float measured_velocity = foc_mode_is_sensorless(feedback_mode)
                    ? axis->sensorless_estimator_.vel_estimate_
                    : axis->encoder_.vel_estimate_;
            axis->controller_.vel_setpoint_ = std::isfinite(measured_velocity)
                    ? measured_velocity : 0.0f;
            axis->controller_.torque_setpoint_ = 0.0f;
            axis->controller_.input_torque_ = 0.0f;
        }
    } else if (mode == Controller::CONTROL_MODE_TORQUE_CONTROL) {
        axis->controller_.config_.input_mode = Controller::INPUT_MODE_PASSTHROUGH;
        apply_foc_torque_safety(axis);
    } else {
        axis->controller_.config_.input_mode = Controller::INPUT_MODE_PASSTHROUGH;
        if (changed) {
            axis->controller_.pos_setpoint_ = axis->encoder_.pos_estimate_;
            axis->controller_.vel_setpoint_ = axis->encoder_.vel_estimate_;
            axis->controller_.input_pos_ = axis->encoder_.pos_estimate_;
            axis->controller_.input_vel_ = 0.0f;
            axis->controller_.input_torque_ = 0.0f;
        }
    }
    if (changed)
        axis->controller_.vel_integrator_torque_ = 0.0f;
    return changed;
}

static bool set_foc_feedback_mode(Axis* axis, uint8_t mode) {
    if (axis->current_state_ != Axis::AXIS_STATE_IDLE || mode > FOC_MODE_SENSORLESS_ABZ_MONITOR)
        return false;

    restore_foc_velocity_tuning(axis);
    restore_foc_position_limits(axis);
    if (foc_mode_is_sensorless(mode)) {
        // Saved ODrive settings override main.cpp defaults. Reapply the
        // project motor profile here so stale NVM cannot break spin-up.
        Axis::LockinConfig_t& ramp = axis->config_.sensorless_ramp;
        ramp.current = std::min(1.5f, axis->motor_.config_.current_lim);
        ramp.ramp_time = 1.5f;
        ramp.ramp_distance = 2.0f * M_PI;
        ramp.accel = 100.0f;
        ramp.vel = 400.0f;
        ramp.finish_on_vel = true;
        ramp.finish_on_distance = false;
        ramp.finish_on_enc_idx = false;
        const float pole_pairs = std::max((float)axis->motor_.config_.pole_pairs, 1.0f);
        axis->sensorless_estimator_.config_.pm_flux_linkage = 5.51328895422f / (pole_pairs * 650.0f);
    }
    Encoder::Mode encoder_mode = Encoder::MODE_INCREMENTAL;
    int32_t encoder_cpr = 4000;
    if (mode == FOC_MODE_SPI || mode == FOC_MODE_SENSORLESS_SPI_MONITOR) {
        encoder_mode = Encoder::MODE_SPI_ABS_AMS;
        encoder_cpr = 16384;
    }

    axis->encoder_.config_.mode = encoder_mode;
    axis->encoder_.config_.cpr = encoder_cpr;
    axis->encoder_.config_.set_bandwidth(100.0f);
    axis->encoder_.mode_ = encoder_mode;
    axis->encoder_.is_ready_ = false;
    axis->encoder_.pos_estimate_valid_ = false;
    axis->encoder_.vel_estimate_valid_ = false;
    axis->encoder_.reset_incremental_velocity_window();
    axis->encoder_.spi_error_rate_ = 0.0f;
    axis->encoder_.error_ = Encoder::ERROR_NONE;
    if (encoder_mode == Encoder::MODE_SPI_ABS_AMS) {
        axis->encoder_.abs_spi_cs_pin_init();
        axis->encoder_.abs_spi_init();
    }
    foc_feedback_modes[axis->axis_num_] = mode;
    return true;
}


// @brief Executes an ASCII protocol command
// @param buffer buffer of ASCII encoded characters
// @param len size of the buffer
void ASCII_protocol_process_line(const uint8_t* buffer, size_t len, StreamSink& response_channel) {
    static_assert(sizeof(char) == sizeof(uint8_t));

    // scan line to find beginning of checksum and prune comment
    uint8_t checksum = 0;
    size_t checksum_start = SIZE_MAX;
    for (size_t i = 0; i < len; ++i) {
        if (buffer[i] == ';') { // ';' is the comment start char
            len = i;
            break;
        }
        if (checksum_start > i) {
            if (buffer[i] == '*') {
                checksum_start = i + 1;
            } else {
                checksum ^= buffer[i];
            }
        }
    }

    // copy everything into a local buffer so we can insert null-termination
    char cmd[MAX_LINE_LENGTH + 1];
    if (len > MAX_LINE_LENGTH) len = MAX_LINE_LENGTH;
    memcpy(cmd, buffer, len);
    cmd[len] = 0; // null-terminate

    // optional checksum validation
    bool use_checksum = (checksum_start < len);
    if (use_checksum) {
        unsigned int received_checksum;
        int numscan = sscanf((const char *)cmd + checksum_start, "%u", &received_checksum);
        if ((numscan < 1) || (received_checksum != checksum))
            return;
        len = checksum_start - 1; // prune checksum and asterisk
        cmd[len] = 0; // null-terminate
    }

    // A completed or fault-aborted cogging scan leaves the axis in Idle, but
    // the scan limits must not leak into the next speed command or a Flash
    // save. Telemetry/commands are serviced frequently, so clean this up at
    // the protocol boundary as soon as the controller clears its scan flag.
    for (unsigned axis_num = 0; axis_num < AXIS_COUNT; ++axis_num)
        restore_completed_cogging_limits(axes[axis_num]);


    // check incoming packet type
    if (cmd[0] == 'm') { // FOC Studio feedback mode (idle-only)
        unsigned motor_number;
        unsigned mode;
        int numscan = sscanf(cmd, "m %u %u", &motor_number, &mode);
        if (numscan < 2 || motor_number >= AXIS_COUNT || mode > FOC_MODE_SENSORLESS_ABZ_MONITOR) {
            respond(response_channel, use_checksum, "err mode");
        } else if (!set_foc_feedback_mode(axes[motor_number], (uint8_t)mode)) {
            respond(response_channel, use_checksum, "err not-idle");
        } else {
            respond(response_channel, use_checksum, "ok mode %u", mode);
        }

    } else if (cmd[0] == 'g') { // FOC Studio high-rate scope telemetry
        unsigned motor_number;
        int numscan = sscanf(cmd, "g %u", &motor_number);
        if (numscan < 1 || motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "err axis");
        } else {
            Axis* axis = axes[motor_number];
            const uint8_t mode = get_foc_feedback_mode(motor_number);
            const float raw_velocity = foc_mode_is_sensorless(mode)
                    ? axis->sensorless_estimator_.vel_estimate_
                    : axis->encoder_.vel_estimate_;
            const float velocity = axis->controller_.velocity_feedback_for_telemetry(raw_velocity);
            const bool power_stage_active =
                    axis->current_state_ != Axis::AXIS_STATE_IDLE &&
                    axis->motor_.armed_state_ != Motor::ARMED_STATE_DISARMED;
            const Motor::CurrentControl_t& current_control = axis->motor_.current_control_;
            const float reported_current = power_stage_active ? current_control.Iq_measured : 0.0f;
            const float v_alpha = power_stage_active ? current_control.final_v_alpha : 0.0f;
            const float v_beta = power_stage_active ? current_control.final_v_beta : 0.0f;
            axis->watchdog_feed();
            respond(response_channel, use_checksum,
                    "! %u %.6g %.6g %.6g %.6g %.6g %.6g %.6g %.6g %.6g %.6g %.6g %.6g %.6g %.6g %.6g %.6g %.6g %u %.6g %.6g %.6g %.6g %.6g %.6g %.6g %u %.6g",
                    (unsigned)axis->current_state_, (double)velocity, (double)reported_current,
                    (double)axis->encoder_.pos_estimate_, (double)vbus_voltage,
                    (double)v_alpha,
                    (double)(-0.5f * v_alpha + 0.8660254038f * v_beta),
                    (double)(-0.5f * v_alpha - 0.8660254038f * v_beta),
                    (double)(power_stage_active ? current_control.Id_measured : 0.0f),
                    (double)(power_stage_active ? current_control.Iq_setpoint : 0.0f),
                    (double)(power_stage_active ? current_control.Id_setpoint : 0.0f),
                    (double)axis->controller_.vel_setpoint_, (double)raw_velocity,
                    (double)axis->encoder_.incremental_window_velocity_,
                    (double)axis->controller_.vel_integrator_torque_,
                    (double)axis->controller_.low_speed_friction_torque_,
                    (double)axis->controller_.pos_setpoint_,
                    (double)axis->controller_.position_error_,
                    (unsigned)axis->controller_.low_speed_compensator_state_,
                    (double)axis->controller_.velocity_proportional_torque_,
                    (double)axis->controller_.anticogging_torque_,
                    (double)axis->controller_.final_torque_,
                    (double)axis->motor_.max_available_torque(),
                    (double)axis->encoder_.control_velocity_estimate_,
                    (double)axis->controller_.velocity_error_,
                    (double)axis->controller_.torque_unsaturated_,
                    (unsigned)axis->controller_.torque_saturated_,
                    (double)axis->encoder_.incremental_velocity_estimator_.time_since_last_edge());
        }

    } else if (cmd[0] == 'j') { // FOC Studio aggregate telemetry
        unsigned motor_number;
        int numscan = sscanf(cmd, "j %u", &motor_number);
        if (numscan < 1 || motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "err axis");
        } else {
            Axis* axis = axes[motor_number];
            uint8_t mode = get_foc_feedback_mode(motor_number);
            const float raw_velocity = foc_mode_is_sensorless(mode)
                    ? axis->sensorless_estimator_.vel_estimate_
                    : axis->encoder_.vel_estimate_;
            float velocity = axis->controller_.velocity_feedback_for_telemetry(raw_velocity);
            float angle_error = 0.0f;
            if (mode == FOC_MODE_SENSORLESS_SPI_MONITOR || mode == FOC_MODE_SENSORLESS_ABZ_MONITOR) {
                angle_error = wrap_pm_pi(axis->sensorless_estimator_.phase_ - axis->encoder_.phase_) / (2.0f * M_PI);
            }
            // Keep the first eleven fields backward compatible. The extra
            // fields let FOC Studio explain the aggregate axis fault without
            // requiring a second burst of property reads.
            const bool power_stage_active =
                    axis->current_state_ != Axis::AXIS_STATE_IDLE &&
                    axis->motor_.armed_state_ != Motor::ARMED_STATE_DISARMED;
            const Motor::CurrentControl_t& current_control = axis->motor_.current_control_;
            const float reported_current = power_stage_active ? current_control.Iq_measured : 0.0f;
            const float v_alpha = power_stage_active ? current_control.final_v_alpha : 0.0f;
            const float v_beta = power_stage_active ? current_control.final_v_beta : 0.0f;
            const float phase_a_voltage = v_alpha;
            const float phase_b_voltage = -0.5f * v_alpha + 0.8660254038f * v_beta;
            const float phase_c_voltage = -0.5f * v_alpha - 0.8660254038f * v_beta;
            const float id_measured = power_stage_active ? current_control.Id_measured : 0.0f;
            const float iq_setpoint = power_stage_active ? current_control.Iq_setpoint : 0.0f;
            const float id_setpoint = power_stage_active ? current_control.Id_setpoint : 0.0f;
            axis->watchdog_feed();
            respond(response_channel, use_checksum, "@ %u %lu %.6g %.6g %.6g %.6g %.6g %u %.6g %u %lu %lu %lu %lu %u %u %u %ld %lu %lu %u %.6g %.6g %.6g %.6g %.6g %.6g %u %u %lu %u",
                    (unsigned)axis->current_state_, (unsigned long)axis->error_,
                    (double)velocity, (double)reported_current,
                    (double)axis->encoder_.pos_estimate_, (double)vbus_voltage,
                    (double)axis->fet_thermistor_.temperature_,
                    (unsigned)axis->sensorless_estimator_.estimator_good_,
                    (double)angle_error, (unsigned)mode,
                    (unsigned long)axis->motor_.error_,
                    (unsigned long)axis->encoder_.error_,
                    (unsigned long)axis->controller_.error_,
                    (unsigned long)axis->sensorless_estimator_.error_,
                    (unsigned)axis->motor_.armed_state_,
                    (unsigned)axis->encoder_.is_ready_,
                    (unsigned)axis->motor_.is_calibrated_,
                    (long)axis->motor_.config_.direction,
                    (unsigned long)axis->fet_thermistor_.error_,
                    (unsigned long)axis->motor_thermistor_.error_,
                    (unsigned)axis->controller_.config_.control_mode,
                    (double)phase_a_voltage, (double)phase_b_voltage,
                    (double)phase_c_voltage, (double)id_measured,
                    (double)iq_setpoint, (double)id_setpoint,
                    (unsigned)axis->controller_.anticogging_valid_,
                    (unsigned)axis->controller_.config_.anticogging.calib_anticogging,
                    (unsigned long)axis->controller_.config_.anticogging.index,
                    (unsigned)axis->controller_.anticogging_valid_bin_count_);
        }

    } else if (cmd[0] == 'x') { // FOC Studio immediate stop
        unsigned motor_number;
        int numscan = sscanf(cmd, "x %u", &motor_number);
        if (numscan < 1 || motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "err axis");
        } else {
            Axis* axis = axes[motor_number];
            restore_foc_velocity_tuning(axis);
            restore_foc_position_limits(axis);
            axis->controller_.input_torque_ = 0.0f;
            axis->controller_.input_vel_ = 0.0f;
            axis->controller_.input_pos_ = axis->encoder_.pos_estimate_;
            axis->controller_.reset();
            axis->motor_.reset_current_control();
            safety_critical_disarm_motor_pwm(axis->motor_);
            axis->requested_state_ = Axis::AXIS_STATE_IDLE;
            respond(response_channel, use_checksum, "ok stopped");
        }

    } else if (cmd[0] == 'k') { // FOC Studio clear errors
        unsigned motor_number;
        int numscan = sscanf(cmd, "k %u", &motor_number);
        if (numscan < 1 || motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "err axis");
        } else if (axes[motor_number]->current_state_ != Axis::AXIS_STATE_IDLE) {
            respond(response_channel, use_checksum, "err not-idle");
        } else {
            axes[motor_number]->clear_errors();
            respond(response_channel, use_checksum, "ok clear");
        }

    } else if (cmd[0] == 'a') { // FOC Studio mode-aware calibration
        unsigned motor_number;
        int numscan = sscanf(cmd, "a %u", &motor_number);
        if (numscan < 1 || motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "err axis");
        } else if (axes[motor_number]->current_state_ != Axis::AXIS_STATE_IDLE) {
            respond(response_channel, use_checksum, "err not-idle");
        } else {
            axes[motor_number]->requested_state_ = foc_mode_is_sensorless(get_foc_feedback_mode(motor_number))
                    ? Axis::AXIS_STATE_MOTOR_CALIBRATION
                    : Axis::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
            respond(response_channel, use_checksum, "ok calibrating");
        }

    } else if (cmd[0] == 'b') { // FOC Studio ABZ anti-cogging calibration
        unsigned motor_number;
        int numscan = sscanf(cmd, "b %u", &motor_number);
        if (numscan < 1 || motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "err axis");
        } else {
            Axis* axis = axes[motor_number];
            if (get_foc_feedback_mode(motor_number) != FOC_MODE_ABZ) {
                respond(response_channel, use_checksum, "err abz-only");
            } else if (axis->current_state_ != Axis::AXIS_STATE_IDLE) {
                respond(response_channel, use_checksum, "err not-idle");
            } else if (axis->error_ != Axis::ERROR_NONE ||
                    !axis->motor_.is_calibrated_ || !axis->encoder_.is_ready_) {
                respond(response_channel, use_checksum, "err not-ready");
            } else {
                restore_foc_velocity_tuning(axis);
                restore_foc_torque_safety(axis);
                // Position-hold calibration: step through the mechanical cycle,
                // settle, and sample the holding torque at each position. This
                // does not depend on the velocity loop being stable at scan
                // speed, so it is the preferred calibration for the ABZ axis.
                set_foc_control_mode(axis, Controller::CONTROL_MODE_POSITION_CONTROL);
                apply_foc_position_limits(axis, 1.0f, 0.015f);
                foc_position_limits[axis->axis_num_].cogging_calibration = true;
                axis->controller_.start_anticogging_calibration(false);
                axis->requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
                axis->watchdog_feed();
                respond(response_channel, use_checksum, "ok cogging-calibrating");
            }
        }

    } else if (cmd[0] == 'p') { // position control
        unsigned motor_number;
        float pos_setpoint, vel_feed_forward, torque_feed_forward;
        int numscan = sscanf(cmd, "p %u %f %f %f", &motor_number, &pos_setpoint, &vel_feed_forward, &torque_feed_forward);
        if (numscan < 2) {
            respond(response_channel, use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "invalid motor %u", motor_number);
        } else {
            Axis* axis = axes[motor_number];
            set_foc_control_mode(axis, Controller::CONTROL_MODE_POSITION_CONTROL);
            axis->controller_.input_pos_ = pos_setpoint;
            axis->controller_.input_vel_ = 0.0f;
            axis->controller_.input_torque_ = 0.0f;
            if (numscan >= 3)
                axis->controller_.input_vel_ = vel_feed_forward;
            if (numscan >= 4)
                axis->controller_.input_torque_ = torque_feed_forward;
            axis->controller_.input_pos_updated();
            axis->watchdog_feed();
        }

    } else if (cmd[0] == 'q') { // position control with limits
        unsigned motor_number;
        float pos_setpoint, vel_limit, torque_lim;
        int numscan = sscanf(cmd, "q %u %f %f %f", &motor_number, &pos_setpoint, &vel_limit, &torque_lim);
        if (numscan < 2) {
            respond(response_channel, use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "invalid motor %u", motor_number);
        } else {
            Axis* axis = axes[motor_number];
            set_foc_control_mode(axis, Controller::CONTROL_MODE_POSITION_CONTROL);
            axis->controller_.input_pos_ = pos_setpoint;
            axis->controller_.input_vel_ = 0.0f;
            axis->controller_.input_torque_ = 0.0f;
            if (numscan >= 3)
                axis->controller_.config_.vel_limit = std::abs(vel_limit);
            if (numscan >= 4)
                axis->motor_.config_.torque_lim = std::abs(torque_lim);
            axis->controller_.input_pos_updated();
            axis->watchdog_feed();
        }

    } else if (cmd[0] == 'v') { // velocity control
        unsigned motor_number;
        float vel_setpoint, torque_feed_forward;
        int numscan = sscanf(cmd, "v %u %f %f", &motor_number, &vel_setpoint, &torque_feed_forward);
        if (numscan < 2) {
            respond(response_channel, use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "invalid motor %u", motor_number);
        } else {
            Axis* axis = axes[motor_number];
            set_foc_control_mode(axis, Controller::CONTROL_MODE_VELOCITY_CONTROL);
            if (std::abs(axis->controller_.input_vel_ - vel_setpoint) > 0.01f)
                axis->controller_.vel_integrator_torque_ = 0.0f;
            axis->controller_.input_vel_ = vel_setpoint;
            axis->controller_.input_torque_ = 0.0f;
            if (numscan >= 3)
                axis->controller_.input_torque_ = torque_feed_forward;
            axis->watchdog_feed();
        }

    } else if (cmd[0] == 'c') { // torque control
        unsigned motor_number;
        float torque_setpoint;
        int numscan = sscanf(cmd, "c %u %f", &motor_number, &torque_setpoint);
        if (numscan < 2) {
            respond(response_channel, use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "invalid motor %u", motor_number);
        } else {
            Axis* axis = axes[motor_number];
            set_foc_control_mode(axis, Controller::CONTROL_MODE_TORQUE_CONTROL);
            axis->controller_.input_vel_ = 0.0f;
            axis->controller_.input_torque_ = torque_setpoint;
            axis->watchdog_feed();
        }

    } else if (cmd[0] == 't') { // trapezoidal trajectory
        unsigned motor_number;
        float goal_point;
        int numscan = sscanf(cmd, "t %u %f", &motor_number, &goal_point);
        if (numscan < 2) {
            respond(response_channel, use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "invalid motor %u", motor_number);
        } else {
            Axis* axis = axes[motor_number];
            set_foc_control_mode(axis, Controller::CONTROL_MODE_POSITION_CONTROL);
            axis->trap_traj_.config_.vel_limit = 0.30f;
            axis->trap_traj_.config_.accel_limit = 0.60f;
            axis->trap_traj_.config_.decel_limit = 0.60f;
            axis->controller_.config_.input_mode = Controller::INPUT_MODE_TRAP_TRAJ;
            axis->controller_.input_pos_ = goal_point;
            axis->controller_.input_vel_ = 0.0f;
            axis->controller_.input_torque_ = 0.0f;
            axis->controller_.input_pos_updated();
            axis->watchdog_feed();
        }

    } else if (cmd[0] == 'f') { // feedback
        unsigned motor_number;
        int numscan = sscanf(cmd, "f %u", &motor_number);
        if (numscan < 1) {
            respond(response_channel, use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "invalid motor %u", motor_number);
        } else {
            respond(response_channel, use_checksum, "%f %f",
                    (double)axes[motor_number]->encoder_.pos_estimate_,
                    (double)axes[motor_number]->encoder_.vel_estimate_);
        }

    } else if (cmd[0] == 'h') {  // Help
        respond(response_channel, use_checksum, "Please see documentation for more details");
        respond(response_channel, use_checksum, "");
        respond(response_channel, use_checksum, "Available commands syntax reference:");
        respond(response_channel, use_checksum, "Position: q axis pos vel-lim I-lim");
        respond(response_channel, use_checksum, "Position: p axis pos vel-ff I-ff");
        respond(response_channel, use_checksum, "Velocity: v axis vel I-ff");
        respond(response_channel, use_checksum, "Torque: c axis T");
        respond(response_channel, use_checksum, "ABZ cogging calibration: b axis");
        respond(response_channel, use_checksum, "");
        respond(response_channel, use_checksum, "Properties start at odrive root, such as axis0.requested_state");
        respond(response_channel, use_checksum, "Read: r property");
        respond(response_channel, use_checksum, "Write: w property value");
        respond(response_channel, use_checksum, "");
        respond(response_channel, use_checksum, "Save config: ss");
        respond(response_channel, use_checksum, "Erase config: se");
        respond(response_channel, use_checksum, "Reboot: sr");

    } else if (cmd[0] == 'i'){ // Dump device info
        // respond(response_channel, use_checksum, "Signature: %#x", STM_ID_GetSignature());
        // respond(response_channel, use_checksum, "Revision: %#x", STM_ID_GetRevision());
        // respond(response_channel, use_checksum, "Flash Size: %#x KiB", STM_ID_GetFlashSize());
        respond(response_channel, use_checksum, "Hardware version: %d.%d-%dV", odrv.hw_version_major_, odrv.hw_version_minor_, odrv.hw_version_variant_);
        respond(response_channel, use_checksum, "Firmware version: %d.%d.%d", odrv.fw_version_major_, odrv.fw_version_minor_, odrv.fw_version_revision_);
        respond(response_channel, use_checksum, "Serial number: %s", serial_number_str);

    } else if (cmd[0] == 's'){ // System
        if(cmd[1] == 's') { // Save config
            bool all_idle = true;
            for (Axis* axis : axes) {
                all_idle = all_idle && axis->current_state_ == Axis::AXIS_STATE_IDLE;
            }
            if (!all_idle) {
                respond(response_channel, use_checksum, "err not-idle");
            } else {
                for (Axis* axis : axes) {
                    restore_foc_velocity_tuning(axis);
                    restore_foc_position_limits(axis);
                }
                odrv.save_configuration();
                respond(response_channel, use_checksum, "ok saved");
            }
        } else if (cmd[1] == 'e'){ // Erase config
            odrv.erase_configuration();
        } else if (cmd[1] == 'r'){ // Reboot
            odrv.reboot();
        }

    } else if (cmd[0] == 'r') { // read property
        char name[MAX_LINE_LENGTH];
        int numscan = sscanf(cmd, "r %255s", name);
        if (numscan < 1) {
            respond(response_channel, use_checksum, "invalid command format");
        } else {
            Introspectable property = root_obj.get_child(name, sizeof(name));
            const StringConvertibleTypeInfo* type_info = dynamic_cast<const StringConvertibleTypeInfo*>(property.get_type_info());
            if (!type_info) {
                respond(response_channel, use_checksum, "invalid property");
            } else {
                char response[10];
                bool success = type_info->get_string(property, response, sizeof(response));
                if (!success)
                    respond(response_channel, use_checksum, "not implemented");
                else
                    respond(response_channel, use_checksum, response);
            }
        }

    } else if (cmd[0] == 'w') { // write property
        char name[MAX_LINE_LENGTH];
        char value[MAX_LINE_LENGTH];
        int numscan = sscanf(cmd, "w %255s %255s", name, value);
        if (numscan < 1) {
            respond(response_channel, use_checksum, "invalid command format");
        } else {
            Introspectable property = root_obj.get_child(name, sizeof(name));
            const StringConvertibleTypeInfo* type_info = dynamic_cast<const StringConvertibleTypeInfo*>(property.get_type_info());
            if (!type_info) {
                respond(response_channel, use_checksum, "invalid property");
            } else {
                bool success = type_info->set_string(property, value, sizeof(value));
                if (!success)
                    respond(response_channel, use_checksum, "not implemented");
            }
        }

    } else if (cmd[0] == 'u') { // Update axis watchdog. 
        unsigned motor_number;
        int numscan = sscanf(cmd, "u %u", &motor_number);
        if(numscan < 1){
            respond(response_channel, use_checksum, "invalid command format");
        } else if (motor_number >= AXIS_COUNT) {
            respond(response_channel, use_checksum, "invalid motor %u", motor_number);
        }else {
            axes[motor_number]->watchdog_feed();
        }

    } else if (cmd[0] != 0) {
        respond(response_channel, use_checksum, "unknown command");
    }
}

void ASCII_protocol_parse_stream(const uint8_t* buffer, size_t len, StreamSink& response_channel) {
    static uint8_t parse_buffer[MAX_LINE_LENGTH];
    static bool read_active = true;
    static uint32_t parse_buffer_idx = 0;

    while (len--) {
        // if the line becomes too long, reset buffer and wait for the next line
        if (parse_buffer_idx >= MAX_LINE_LENGTH) {
            read_active = false;
            parse_buffer_idx = 0;
        }

        // Fetch the next char
        uint8_t c = *(buffer++);
        bool is_end_of_line = (c == '\r' || c == '\n' || c == '!');
        if (is_end_of_line) {
            if (read_active)
                ASCII_protocol_process_line(parse_buffer, parse_buffer_idx, response_channel);
            parse_buffer_idx = 0;
            read_active = true;
        } else {
            if (read_active) {
                parse_buffer[parse_buffer_idx++] = c;
            }
        }
    }
}
