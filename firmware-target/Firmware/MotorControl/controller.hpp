#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

#include "friction_compensator.hpp"
#include "abz_velocity_observer.hpp"
#include "abz_velocity_utils.hpp"

class Controller : public ODriveIntf::ControllerIntf {
public:
    typedef struct {
        uint32_t index = 0;
        float cogging_map[3600];
        bool pre_calibrated = false;
        bool calib_anticogging = false;
        float calib_pos_threshold = 2.0f;
        float calib_vel_threshold = 8.0f;
        float cogging_ratio = 1.0f;
        bool anticogging_enabled = true;
    } Anticogging_t;

    struct Config_t {
        ControlMode control_mode = CONTROL_MODE_POSITION_CONTROL;  //see: ControlMode_t
        InputMode input_mode = INPUT_MODE_PASSTHROUGH;             //see: InputMode_t
        float pos_gain = 20.0f;                  // [(turn/s) / turn]
        float vel_gain = 1.0f / 6.0f;            // [Nm/(turn/s)]
        // float vel_gain = 0.2f / 200.0f,       // [Nm/(rad/s)] <sensorless example>
        float vel_integrator_gain = 2.0f / 6.0f; // [Nm/(turn/s * s)]
        float vel_limit = 2.0f;                  // [turn/s] Infinity to disable.
        float vel_limit_tolerance = 1.2f;        // ratio to vel_lim. Infinity to disable.
        float vel_ramp_rate = 1.0f;              // [(turn/s) / s]
        float torque_ramp_rate = 0.01f;          // Nm / sec
        bool circular_setpoints = false;
        float circular_setpoint_range = 1.0f; // Circular range when circular_setpoints is true. [turn]
        float inertia = 0.0f;                 // [Nm/(turn/s^2)]
        float input_filter_bandwidth = 2.0f;  // [1/s]
        float homing_speed = 0.25f;           // [turn/s]
        Anticogging_t anticogging;
        float gain_scheduling_width = 10.0f;
        bool enable_gain_scheduling = false;
        bool enable_vel_limit = true;
        bool enable_overspeed_error = true;
        bool enable_current_mode_vel_limit = true;  // enable velocity limit in current control mode (requires a valid velocity estimator)
        // Low-speed friction/breakaway feed-forward. Enabled for the ABZ
        // velocity loop so static friction can be broken without raising Kp.
        bool enable_low_speed_compensation = true;
        // ABZ-specific velocity PI gains (the generic vel_gain/vel_integrator
        // gain are not suitable for the 4000 CPR incremental encoder).  These
        // two gains are the ONLY velocity PI gains used by the ABZ velocity
        // loop, and FOC Studio must write exactly these when tuning ABZ.
        float abz_vel_gain = 0.002f;                 // [Nm/(turn/s)]
        float abz_vel_integrator_gain = 0.002f;      // [Nm/(turn/s * s)]
        // ABZ velocity-control-only low-speed PI scheduling. Below schedule_start
        // the effective gains are the low-speed gains; above schedule_end they are
        // the base gains. The handoff uses smoothstep, never a hard switch.
        bool enable_abz_low_speed_gain_scheduling = true;
        float abz_low_speed_vel_gain = 0.004f;            // [Nm/(turn/s)]
        float abz_low_speed_vel_integrator_gain = 0.0025f;// [Nm/(turn/s * s)]
        float abz_low_speed_gain_schedule_start = 0.5f;   // [turn/s]
        float abz_low_speed_gain_schedule_end = 1.2f;     // [turn/s]
        // Control velocity observer adaptive bandwidth bounds [Hz].  The
        // observer bandwidth follows the commanded speed (see
        // AbzVelocityObserver): ~15 Hz at standstill rising smoothly to
        // ~50 Hz at 4 turn/s, clamped to these two values.
        float abz_velocity_observer_min_bandwidth = 15.0f;
        float abz_velocity_observer_max_bandwidth = 50.0f;
        // ABZ velocity-loop torque limit [Nm], applied to P+I+FF before the
        // motor global torque limit. <= 0 or non-finite disables it.
        float abz_velocity_torque_limit = 0.015f;
        // Bound on the velocity integrator torque [Nm] (<= torque limit).
        float abz_velocity_integrator_limit = 0.015f;
        // Coulomb/dynamic friction feed-forward and static breakaway torque
        // [Nm] for the ABZ velocity loop. breakaway >= coulomb is enforced.
        float abz_coulomb_friction_torque = 0.0015f;
        float abz_breakaway_torque = 0.0055f;
        // Friction/breakaway tuning (was hardcoded in FrictionCompensator).
        float friction_command_threshold = 0.02f;         // [turn/s]
        float friction_stall_confirm_time = 0.06f;        // [s]
        float friction_recovery_speed_ratio = 0.85f;      // []
        float friction_recovery_confirm_time = 0.09f;     // [s]
        float friction_stall_velocity_threshold = 0.05f;  // [turn/s] (ratio floor)
        float friction_reverse_velocity_threshold = 0.02f;// [turn/s]
        float friction_breakaway_rise_rate = 0.020f;      // [Nm/s]
        float friction_assist_reengage_rate = 0.080f;     // [Nm/s]
        float friction_recovery_release_rate = 0.020f;    // [Nm/s]
        float friction_disable_fall_rate = 0.60f;         // [Nm/s]
        // Continuous running friction feed-forward. The static-friction hold
        // fades smoothly to zero at friction_running_assist_fade_speed.
        float friction_running_hold_ratio = 0.70f;        // [0,1]
        float friction_running_assist_fade_speed = 1.0f;  // [turn/s]
        // BREAKAWAY exit requires net directional progress, speed ratio and a
        // sustained qualification timer, not just the first few encoder counts.
        int32_t friction_breakaway_exit_progress_counts = 24;
        float friction_breakaway_exit_speed_ratio = 0.50f;// [0.4,0.6]
        float friction_breakaway_exit_confirm_time = 0.040f; // [s]
        // Anticogging feed-forward tuning.
        float anticogging_phase_offset_bins = 0.0f;       // bin, -180..+180
        float anticogging_torque_limit = 0.005f;          // [Nm] map clamp
        // The velocity scan's map is empirically opposite to the low-speed
        // torque direction on this ABZ setup. Apply its polarity and extra
        // low-speed strength only in ABZ velocity control; position/SPI/
        // sensorless/torque behavior remains unchanged.
        float abz_anticogging_polarity = -1.0f;            // -1 or +1
        float abz_anticogging_low_speed_boost = 3.0f;      // >= 1
        float abz_anticogging_low_speed_fade_speed = 1.2f; // [turn/s]
        // Anticogging bidirectional scan tuning (used on next calibration).
        float anticogging_scan_speed = 2.0f;              // [turn/s]
        // The bidirectional scan must be appreciably steadier than the 2 turn/s
        // command before it records a position bin. More turns improve the
        // forward/reverse average and reject velocity-loop transient torque.
        float anticogging_scan_velocity_tolerance = 0.25f; // [turn/s]
        float anticogging_scan_dwell_time = 0.25f;         // [s]
        float anticogging_scan_turns = 10.0f;              // turns/direction
        // Bound each motion/settling phase. A scan that cannot reach its
        // accepted speed must fail safely instead of rotating indefinitely.
        float anticogging_scan_phase_timeout = 15.0f;      // [s]
        uint16_t anticogging_postprocess_bins_per_cycle = 4;
        float anticogging_calibration_accel = 2.0f;       // [turn/s^2]
        float anticogging_calibration_torque_limit = 0.015f; // [Nm]
        // Map quality gate thresholds.
        float anticogging_quality_min_coverage = 0.8f;
        float anticogging_quality_max_abs = 0.012f;
        float anticogging_quality_max_peak_to_peak = 0.020f;
        float anticogging_quality_max_adjacent_jump = 0.003f;
        float anticogging_quality_max_wrap_jump = 0.003f;
        uint8_t axis_to_mirror = -1;
        float mirror_ratio = 1.0f;
        uint8_t load_encoder_axis = -1;  // default depends on Axis number and is set in load_configuration()

        // custom setters
        Controller* parent;
        void set_input_filter_bandwidth(float value) { input_filter_bandwidth = value; parent->update_filter_gains(); }
    };

    explicit Controller(Config_t& config);
    // Reset transient control state.  A motor arm happens immediately before
    // entering closed-loop control; preserve an explicitly pending cogging
    // calibration across that transition so it is not cancelled before the
    // first control tick.  Safety stops use the default (abort=true).
    void reset(bool abort_anticogging = true);
    void set_error(Error error);

    constexpr void input_pos_updated() {
        input_pos_updated_ = true;
    }

    bool select_encoder(size_t encoder_num);

    // Trajectory-Planned control
    void move_to_pos(float goal_point);
    void move_incremental(float displacement, bool from_goal_point);
    
    // TODO: make this more similar to other calibration loops
    void start_anticogging_calibration() override;
    void start_anticogging_calibration(bool velocity_only);
    bool anticogging_calibration(float pos_estimate, float vel_estimate);
    void abort_anticogging_calibration(uint8_t reason);

    void update_filter_gains();
    bool update(float* torque_setpoint);

    // Control selects one ABZ velocity source before optional high-speed
    // conditioning. Telemetry returns the exact feedback that closed the most
    // recent velocity-control cycle.
    float velocity_feedback_for_control(float raw_velocity,
                                        float commanded_velocity) const;
    float velocity_feedback_for_telemetry(float raw_velocity) const;

    // True when the incremental (ABZ) encoder is actually the velocity
    // feedback source of the cascaded velocity/position loop. Sensorless
    // control reuses MODE_INCREMENTAL as the encoder placeholder while the
    // loop is fed from the sensorless observer, so the encoder mode alone is
    // not sufficient to select the ABZ-specific conditioning.
    bool cascaded_abz_control() const;

    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Error error_ = ERROR_NONE;

    float* pos_estimate_linear_src_ = nullptr;
    float* pos_estimate_circular_src_ = nullptr;
    bool* pos_estimate_valid_src_ = nullptr;
    float* vel_estimate_src_ = nullptr;
    bool* vel_estimate_valid_src_ = nullptr;
    float* pos_wrap_src_ = nullptr; 


    float pos_setpoint_ = 0.0f; // [turns]
    float vel_setpoint_ = 0.0f; // [turn/s]
    // float vel_setpoint = 800.0f; <sensorless example>
    float vel_integrator_torque_ = 0.0f;    // [Nm]
    float torque_setpoint_ = 0.0f;  // [Nm]

    float input_pos_ = 0.0f;     // [turns]
    float input_vel_ = 0.0f;     // [turn/s]
    float input_torque_ = 0.0f;  // [Nm]
    float input_filter_kp_ = 0.0f;
    float input_filter_ki_ = 0.0f;

    // Runtime diagnostics for the FOC Studio speed command.  The encoder PLL
    // remains untouched for torque control, commutation and safety checks.
    float velocity_control_feedback_ = 0.0f;
    bool velocity_control_feedback_valid_ = false;
    // Overspeed is safety-critical, but a single ABZ edge/PLL impulse must
    // not abort a running scan or speed command.  Dual-layer qualified
    // detection: normal layer on the control observer, emergency layer on the
    // raw PLL / 50 ms count window (2x the normal limit); both require
    // consecutive cycles before latching (see AbzOverspeedQualifier).
    AbzOverspeedQualifier overspeed_qualifier_;
    // P + I torque actually used by the velocity loop in the previous control
    // cycle. The bidirectional cogging scan samples this rather than I alone.
    float velocity_loop_torque_ = 0.0f;
    float velocity_proportional_torque_ = 0.0f;
    float effective_abz_vel_gain_ = 0.0f;
    float effective_abz_vel_integrator_gain_ = 0.0f;
    float abz_low_speed_gain_blend_ = 0.0f;
    float anticogging_torque_ = 0.0f;
    float anticogging_effective_scale_ = 0.0f;
    float final_torque_ = 0.0f;
    // PI telemetry: velocity error, unsaturated torque and saturation flag.
    float velocity_error_ = 0.0f;
    float torque_unsaturated_ = 0.0f;
    bool motor_torque_saturated_ = false;
    // ABZ velocity-loop torque limit telemetry.
    float abz_velocity_torque_before_limit_ = 0.0f;
    float abz_velocity_torque_after_limit_ = 0.0f;
    bool abz_velocity_torque_saturated_ = false;
    FrictionCompensator friction_compensator_;
    float low_speed_friction_torque_ = 0.0f;
    uint8_t low_speed_compensator_state_ = FrictionCompensator::STATE_IDLE;
    float friction_target_torque_ = 0.0f;
    float friction_continuous_torque_ = 0.0f;
    float friction_breakaway_extra_torque_ = 0.0f;
    float friction_speed_ratio_ = 0.0f;
    float friction_assist_blend_ = 0.0f;
    float friction_running_assist_blend_ = 0.0f;
    float friction_no_progress_time_ = 0.0f;
    float friction_recovery_timer_ = 0.0f;
    float friction_breakaway_exit_timer_ = 0.0f;
    float friction_forward_velocity_ = 0.0f;
    bool friction_reverse_detected_ = false;
    // ABZ mechanical velocity observer: the SINGLE feedback source of the ABZ
    // velocity PI.  Fed with per-tick delta position; adaptive bandwidth.
    AbzVelocityObserver abz_velocity_observer_;
    float control_observer_velocity_ = 0.0f;   // [turn/s]
    bool control_observer_valid_ = false;
    // Effective observer bandwidth of the most recent control cycle [Hz].
    float observer_bandwidth_ = 0.0f;
    // Diagnostic: control observer velocity minus 50 ms window velocity
    // [turn/s].  |value| >> 0 means the estimators disagree (see
    // docs/ABZ_VELOCITY_ESTIMATION.md for how to read it).
    float velocity_estimator_disagreement_ = 0.0f;
    // Diagnostic: number of control ticks whose |delta_enc| exceeded the
    // physically plausible bound (3x the expected counts per tick at the
    // commanded speed).  Pure telemetry; never faults.
    uint32_t abz_count_glitch_count_ = 0;
    float position_error_ = 0.0f;
    bool position_low_speed_active_ = false;

    bool input_pos_updated_ = false;
    
    bool trajectory_done_ = true;

    bool anticogging_valid_ = false;
    bool anticogging_velocity_only_ = false;
    float anticogging_calibration_base_pos_ = 0.0f;
    float anticogging_sample_sum_ = 0.0f;
    uint16_t anticogging_sample_count_ = 0;
    float anticogging_map_sum_ = 0.0f;
    float anticogging_map_mean_ = 0.0f;
    uint16_t anticogging_valid_bin_count_ = 0;

    // Bidirectional velocity scan state.  The forward map is kept in fixed
    // point form so the reverse pass can reuse the persistent cogging map for
    // its running mean without adding another 3600-float buffer.
    enum AnticoggingScanPhase : uint8_t {
        ANTICOGGING_SCAN_IDLE = 0,
        ANTICOGGING_SCAN_RAMP_FORWARD,
        ANTICOGGING_SCAN_FORWARD,
        ANTICOGGING_SCAN_RAMP_REVERSE,
        ANTICOGGING_SCAN_REVERSE,
        ANTICOGGING_SCAN_FINALIZE,
        ANTICOGGING_SCAN_SMOOTH,
        ANTICOGGING_SCAN_STATS,
        ANTICOGGING_SCAN_VALIDATE,
        ANTICOGGING_SCAN_COMPLETE,
        ANTICOGGING_SCAN_FAILED,
    };
    AnticoggingScanPhase anticogging_scan_phase_ = ANTICOGGING_SCAN_IDLE;
    float anticogging_scan_start_pos_ = 0.0f;
    float anticogging_reverse_start_pos_ = 0.0f;
    uint16_t anticogging_finalize_index_ = 0;
    int32_t anticogging_last_sample_position_bin_ = 0;
    bool anticogging_sample_position_valid_ = false;
    int16_t anticogging_forward_map_[3600] = {};
    uint8_t anticogging_forward_count_[3600] = {};
    uint8_t anticogging_reverse_count_[3600] = {};

    // Cogging calibration diagnostics.
    float anticogging_dwell_time_ = 0.0f;
    float anticogging_phase_elapsed_ = 0.0f;
    float anticogging_scan_command_speed_ = 0.0f;
    float anticogging_progress_percent_ = 0.0f;
    float anticogging_scan_velocity_ = 0.0f;
    float anticogging_scan_velocity_error_ = 0.0f;
    uint32_t anticogging_current_bin_ = 0;
    uint32_t anticogging_forward_valid_bins_ = 0;
    uint32_t anticogging_reverse_valid_bins_ = 0;
    uint32_t anticogging_rejected_velocity_samples_ = 0;
    uint32_t anticogging_rejected_reverse_samples_ = 0;
    uint32_t anticogging_rejected_state_samples_ = 0;
    uint32_t anticogging_rejected_saturation_samples_ = 0;
    uint32_t anticogging_rejected_estimator_samples_ = 0;
    float anticogging_map_rms_ = 0.0f;
    float anticogging_map_peak_to_peak_ = 0.0f;
    float anticogging_map_max_jump_ = 0.0f;
    float anticogging_map_wrap_jump_ = 0.0f;
    float anticogging_map_max_abs_ = 0.0f;
    float anticogging_map_min_ = 0.0f;
    float anticogging_map_max_ = 0.0f;
    float anticogging_map_sum_sq_ = 0.0f;
    float anticogging_map_first_ = 0.0f;
    float anticogging_map_last_ = 0.0f;
    uint16_t anticogging_stats_index_ = 0;
    bool anticogging_calibration_failed_ = false;
    uint8_t anticogging_calibration_abort_reason_ = 0;

    // custom setters
    void set_input_pos(float value) { input_pos_ = value; input_pos_updated(); }

};

#endif // __CONTROLLER_HPP
