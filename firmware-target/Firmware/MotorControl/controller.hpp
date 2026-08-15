#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

#include "velocity_filter.hpp"
#include "friction_compensator.hpp"
#include "control_velocity_observer.hpp"

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
        // Low-speed friction/breakaway feed-forward.  Defaults to off so the
        // ABZ velocity loop can be verified as a plain PI controller first.
        bool enable_low_speed_compensation = false;
        // ABZ-specific velocity PI gains (the generic vel_gain/vel_integrator
        // gain are not suitable for the 4000 CPR incremental encoder).
        float abz_vel_gain = 0.002f;                 // [Nm/(turn/s)]
        float abz_vel_integrator_gain = 0.0f;        // [Nm/(turn/s * s)]
        // Bandwidth of the control velocity observer in Hz (tunable 20..80 Hz).
        float control_velocity_observer_bandwidth = 40.0f;
        // ABZ velocity-loop torque limit [Nm], applied to P+I+FF before the
        // motor global torque limit. <= 0 or non-finite disables it.
        float abz_velocity_torque_limit = 0.015f;
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

    // Runtime diagnostics for the FOC Studio speed command. The encoder PLL
    // remains untouched for torque control, commutation and safety checks.
    VelocityFeedbackFilter abz_velocity_feedback_filter_;
    float velocity_control_feedback_ = 0.0f;
    bool velocity_control_feedback_valid_ = false;
    // Overspeed is safety-critical, but a single ABZ edge/PLL impulse must
    // not abort a running scan or speed command. Require a short consecutive
    // violation window before latching the controller fault.
    uint16_t overspeed_violation_count_ = 0;
    uint8_t raw_overspeed_lead_count_ = 0;
    // P + I torque actually used by the velocity loop in the previous control
    // cycle. The bidirectional cogging scan samples this rather than I alone.
    float velocity_loop_torque_ = 0.0f;
    float velocity_proportional_torque_ = 0.0f;
    float anticogging_torque_ = 0.0f;
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
    ControlVelocityObserver control_velocity_observer_;
    float control_observer_velocity_ = 0.0f;   // [turn/s]
    bool control_observer_valid_ = false;
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

    // custom setters
    void set_input_pos(float value) { input_pos_ = value; input_pos_updated(); }

};

#endif // __CONTROLLER_HPP
