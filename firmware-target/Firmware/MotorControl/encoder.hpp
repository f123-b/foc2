#ifndef __ENCODER_HPP
#define __ENCODER_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

#include "incremental_velocity_estimator.hpp"
#include "abz_velocity_window.hpp"


class Encoder : public ODriveIntf::EncoderIntf {
public:
    static constexpr uint32_t MODE_FLAG_ABS = 0x100;

    struct Config_t {
        Mode mode = MODE_INCREMENTAL;
        bool use_index = false;
        bool pre_calibrated = false; // If true, this means the offset stored in
                                    // configuration is valid and does not need
                                    // be determined by run_offset_calibration.
                                    // In this case the encoder will enter ready
                                    // state as soon as the index is found.
        bool zero_count_on_find_idx = true;
        int32_t cpr = (2048 * 4);   // Default resolution of CUI-AMT102 encoder,
        int32_t offset = 0;        // Offset between encoder count and rotor electrical phase
        float offset_float = 0.0f; // Sub-count phase alignment offset
        bool enable_phase_interpolation = true; // Use velocity to interpolate inside the count state
        float calib_range = 0.02f; // Accuracy required to pass encoder cpr check
        float calib_scan_distance = 16.0f * M_PI; // rad electrical
        float calib_scan_omega = 4.0f * M_PI; // rad/s electrical
        float bandwidth = 1000.0f;
        bool find_idx_on_lockin_only = false; // Only be sensitive during lockin scan constant vel state
        bool idx_search_unidirectional = false; // Only allow index search in known direction
        bool ignore_illegal_hall_state = false; // dont error on bad states like 000 or 111
        uint16_t abs_spi_cs_gpio_pin = 1;
        uint16_t sincos_gpio_pin_sin = 3;
        uint16_t sincos_gpio_pin_cos = 4;

        // custom setters
        Encoder* parent = nullptr;
        void set_use_index(bool value) { use_index = value; parent->set_idx_subscribe(); }
        void set_find_idx_on_lockin_only(bool value) { find_idx_on_lockin_only = value; parent->set_idx_subscribe(); }
        void set_abs_spi_cs_gpio_pin(uint16_t value) { abs_spi_cs_gpio_pin = value; parent->abs_spi_cs_pin_init(); }
        void set_pre_calibrated(bool value) { pre_calibrated = value; parent->check_pre_calibrated(); }
        void set_bandwidth(float value) { bandwidth = value; parent->update_pll_gains(); }
    };

    Encoder(const EncoderHardwareConfig_t& hw_config,
            Config_t& config, const Motor::Config_t& motor_config);
    
    void setup();
    void set_error(Error error);
    bool do_checks();

    void enc_index_cb();
    void set_idx_subscribe(bool override_enable = false);
    void update_pll_gains();
    void check_pre_calibrated();

    void set_linear_count(int32_t count);
    void set_circular_count(int32_t count, bool update_offset);
    bool calib_enc_offset(float voltage_magnitude);

    bool run_index_search();
    bool run_direction_find();
    bool run_offset_calibration();
    void sample_now();
    bool update();

    const EncoderHardwareConfig_t& hw_config_;
    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Error error_ = ERROR_NONE;
    bool index_found_ = false;
    bool is_ready_ = false;
    int32_t shadow_count_ = 0;
    int32_t count_in_cpr_ = 0;
    float interpolation_ = 0.0f;
    float phase_ = 0.0f;        // [count]
    float pos_estimate_counts_ = 0.0f;  // [count]
    float pos_cpr_counts_ = 0.0f;  // [count]
    float vel_estimate_counts_ = 0.0f;  // [count/s]
    // Mechanical velocity diagnostics (ABZ incremental mode only).  These are
    // deliberately separate from the PLL state, which keeps driving
    // commutation, phase interpolation and the safety checks:
    //
    //   * velocity_window_50ms_  -> fast mechanical diagnostic reference
    //   * velocity_window_100ms_ -> steady-state mechanical reference
    //   * mt_velocity_estimate_  -> M/T (count-time) edge diagnostic
    //
    // The windows are true sliding windows updated every control tick, derived
    // from current_meas_hz at compile time.  At the nominal 8 kHz current loop
    // the 50 ms window is 400 ticks and the 100 ms window is 800 ticks; both
    // share a single 800-slot ring buffer (3.2 KiB at 4000 CPR).
    static constexpr uint16_t kMechanicalWindowSamples100 =
            static_cast<uint16_t>(current_meas_hz / 10);   // 100 ms
    static constexpr uint16_t kMechanicalWindowSamples50 =
            static_cast<uint16_t>(current_meas_hz / 20);   // 50 ms
    static_assert(kMechanicalWindowSamples100 == 2 * kMechanicalWindowSamples50,
                  "The 100 ms mechanical window must be exactly two 50 ms windows "
                  "(change the sample derivation if current_meas_hz changes)");
    static_assert(kMechanicalWindowSamples100 > 0 && kMechanicalWindowSamples50 > 0,
                  "Mechanical window sizes must be non-zero");

    AbzVelocityWindowT<kMechanicalWindowSamples100, kMechanicalWindowSamples50>
            mechanical_velocity_window_;
    float velocity_window_50ms_ = 0.0f;   // [turn/s]
    float velocity_window_100ms_ = 0.0f;  // [turn/s]
    bool velocity_window_50ms_valid_ = false;
    bool velocity_window_100ms_valid_ = false;
    // Unwrapped 64-bit mechanical count for diagnostics / observer / position
    // tracking.  shadow_count_ (int32) can overflow after hours of continuous
    // running at 4000 CPR; this counter never wraps in practice and costs one
    // addition per control tick.  The fast int32 hardware/electrical path is
    // untouched.
    int64_t mechanical_count_ = 0;
    // M/T (count-time) edge diagnostic estimator.  The PLL state above drives
    // commutation; this estimator only analyses low-speed encoder edges.
    IncrementalVelocityEstimator mt_velocity_estimator_;
    float mt_velocity_estimate_ = 0.0f;  // [turn/s] (M/T diagnostic)
    int32_t last_delta_count_ = 0;             // per-cycle count delta
    float pll_kp_ = 0.0f;   // [count/s / count]
    float pll_ki_ = 0.0f;   // [(count/s^2) / count]
    float calib_scan_response_ = 0.0f; // debug report from offset calib
    int32_t pos_abs_ = 0;
    float spi_error_rate_ = 0.0f;

    float pos_estimate_ = 0.0f; // [turn]
    float vel_estimate_ = 0.0f; // [turn/s]
    float pos_cpr_ = 0.0f;      // [turn]
    float pos_circular_ = 0.0f; // [turn]

    bool pos_estimate_valid_ = false;
    bool vel_estimate_valid_ = false;

    int16_t tim_cnt_sample_ = 0; // 
    // Updated by low_level pwm_adc_cb
    uint8_t hall_state_ = 0x0; // bit[0] = HallA, .., bit[2] = HallC
    float sincos_sample_s_ = 0.0f;
    float sincos_sample_c_ = 0.0f;

    bool abs_spi_init();
    bool abs_spi_start_transaction();
    void abs_spi_cb();
    void abs_spi_cs_pin_init();
    uint16_t abs_spi_dma_tx_[1] = {0xFFFF};
    uint16_t abs_spi_dma_rx_[1];
    bool abs_spi_pos_updated_ = false;
    Mode mode_ = MODE_INCREMENTAL;
    GPIO_TypeDef* abs_spi_cs_port_;
    uint16_t abs_spi_cs_pin_;
    uint32_t abs_spi_cr1;
    uint32_t abs_spi_cr2;

    void reset_mechanical_velocity_estimators() {
        mechanical_velocity_window_.reset();
        velocity_window_50ms_ = 0.0f;
        velocity_window_100ms_ = 0.0f;
        velocity_window_50ms_valid_ = false;
        velocity_window_100ms_valid_ = false;
        mechanical_count_ = 0;
        mt_velocity_estimator_.reset();
        mt_velocity_estimate_ = 0.0f;
        last_delta_count_ = 0;
    }

    constexpr float getCoggingRatio(){
        return 1.0f / 3600.0f;
    }
};

#endif // __ENCODER_HPP
