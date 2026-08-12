#pragma once

#include <cmath>
#include <cstdint>

namespace foc2 {

struct MotorConfig {
    static constexpr std::int32_t pole_pairs = 10;
    static constexpr float phase_resistance_ohm = 0.1f;
    static constexpr float phase_inductance_h = 42.3e-6f;
    static constexpr float kv_rpm_per_volt = 650.0f;
    static constexpr float torque_constant_nm_per_a = 8.27f / kv_rpm_per_volt;
    static constexpr float pm_flux_linkage_v_per_rad_s =
        5.51328895422f / (static_cast<float>(pole_pairs) * kv_rpm_per_volt);
    static constexpr float max_current_a = 17.0f;
    static constexpr float max_torque_nm = 0.2f;
    static constexpr float max_speed_rpm = 7800.0f;
    static constexpr float max_speed_turns_per_second = max_speed_rpm / 60.0f;
    static constexpr float current_control_bandwidth_rad_s = 500.0f;
    static constexpr float current_p_gain_v_per_a =
        current_control_bandwidth_rad_s * phase_inductance_h;
    static constexpr float current_i_gain_v_per_as =
        (phase_resistance_ohm / phase_inductance_h) * current_p_gain_v_per_a;

    // Bring-up defaults. The hardware/motor maximum remains above these values.
    static constexpr float bringup_current_limit_a = 2.0f;
    static constexpr float sensorless_start_current_a = 2.0f;
    static constexpr float sensorless_handoff_electrical_rad_s = 400.0f;
};

struct EncoderConfig {
    static constexpr std::uint32_t spi_raw_cpr = 16384;
    static constexpr std::uint32_t abz_cpr = 4000;
    static constexpr std::uint8_t spi_cs_gpio = 4;
    static constexpr bool use_index = true;
};

static_assert(MotorConfig::pole_pairs > 0);
static_assert(MotorConfig::phase_resistance_ohm > 0.0f);
static_assert(MotorConfig::phase_inductance_h > 0.0f);
static_assert(EncoderConfig::spi_raw_cpr == (1u << 14));

}  // namespace foc2
