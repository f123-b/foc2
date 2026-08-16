#ifndef __ABZ_VELOCITY_UTILS_HPP
#define __ABZ_VELOCITY_UTILS_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

// Shared, host-testable helpers for the ABZ mechanical velocity architecture.
// They are deliberately free of STM32 dependencies so the EXACT code used by
// Controller::update() / ascii_protocol.cpp can be exercised on the host
// (see firmware-target/Firmware/Tests/test_velocity_estimators.cpp).

// ---------------------------------------------------------------------------
// Observer initial-velocity seed.
//
// Priority: 50 ms window -> M/T -> encoder PLL -> 0.  Used to (re)initialize
// the control velocity observer while the rotor may be moving, so switching
// into velocity mode does not start the loop from zero and create a torque
// impulse.  Both Controller::update() and set_foc_control_mode() must use this
// same chain so the velocity setpoint and the observer seed never disagree.
inline float abz_observer_seed_velocity(bool window50_valid, float window50,
                                        bool mt_valid, float mt_velocity,
                                        bool pll_valid, float pll_velocity) {
    if (window50_valid)
        return window50;
    if (mt_valid)
        return mt_velocity;
    if (pll_valid && std::isfinite(pll_velocity))
        return pll_velocity;
    return 0.0f;
}

// ---------------------------------------------------------------------------
// ABZ count-glitch threshold (diagnostic only, never faults).
//
// A per-tick |delta_count| is physically plausible when it is consistent with
// the current speed envelope: the fastest of the commanded speed, the control
// observer and the 50 ms mechanical window.  The raw encoder PLL is
// deliberately NOT included: the PLL is the signal that spikes on a glitch,
// so including it would raise the threshold exactly when a glitch happens.
//
//   threshold = max(2, ceil(3 * max_speed * CPR * period))
//
// A delta strictly above the threshold increments the diagnostic glitch
// counter.  Legitimate fast motion (external drag, overshoot, deceleration,
// command = 0 while the rotor still spins) raises max_speed through the
// observer / window and is therefore never "continuously flagged".
inline int32_t abz_count_glitch_threshold(float max_plausible_speed_turn_per_s,
                                          int32_t cpr, float period) {
    const float speed = std::max(0.0f, max_plausible_speed_turn_per_s);
    const float expected_counts_per_tick =
            speed * (float)std::max<int32_t>(1, cpr) * std::max(0.0f, period);
    const int32_t threshold = (int32_t)(expected_counts_per_tick * 3.0f + 0.999f);
    return std::max((int32_t)2, threshold);
}

// ---------------------------------------------------------------------------
// Dual-layer overspeed qualification (see docs/ABZ_VELOCITY_ESTIMATION.md).
//
//   Layer A (normal):   the ABZ control feedback (observer) exceeds the normal
//                       limit (vel_limit * vel_limit_tolerance).
//   Layer B (emergency):the raw encoder PLL or the 50 ms count window exceeds
//                       the higher emergency limit (2x the normal limit), so a
//                       real runaway is not entirely dependent on the
//                       low-bandwidth observer.
//
// Both layers share one consecutive-cycle counter: a single raw PLL spike
// never drops PWM, but a sustained violation latches ERROR_OVERSPEED after
// kQualificationCycles consecutive cycles.
class AbzOverspeedQualifier {
public:
    static constexpr uint16_t kQualificationCycles = 16;
    static constexpr float kEmergencyLimitRatio = 2.0f;

    void reset() { violation_count_ = 0; }

    // Returns true when ERROR_OVERSPEED should be latched.
    bool update(bool normal_layer_exceeded, bool emergency_layer_exceeded) {
        if (normal_layer_exceeded || emergency_layer_exceeded) {
            ++violation_count_;
            if (violation_count_ >= kQualificationCycles)
                return true;
        } else {
            violation_count_ = 0;
        }
        return false;
    }

    static float emergency_limit(float normal_limit) {
        return normal_limit * kEmergencyLimitRatio;
    }

    uint16_t violation_count() const { return violation_count_; }

private:
    uint16_t violation_count_ = 0;
};

#endif // __ABZ_VELOCITY_UTILS_HPP
