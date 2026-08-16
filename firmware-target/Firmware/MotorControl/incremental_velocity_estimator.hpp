#ifndef __INCREMENTAL_VELOCITY_ESTIMATOR_HPP
#define __INCREMENTAL_VELOCITY_ESTIMATOR_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

// Count-time (M/T) velocity estimator.  It is a DIAGNOSTIC estimator for the
// ABZ encoder: the FOC electrical angle / phase interpolation / commutation
// keep using the encoder PLL, and the ABZ velocity PI closes on
// AbzVelocityObserver.  This estimator is used to analyze low-speed encoder
// edges, to compare against the observer and the rolling windows, and to
// detect abnormal count behaviour.
//
// Each control tick it accumulates the wrapped encoder count delta and the
// elapsed time, then publishes
//
//     velocity = delta_count / (cpr * delta_time)
//
// when either enough counts have accumulated (the low-speed path) or a maximum
// observation window has elapsed (the high-speed and stopped path).  Between
// publishes the previous estimate is held, so a single control tick without an
// encoder edge never reads as velocity == 0.
//
// Zero-speed behaviour (important at low speed): with a fixed
// max_publish_time, a slow rotor that produces no edge inside one publish
// interval would periodically publish 0 and make the trace flicker
// "0 -> non-zero -> 0".  Instead:
//   * while edges are still EXPECTED (time since last edge is below a
//     speed-dependent timeout of ~4x the expected edge interval), a publish
//     with zero accumulated counts HOLDS the previous estimate;
//   * once the edge timeout is exceeded (the rotor has genuinely stopped or
//     slowed far below the last estimate), the velocity decays exponentially
//     toward zero instead of snapping.
// The hold/decay never fabricates a sustained speed: the decay reaches <0.1 %
// of the last estimate within ~0.7 s and clamps to exactly 0.
class IncrementalVelocityEstimator {
public:
    struct Config {
        // Publish as soon as this many counts accumulate (the low-speed path).
        float min_publish_counts = 3.0f;
        // Keep the publish interval no shorter than this at any speed; at high
        // speed the count threshold is raised to reach this interval.
        float min_publish_time = 0.002f;
        // Publish no slower than this even when counts are sparse; this is what
        // detects a genuinely stopped rotor without a fixed speed threshold.
        float max_publish_time = 0.010f;
        // Upper bound on the speed-scaled count threshold.
        float max_publish_counts = 64.0f;
        // Output slew limit [turn/s^2].  Rejects single-count glitches and
        // bounds the estimator phase response.
        float max_velocity_slew = 100.0f;
    };

    // Multiplier on the expected edge interval before the rotor is declared
    // idle and the estimate starts decaying toward zero.
    static constexpr float kEdgeTimeoutRatio = 4.0f;
    // Floor of the edge timeout so the decay never triggers on sub-millisecond
    // edge intervals of a fast rotor.
    static constexpr float kEdgeTimeoutMin = 0.030f;   // [s]
    // Exponential decay time constant once idle [s]: reaches ~0.1 % of the
    // last estimate in ~0.7 s.
    static constexpr float kDecayTau = 0.100f;         // [s]
    // Estimates below this magnitude are snapped to exactly zero.
    static constexpr float kZeroFloor = 1e-4f;         // [turn/s]

    void reset() {
        count_accum_ = 0;
        time_accum_ = 0.0f;
        velocity_ = 0.0f;
        valid_ = false;
        time_since_last_edge_ = 0.0f;
    }

    float update(int32_t delta_count, float dt, float cpr) {
        if (!(dt > 0.0f) || !(cpr > 0.0f)) {
            reset();
            return 0.0f;
        }

        count_accum_ += delta_count;
        time_accum_ += dt;
        time_since_last_edge_ = (delta_count != 0) ? 0.0f : time_since_last_edge_ + dt;

        // Speed-dependent edge timeout: Tedge = 1 / (CPR * |v|).  Once the
        // time since the last edge exceeds ~4x Tedge (with a floor), the rotor
        // is treated as stopped relative to the last estimate and the estimate
        // decays smoothly to zero.
        const float speed = std::fabs(velocity_);
        const float expected_edge_interval = (speed > 1e-4f)
                ? 1.0f / (cpr * speed) : 1.0f;
        const float edge_timeout = std::max(
                kEdgeTimeoutMin, kEdgeTimeoutRatio * expected_edge_interval);
        if (time_since_last_edge_ >= edge_timeout) {
            if (speed > 0.0f) {
                // First-order exponential decay toward zero:
                //   v *= exp(-dt / tau)  ≈  v * (1 - dt / tau)
                // dt/tau is ~0.00125 at 8 kHz with tau = 0.1 s, so the
                // approximation error is < 1e-6 relative and no exp() call is
                // needed inside the control loop.
                const float decay = 1.0f - dt * (1.0f / kDecayTau);
                velocity_ *= (decay > 0.0f) ? decay : 0.0f;
                if (std::fabs(velocity_) < kZeroFloor)
                    velocity_ = 0.0f;
            }
            // Discard the stale accumulation so a later restart publishes a
            // fresh interval instead of one that includes the idle period.
            count_accum_ = 0;
            time_accum_ = 0.0f;
            return velocity_;
        }

        // Speed-scaled publish threshold: raise the count requirement so the
        // publish interval stays at least min_publish_time at higher speeds.
        const float counts_for_min_time =
                cpr * std::fabs(velocity_) * config_.min_publish_time;
        const float effective_min_counts = std::min(
                config_.max_publish_counts,
                std::max(config_.min_publish_counts, counts_for_min_time));

        const bool publish =
                std::fabs(static_cast<float>(count_accum_)) >= effective_min_counts ||
                time_accum_ >= config_.max_publish_time;

        if (publish && time_accum_ > 0.0f) {
            if (count_accum_ == 0) {
                // No counts in this interval but edges are still expected (the
                // edge timeout above has not fired).  Publish nothing: holding
                // the previous estimate avoids the 0 / non-zero flicker that a
                // raw 0 publish would produce at low speed.
                count_accum_ = 0;
                time_accum_ = 0.0f;
                return velocity_;
            }
            const float raw = static_cast<float>(count_accum_) / (cpr * time_accum_);
            const float max_delta = config_.max_velocity_slew * time_accum_;
            velocity_ += std::clamp(raw - velocity_, -max_delta, max_delta);
            valid_ = true;
            count_accum_ = 0;
            time_accum_ = 0.0f;
        }

        return velocity_;
    }

    float velocity() const { return velocity_; }
    bool valid() const { return valid_; }
    float time_since_last_edge() const { return time_since_last_edge_; }

private:
    Config config_;
    int64_t count_accum_ = 0;
    float time_accum_ = 0.0f;
    float velocity_ = 0.0f;
    bool valid_ = false;
    float time_since_last_edge_ = 0.0f;
};

#endif // __INCREMENTAL_VELOCITY_ESTIMATOR_HPP
