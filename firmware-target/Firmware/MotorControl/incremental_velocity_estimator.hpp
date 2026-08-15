#ifndef __INCREMENTAL_VELOCITY_ESTIMATOR_HPP
#define __INCREMENTAL_VELOCITY_ESTIMATOR_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

// Count-time (M/T) velocity estimator for the ABZ velocity/position
// controller.  It is deliberately independent of the encoder PLL so that the
// FOC electrical angle, phase interpolation and commutation paths keep using
// the PLL exactly as before.
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
// The publish threshold is not a fixed speed: at low speed it waits for a few
// real counts, while at higher speed it is raised proportionally so the
// publish interval does not collapse below a minimum time and amplify
// count-arrival timing jitter.  The output is slew limited so a single
// erroneous count cannot become a large velocity spike.
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
