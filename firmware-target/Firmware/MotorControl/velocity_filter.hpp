#ifndef __VELOCITY_FILTER_HPP
#define __VELOCITY_FILTER_HPP

#include <algorithm>
#include <cmath>

// A runtime-only low-pass filter for velocity feedback.  It is intentionally
// separate from Encoder::vel_estimate_: torque and position paths must keep
// using the estimator that also drives phase advance and safety checks.
class VelocityFeedbackFilter {
public:
    void clear() {
        initialized_ = false;
        value_ = 0.0f;
    }

    void reset(float value) {
        value_ = value;
        initialized_ = true;
    }

    float update(float sample, float period, float bandwidth_hz) {
        if (!std::isfinite(sample)) {
            clear();
            return sample;
        }
        if (!initialized_) {
            reset(sample);
            return value_;
        }
        if (!(period > 0.0f) || !(bandwidth_hz > 0.0f)) {
            value_ = sample;
            return value_;
        }

        // The ABZ count window can occasionally contain a position-synchronous
        // burst. Limit the apparent acceleration before the one-pole filter so
        // one burst cannot become a visible torque kick. This is deliberately
        // much faster than FOC Studio's normal 1 turn/s^2 command ramp and
        // therefore does not slow a real command or scan reversal.
        constexpr float max_feedback_acceleration = 30.0f; // turn/s^2
        const float max_delta = max_feedback_acceleration * period;
        const float limited_sample = value_ + std::clamp(
                sample - value_, -max_delta, max_delta);

        constexpr float two_pi = 6.28318530717958647692f;
        const float omega_period = two_pi * bandwidth_hz * period;
        const float alpha = std::clamp(omega_period / (1.0f + omega_period), 0.0f, 1.0f);
        value_ += alpha * (limited_sample - value_);
        return value_;
    }

    // A filtered value is useful for telemetry as well as the control error.
    // Returning the raw sample before the first update would re-introduce the
    // encoder startup spike into the speed plot.
    float value_or(float fallback) const {
        return initialized_ ? value_ : fallback;
    }

    bool initialized() const { return initialized_; }
    float value() const { return value_; }

private:
    bool initialized_ = false;
    float value_ = 0.0f;
};

#endif
