#ifndef __CONTROL_VELOCITY_OBSERVER_HPP
#define __CONTROL_VELOCITY_OBSERVER_HPP

#include <algorithm>
#include <cmath>

// Low-bandwidth position-tracking PLL (an alpha-beta observer) that produces
// the smooth velocity used to close the ABZ velocity PI.  It is deliberately
// independent of the encoder PLL: the encoder PLL keeps driving commutation,
// phase interpolation and the safety checks, while this observer only feeds
// the mechanical velocity loop.
//
//   position (turn) -> [ kp, ki ] -> velocity (turn/s)
//
// The gains are derived from a single bandwidth parameter (in Hz) so the
// mechanical loop can be tuned between roughly 20 and 80 Hz without touching
// the commutation PLL.  A critically damped pair is used:
//
//   omega_n = 2*pi*bandwidth
//   kp      = 2*omega_n
//   ki      = 0.25*kp^2   (critically damped)
class ControlVelocityObserver {
public:
    void configure(float bandwidth_hz) {
        const float two_pi = 6.28318530717958647692f;
        const float omega = two_pi * std::max(0.1f, bandwidth_hz);
        kp_ = 2.0f * omega;        // [1/s]
        ki_ = 0.25f * kp_ * kp_;   // [1/s^2]
    }

    void reset(float position_turn) {
        pos_hat_ = position_turn;
        vel_hat_ = 0.0f;
        initialized_ = true;
    }

    // Feed the raw (unbounded) mechanical position in turns. Returns the
    // smoothed velocity in turn/s.
    float update(float position_turn, float dt) {
        if (!(dt > 0.0f) || !std::isfinite(position_turn)) {
            reset(position_turn);
            return 0.0f;
        }
        if (!initialized_) {
            reset(position_turn);
            return 0.0f;
        }
        // Predict position ahead by the current velocity estimate.
        pos_hat_ += dt * vel_hat_;
        // Correct from the measured position.
        const float pos_err = position_turn - pos_hat_;
        pos_hat_ += dt * kp_ * pos_err;
        vel_hat_ += dt * ki_ * pos_err;
        return vel_hat_;
    }

    float velocity() const { return vel_hat_; }
    float position() const { return pos_hat_; }
    bool initialized() const { return initialized_; }

private:
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float pos_hat_ = 0.0f;
    float vel_hat_ = 0.0f;
    bool initialized_ = false;
};

#endif // __CONTROL_VELOCITY_OBSERVER_HPP
