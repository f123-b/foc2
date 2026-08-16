#ifndef __ABZ_VELOCITY_WINDOW_HPP
#define __ABZ_VELOCITY_WINDOW_HPP

#include <cstdint>

// True sliding-window mechanical velocity for the ABZ encoder.  The window is
// fed once per control tick with the per-tick encoder count delta and
// maintains two sums over a SINGLE ring buffer:
//
//   * a 50 ms window  (fast mechanical diagnostic / observer seed)
//   * a 100 ms window (steady-state mechanical reference)
//
//   velocity = sum(delta_count) / (CPR * window_elapsed)
//
// Both windows update every control tick (no 50/100 ms periodic refresh), are
// correct for both directions, read exactly 0 at standstill and never touch
// the electrical angle / commutation path.  Memory is a fixed-size array (no
// dynamic allocation); the two windows share one buffer so a 4000 CPR axis
// costs 800 * 4 B = 3.2 KiB instead of two separate arrays.
//
// The sample counts are template parameters so the class is host-testable with
// small buffers and the firmware instantiates it from `current_meas_hz`
// (compile-time derived, see encoder.hpp).  `Samples100` must be a multiple of
// `Samples50` so the 50 ms window can be a strict sub-window of the 100 ms
// ring (the typical case is Samples100 == 2 * Samples50).
template <uint16_t Samples100, uint16_t Samples50>
class AbzVelocityWindowT {
    static_assert(Samples100 > 0, "100 ms window must not be empty");
    static_assert(Samples50 > 0 && Samples50 <= Samples100,
                  "50 ms window must be a non-empty sub-window of the 100 ms ring");
    static_assert(Samples100 % Samples50 == 0,
                  "100 ms window must be an integer multiple of the 50 ms window");

public:
    // Clear all state.  Called only on feedback-mode / calibration changes
    // (never inside the control loop hot path).
    void reset() {
        for (uint16_t i = 0; i < Samples100; ++i)
            ring_[i] = 0;
        head_ = 0;
        count100_ = 0;
        count50_ = 0;
        sum100_ = 0;
        sum50_ = 0;
    }

    // Push one per-tick count delta.  O(1): add the newest delta, subtract the
    // deltas that leave each window.  The 100 ms window drops the slot about
    // to be overwritten; the 50 ms window drops the slot written exactly
    // Samples50 ticks ago, which sits `Samples100 - Samples50` slots behind
    // the current write position.
    void push(int32_t delta) {
        if (count100_ < Samples100)
            ++count100_;
        else
            sum100_ -= ring_[head_];
        sum100_ += delta;

        if (count50_ < Samples50)
            ++count50_;
        else
            sum50_ -= ring_[(head_ + Samples100 - Samples50) % Samples100];
        sum50_ += delta;

        ring_[head_] = delta;
        head_ = static_cast<uint16_t>((head_ + 1) % Samples100);
    }

    bool valid50() const { return count50_ == Samples50; }
    bool valid100() const { return count100_ == Samples100; }
    uint16_t count50() const { return count50_; }
    uint16_t count100() const { return count100_; }

    // Sum of the last `Samples` count deltas.  int64 avoids overflow: at
    // 4000 CPR and 10 turn/s a 100 ms window accumulates at most 4000 counts,
    // but int64 also keeps the math exact when the window is used for
    // diagnostics over long sessions.
    int64_t sum50() const { return sum50_; }
    int64_t sum100() const { return sum100_; }

    // Velocity in turn/s.  `cpr` is the encoder counts per revolution,
    // `period` the control period in seconds.  Callers should gate on
    // valid50()/valid100() before using the value.
    float velocity50(float cpr, float period) const {
        if (!valid50() || !(cpr > 0.0f) || !(period > 0.0f))
            return 0.0f;
        return static_cast<float>(sum50_) / (cpr * static_cast<float>(Samples50) * period);
    }

    float velocity100(float cpr, float period) const {
        if (!valid100() || !(cpr > 0.0f) || !(period > 0.0f))
            return 0.0f;
        return static_cast<float>(sum100_) / (cpr * static_cast<float>(Samples100) * period);
    }

private:
    int32_t ring_[Samples100] = {};
    uint16_t head_ = 0;
    uint16_t count100_ = 0;
    uint16_t count50_ = 0;
    int64_t sum100_ = 0;
    int64_t sum50_ = 0;
};

#endif // __ABZ_VELOCITY_WINDOW_HPP
