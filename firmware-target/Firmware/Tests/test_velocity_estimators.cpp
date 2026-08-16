// Standalone host test for the ABZ mechanical velocity estimators.
//
// Compiles on any C++17 toolchain (no STM32 dependencies) so the exact
// algorithm headers used by the firmware can be exercised on the host:
//
//   abz_velocity_window.hpp        50/100 ms true sliding windows
//   incremental_velocity_estimator.hpp  M/T count-time diagnostic
//   abz_velocity_observer.hpp      ABZ mechanical velocity observer
//   abz_velocity_utils.hpp         seed / glitch-threshold / overspeed helpers
//
// Coverage (docs/ABZ_VELOCITY_ESTIMATION.md):
//   1.  +1 turn/s  -> window50 ~ 1, window100 ~ 1
//   2.  -1 turn/s  -> ~ -1
//   3.   0.2 turn/s-> no periodic 0 / 0.4 / 0 jumps
//   4.  zero speed -> all estimators converge to 0
//   5.  +1 -> -1 reversal: no NaN, no huge spike
//   6.  single-count irregular timing: M/T varies, windows are steadier
//   7.  single erroneous count: slew-limited; glitch threshold derivation
//   8.  observer reset while rotor moving: no velocity-from-zero impulse
//   9.  (protocol field count is covered by host/foc-studio/test_protocol.mjs)
//  10.  long-duration count handling: no overflow / float precision regression
//  11.  observer seed fallback: window/M-T invalid, PLL valid -> seed ~ PLL
//  12.  glitch threshold: legitimate 2 turn/s stream at command 0 is NOT
//      flagged; a single impossible count spike IS flagged
//  13.  overspeed qualifier: a single raw PLL spike does not fault; a
//      sustained runaway does
//  26.  speed sweep 0.1/0.2/0.5/1.0/2.0 with edge timing jitter +/-10% / +/-20%
//      -> window100 is the steadiest, window50 second, observer tracks for
//         closed-loop use.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "abz_velocity_window.hpp"
#include "incremental_velocity_estimator.hpp"
#include "abz_velocity_observer.hpp"
#include "abz_velocity_utils.hpp"

namespace {

int g_failures = 0;
int g_checks = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        ++g_checks;                                                       \
        if (!(cond)) {                                                    \
            ++g_failures;                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
        }                                                                 \
    } while (0)

#define CHECK_NEAR(actual, expected, eps)                                 \
    do {                                                                  \
        ++g_checks;                                                       \
        const double a_ = (actual), e_ = (expected), t_ = (eps);          \
        if (!(std::fabs(a_ - e_) <= t_)) {                                \
            ++g_failures;                                                 \
            std::printf("FAIL %s:%d: %s = %g, expected %g +- %g\n",       \
                        __FILE__, __LINE__, #actual, a_, e_, t_);         \
        }                                                                 \
    } while (0)

constexpr double kDt = 1.0 / 8000.0;          // 8 kHz control period
constexpr double kCpr = 4000.0;               // ABZ encoder counts/rev
constexpr double kCprD = 4000.0;

// Deterministic pseudo-random jitter in [-jitter, +jitter].
uint64_t lcg = 123456789;
double rand_unit() {
    lcg = lcg * 6364136223846793005ULL + 1442695040888963407ULL;
    return (double)((lcg >> 33) & 0x7fffffff) / (double)0x7fffffff;  // [0,1)
}

// Simulated rotor: returns the per-tick count delta of a rotor moving at
// `speed` turn/s.  `jitter` optionally modulates the effective per-tick speed
// (edge timing jitter).  Returns NaN-marker-free int32 deltas.
struct Rotor {
    double pos_counts = 0.0;
    int32_t prev_count = 0;

    int32_t tick(double speed, double jitter = 0.0) {
        const double eff = speed * (1.0 + jitter * (2.0 * rand_unit() - 1.0));
        pos_counts += eff * kCprD * kDt;
        const int32_t count = (int32_t)std::floor(pos_counts);
        const int32_t delta = count - prev_count;
        prev_count = count;
        return delta;
    }
};

// Run the two windows + M/T + observer on the same rotor stream.
struct Harness {
    AbzVelocityWindowT<800, 400> window;
    IncrementalVelocityEstimator mt;
    AbzVelocityObserver observer;

    Harness() {
        observer.configure(15.0f, 50.0f);
        observer.set_bandwidth(observer.bandwidth_for(1.0f));
    }

    // Feed one tick; returns current estimates.
    void feed(int32_t delta, float command_velocity) {
        window.push(delta);
        mt.update(delta, (float)kDt, (float)kCpr);
        observer.set_bandwidth(observer.bandwidth_for(std::fabs(command_velocity)));
        if (!observer.initialized())
            observer.reset(0.0f, 0.0f);
        observer.update((float)delta / (float)kCpr, (float)kDt);
    }
};

// --- test 1/2: constant +/- 1 turn/s ---------------------------------------
void test_constant_speed() {
    for (double speed : {1.0, -1.0}) {
        Harness h;
        Rotor rotor;
        for (int i = 0; i < 2400; ++i)
            h.feed(rotor.tick(speed), (float)speed);
        CHECK_NEAR(h.window.velocity50((float)kCpr, (float)kDt), speed, 0.01);
        CHECK_NEAR(h.window.velocity100((float)kCpr, (float)kDt), speed, 0.01);
        CHECK_NEAR(h.mt.velocity(), speed, 0.05);
        CHECK_NEAR(h.observer.velocity(), speed, 0.02);
    }
}

// --- test 3: 0.2 turn/s must not produce periodic 0 / 0.4 / 0 --------------
void test_low_speed_no_flicker() {
    const double speed = 0.2;
    Harness h;
    Rotor rotor;
    for (int i = 0; i < 8000; ++i)
        h.feed(rotor.tick(speed), (float)speed);
    // Steady state: watch 0.5 s.
    double min_w50 = 1e9, max_w50 = -1e9;
    double min_w100 = 1e9, max_w100 = -1e9;
    double min_mt = 1e9, max_mt = -1e9;
    for (int i = 0; i < 4000; ++i) {
        h.feed(rotor.tick(speed), (float)speed);
        const float w50 = h.window.velocity50((float)kCpr, (float)kDt);
        const float w100 = h.window.velocity100((float)kCpr, (float)kDt);
        const float mt = h.mt.velocity();
        min_w50 = std::min(min_w50, (double)w50); max_w50 = std::max(max_w50, (double)w50);
        min_w100 = std::min(min_w100, (double)w100); max_w100 = std::max(max_w100, (double)w100);
        min_mt = std::min(min_mt, (double)mt); max_mt = std::max(max_mt, (double)mt);
    }
    // No 0 / 2x / 0 pattern in any estimator at steady state.
    CHECK(min_w50 > 0.15);
    CHECK(max_w50 < 0.25);
    CHECK(min_w100 > 0.17);
    CHECK(max_w100 < 0.23);
    CHECK(min_mt > 0.10);   // M/T is noisier but never collapses to 0
    CHECK(max_mt < 0.35);
    // Windows must be much steadier than the raw edge stream.
    CHECK(max_w50 - min_w50 < 0.02);
    CHECK(max_w100 - min_w100 < 0.01);
}

// --- test 4: zero speed converges ------------------------------------------
void test_zero_speed() {
    Harness h;
    Rotor rotor;
    for (int i = 0; i < 2000; ++i)
        h.feed(rotor.tick(1.0), 1.0f);
    for (int i = 0; i < 16000; ++i)   // 2 s stopped
        h.feed(rotor.tick(0.0), 0.0f);
    CHECK(h.window.velocity50((float)kCpr, (float)kDt) == 0.0f);
    CHECK(h.window.velocity100((float)kCpr, (float)kDt) == 0.0f);
    CHECK(std::fabs(h.mt.velocity()) < 0.005);
    CHECK(std::fabs(h.observer.velocity()) < 0.01);
}

// --- test 5: reversal +1 -> -1 ---------------------------------------------
void test_reversal() {
    Harness h;
    Rotor rotor;
    for (int i = 0; i < 2000; ++i)
        h.feed(rotor.tick(1.0), 1.0f);
    double max_abs = 0.0;
    bool any_nan = false;
    for (int i = 0; i < 16000; ++i) {
        const int32_t delta = rotor.tick(i < 8000 ? -1.0 : 1.0);
        h.feed(delta, (float)(i < 8000 ? -1.0 : 1.0));
        const float w50 = h.window.velocity50((float)kCpr, (float)kDt);
        const float w100 = h.window.velocity100((float)kCpr, (float)kDt);
        const float mt = h.mt.velocity();
        const float obs = h.observer.velocity();
        for (float v : {w50, w100, mt, obs}) {
            if (!std::isfinite(v)) any_nan = true;
            max_abs = std::max(max_abs, (double)std::fabs(v));
        }
    }
    CHECK(!any_nan);
    CHECK(max_abs < 1.6);   // no huge spike beyond the reversal itself
    CHECK_NEAR(h.window.velocity100((float)kCpr, (float)kDt), 1.0, 0.01);
    CHECK_NEAR(h.mt.velocity(), 1.0, 0.1);
    CHECK_NEAR(h.observer.velocity(), 1.0, 0.05);
}

// --- test 6: single-count irregular timing ---------------------------------
void test_irregular_edges() {
    Harness h;
    Rotor rotor;
    // 0.05 turn/s: an edge arrives roughly every 5 ms -> very irregular at
    // the 125 us tick rate. Windows must be much steadier than the M/T.
    for (int i = 0; i < 8000; ++i)
        h.feed(rotor.tick(0.05), 0.05f);
    double min_w50 = 1e9, max_w50 = -1e9, min_w100 = 1e9, max_w100 = -1e9;
    double min_mt = 1e9, max_mt = -1e9;
    for (int i = 0; i < 8000; ++i) {
        h.feed(rotor.tick(0.05), 0.05f);
        const float w50 = h.window.velocity50((float)kCpr, (float)kDt);
        const float w100 = h.window.velocity100((float)kCpr, (float)kDt);
        const float mt = h.mt.velocity();
        min_w50 = std::min(min_w50, (double)w50); max_w50 = std::max(max_w50, (double)w50);
        min_w100 = std::min(min_w100, (double)w100); max_w100 = std::max(max_w100, (double)w100);
        min_mt = std::min(min_mt, (double)mt); max_mt = std::max(max_mt, (double)mt);
    }
    const double ripple_w50 = max_w50 - min_w50;
    const double ripple_w100 = max_w100 - min_w100;
    const double ripple_mt = max_mt - min_mt;
    CHECK(ripple_mt > ripple_w50);    // M/T varies tick to tick
    CHECK(ripple_w50 > ripple_w100);  // window100 is the steadiest
    CHECK_NEAR(h.window.velocity100((float)kCpr, (float)kDt), 0.05, 0.01);
    CHECK_NEAR(h.mt.velocity(), 0.05, 0.03);
}

// --- test 7: single erroneous count ----------------------------------------
void test_single_glitch() {
    Harness h;
    Rotor rotor;
    for (int i = 0; i < 4000; ++i)
        h.feed(rotor.tick(0.2), 0.2f);
    // One stray +5 count on a rotor commanded at 0.2 turn/s.
    h.feed(5, 0.2f);
    float max_after = 0.0f;
    for (int i = 0; i < 4000; ++i) {
        h.feed(rotor.tick(0.2), 0.2f);
        max_after = std::max(max_after, std::fabs(h.mt.velocity()));
        max_after = std::max(max_after, std::fabs(h.observer.velocity()));
    }
    // Slew-limited: never a huge spike, and the loop returns to 0.2.
    CHECK(max_after < 0.5f);
    CHECK_NEAR(h.mt.velocity(), 0.2, 0.05);

    // --- test 7b: glitch threshold (shared helper used by Controller) -------
    // threshold = max(2, ceil(3 * plausible_speed * CPR * period))
    auto threshold_for = [](float plausible_speed) {
        return abz_count_glitch_threshold(plausible_speed, (int32_t)kCprD, (float)kDt);
    };
    CHECK(threshold_for(0.0f) == 2);    // standstill floor: a 2-count tick is the limit
    CHECK(threshold_for(0.2f) == 2);    // 0.075 -> ceil -> 1 -> floor 2
    CHECK(threshold_for(1.0f) == 2);    // 1.5 -> 2
    CHECK(threshold_for(2.0f) == 3);    // 3.0 -> 3
    CHECK(threshold_for(10.0f) == 15);  // 15.0 -> 15
    // The +5 stray above is far above the ~2 threshold at a 0.2 turn/s envelope.
    CHECK(std::abs(5) > threshold_for(0.2f));

    // --- test 7c: legitimate fast counts at command = 0 are NOT glitches -----
    // Command is 0 but the rotor is dragged at 2 turn/s: observer and window50
    // converge to ~2, so the plausible envelope is ~2 and the stream (<= 2
    // counts/tick) never exceeds the threshold.  No glitch counter increment.
    {
        Harness h;
        Rotor rotor;
        uint32_t glitches = 0;
        for (int i = 0; i < 16000; ++i) {
            const int32_t delta = rotor.tick(2.0);   // legitimate 2 turn/s stream
            h.feed(delta, 0.0f);                     // command = 0
            const float plausible = std::max(
                    std::max(0.0f, std::fabs(h.observer.velocity())),
                    h.window.valid50() ? std::fabs(h.window.velocity50((float)kCpr, (float)kDt)) : 0.0f);
            if (std::abs(delta) > threshold_for(plausible))
                ++glitches;
        }
        CHECK(glitches == 0);
    }

    // --- test 7d: an impossible single count spike IS flagged ----------------
    // All estimators agree the rotor is at ~0.2 turn/s; a +50 count in one
    // tick (160 turn/s instantaneous) is far beyond the envelope.
    {
        Harness h;
        Rotor rotor;
        for (int i = 0; i < 4000; ++i)
            h.feed(rotor.tick(0.2), 0.2f);
        uint32_t glitches = 0;
        for (int i = 0; i < 64; ++i) {
            const int32_t delta = (i == 32) ? 50 : rotor.tick(0.2);
            h.feed(delta, 0.2f);
            const float plausible = std::max(
                    std::max(0.2f, std::fabs(h.observer.velocity())),
                    h.window.valid50() ? std::fabs(h.window.velocity50((float)kCpr, (float)kDt)) : 0.0f);
            if (std::abs(delta) > threshold_for(plausible))
                ++glitches;
        }
        CHECK(glitches == 1);
    }
}

// --- test 11: observer seed fallback (shared helper used by Controller) ------
void test_observer_seed_fallback() {
    // window invalid, M/T invalid, raw PLL valid = 1 turn/s -> seed = PLL.
    CHECK_NEAR(abz_observer_seed_velocity(
            false, 0.0f, false, 0.0f, true, 1.0f), 1.0, 1e-6);
    // window valid wins over M/T and PLL.
    CHECK_NEAR(abz_observer_seed_velocity(
            true, 0.4f, true, 1.2f, true, 1.0f), 0.4, 1e-6);
    // M/T wins over PLL when window is invalid.
    CHECK_NEAR(abz_observer_seed_velocity(
            false, 0.0f, true, -0.7f, true, 1.0f), -0.7, 1e-6);
    // PLL wins when window and M/T are invalid.
    CHECK_NEAR(abz_observer_seed_velocity(
            false, 0.0f, false, 0.0f, true, -1.0f), -1.0, 1e-6);
    // Non-finite PLL falls through to 0.
    CHECK(abz_observer_seed_velocity(
            false, 0.0f, false, 0.0f, true, NAN) == 0.0f);
    CHECK(abz_observer_seed_velocity(
            false, 0.0f, false, 0.0f, true, INFINITY) == 0.0f);
    // Everything invalid -> 0.
    CHECK(abz_observer_seed_velocity(
            false, 0.0f, false, 0.0f, false, 0.0f) == 0.0f);
}

// --- test 13: dual-layer overspeed qualifier ---------------------------------
void test_overspeed_qualifier() {
    AbzOverspeedQualifier q;

    // A single raw PLL spike (emergency layer only, one cycle) never faults.
    q.reset();
    bool faulted = false;
    for (int i = 0; i < 200; ++i) {
        const bool spike = (i == 50);
        if (q.update(false, spike))
            faulted = true;
    }
    CHECK(!faulted);

    // Normal-layer qualification: observer over normal limit for many cycles.
    q.reset();
    faulted = false;
    for (int i = 0; i < 15; ++i)
        faulted = q.update(true, false) || faulted;
    CHECK(!faulted);  // 15 < 16
    CHECK(q.update(true, false));  // 16th cycle latches

    // Sustained emergency runaway (raw PLL beyond emergency limit) latches.
    q.reset();
    faulted = false;
    for (int i = 0; i < 100; ++i) {
        if (q.update(false, true))
            faulted = true;
    }
    CHECK(faulted);

    // Intermittent raw spikes reset the counter and never latch.
    q.reset();
    faulted = false;
    for (int i = 0; i < 1000; ++i) {
        const bool spike = (i % 40 == 0);  // one spike every 40 cycles
        if (q.update(false, spike))
            faulted = true;
    }
    CHECK(!faulted);

    // Emergency limit is 2x the normal limit.
    CHECK_NEAR(AbzOverspeedQualifier::emergency_limit(3.6f), 7.2, 1e-6);
    CHECK_NEAR(AbzOverspeedQualifier::emergency_limit(24.0f), 48.0, 1e-6);
}

// --- test 8: observer reset while rotor moving ------------------------------
void test_observer_reset_while_moving() {
    AbzVelocityObserver observer;
    observer.configure(15.0f, 50.0f);
    observer.set_bandwidth(observer.bandwidth_for(0.5f));
    // Rotor already at 0.5 turn/s; a naive reset would seed vel_hat_ = 0 and
    // create a large initial velocity error (torque impulse).  Seeding with
    // the measured speed must not.
    observer.reset(0.0f, 0.5f);
    const float first = observer.update(0.5f * (float)kDt, (float)kDt);
    CHECK_NEAR(first, 0.5, 0.02);
    float v = first;
    for (int i = 0; i < 8000; ++i)
        v = observer.update(0.5f * (float)kDt, (float)kDt);
    CHECK_NEAR(v, 0.5, 0.01);
    // And the local frame must stay small even after a huge initial position.
    observer.reset(123456.0f, 0.0f);
    observer.update(0.0f, (float)kDt);
    CHECK(std::fabs(observer.position()) < 16.0f);
}

// --- test 10: long-duration count handling ----------------------------------
void test_long_duration() {
    Harness h;
    Rotor rotor;
    // 12 hours at 2 turn/s = 86400 s * 2 * 4000 = 691,200,000 counts, far past
    // the int32 shadow-count overflow; the observer/window must stay exact.
    const long long ticks = (long long)(2 * 4000.0 * 2.0);  // ~1 s scaled run
    for (long long i = 0; i < ticks; ++i)
        h.feed(rotor.tick(2.0), 2.0f);
    CHECK_NEAR(h.window.velocity100((float)kCpr, (float)kDt), 2.0, 0.01);
    CHECK_NEAR(h.observer.velocity(), 2.0, 0.02);
    CHECK(std::isfinite(h.window.velocity50((float)kCpr, (float)kDt)));
    CHECK(std::isfinite(h.mt.velocity()));
}

// --- test 26: speed sweep with edge timing jitter ---------------------------
void test_jitter_sweep() {
    for (double speed : {0.1, 0.2, 0.5, 1.0, 2.0}) {
        for (double jitter : {0.0, 0.10, 0.20}) {
            Harness h;
            Rotor rotor;
            // Settle, then measure steady-state ripple.
            for (int i = 0; i < 4000; ++i)
                h.feed(rotor.tick(speed, jitter), (float)speed);
            double ripple50 = 0.0, ripple100 = 0.0;
            double min50 = 1e9, max50 = -1e9, min100 = 1e9, max100 = -1e9;
            for (int i = 0; i < 8000; ++i) {
                h.feed(rotor.tick(speed, jitter), (float)speed);
                const float w50 = h.window.velocity50((float)kCpr, (float)kDt);
                const float w100 = h.window.velocity100((float)kCpr, (float)kDt);
                min50 = std::min(min50, (double)w50); max50 = std::max(max50, (double)w50);
                min100 = std::min(min100, (double)w100); max100 = std::max(max100, (double)w100);
            }
            ripple50 = max50 - min50;
            ripple100 = max100 - min100;
            // window100 is always at least as steady as window50.
            CHECK(ripple100 <= ripple50 + 1e-9);
            // Mean stays on the commanded speed.
            CHECK_NEAR(0.5 * (min50 + max50), speed, std::max(0.02, 0.1 * speed));
            CHECK_NEAR(0.5 * (min100 + max100), speed, std::max(0.015, 0.08 * speed));
            // Observer tracks the commanded speed for closed-loop use.
            CHECK_NEAR(h.observer.velocity(), speed, std::max(0.05, 0.15 * speed));
            CHECK(std::isfinite(h.observer.velocity()));
        }
    }
}

}  // namespace

int main() {
    test_constant_speed();
    test_low_speed_no_flicker();
    test_zero_speed();
    test_reversal();
    test_irregular_edges();
    test_single_glitch();
    test_observer_reset_while_moving();
    test_long_duration();
    test_observer_seed_fallback();
    test_overspeed_qualifier();
    test_jitter_sweep();

    std::printf("velocity estimator tests: %d checks, %d failures\n",
                g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
