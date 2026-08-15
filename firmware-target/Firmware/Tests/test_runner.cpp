

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#define DOCTEST_CONFIG_TREAT_CHAR_STAR_AS_STRING
#define DOCTEST_CONFIG_USE_STD_HEADERS
#define DOCTEST_CONFIG_NO_TRY_CATCH_IN_ASSERTS
#define DOCTEST_CONFIG_NO_EXCEPTIONS
#define DOCTEST_CONFIG_NO_WINDOWS_SEH
#define DOCTEST_CONFIG_NO_POSIX_SIGNALS
// #define DOCTEST_CONFIG_VOID_CAST_EXPRESSIONS

#include <doctest.h>

#include <communication/can_helpers.hpp>
#include <incremental_velocity_estimator.hpp>
#include <friction_compensator.hpp>
#include <control_velocity_observer.hpp>
#include <velocity_filter.hpp>

using std::cout;
using std::endl;

TEST_SUITE("delta_enc") {
    // Modulo (as opposed to remainder), per https://stackoverflow.com/a/19288271
    int mod(int dividend, int divisor) {
        int r = dividend % divisor;
        return (r < 0) ? (r + divisor) : r;
    }

    int getDelta(int pos_abs, int count_in_cpr, int cpr) {
        int delta_enc = pos_abs - count_in_cpr;
        delta_enc = mod(delta_enc, cpr);
        if (delta_enc > (cpr / 2))
            delta_enc -= cpr;
        return delta_enc;
    }

    TEST_CASE("mod") {
        int cpr = 1000;

        // Check moves around 0
        CHECK(getDelta(1, 0, cpr) == 1);
        CHECK(getDelta(0, 1, cpr) == -1);
        CHECK(getDelta(999, 0, cpr) == -1);
        CHECK(getDelta(50, 650, cpr) == 400);
        CHECK(getDelta(650, 50, cpr) == -400);
        CHECK(getDelta(50, 500, cpr) == -450);
        CHECK(getDelta(500, 50, cpr) == 450);

        // Test moving a distance larger than cpr / 2
        CHECK(getDelta(950, 450, cpr) == 500);
        CHECK(getDelta(451, 950, cpr) == -499);
        CHECK(getDelta(450, 950, cpr) == 500);

        // Test handling around mid-point
        CHECK(getDelta(501, 499, cpr) == 2);
        CHECK(getDelta(499, 501, cpr) == -2);
        CHECK(getDelta(550, 450, cpr) == 100);
        CHECK(getDelta(450, 550, cpr) == -100);
    }
}

TEST_SUITE("velLimiter") {
// Velocity limiting in current mode
#include <algorithm>
    using doctest::Approx;

    auto limitVel(float vel_limit, float vel_estimate, float vel_gain, float Iq) {
        float Imax = (vel_limit - vel_estimate) * vel_gain;
        float Imin = (-vel_limit - vel_estimate) * vel_gain;
        return std::clamp(Iq, Imin, Imax);
    }

    TEST_CASE("limit Vel") {
        CHECK(limitVel(0, 0, 0, 0) == 0.0f);
        CHECK(limitVel(1000.0f, 1.0f, 0.0f, 0.0f) == 0.0f);
        CHECK(limitVel(1000.0f, 500.0f, 1.0f, 1.0f) == 1.0f);
        CHECK(limitVel(1000.0f, 500.0f, 1.0f, -20.0f) == -20.0f);
        CHECK(limitVel(1000.0f, 999.0f, 1.0f, 2.0f) == 1.0f);
        CHECK(limitVel(1000.0f, 999.0f, 1.0f, -5.0f) == -5.0f);
        CHECK(limitVel(1000.0f, -999.0f, 1.0f, -5.0f) == -1.0f);
        CHECK(limitVel(1000.0f, -999.0f, 1.0f, 5.0f) == 5.0f);
        CHECK(limitVel(1000.0f, 0.0f, 1.0f, 1.0f) == 1.0f);
        CHECK(limitVel(1000.0f, 0.0f, 1.0f, -1.0f) == -1.0f);
    }

    TEST_CASE("Accelerating") {
        CHECK(limitVel(200000.0f, 195000.0f, 5.0E-4f, 30.0f) == 2.5f);
        CHECK(limitVel(200000.0f, 205000.0f, 5.0E-4f, 30.0f) == -2.5f);
        CHECK(limitVel(200000.0f, -195000.0f, 5.0E-4, -30.0f) == -2.5f);
        CHECK(limitVel(200000.0f, -205000.0f, 5.0E-4f, -30.0f) == 2.5f);
    }

    TEST_CASE("Decelerating") {
        CHECK(limitVel(200000.0f, 195000.0f, 5.0E-4f, -30.0f) == -30.0f);
        CHECK(limitVel(200000.0f, 205000.0f, 5.0E-4f, -30.0f) == -30.0f);
        CHECK(limitVel(200000.0f, -195000.0f, 5.0E-4, 30.0f) == 30.0f);
        CHECK(limitVel(200000.0f, -205000.0f, 5.0E-4f, 30.0f) == 30.0f);
    }

    TEST_CASE("Over-Center") {
        CHECK(limitVel(20000.0f, 1000.0f, 5.0E-4f, 30.0f) == 9.5f);
        CHECK(limitVel(20000.0f, -1000.0f, 5.0E-4f, 30.0f) == Approx(10.5f));
    }
}

TEST_SUITE("velocity_feedback_filter") {
    TEST_CASE("initializes without a kick") {
        VelocityFeedbackFilter filter;
        CHECK(filter.update(1.25f, 0.000125f, 15.0f) == doctest::Approx(1.25f));
        CHECK(filter.initialized());
    }

    TEST_CASE("attenuates alternating measurement noise") {
        VelocityFeedbackFilter filter;
        filter.reset(1.0f);
        float output = 1.0f;
        for (int i = 0; i < 400; ++i) {
            output = filter.update((i & 1) ? 2.0f : 0.0f, 0.000125f, 15.0f);
        }
        CHECK(output == doctest::Approx(1.0f).epsilon(0.02f));
    }

    TEST_CASE("limits a single feedback burst") {
        VelocityFeedbackFilter filter;
        filter.reset(0.2f);
        const float output = filter.update(8.0f, 0.000125f, 12.0f);
        CHECK(output < 0.21f);
    }

    TEST_CASE("converges to a real speed change") {
        VelocityFeedbackFilter filter;
        filter.reset(0.0f);
        for (int i = 0; i < 1600; ++i)
            filter.update(2.0f, 0.000125f, 15.0f);
        CHECK(filter.value() == doctest::Approx(2.0f).epsilon(0.01f));
    }
}

TEST_SUITE("friction_compensator") {
    constexpr float dt = 0.000125f;
    constexpr float coulomb_torque = 0.0015f;
    constexpr float breakaway_torque = 0.0055f;

    TEST_CASE("running applies coulomb feed-forward in the command direction") {
        FrictionCompensator compensator;
        FrictionCompensationResult result;
        for (int i = 0; i < 4000; ++i) {
            result = compensator.update(
                    true, 0.5f, 0.5f, 0.0f, i / 10, dt);
        }
        CHECK(result.state == FrictionCompensator::STATE_RUNNING);
        CHECK(result.friction_torque == doctest::Approx(
                coulomb_torque).epsilon(0.02f));
    }

    TEST_CASE("feed-forward fades toward zero near zero command") {
        FrictionCompensator compensator;
        FrictionCompensationResult result;
        for (int i = 0; i < 4000; ++i) {
            result = compensator.update(
                    true, 0.005f, 0.005f, 0.0f, i / 10, dt);
        }
        // 0.005 / 0.02 = 0.25 of the coulomb level, never full coulomb.
        CHECK(result.friction_torque < coulomb_torque);
        CHECK(result.friction_torque > 0.0f);
    }

    TEST_CASE("breakaway engages when stalled with persistent error") {
        FrictionCompensator compensator;
        FrictionCompensationResult result;
        for (int i = 0; i < 12000; ++i) {
            result = compensator.update(
                    true, 0.2f, 0.0f, 0.2f, 0, dt);
        }
        CHECK(result.state == FrictionCompensator::STATE_BREAKAWAY);
        CHECK(result.friction_torque == doctest::Approx(
                breakaway_torque).epsilon(0.02f));
    }

    TEST_CASE("breakaway ramps instead of stepping") {
        FrictionCompensator compensator;
        FrictionCompensationResult result;
        for (int i = 0; i < 600; ++i) {
            result = compensator.update(
                    true, 0.2f, 0.0f, 0.2f, 0, dt);
        }
        // Just past the stall-confirm time: breakaway state, but the slew
        // limit means the torque has not yet reached the breakaway peak.
        CHECK(result.state == FrictionCompensator::STATE_BREAKAWAY);
        CHECK(result.friction_torque < breakaway_torque);
    }

    TEST_CASE("confirmed progress releases breakaway back to coulomb") {
        FrictionCompensator compensator;
        for (int i = 0; i < 12000; ++i) {
            compensator.update(true, 0.2f, 0.0f, 0.2f, 0, dt);
        }
        FrictionCompensationResult result;
        int32_t count = 0;
        for (int i = 0; i < 4000; ++i) {
            count += 4;  // clear forward progress every cycle
            result = compensator.update(
                    true, 0.2f, 0.2f, 0.0f, count, dt);
        }
        CHECK(result.state == FrictionCompensator::STATE_RUNNING);
        CHECK(result.friction_torque <=
                coulomb_torque + 0.0001f);
    }

    TEST_CASE("direction reversal cannot reuse positive compensation") {
        FrictionCompensator compensator;
        for (int i = 0; i < 4000; ++i) {
            compensator.update(true, 0.2f, 0.2f, 0.0f, i / 10, dt);
        }
        const auto result = compensator.update(
                true, -0.2f, 0.0f, -0.2f, 0, dt);
        CHECK(result.friction_torque <= 0.0f);
    }

    TEST_CASE("zero command gracefully ramps out") {
        FrictionCompensator compensator;
        for (int i = 0; i < 4000; ++i) {
            compensator.update(true, 0.2f, 0.2f, 0.0f, i / 10, dt);
        }
        const float before = compensator.friction_torque();
        CHECK(before > 0.0f);
        // A single zero-command cycle must not hard-jump to zero; it marks the
        // compensator idle and begins a graceful ramp-out.
        const auto result = compensator.update(
                false, 0.0f, 0.0f, 0.0f, 0, dt);
        CHECK(result.state == FrictionCompensator::STATE_IDLE);
        CHECK(result.friction_torque < before);
        CHECK(result.friction_torque > 0.0f);
    }

    TEST_CASE("isolated count dither does not reset the stall timer") {
        FrictionCompensator compensator;
        FrictionCompensationResult result;
        for (int i = 0; i < 12000; ++i) {
            // One-count ABZ bounce that returns to the same position.
            const int32_t count = (i / 10) & 1;
            result = compensator.update(
                    true, 0.2f, 0.0f, 0.2f, count, dt);
        }
        CHECK(result.state == FrictionCompensator::STATE_BREAKAWAY);
    }

    TEST_CASE("breakaway ramp is symmetric between + and - direction") {
        // Same number of stalled cycles in each direction must reach the same
        // torque magnitude. The old target>=value slew used the fall rate for
        // every negative move, making negative ramps much faster.
        FrictionCompensator positive;
        for (int i = 0; i < 600; ++i)
            positive.update(true, 0.2f, 0.0f, 0.2f, 0, dt);

        FrictionCompensator negative;
        for (int i = 0; i < 600; ++i)
            negative.update(true, -0.2f, 0.0f, -0.2f, 0, dt);

        CHECK(negative.friction_torque() == doctest::Approx(
                -positive.friction_torque()).epsilon(0.01f));
    }

    TEST_CASE("disable ramps to zero instead of stepping") {
        FrictionCompensator compensator;
        for (int i = 0; i < 12000; ++i)
            compensator.update(true, 0.2f, 0.0f, 0.2f, 0, dt);
        const float before = compensator.friction_torque();
        CHECK(before == doctest::Approx(
                breakaway_torque).epsilon(0.02f));

        // One disable cycle must not jump to zero.
        const float after_one = compensator.disable(dt);
        CHECK(std::abs(after_one) > 0.0f);
        CHECK(std::abs(after_one) < std::abs(before));

        // Over many cycles it decays to zero.
        float v = after_one;
        for (int i = 0; i < 4000; ++i)
            v = compensator.disable(dt);
        CHECK(std::abs(v) < 0.0002f);
    }

    TEST_CASE("configure clamps breakaway >= coulomb and rejects negatives") {
        FrictionCompensator compensator;
        compensator.configure(0.004f, 0.002f);  // breakaway < coulomb
        CHECK(compensator.coulomb_torque() == doctest::Approx(0.004f));
        CHECK(compensator.breakaway_torque() == doctest::Approx(0.004f));

        compensator.configure(-1.0f, -2.0f);    // negatives -> 0
        CHECK(compensator.coulomb_torque() == 0.0f);
        CHECK(compensator.breakaway_torque() == 0.0f);

        compensator.configure(0.0015f, 0.0055f); // normal
        CHECK(compensator.coulomb_torque() == doctest::Approx(0.0015f));
        CHECK(compensator.breakaway_torque() == doctest::Approx(0.0055f));
    }

    TEST_CASE("directional speed ratio rejects reverse motion and is symmetric") {
        // speed_ratio must be the velocity PROJECTED onto the command
        // direction, never abs(measured). Reverse motion -> 0.
        auto ratio = [](float command, float measured) {
            FrictionCompensator c;
            const auto r = c.update(true, command, measured,
                    command - measured, 0, dt);
            return r.speed_ratio;
        };
        CHECK(ratio(0.2f, 0.2f) == doctest::Approx(1.0f).epsilon(0.01f));
        CHECK(ratio(0.2f, 0.1f) == doctest::Approx(0.5f).epsilon(0.01f));
        CHECK(ratio(0.2f, 0.0f) == 0.0f);
        CHECK(ratio(0.2f, -0.2f) == 0.0f);
        CHECK(ratio(-0.2f, -0.2f) == doctest::Approx(1.0f).epsilon(0.01f));
        CHECK(ratio(-0.2f, 0.2f) == 0.0f);
    }

    TEST_CASE("reverse motion is detected and keeps assist at breakaway") {
        FrictionCompensator compensator;
        const auto r = compensator.update(
                true, 0.2f, -0.2f, 0.4f, 0, dt);
        CHECK(r.reverse_detected == true);
        CHECK(r.speed_ratio == 0.0f);
        CHECK(r.assist_blend == doctest::Approx(1.0f));
    }
}

TEST_SUITE("control_velocity_observer") {
    constexpr float dt = 0.000125f;

    float run_constant_speed(ControlVelocityObserver& obs, float speed,
                             int cycles) {
        double pos = 0.0;
        float v = 0.0f;
        for (int i = 0; i < cycles; ++i) {
            pos += (double)speed * (double)dt;
            v = obs.update((float)pos, dt);
        }
        return v;
    }

    TEST_CASE("converges to a constant speed") {
        ControlVelocityObserver obs;
        obs.configure(30.0f);
        obs.reset(0.0f);
        CHECK(run_constant_speed(obs, 1.0f, 8000) ==
                doctest::Approx(1.0f).epsilon(0.01f));
    }

    TEST_CASE("smooths quantized count input without staircase ripple") {
        ControlVelocityObserver obs;
        obs.configure(30.0f);
        obs.reset(0.0f);
        // Feed integer-count position (0.00025 turn per count at 4000 CPR)
        // while the rotor moves at 1 turn/s. The observer output must not
        // reproduce the count staircase.
        double pos_counts = 0.0;
        float v = 0.0f;
        float max_v = 0.0f, min_v = 1e9f;
        for (int i = 0; i < 16000; ++i) {
            pos_counts += 1.0 * 4000.0 * (double)dt;
            const int32_t count = (int32_t)std::floor(pos_counts);
            v = obs.update((float)count / 4000.0f, dt);
            if (i > 2000) {
                max_v = std::max(max_v, v);
                min_v = std::min(min_v, v);
            }
        }
        CHECK(v == doctest::Approx(1.0f).epsilon(0.01f));
        // Peak-to-peak ripple far below a single-count step (~0.125 turn/s
        // over 2 ms).
        CHECK(max_v - min_v < 0.05f);
    }

    TEST_CASE("tracks negative speed") {
        ControlVelocityObserver obs;
        obs.configure(30.0f);
        obs.reset(0.0f);
        CHECK(run_constant_speed(obs, -1.0f, 8000) ==
                doctest::Approx(-1.0f).epsilon(0.01f));
    }

    TEST_CASE("reset produces no startup spike") {
        ControlVelocityObserver obs;
        obs.configure(30.0f);
        obs.reset(0.0f);
        const float v = obs.update(0.0f, dt);
        CHECK(v == 0.0f);
    }
}

TEST_SUITE("vel_ramp") {
    float vel_ramp_old(float input_vel_, float vel_setpoint_, float vel_ramp_rate) {
        float max_step_size = 0.000125f * vel_ramp_rate;
        float full_step = input_vel_ - vel_setpoint_;
        float step;
        if (std::abs(full_step) > max_step_size) {
            step = std::copysignf(max_step_size, full_step);
        } else {
            step = full_step;
        }
        return step;
    }

    float vel_ramp_new(float input_vel_, float vel_setpoint_, float vel_ramp_rate) {
        float max_step_size = 0.000125f * vel_ramp_rate;
        float full_step = input_vel_ - vel_setpoint_;
        return std::clamp(full_step, -max_step_size, max_step_size);
    }
    
    uint8_t parity(uint16_t v) {
        v ^= v >> 8;
        v ^= v >> 4;
        v ^= v >> 2;
        v ^= v >> 1;
        return v & 1;
    }

    TEST_CASE("Equivalence") {
        float vel_setpoint = 0.0f;
        float vel_ramp_rate = 8000;
        float input_vel = 0.0f;
        CHECK(vel_ramp_old(input_vel, vel_setpoint, vel_ramp_rate) == vel_ramp_new(input_vel, vel_setpoint, vel_ramp_rate));

        input_vel = 10.0f;
        CHECK(vel_ramp_old(input_vel, vel_setpoint, vel_ramp_rate) == vel_ramp_new(input_vel, vel_setpoint, vel_ramp_rate));

        input_vel = 10000.0f;
        CHECK(vel_ramp_old(input_vel, vel_setpoint, vel_ramp_rate) == vel_ramp_new(input_vel, vel_setpoint, vel_ramp_rate));

        input_vel = -10000.0f;
        CHECK(vel_ramp_old(input_vel, vel_setpoint, vel_ramp_rate) == vel_ramp_new(input_vel, vel_setpoint, vel_ramp_rate));

        input_vel = -0.1234f;
        CHECK(vel_ramp_old(input_vel, vel_setpoint, vel_ramp_rate) == vel_ramp_new(input_vel, vel_setpoint, vel_ramp_rate));

        input_vel = 0.1234f;
        CHECK(vel_ramp_old(input_vel, vel_setpoint, vel_ramp_rate) == vel_ramp_new(input_vel, vel_setpoint, vel_ramp_rate));
    }

    TEST_CASE("Parity") {
        CHECK(parity(0x0DDF & 0x7FFF) == 0);
        CHECK(parity(0x8DDF & 0x7FFF) == 0);
        CHECK(parity(0x5BFF & 0x7FFF) == 1);
    }
}

TEST_SUITE("incremental_velocity_estimator") {
    constexpr float dt = 0.000125f;  // 8 kHz control period

    // Feed a constant mechanical speed using a sub-count position accumulator
    // so the emitted integer count deltas are the exact sequence a real ABZ
    // encoder would produce.
    float run_constant_speed(IncrementalVelocityEstimator& est, float speed,
                             float cpr, float dt, int cycles) {
        double pos = 0.0;
        int32_t prev_count = 0;
        float v = 0.0f;
        for (int i = 0; i < cycles; ++i) {
            pos += (double)speed * (double)cpr * (double)dt;
            const int32_t count = (int32_t)std::floor(pos);
            const int32_t delta = count - prev_count;
            prev_count = count;
            v = est.update(delta, dt, cpr);
        }
        return v;
    }

    TEST_CASE("constant positive speed for several CPR values") {
        for (float cpr : {4000.0f, 8192.0f, 16384.0f}) {
            for (float speed : {0.2f, 0.5f, 1.0f, 2.0f}) {
                IncrementalVelocityEstimator est;
                const float v = run_constant_speed(est, speed, cpr, dt, 40000);
                CHECK(v == doctest::Approx(speed).epsilon(0.05f));
            }
        }
    }

    TEST_CASE("constant negative speed") {
        IncrementalVelocityEstimator est;
        const float v = run_constant_speed(est, -1.0f, 4000.0f, dt, 40000);
        CHECK(v == doctest::Approx(-1.0f).epsilon(0.05f));
    }

    TEST_CASE("startup from zero converges without a huge overshoot") {
        IncrementalVelocityEstimator est;
        float v = 0.0f;
        float max_v = 0.0f;
        double pos = 0.0;
        int32_t prev_count = 0;
        for (int i = 0; i < 40000; ++i) {
            pos += 0.2 * 4000.0 * (double)dt;
            const int32_t count = (int32_t)std::floor(pos);
            const int32_t delta = count - prev_count;
            prev_count = count;
            v = est.update(delta, dt, 4000.0f);
            max_v = std::max(max_v, v);
        }
        CHECK(v == doctest::Approx(0.2f).epsilon(0.05f));
        CHECK(max_v < 0.4f);  // slew-limited, no runaway spike
    }

    TEST_CASE("sudden stop decays to zero") {
        IncrementalVelocityEstimator est;
        run_constant_speed(est, 1.0f, 4000.0f, dt, 20000);
        float v = 0.0f;
        for (int i = 0; i < 4000; ++i)
            v = est.update(0, dt, 4000.0f);
        CHECK(std::fabs(v) < 0.01f);
    }

    TEST_CASE("reversal crosses zero and tracks the new direction") {
        IncrementalVelocityEstimator est;
        run_constant_speed(est, 1.0f, 4000.0f, dt, 20000);
        const float v = run_constant_speed(est, -1.0f, 4000.0f, dt, 40000);
        CHECK(v == doctest::Approx(-1.0f).epsilon(0.05f));
    }

    TEST_CASE("multiple counts per control cycle at high speed") {
        IncrementalVelocityEstimator est;
        const float v = run_constant_speed(est, 10.0f, 4000.0f, dt, 40000);
        CHECK(v == doctest::Approx(10.0f).epsilon(0.05f));
    }

    TEST_CASE("sparse counts at very low speed do not read zero") {
        IncrementalVelocityEstimator est;
        run_constant_speed(est, 0.05f, 4000.0f, dt, 5000);  // warm up
        double pos = 5000 * 0.05 * 4000.0 * (double)dt;
        int32_t prev_count = (int32_t)std::floor(pos);
        float v = 0.0f;
        float min_v = 1e9f;
        for (int i = 0; i < 20000; ++i) {
            pos += 0.05 * 4000.0 * (double)dt;
            const int32_t count = (int32_t)std::floor(pos);
            const int32_t delta = count - prev_count;
            prev_count = count;
            v = est.update(delta, dt, 4000.0f);
            min_v = std::min(min_v, v);
        }
        CHECK(v == doctest::Approx(0.05f).epsilon(0.20f));
        CHECK(min_v > 0.0f);  // never collapses to zero at constant speed
    }

    TEST_CASE("a single stray count is slew limited, not a spike") {
        IncrementalVelocityEstimator est;
        run_constant_speed(est, 0.0f, 4000.0f, dt, 100);  // settle at zero
        est.update(1, dt, 4000.0f);  // one glitch count
        float v = 0.0f;
        for (int i = 0; i < 100; ++i)
            v = est.update(0, dt, 4000.0f);  // let max publish window fire
        CHECK(v < 0.05f);
    }
}

TEST_SUITE("velocity_integrator_antiwindup") {
    // Reference of the controller's conditional-integration step so the logic
    // itself is covered on the host even though the full axis is not linked.
    float step(float integrator, float ki, float v_err, float dt,
               float torque_unsaturated, float Tlim) {
        const bool saturated_high = torque_unsaturated > Tlim;
        const bool saturated_low = torque_unsaturated < -Tlim;
        const bool windup_forward = saturated_high && v_err > 0.0f;
        const bool windup_reverse = saturated_low && v_err < 0.0f;
        if (!windup_forward && !windup_reverse)
            integrator += ki * v_err * dt;
        return std::clamp(integrator, -Tlim, Tlim);
    }

    TEST_CASE("integrator does not wind up in forward saturation") {
        const float Tlim = 0.025f;
        const float ki = 0.01f;
        const float dt = 0.000125f;
        float integrator = 0.0f;
        const float before = integrator;
        // Torque is saturated high while error keeps pushing forward: hold.
        integrator = step(integrator, ki, 1.0f, dt, Tlim + 0.1f, Tlim);
        CHECK(integrator == doctest::Approx(before));
    }

    TEST_CASE("integrator releases when the error reverses") {
        const float Tlim = 0.025f;
        const float ki = 0.01f;
        const float dt = 0.000125f;
        float integrator = 0.001f;
        // Saturated high but error negative now: allow integration (release).
        const float released = step(integrator, ki, -1.0f, dt, Tlim + 0.1f, Tlim);
        CHECK(released < integrator);
    }

    TEST_CASE("integrator integrates normally when unsaturated") {
        const float Tlim = 0.025f;
        const float ki = 0.01f;
        const float dt = 0.000125f;
        float integrator = 0.0f;
        integrator = step(integrator, ki, 1.0f, dt, 0.0f, Tlim);
        CHECK(integrator == doctest::Approx(ki * 1.0f * dt));
    }

    TEST_CASE("integrator is bounded by the torque limit") {
        const float Tlim = 0.025f;
        const float ki = 0.01f;
        const float dt = 0.000125f;
        float integrator = 0.0f;
        for (int i = 0; i < 100000; ++i)
            integrator = step(integrator, ki, 0.001f, dt, 0.0f, Tlim);
        CHECK(integrator <= Tlim);
        CHECK(integrator >= -Tlim);
    }
}
