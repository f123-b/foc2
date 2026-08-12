

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
#include <low_speed_compensator.hpp>
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

    TEST_CASE("converges to a real speed change") {
        VelocityFeedbackFilter filter;
        filter.reset(0.0f);
        for (int i = 0; i < 1600; ++i)
            filter.update(2.0f, 0.000125f, 15.0f);
        CHECK(filter.value() == doctest::Approx(2.0f).epsilon(0.01f));
    }
}

TEST_SUITE("low_speed_compensator") {
    TEST_CASE("count progress prevents false stall detection") {
        LowSpeedCompensator compensator;
        LowSpeedCompensationResult result;
        int32_t count = 0;
        for (int i = 0; i < 8000; ++i) {
            if (i % 10 == 0)
                ++count;
            result = compensator.update(
                    true, 1.0f, 0.2f, 0.2f, 0.0f, 0.0f,
                    count, 0.000125f);
        }
        CHECK(result.state == LowSpeedCompensator::STATE_RUNNING);
        CHECK_FALSE(result.hold_integrator);
        CHECK(result.friction_torque == doctest::Approx(
                LowSpeedCompensator::running_torque).epsilon(0.02f));
    }

    TEST_CASE("holds the integrator and ramps breakaway torque while stalled") {
        LowSpeedCompensator compensator;
        LowSpeedCompensationResult result;
        for (int i = 0; i < 12000; ++i) {
            result = compensator.update(
                    true, 1.0f, 0.2f, 0.0f, 0.2f, 0.001f,
                    0, 0.000125f);
        }
        CHECK(result.state == LowSpeedCompensator::STATE_BREAKAWAY);
        CHECK(result.hold_integrator);
        CHECK(result.friction_torque == doctest::Approx(
                LowSpeedCompensator::breakaway_torque).epsilon(0.02f));
    }

    TEST_CASE("count dither cannot release breakaway torque") {
        LowSpeedCompensator compensator;
        for (int i = 0; i < 12000; ++i) {
            compensator.update(
                    true, 1.0f, 0.2f, 0.0f, 0.2f, 0.0f,
                    0, 0.000125f);
        }
        auto result = compensator.update(
                true, 1.0f, 0.2f, 0.1f, 0.1f, 0.0f,
                1, 0.000125f);
        CHECK(result.state == LowSpeedCompensator::STATE_BREAKAWAY);
        for (int i = 0; i < 200; ++i) {
            result = compensator.update(
                    true, 1.0f, 0.2f, 0.0f, 0.2f, 0.0f,
                    (i & 1) ? 0 : 1, 0.000125f);
        }
        CHECK(result.state == LowSpeedCompensator::STATE_BREAKAWAY);
        CHECK(result.hold_integrator);
    }

    TEST_CASE("sustained forward counts enter recovery and unload promptly") {
        LowSpeedCompensator compensator;
        for (int i = 0; i < 12000; ++i) {
            compensator.update(
                    true, 1.0f, 0.2f, 0.0f, 0.2f, 0.0f,
                    0, 0.000125f);
        }
        LowSpeedCompensationResult result;
        for (int32_t count = 1; count <=
                LowSpeedCompensator::recovery_progress_counts; ++count) {
            result = compensator.update(
                    true, 1.0f, 0.2f, 0.1f, 0.1f, 0.0f,
                    count, 0.000125f);
        }
        CHECK(result.state == LowSpeedCompensator::STATE_RECOVERING);
        CHECK(result.hold_integrator);
        for (int i = 0; i < 600; ++i) {
            result = compensator.update(
                    true, 1.0f, 0.2f, 0.2f, 0.0f, 0.0f,
                    LowSpeedCompensator::recovery_progress_counts + i / 10,
                    0.000125f);
        }
        CHECK(result.state == LowSpeedCompensator::STATE_RUNNING);
        CHECK_FALSE(result.hold_integrator);
        CHECK(result.friction_torque == doctest::Approx(
                LowSpeedCompensator::running_torque).epsilon(0.03f));
    }

    TEST_CASE("removes drive torque during overspeed without changing sign") {
        LowSpeedCompensator compensator;
        LowSpeedCompensationResult result;
        for (int i = 0; i < 400; ++i)
            result = compensator.update(
                    true, 1.0f, 0.2f, 0.5f, -0.3f, 0.0f,
                    i, 0.000125f);
        CHECK(result.friction_torque >= 0.0f);
        CHECK(result.friction_torque < 0.00001f);
    }

    TEST_CASE("direction reversal cannot reuse positive compensation") {
        LowSpeedCompensator compensator;
        for (int i = 0; i < 8000; ++i) {
            compensator.update(
                    true, 1.0f, 0.2f, 0.0f, 0.2f, 0.0f,
                    0, 0.000125f);
        }
        const auto result = compensator.update(
                true, -1.0f, -0.2f, 0.0f, -0.2f, 0.0f,
                0, 0.000125f);
        CHECK(result.friction_torque <= 0.0f);
        CHECK(std::abs(result.friction_torque) < 0.00001f);
    }

    TEST_CASE("zero command clears compensation immediately") {
        LowSpeedCompensator compensator;
        for (int i = 0; i < 8000; ++i) {
            compensator.update(
                    true, 1.0f, 0.2f, 0.0f, 0.2f, 0.0f,
                    0, 0.000125f);
        }
        const auto result = compensator.update(
                false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0, 0.000125f);
        CHECK(result.friction_torque == 0.0f);
        CHECK_FALSE(result.hold_integrator);
        CHECK(compensator.compensation_magnitude() == 0.0f);
        CHECK(compensator.state() == LowSpeedCompensator::STATE_IDLE);
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
