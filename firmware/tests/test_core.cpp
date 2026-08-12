#include "feedback_mode.hpp"
#include "foc_config.hpp"
#include "safety_state.hpp"

#include <cassert>
#include <cmath>

int main() {
    using namespace foc2;

    assert(EncoderConfig::spi_raw_cpr == 16384);
    assert(EncoderConfig::abz_cpr == 4000);
    assert(std::fabs(MotorConfig::torque_constant_nm_per_a - 0.012723f) < 0.00001f);
    assert(std::fabs(MotorConfig::pm_flux_linkage_v_per_rad_s - 0.000848f) < 0.00001f);
    assert(std::fabs(MotorConfig::max_speed_turns_per_second - 130.0f) < 0.001f);

    FeedbackRouter router;
    assert(router.mode() == FeedbackMode::SpiAbsolute);
    assert(router.request_mode(FeedbackMode::IncrementalAbz, AxisState::Idle));
    assert(!router.request_mode(FeedbackMode::Sensorless, AxisState::Running));
    assert(supports_position_control(FeedbackMode::IncrementalAbz));
    assert(!supports_position_control(FeedbackMode::Sensorless));

    FeedbackSample encoder;
    encoder.position_turns = 2.0f;
    encoder.velocity_turns_per_second = 10.0f;
    encoder.valid = true;
    FeedbackSample observer;
    observer.position_turns = 1.9f;
    observer.velocity_turns_per_second = 9.5f;
    observer.valid = true;
    router.update_encoder(encoder);
    router.update_observer(observer);
    assert(router.request_mode(FeedbackMode::SensorlessWithSpiMonitor, AxisState::Idle));
    const auto diagnostics = router.diagnostics();
    assert(diagnostics.comparison_valid);
    assert(std::fabs(diagnostics.position_error_turns - 0.1f) < 0.0001f);

    encoder.position_turns = 0.01f;
    observer.position_turns = 0.99f;
    router.update_encoder(encoder);
    router.update_observer(observer);
    assert(std::fabs(router.diagnostics().position_error_turns - 0.02f) < 0.0001f);

    SafetyStateMachine safety;
    safety.boot_complete();
    assert(safety.state() == AxisState::Idle);
    assert(safety.arm());
    assert(safety.begin_running());
    safety.raise_fault(FaultOverCurrent);
    assert(!safety.pwm_enabled());
    assert(safety.state() == AxisState::Fault);
    safety.clear_faults();
    assert(safety.state() == AxisState::Idle);

    return 0;
}
