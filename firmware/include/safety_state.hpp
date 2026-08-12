#pragma once

#include "feedback_mode.hpp"

#include <cstdint>

namespace foc2 {

enum Fault : std::uint32_t {
    FaultNone = 0,
    FaultOverCurrent = 1u << 0,
    FaultBusUnderVoltage = 1u << 1,
    FaultBusOverVoltage = 1u << 2,
    FaultOverTemperature = 1u << 3,
    FaultEncoder = 1u << 4,
    FaultSensorlessLost = 1u << 5,
    FaultWatchdog = 1u << 6,
    FaultGateDriver = 1u << 7,
    FaultOverspeed = 1u << 8,
};

class SafetyStateMachine {
public:
    AxisState state() const { return state_; }
    std::uint32_t faults() const { return faults_; }
    bool pwm_enabled() const { return state_ == AxisState::Armed || state_ == AxisState::Running; }

    void boot_complete();
    bool begin_calibration();
    bool arm();
    bool begin_running();
    void disarm();
    void raise_fault(std::uint32_t fault);
    void clear_faults();

private:
    AxisState state_ = AxisState::Boot;
    std::uint32_t faults_ = FaultNone;
};

}  // namespace foc2
