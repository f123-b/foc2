#include "safety_state.hpp"

namespace foc2 {

void SafetyStateMachine::boot_complete() {
    if (state_ == AxisState::Boot) {
        state_ = AxisState::Idle;
    }
}

bool SafetyStateMachine::begin_calibration() {
    if (state_ != AxisState::Idle || faults_ != FaultNone) {
        return false;
    }
    state_ = AxisState::Calibrating;
    return true;
}

bool SafetyStateMachine::arm() {
    if (state_ != AxisState::Idle || faults_ != FaultNone) {
        return false;
    }
    state_ = AxisState::Armed;
    return true;
}

bool SafetyStateMachine::begin_running() {
    if (state_ != AxisState::Armed || faults_ != FaultNone) {
        return false;
    }
    state_ = AxisState::Running;
    return true;
}

void SafetyStateMachine::disarm() {
    if (faults_ == FaultNone) {
        state_ = AxisState::Idle;
    }
}

void SafetyStateMachine::raise_fault(std::uint32_t fault) {
    faults_ |= fault;
    state_ = AxisState::Fault;
}

void SafetyStateMachine::clear_faults() {
    if (state_ == AxisState::Fault) {
        faults_ = FaultNone;
        state_ = AxisState::Idle;
    }
}

}  // namespace foc2
