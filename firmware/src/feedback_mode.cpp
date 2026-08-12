#include "feedback_mode.hpp"

#include <cmath>

namespace foc2 {

bool is_sensorless(FeedbackMode mode) {
    return mode == FeedbackMode::Sensorless ||
           mode == FeedbackMode::SensorlessWithSpiMonitor ||
           mode == FeedbackMode::SensorlessWithAbzMonitor;
}

bool uses_spi_encoder(FeedbackMode mode) {
    return mode == FeedbackMode::SpiAbsolute ||
           mode == FeedbackMode::SensorlessWithSpiMonitor;
}

bool uses_abz_encoder(FeedbackMode mode) {
    return mode == FeedbackMode::IncrementalAbz ||
           mode == FeedbackMode::SensorlessWithAbzMonitor;
}

bool supports_position_control(FeedbackMode mode) {
    return mode == FeedbackMode::SpiAbsolute || mode == FeedbackMode::IncrementalAbz;
}

const char* feedback_mode_name(FeedbackMode mode) {
    switch (mode) {
        case FeedbackMode::SpiAbsolute: return "spi-absolute";
        case FeedbackMode::IncrementalAbz: return "incremental-abz";
        case FeedbackMode::Sensorless: return "sensorless";
        case FeedbackMode::SensorlessWithSpiMonitor: return "sensorless-spi-monitor";
        case FeedbackMode::SensorlessWithAbzMonitor: return "sensorless-abz-monitor";
    }
    return "unknown";
}

FeedbackRouter::FeedbackRouter(FeedbackMode mode) : mode_(mode) {}

bool FeedbackRouter::request_mode(FeedbackMode next, AxisState state) {
    if (state != AxisState::Idle && state != AxisState::Boot) {
        return false;
    }
    mode_ = next;
    return true;
}

FeedbackSample FeedbackRouter::control_sample() const {
    return is_sensorless(mode_) ? observer_ : encoder_;
}

FeedbackDiagnostics FeedbackRouter::diagnostics() const {
    FeedbackDiagnostics result;
    if (!is_sensorless(mode_) || !encoder_.valid || !observer_.valid) {
        return result;
    }

    result.position_error_turns = encoder_.position_turns - observer_.position_turns;
    while (result.position_error_turns > 0.5f) result.position_error_turns -= 1.0f;
    while (result.position_error_turns < -0.5f) result.position_error_turns += 1.0f;
    result.velocity_error_turns_per_second =
        encoder_.velocity_turns_per_second - observer_.velocity_turns_per_second;
    result.comparison_valid = true;
    return result;
}

void FeedbackRouter::update_encoder(const FeedbackSample& sample) {
    encoder_ = sample;
}

void FeedbackRouter::update_observer(const FeedbackSample& sample) {
    observer_ = sample;
}

}  // namespace foc2
