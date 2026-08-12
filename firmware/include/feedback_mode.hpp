#pragma once

#include <cstdint>

namespace foc2 {

enum class FeedbackMode : std::uint8_t {
    SpiAbsolute = 0,
    IncrementalAbz = 1,
    Sensorless = 2,
    SensorlessWithSpiMonitor = 3,
    SensorlessWithAbzMonitor = 4,
};

enum class AxisState : std::uint8_t {
    Boot = 0,
    Idle = 1,
    Calibrating = 2,
    Armed = 3,
    Running = 4,
    Fault = 5,
};

struct FeedbackSample {
    float position_turns = 0.0f;
    float velocity_turns_per_second = 0.0f;
    float electrical_phase_rad = 0.0f;
    bool valid = false;
    bool index_found = false;
};

struct FeedbackDiagnostics {
    float position_error_turns = 0.0f;
    float velocity_error_turns_per_second = 0.0f;
    bool comparison_valid = false;
};

bool is_sensorless(FeedbackMode mode);
bool uses_spi_encoder(FeedbackMode mode);
bool uses_abz_encoder(FeedbackMode mode);
bool supports_position_control(FeedbackMode mode);
const char* feedback_mode_name(FeedbackMode mode);

class FeedbackRouter {
public:
    explicit FeedbackRouter(FeedbackMode mode = FeedbackMode::SpiAbsolute);

    bool request_mode(FeedbackMode next, AxisState state);
    FeedbackMode mode() const { return mode_; }
    FeedbackSample control_sample() const;
    FeedbackDiagnostics diagnostics() const;

    void update_encoder(const FeedbackSample& sample);
    void update_observer(const FeedbackSample& sample);

private:
    FeedbackMode mode_;
    FeedbackSample encoder_{};
    FeedbackSample observer_{};
};

}  // namespace foc2
