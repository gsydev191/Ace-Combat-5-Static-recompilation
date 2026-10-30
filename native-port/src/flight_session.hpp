#pragma once

#include "flight_model.hpp"

namespace ac5 {

// Platform-independent 60 Hz simulation clock for a native aircraft.
class AircraftFlightSession {
public:
    AircraftFlightSession(AircraftFlightRules rules,
                          AircraftFlightState initial_state = {}) noexcept;

    // Advance by elapsed wall-clock time, capped at 100 ms to bound catch-up.
    // State and accumulator are committed only if every simulated tick succeeds.
    [[nodiscard]] bool advance_frame(double elapsed_seconds,
                                     const AircraftFlightInput& input) noexcept;

    [[nodiscard]] const AircraftFlightState& state() const noexcept { return state_; }
    [[nodiscard]] AircraftFlightState& state() noexcept { return state_; }
    void reset_clock() noexcept { accumulator_seconds_ = 0.0; }

private:
    AircraftFlightRules rules_;
    AircraftFlightState state_;
    double accumulator_seconds_ = 0.0;
};

} // namespace ac5
