#pragma once

#include "flight_model.hpp"

#include <algorithm>
#include <cmath>
#include <type_traits>
#include <utility>

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

    // The callback observes each successfully simulated fixed tick before the
    // session commits its frame. Callers can stage coupled state locally and
    // commit it only when this method returns true.
    template <typename OnTick>
    [[nodiscard]] bool advance_frame(double elapsed_seconds,
                                     const AircraftFlightInput& input,
                                     OnTick&& on_tick) noexcept {
        static_assert(std::is_nothrow_invocable_v<OnTick&, const AircraftFlightState&>,
                      "flight tick callbacks must be noexcept");
        if (!std::isfinite(elapsed_seconds) || elapsed_seconds < 0.0 ||
            !std::isfinite(accumulator_seconds_)) return false;

        constexpr double tick_seconds = 1.0 / 60.0;
        constexpr double maximum_frame_seconds = 0.1;
        double next_accumulator = std::min(accumulator_seconds_ + elapsed_seconds,
                                           maximum_frame_seconds);
        AircraftFlightState next_state = state_;
        while (next_accumulator >= tick_seconds) {
            if (!advance_aircraft_carrier_tick(rules_, input, next_state)) return false;
            on_tick(next_state);
            next_accumulator -= tick_seconds;
        }
        state_ = std::move(next_state);
        accumulator_seconds_ = next_accumulator;
        return true;
    }

    [[nodiscard]] const AircraftFlightState& state() const noexcept { return state_; }
    [[nodiscard]] AircraftFlightState& state() noexcept { return state_; }
    void reset_clock() noexcept { accumulator_seconds_ = 0.0; }

private:
    AircraftFlightRules rules_;
    AircraftFlightState state_;
    double accumulator_seconds_ = 0.0;
};

} // namespace ac5
