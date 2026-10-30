#include "flight_session.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ac5 {
namespace {

constexpr double tick_seconds = 1.0 / 60.0;
constexpr double maximum_frame_seconds = 0.1;

} // namespace

AircraftFlightSession::AircraftFlightSession(AircraftFlightRules rules,
                                             AircraftFlightState initial_state) noexcept
    : rules_(std::move(rules)), state_(std::move(initial_state)) {}

bool AircraftFlightSession::advance_frame(double elapsed_seconds,
                                          const AircraftFlightInput& input) noexcept {
    if (!std::isfinite(elapsed_seconds) || elapsed_seconds < 0.0 ||
        !std::isfinite(accumulator_seconds_)) return false;

    double next_accumulator = std::min(accumulator_seconds_ + elapsed_seconds,
                                       maximum_frame_seconds);
    AircraftFlightState next_state = state_;
    while (next_accumulator >= tick_seconds) {
        if (!advance_aircraft_carrier_tick(rules_, input, next_state)) return false;
        next_accumulator -= tick_seconds;
    }
    state_ = std::move(next_state);
    accumulator_seconds_ = next_accumulator;
    return true;
}

} // namespace ac5
