#include "flight_session.hpp"

namespace ac5 {

AircraftFlightSession::AircraftFlightSession(AircraftFlightRules rules,
                                             AircraftFlightState initial_state) noexcept
    : rules_(std::move(rules)), state_(std::move(initial_state)) {}

bool AircraftFlightSession::advance_frame(double elapsed_seconds,
                                          const AircraftFlightInput& input) noexcept {
    return advance_frame(elapsed_seconds, input,
                         [](const AircraftFlightState&) noexcept {});
}

} // namespace ac5
