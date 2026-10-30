#pragma once

#include "flight_rules_loader.hpp"

#include <array>

namespace ac5 {

struct AircraftFlightState {
    std::array<double, 3> euler{}; // body pitch, yaw, roll in radians
    std::array<double, 3> rates{}; // pitch, yaw, roll in radians per tick
    double speed_units_per_tick = 0.0;
    double stall_factor = 0.0;
};

struct AircraftFlightInput {
    double altitude_units = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    double yaw = 0.0;
    double throttle_lever = 0.0;
    int ticks_per_second = 60;
    bool reduced_speed_mode = false;
};

// Advance the airborne speed/rate/attitude portion of one native flight tick.
// Ground handling, trim, stall nose-drop, and position integration are separate.
[[nodiscard]] bool advance_aircraft_flight_tick(
    const AircraftFlightRules& rules, const AircraftFlightInput& input,
    AircraftFlightState& state) noexcept;

} // namespace ac5
