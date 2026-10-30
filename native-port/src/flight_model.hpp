#pragma once

#include "flight_rules_loader.hpp"

#include <array>

namespace ac5 {

struct AircraftFlightState {
    std::array<double, 3> position{0.0, 3000.0, 0.0};
    std::array<double, 3> euler{}; // body pitch, yaw, roll in radians
    std::array<double, 3> path_euler{};
    std::array<double, 3> rates{}; // pitch, yaw, roll in radians per tick
    double speed_units_per_tick = 0.0;
    double stall_factor = 0.0;
    double path_lag = 1.0;
    double approach_pitch = 0.0;
};

struct AircraftFlightInput {
    double pitch = 0.0;
    double roll = 0.0;
    double yaw = 0.0;
    double throttle_lever = 0.0;
    int ticks_per_second = 60;
    bool reduced_speed_mode = false;
    bool suppress_inverted_sink = false;
    bool approach_control = false;
};

// Advance one native airborne speed/rate/attitude tick. Ground handling and
// terrain collision are separate; approach_control selects the recovered
// approach stick law and skips steady trim.
[[nodiscard]] bool advance_aircraft_flight_tick(
    const AircraftFlightRules& rules, const AircraftFlightInput& input,
    AircraftFlightState& state) noexcept;

} // namespace ac5
