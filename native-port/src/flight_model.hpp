#pragma once

#include "flight_rules_loader.hpp"

#include <array>
#include <cstdint>

namespace ac5 {

struct AircraftFlightFlags {
    std::uint8_t contact = 0;
    std::uint8_t mode = 0;
};

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
    AircraftFlightFlags flags{};
    bool suppress_inverted_sink = false;
    bool reset_contact_speed = false;
    double contact_acceleration_scale = 1.0;
    double contact_acceleration_bias = 0.0;
};

// Set the recovered ground/contact modes while preserving unrelated flag bits.
void set_aircraft_flight_mode(AircraftFlightFlags& flags,
                              std::uint8_t command) noexcept;
void set_aircraft_approach_control(AircraftFlightFlags& flags,
                                   bool enabled) noexcept;

// Advance one native flight tick. Contact bit 0 selects the recovered scalar
// contact-speed law; terrain collision and ground-marker alignment are separate.
[[nodiscard]] bool advance_aircraft_flight_tick(
    const AircraftFlightRules& rules, const AircraftFlightInput& input,
    AircraftFlightState& state) noexcept;

} // namespace ac5
