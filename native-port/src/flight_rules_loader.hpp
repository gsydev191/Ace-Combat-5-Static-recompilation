#pragma once

#include "flight_dynamics.hpp"

#include <cstdint>
#include <span>
#include <string>

namespace ac5 {

struct AircraftFlightRules {
    FlightSpeedRules speed;
    FlightAxisRules pitch;
    FlightAxisRules roll;
    FlightAxisRules yaw;
    double trim_pitch_gain = 0.0;
    double trim_yaw_gain = 0.0;
    double stall_drop_percent = 0.0;
};

// Decode the 400-byte player-aircraft .pmb flight-parameter bundle into native
// values. This reads file data directly and does not retain guest addresses.
bool decode_aircraft_flight_rules(std::span<const std::uint8_t> bytes,
                                  AircraftFlightRules& rules,
                                  std::string& error);

bool load_aircraft_flight_rules(const std::string& path,
                                AircraftFlightRules& rules,
                                std::string& error);

} // namespace ac5
