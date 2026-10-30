#pragma once

#include <array>

namespace ac5 {

// Aircraft values recovered from the speed block and supplied by a native
// ruleset loader. Speeds are km/h; gains and climb penalty are percentages.
struct FlightSpeedRules {
    double spool_up_gain = 0.0;
    double spool_down_gain = 0.0;
    double climb_penalty_percent = 0.0;
    double inverted_sink_mps = 0.0;
    double cruise_kmh = 0.0;
    double maximum_kmh = 0.0;
    double minimum_kmh = 0.0;
    double stall_kmh = 0.0;
    double full_control_kmh = 0.0;
    double dive_limit_kmh = 0.0;
    double dive_extra_kmh = 0.0;
};

struct FlightSpeedInput {
    double altitude_units = 0.0;
    double pitch_radians = 0.0;
    double throttle_lever = 0.0;
    double current_speed_units_per_tick = 0.0;
    int ticks_per_second = 60;
    bool reduced_speed_mode = false;
};

struct FlightSpeedOutput {
    bool valid = false;
    double speed_units_per_tick = 0.0;
    double target_units_per_tick = 0.0;
    double stall_factor = 0.0;
    double stall_speed_units_per_tick = 0.0;
    double full_control_speed_units_per_tick = 0.0;
};

// Airborne scalar speed law recovered from generated routine 0x0016d408.
// Ground-contact acceleration, attitude integration, and world displacement
// are separate native boundaries.
[[nodiscard]] FlightSpeedOutput advance_airborne_speed(
    const FlightSpeedRules& rules, const FlightSpeedInput& input) noexcept;

struct FlightAxisRules {
    std::array<double, 8> breakpoints_kmh{};
    std::array<double, 8> rates_degrees_per_second{};
    double low_input_gain_percent = 0.0;
    double full_input_gain_percent = 0.0;
};

struct FlightAxisInput {
    double stick = 0.0;
    double speed_units_per_tick = 0.0;
    double stall_speed_units_per_tick = 0.0;
    double full_control_speed_units_per_tick = 0.0;
    double current_rate_radians_per_tick = 0.0;
    int ticks_per_second = 60;
};

struct FlightAxisOutput {
    bool valid = false;
    double target_rate_radians_per_tick = 0.0;
    double rate_radians_per_tick = 0.0;
    double rate_degrees_per_second = 0.0;
};

// Port of the rate lookup and slew helper at generated routine 0x0016b828.
[[nodiscard]] FlightAxisOutput advance_flight_axis(
    const FlightAxisRules& rules, const FlightAxisInput& input) noexcept;

} // namespace ac5
