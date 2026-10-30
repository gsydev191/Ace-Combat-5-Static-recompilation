#pragma once

#include "mission_units.hpp"

#include <array>

namespace ac5 {

struct Ac5MissionSteeringState {
    std::array<double, 4> position{};
    // Pitch, yaw, roll in radians.
    std::array<double, 3> attitude{};
    std::array<double, 4> velocity{};
    std::array<double, 4> order_start_position{};
    std::array<double, 3> order_start_attitude{};
    double speed_world_units_per_tick = 0.0;
    std::uint16_t flags = 0;
    std::uint8_t waypoint_parameter_3c = 0;
    std::uint8_t waypoint_parameter_3d = 0;
    std::size_t path_segment = 0;
    bool waypoint_complete = false;
    std::uint8_t active_mode = 0xff;
    bool waypoint_order_started = false;
};

struct Ac5MissionSteeringInput {
    double ticks_per_second = 60.0;
    // Target includes model altitude and terrain adjustments, when applicable.
    std::array<double, 4> target_position{};
    bool reference_position_available = false;
    std::array<double, 4> reference_position{};
};

// Initialize mode-dependent speed and velocity state when a waypoint begins.
[[nodiscard]] bool begin_ac5_mission_steering_order(
    const Ac5MissionOrder& order, double ticks_per_second,
    Ac5MissionSteeringState& state) noexcept;

// Apply the recovered waypoint modes 0-8. Returns false for invalid or
// unsupported input; completion is reported in state.waypoint_complete.
[[nodiscard]] bool advance_ac5_mission_steering(
    const Ac5MissionOrder& order, const Ac5MissionSteeringInput& input,
    Ac5MissionSteeringState& state) noexcept;

} // namespace ac5
