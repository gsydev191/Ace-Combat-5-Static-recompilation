#pragma once

#include "flight_rules_loader.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <optional>

namespace ac5 {

// Native flight/world scale recovered from the speed and gravity conversions.
inline constexpr double aircraft_world_units_per_meter = 5.12;
inline constexpr double aircraft_world_units_per_kmh_per_second = 1.4222219;

struct AircraftFlightFlags {
    std::uint8_t contact = 0;
    std::uint8_t mode = 0;
};

// Spatial ground-contact latch mirrored from the generated runtime. A brief
// loss of terrain contact does not immediately switch the aircraft to flight.
struct AircraftGroundContactState {
    std::uint8_t flags = 0;
    int decay_ticks = 0;
};

using AircraftGroundMarkers = std::array<std::array<double, 3>, 3>;

struct AircraftGroundTraceModel {
    // The callback samples marker positions at a gear animation channel, or
    // current channels when the argument is empty. Marker order is I, K, J.
    std::uint8_t gear_state = 0;
    std::function<std::optional<AircraftGroundMarkers>(std::optional<double>)> sample;
};

struct AircraftGroundSweepHit {
    int result = 0;
    double fraction = 0.0;
    double normal_y = 0.0;
    int kind = 0;
};

struct AircraftGroundVerticalHit {
    int result = 0;
    std::array<double, 3> point{};
};

struct AircraftGroundTraceTerrain {
    std::function<double(const std::array<double, 3>&)> height;
    std::function<AircraftGroundSweepHit(const std::array<double, 3>& end,
                                         const std::array<double, 3>& start)> sweep;
    std::function<AircraftGroundVerticalHit(const std::array<double, 3>& upper,
                                            const std::array<double, 3>& lower)> vertical;
};

struct AircraftGroundTraceResult {
    bool hit = false;
    int anchor = -1; // I=0, K=1, J=2
    std::array<double, 3> fractions{};
};

struct AircraftFlightState;

[[nodiscard]] bool update_aircraft_ground_contact(
    AircraftGroundContactState& state, bool hit, int ticks_per_second,
    bool suppress_effect = false) noexcept;

[[nodiscard]] std::optional<std::array<double, 3>> aircraft_ground_marker_attitude(
    const AircraftGroundMarkers& markers) noexcept;
[[nodiscard]] std::optional<AircraftGroundTraceResult> normal_aircraft_ground_trace(
    AircraftFlightState& state, const AircraftGroundTraceModel& model,
    const AircraftGroundTraceTerrain* terrain);

struct AircraftFlightState {
    AircraftFlightFlags flags{};
    std::array<double, 3> position{0.0, 3000.0, 0.0};
    std::array<double, 3> euler{}; // body pitch, yaw, roll in radians
    std::array<double, 3> path_euler{};
    std::array<double, 3> rates{}; // pitch, yaw, roll in radians per tick
    double speed_units_per_tick = 0.0;
    double stall_factor = 0.0;
    double path_lag = 1.0;
    double approach_pitch = 0.0;
    std::uint8_t carrier_hold_ticks = 0;
};

struct AircraftFlightInput {
    double pitch = 0.0;
    double roll = 0.0;
    double yaw = 0.0;
    double throttle_lever = 0.0;
    int ticks_per_second = 60;
    bool suppress_inverted_sink = false;
    bool reset_contact_speed = false;
    double contact_acceleration_scale = 1.0;
    double contact_acceleration_bias = 0.0;
    // Spatial on-ground state supplied by a native world query. When absent,
    // the recovered contact bit remains the source of the ground-speed mode.
    std::optional<bool> ground_contact_override;
};

// Set the recovered ground/contact modes while preserving unrelated flag bits.
void set_aircraft_flight_mode(AircraftFlightFlags& flags,
                              std::uint8_t command) noexcept;
void set_aircraft_approach_control(AircraftFlightFlags& flags,
                                   bool enabled) noexcept;
void advance_aircraft_flight_flags(AircraftFlightFlags& flags) noexcept;

// World-space forward direction used by the native flight integrator, with
// zero attitude pointing along negative Z.
[[nodiscard]] std::array<double, 3> aircraft_forward_direction(
    const std::array<double, 3>& euler) noexcept;

// Body axes used by the native aircraft model and the six-point collision
// test. Each row is one local body axis expressed in world coordinates.
[[nodiscard]] std::array<std::array<double, 3>, 3> aircraft_body_axes(
    const std::array<double, 3>& euler) noexcept;

// Advance one native flight tick. Contact bit 0 selects the recovered scalar
// contact-speed law; terrain collision and ground-marker alignment are separate.
[[nodiscard]] bool advance_aircraft_flight_tick(
    const AircraftFlightRules& rules, const AircraftFlightInput& input,
    AircraftFlightState& state) noexcept;

// Port the carrier launch hold wrapper, including its temporary throttle
// suppression, contact acceleration tuning, and delayed flag transitions.
[[nodiscard]] bool advance_aircraft_carrier_tick(
    const AircraftFlightRules& rules, const AircraftFlightInput& input,
    AircraftFlightState& state) noexcept;

} // namespace ac5
