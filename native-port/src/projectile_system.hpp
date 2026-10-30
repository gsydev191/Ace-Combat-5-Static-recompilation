#pragma once

#include "flight_model.hpp"
#include "weapon_controller.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace ac5 {

inline constexpr std::size_t projectile_capacity = 128;
// Native presentation cap keeps uncollided shots from occupying the pool forever.
inline constexpr double projectile_lifetime_seconds = 12.0;

struct AircraftProjectile {
    AircraftShotKind kind = AircraftShotKind::Gun;
    std::array<double, 3> position{};
    std::array<double, 3> velocity{};
    double gravity_world_units_per_second_squared = 0.0;
    double age_seconds = 0.0;
    bool active = false;
};

struct AircraftProjectileSystem {
    std::array<AircraftProjectile, projectile_capacity> projectiles{};
    std::size_t next_slot = 0;
};

// Launches from the aircraft origin. Weapon and pylon-specific offsets,
// homing, collision, and damage are intentionally left to later systems.
[[nodiscard]] bool launch_aircraft_projectile(
    const AircraftWeaponRules& rules, const AircraftWeaponShot& shot,
    const AircraftFlightState& aircraft,
    const std::array<double, 3>& aircraft_velocity_units_per_second,
    AircraftProjectileSystem& projectiles) noexcept;

void advance_aircraft_projectiles(AircraftProjectileSystem& projectiles,
                                  double elapsed_seconds) noexcept;

} // namespace ac5
