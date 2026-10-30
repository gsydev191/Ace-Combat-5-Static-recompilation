#pragma once

#include "flight_model.hpp"
#include "weapon_controller.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace ac5 {

inline constexpr std::size_t projectile_capacity = 128;
// Native presentation cap keeps uncollided shots from occupying the pool forever.
inline constexpr double projectile_lifetime_seconds = 12.0;

struct AircraftProjectile {
    AircraftShotKind kind = AircraftShotKind::Gun;
    std::array<double, 3> position{};
    std::array<double, 3> velocity{};
    std::array<double, 3> guidance_target{};
    double turn_rate_radians_per_second = 0.0;
    double gravity_world_units_per_second_squared = 0.0;
    double age_seconds = 0.0;
    bool active = false;
    bool has_guidance_target = false;
};

struct AircraftProjectileSystem {
    std::array<AircraftProjectile, projectile_capacity> projectiles{};
    std::size_t next_slot = 0;
};

struct ProjectileTargetSphere {
    std::uint64_t id = 0;
    std::array<double, 3> position{};
    double radius = 0.0;
};

struct AircraftProjectileImpact {
    std::size_t projectile_slot = 0;
    std::uint64_t target_id = 0;
    AircraftShotKind kind = AircraftShotKind::Gun;
    std::array<double, 3> position{};
};

struct AircraftProjectileImpacts {
    std::array<AircraftProjectileImpact, projectile_capacity> items{};
    std::size_t count = 0;
};

// Launches a gun round or standard missile from the aircraft origin. Special
// weapon projectiles are rejected until their own launch rules are decoded.
// An optional target is accepted for standard missiles only and must be inside
// the decoded seeker cone; missile turn rate then bounds guidance. Acquisition
// and moving-target updates remain responsibilities of a higher-level system.
[[nodiscard]] bool launch_aircraft_projectile(
    const AircraftWeaponRules& rules, const AircraftWeaponShot& shot,
    const AircraftFlightState& aircraft,
    const std::array<double, 3>& aircraft_velocity_units_per_second,
    AircraftProjectileSystem& projectiles,
    std::optional<std::array<double, 3>> guidance_target = std::nullopt) noexcept;

// Replace the point followed by an already-guided standard missile. This lets
// a higher-level target system refresh a moving target without owning the
// projectile's steering implementation.
[[nodiscard]] bool update_aircraft_projectile_target(
    AircraftProjectile& projectile,
    const std::array<double, 3>& target_position) noexcept;

void advance_aircraft_projectiles(AircraftProjectileSystem& projectiles,
                                  double elapsed_seconds) noexcept;

// Advance projectiles and detect swept hits against caller-owned spherical
// target bounds. On invalid target or active-projectile state, leave both
// projectile state and impacts intact. This reports collision events only;
// target damage rules are caller-owned.
[[nodiscard]] bool advance_aircraft_projectiles(
    AircraftProjectileSystem& projectiles, double elapsed_seconds,
    std::span<const ProjectileTargetSphere> targets,
    AircraftProjectileImpacts& impacts) noexcept;

} // namespace ac5
