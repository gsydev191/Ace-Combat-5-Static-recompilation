#include "projectile_system.hpp"

#include <algorithm>
#include <cmath>

namespace ac5 {

bool launch_aircraft_projectile(const AircraftWeaponRules& rules,
                                const AircraftWeaponShot& shot,
                                const AircraftFlightState& aircraft,
                                const std::array<double, 3>& aircraft_velocity,
                                AircraftProjectileSystem& system) noexcept {
    const bool gun = shot.kind == AircraftShotKind::Gun;
    const double launch_speed = static_cast<double>(gun
        ? rules.gun_muzzle_speed_kmh : rules.missile_launch_speed_kmh) *
        aircraft_world_units_per_kmh_per_second;
    const double gravity = static_cast<double>(gun
        ? rules.gun_gravity_mps2 : rules.missile_gravity_mps2) *
        aircraft_world_units_per_meter;
    if (system.next_slot >= system.projectiles.size() ||
        !std::isfinite(launch_speed) || !std::isfinite(gravity) || gravity < 0.0 ||
        !std::all_of(aircraft.position.begin(), aircraft.position.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::all_of(aircraft.euler.begin(), aircraft.euler.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::all_of(aircraft_velocity.begin(), aircraft_velocity.end(),
                     [](double value) { return std::isfinite(value); }))
        return false;

    const std::array<double, 3> forward = aircraft_forward_direction(aircraft.euler);
    AircraftProjectile projectile;
    projectile.kind = shot.kind;
    projectile.position = aircraft.position;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        projectile.velocity[axis] = aircraft_velocity[axis] +
            forward[axis] * launch_speed;
        if (!std::isfinite(projectile.velocity[axis])) return false;
    }
    projectile.gravity_world_units_per_second_squared = gravity;
    projectile.active = true;
    system.projectiles[system.next_slot] = projectile;
    system.next_slot = (system.next_slot + 1) % system.projectiles.size();
    return true;
}

void advance_aircraft_projectiles(AircraftProjectileSystem& system,
                                  double elapsed_seconds) noexcept {
    if (!std::isfinite(elapsed_seconds) || elapsed_seconds <= 0.0) return;
    constexpr double maximum_step = 1.0 / 60.0;
    double remaining = std::min(elapsed_seconds, 0.1);
    while (remaining > 0.0) {
        const double step = std::min(remaining, maximum_step);
        for (AircraftProjectile& projectile : system.projectiles) {
            if (!projectile.active) continue;
            projectile.velocity[1] -=
                projectile.gravity_world_units_per_second_squared * step;
            for (std::size_t axis = 0; axis < 3; ++axis)
                projectile.position[axis] += projectile.velocity[axis] * step;
            projectile.age_seconds += step;
            if (projectile.age_seconds >= projectile_lifetime_seconds ||
                !std::all_of(projectile.position.begin(), projectile.position.end(),
                             [](double value) { return std::isfinite(value); }))
                projectile.active = false;
        }
        remaining -= step;
    }
}

} // namespace ac5
