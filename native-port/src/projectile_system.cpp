#include "projectile_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace ac5 {
namespace {

using Vector3 = std::array<double, 3>;

double dot(const Vector3& a, const Vector3& b) noexcept {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vector3 cross(const Vector3& a, const Vector3& b) noexcept {
    return {a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}

bool normalize(Vector3& value) noexcept {
    const double length = std::sqrt(dot(value, value));
    if (!std::isfinite(length) || length <= 1.0e-9) return false;
    for (double& component : value) component /= length;
    return true;
}

bool valid_projectile(const AircraftProjectile& projectile) noexcept {
    if (projectile.kind != AircraftShotKind::Gun &&
        projectile.kind != AircraftShotKind::StandardMissile)
        return false;
    if (!std::all_of(projectile.position.begin(), projectile.position.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::all_of(projectile.velocity.begin(), projectile.velocity.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::isfinite(projectile.age_seconds) || projectile.age_seconds < 0.0 ||
        !std::isfinite(projectile.gravity_world_units_per_second_squared) ||
        projectile.gravity_world_units_per_second_squared < 0.0 ||
        !std::isfinite(projectile.turn_rate_radians_per_second) ||
        projectile.turn_rate_radians_per_second < 0.0)
        return false;
    if (projectile.has_guidance_target &&
        (projectile.kind != AircraftShotKind::StandardMissile ||
         !std::all_of(projectile.guidance_target.begin(),
                      projectile.guidance_target.end(),
                      [](double value) { return std::isfinite(value); })))
        return false;
    return true;
}

void turn_toward_target(AircraftProjectile& projectile, double maximum_turn) noexcept {
    Vector3 direction = projectile.velocity;
    Vector3 desired{projectile.guidance_target[0] - projectile.position[0],
                    projectile.guidance_target[1] - projectile.position[1],
                    projectile.guidance_target[2] - projectile.position[2]};
    if (!normalize(direction) || !normalize(desired)) return;
    const double cosine = std::clamp(dot(direction, desired), -1.0, 1.0);
    const double angle = std::acos(cosine);
    if (!std::isfinite(angle) || angle <= 1.0e-9) return;
    const double step = std::min(angle, maximum_turn);
    Vector3 axis = cross(direction, desired);
    if (!normalize(axis)) {
        if (cosine > 0.0) return;
        const Vector3 reference = std::abs(direction[1]) < 0.9
            ? Vector3{0.0, 1.0, 0.0} : Vector3{1.0, 0.0, 0.0};
        axis = cross(direction, reference);
        if (!normalize(axis)) return;
    }
    const Vector3 perpendicular = cross(axis, direction);
    const double c = std::cos(step), s = std::sin(step);
    const double speed = std::sqrt(dot(projectile.velocity, projectile.velocity));
    for (std::size_t i = 0; i < 3; ++i)
        projectile.velocity[i] = (direction[i] * c + perpendicular[i] * s) * speed;
}

} // namespace

bool launch_aircraft_projectile(const AircraftWeaponRules& rules,
                                const AircraftWeaponShot& shot,
                                const AircraftFlightState& aircraft,
                                const std::array<double, 3>& aircraft_velocity,
                                AircraftProjectileSystem& system,
                                std::optional<std::array<double, 3>> guidance_target) noexcept {
    if (shot.kind != AircraftShotKind::Gun &&
        shot.kind != AircraftShotKind::StandardMissile)
        return false;
    const bool gun = shot.kind == AircraftShotKind::Gun;
    const double launch_speed = static_cast<double>(gun
        ? rules.gun_muzzle_speed_kmh : rules.missile_launch_speed_kmh) *
        aircraft_world_units_per_kmh_per_second;
    const double gravity = static_cast<double>(gun
        ? rules.gun_gravity_mps2 : rules.missile_gravity_mps2) *
        aircraft_world_units_per_meter;
    const double turn_rate = static_cast<double>(rules.missile_turn_rate_degrees_per_second) *
                             (3.14159265358979323846 / 180.0);
    if (system.next_slot >= system.projectiles.size() ||
        !std::isfinite(launch_speed) || !std::isfinite(gravity) || gravity < 0.0 ||
        !std::isfinite(turn_rate) || turn_rate < 0.0 ||
        !std::all_of(aircraft.position.begin(), aircraft.position.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::all_of(aircraft.euler.begin(), aircraft.euler.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::all_of(aircraft_velocity.begin(), aircraft_velocity.end(),
                     [](double value) { return std::isfinite(value); }))
        return false;

    if (guidance_target) {
        if (shot.kind != AircraftShotKind::StandardMissile ||
            rules.seeker_half_angle_degrees > 180 ||
            !std::all_of(guidance_target->begin(), guidance_target->end(),
                [](double value) { return std::isfinite(value); })) return false;
        Vector3 target_direction{(*guidance_target)[0] - aircraft.position[0],
                                 (*guidance_target)[1] - aircraft.position[1],
                                 (*guidance_target)[2] - aircraft.position[2]};
        Vector3 forward = aircraft_forward_direction(aircraft.euler);
        if (!normalize(target_direction) || !normalize(forward)) return false;
        const double seeker_cosine = std::cos(
            static_cast<double>(rules.seeker_half_angle_degrees) *
            (3.14159265358979323846 / 180.0));
        if (dot(forward, target_direction) + 1.0e-9 < seeker_cosine) return false;
    }

    const std::array<double, 3> forward = aircraft_forward_direction(aircraft.euler);
    AircraftProjectile projectile;
    projectile.kind = shot.kind;
    projectile.position = aircraft.position;
    if (guidance_target) {
        projectile.guidance_target = *guidance_target;
        projectile.turn_rate_radians_per_second = turn_rate;
        projectile.has_guidance_target = true;
    }
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

bool update_aircraft_projectile_target(
    AircraftProjectile& projectile,
    const std::array<double, 3>& target_position) noexcept {
    if (!projectile.active || projectile.kind != AircraftShotKind::StandardMissile ||
        !projectile.has_guidance_target ||
        !std::all_of(target_position.begin(), target_position.end(),
                     [](double value) { return std::isfinite(value); }))
        return false;
    projectile.guidance_target = target_position;
    return true;
}

void advance_aircraft_projectiles(AircraftProjectileSystem& system,
                                  double elapsed_seconds) noexcept {
    AircraftProjectileImpacts ignored;
    (void)advance_aircraft_projectiles(system, elapsed_seconds,
                                       std::span<const ProjectileTargetSphere>{},
                                       ignored);
}

bool advance_aircraft_projectiles(
    AircraftProjectileSystem& system, double elapsed_seconds,
    std::span<const ProjectileTargetSphere> targets,
    AircraftProjectileImpacts& impacts) noexcept {
    if (!std::isfinite(elapsed_seconds) || elapsed_seconds < 0.0 ||
        system.next_slot >= system.projectiles.size()) return false;
    for (const AircraftProjectile& projectile : system.projectiles)
        if (projectile.active && !valid_projectile(projectile)) return false;
    for (const ProjectileTargetSphere& target : targets) {
        if (!std::isfinite(target.position[0]) ||
            !std::isfinite(target.position[1]) ||
            !std::isfinite(target.position[2]) ||
            !std::isfinite(target.radius) || target.radius <= 0.0)
            return false;
    }

    AircraftProjectileSystem advanced = system;
    AircraftProjectileImpacts detected{};
    if (elapsed_seconds == 0.0) {
        impacts = detected;
        return true;
    }
    constexpr double maximum_step = 1.0 / 60.0;
    // No projectile can remain active beyond its native presentation lifetime.
    // Simulate the complete useful interval instead of silently dropping time
    // for callers that advance more than a single short frame.
    double remaining = std::min(elapsed_seconds, projectile_lifetime_seconds);
    while (remaining > 0.0) {
        const double step = std::min(remaining, maximum_step);
        for (std::size_t slot = 0; slot < advanced.projectiles.size(); ++slot) {
            AircraftProjectile& projectile = advanced.projectiles[slot];
            if (!projectile.active) continue;
            if (!std::isfinite(projectile.age_seconds) ||
                projectile.age_seconds < 0.0) return false;
            const double lifetime_remaining =
                projectile_lifetime_seconds - projectile.age_seconds;
            if (lifetime_remaining <= 0.0) {
                projectile.active = false;
                continue;
            }
            const double projectile_step = std::min(step, lifetime_remaining);
            const std::array<double, 3> start = projectile.position;
            if (projectile.has_guidance_target &&
                projectile.turn_rate_radians_per_second > 0.0)
                turn_toward_target(projectile,
                    projectile.turn_rate_radians_per_second * projectile_step);
            projectile.velocity[1] -=
                projectile.gravity_world_units_per_second_squared * projectile_step;
            for (std::size_t axis = 0; axis < 3; ++axis)
                projectile.position[axis] += projectile.velocity[axis] * projectile_step;
            projectile.age_seconds += projectile_step;
            if (!std::all_of(projectile.position.begin(), projectile.position.end(),
                             [](double value) { return std::isfinite(value); })) {
                projectile.active = false;
                continue;
            }

            const Vector3 movement{projectile.position[0] - start[0],
                                   projectile.position[1] - start[1],
                                   projectile.position[2] - start[2]};
            const double movement_squared = dot(movement, movement);
            if (!std::isfinite(movement_squared)) return false;
            double earliest_time = std::numeric_limits<double>::infinity();
            std::uint64_t hit_target = 0;
            for (const ProjectileTargetSphere& target : targets) {
                const Vector3 offset{start[0] - target.position[0],
                                     start[1] - target.position[1],
                                     start[2] - target.position[2]};
                const double radius_squared = target.radius * target.radius;
                const double c = dot(offset, offset) - radius_squared;
                if (!std::isfinite(radius_squared) || !std::isfinite(c)) return false;
                double time = std::numeric_limits<double>::infinity();
                if (c <= 0.0) {
                    time = 0.0;
                } else if (movement_squared > 1.0e-18) {
                    const double projection = dot(offset, movement);
                    const double discriminant = projection * projection -
                                                movement_squared * c;
                    if (!std::isfinite(projection) || !std::isfinite(discriminant))
                        return false;
                    if (discriminant >= 0.0) {
                        const double candidate =
                            (-projection - std::sqrt(discriminant)) / movement_squared;
                        if (candidate >= 0.0 && candidate <= 1.0) time = candidate;
                    }
                }
                if (time < earliest_time) {
                    earliest_time = time;
                    hit_target = target.id;
                }
            }
            if (std::isfinite(earliest_time)) {
                for (std::size_t axis = 0; axis < 3; ++axis)
                    projectile.position[axis] =
                        start[axis] + movement[axis] * earliest_time;
                projectile.active = false;
                detected.items[detected.count++] = {
                    slot, hit_target, projectile.kind, projectile.position};
            } else if (projectile.age_seconds >= projectile_lifetime_seconds) {
                projectile.active = false;
            }
        }
        remaining -= step;
    }
    system = std::move(advanced);
    impacts = std::move(detected);
    return true;
}

} // namespace ac5
