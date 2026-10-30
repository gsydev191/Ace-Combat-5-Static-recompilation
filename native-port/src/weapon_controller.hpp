#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace ac5 {

inline constexpr std::uint16_t unlimited_ammunition_sentinel = 10000;

struct AircraftWeaponRules {
    std::uint16_t gun_ammunition = 0;
    std::uint8_t missile_ammunition = 0;
    std::uint8_t special_ammunition = 0;
    std::uint8_t special_weapon_id = 0;
    std::uint8_t standard_pylon_mask = 0x03;
    std::uint8_t special_pylon_mask = 0x0f;
    float gun_interval_seconds = 0.15f;
    float gun_spread_degrees = 0.0f;
    float gun_gravity_mps2 = 9.8f;
    std::uint16_t gun_muzzle_speed_kmh = 0;
    float standard_reload_seconds = 4.0f;
    float missile_gravity_mps2 = 9.8f;
    float missile_turn_rate_degrees_per_second = 0.0f;
    std::uint16_t missile_launch_speed_kmh = 0;
    std::uint8_t seeker_half_angle_degrees = 0;
    float special_reload_seconds = 4.0f;
};

enum class AircraftWeaponSlot : std::uint8_t { Standard, Special };
enum class AircraftWeaponStatus : std::uint8_t {
    StandardReady = 1,
    SpecialReady = 2,
    StandardEmpty = 3,
    SpecialEmpty = 4,
    GunEmpty = 5,
};
enum class AircraftShotKind : std::uint8_t { Gun, StandardMissile, Special };

struct AircraftWeaponState {
    AircraftWeaponSlot selected = AircraftWeaponSlot::Standard;
    AircraftWeaponStatus status = AircraftWeaponStatus::StandardEmpty;
    std::uint16_t gun_ammunition = 0;
    std::uint8_t missile_ammunition = 0;
    std::uint8_t special_ammunition = 0;
    std::uint8_t next_standard_pylon = 0;
    std::uint8_t next_special_pylon = 0;
    std::uint32_t gun_cooldown_ticks = 0;
    std::array<std::uint32_t, 2> standard_pylon_cooldowns{};
    std::array<std::uint32_t, 4> special_pylon_cooldowns{};
    bool unlimited_gun_ammunition = false;
};

struct AircraftWeaponInput {
    bool switch_weapon_pressed = false;
    bool fire_selected_pressed = false;
    bool fire_gun_held = false;
};

struct AircraftWeaponShot {
    AircraftShotKind kind = AircraftShotKind::Gun;
    std::uint8_t pylon = 0;
};

struct AircraftWeaponTick {
    std::array<AircraftWeaponShot, 4> shots{};
    std::size_t shot_count = 0;
    bool switched_weapon = false;
    bool special_sequence_unavailable = false;
};

// Decode the in-mission aircraft bundle slots 7 (loadout), 8 (gun), and 9
// (missile). Spans are complete slot payloads, including their 0x10-byte
// headers and relative-pointer fields.
[[nodiscard]] bool decode_aircraft_weapon_rules(
    std::span<const std::uint8_t> loadout_slot,
    std::span<const std::uint8_t> gun_slot,
    std::span<const std::uint8_t> missile_slot,
    AircraftWeaponRules& rules, std::string& error);

[[nodiscard]] bool initialize_aircraft_weapons(
    const AircraftWeaponRules& rules, AircraftWeaponState& state) noexcept;
// Invalid rules or state produce an empty tick and leave state unchanged.
[[nodiscard]] AircraftWeaponTick advance_aircraft_weapons(
    const AircraftWeaponRules& rules, const AircraftWeaponInput& input,
    AircraftWeaponState& state) noexcept;

} // namespace ac5
