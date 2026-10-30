#include "weapon_controller.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>

namespace ac5 {
namespace {

bool read_u16(std::span<const std::uint8_t> bytes, std::size_t offset,
              std::uint16_t& value) noexcept {
    if (offset > bytes.size() || bytes.size() - offset < 2) return false;
    value = static_cast<std::uint16_t>(bytes[offset]) |
            static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[offset + 1]) << 8);
    return true;
}

bool read_f32(std::span<const std::uint8_t> bytes, std::size_t offset,
              float& value) noexcept {
    if (offset > bytes.size() || bytes.size() - offset < 4) return false;
    const std::uint32_t bits = static_cast<std::uint32_t>(bytes[offset]) |
        (static_cast<std::uint32_t>(bytes[offset + 1]) << 8) |
        (static_cast<std::uint32_t>(bytes[offset + 2]) << 16) |
        (static_cast<std::uint32_t>(bytes[offset + 3]) << 24);
    value = std::bit_cast<float>(bits);
    return std::isfinite(value);
}

std::uint32_t ticks_for_seconds(float seconds) noexcept {
    if (!(seconds > 0.0f)) return 0;
    const double ticks = std::ceil(static_cast<double>(seconds) * 60.0);
    return ticks >= static_cast<double>(std::numeric_limits<std::uint32_t>::max())
        ? std::numeric_limits<std::uint32_t>::max()
        : static_cast<std::uint32_t>(ticks);
}

template <std::size_t N>
void reduce_cooldowns(std::array<std::uint32_t, N>& cooldowns) noexcept {
    for (std::uint32_t& cooldown : cooldowns)
        if (cooldown != 0) --cooldown;
}

template <std::size_t N>
int find_ready_pylon(std::uint8_t mask, std::uint8_t first,
                     const std::array<std::uint32_t, N>& cooldowns) noexcept {
    for (std::size_t offset = 0; offset < N; ++offset) {
        const std::size_t pylon = (static_cast<std::size_t>(first) + offset) % N;
        if ((mask & (1u << pylon)) != 0 && cooldowns[pylon] == 0)
            return static_cast<int>(pylon);
    }
    return -1;
}

void push_shot(AircraftWeaponTick& output, AircraftShotKind kind,
               std::uint8_t pylon) noexcept {
    if (output.shot_count < output.shots.size())
        output.shots[output.shot_count++] = {kind, pylon};
}

bool valid_weapon_rules(const AircraftWeaponRules& rules) noexcept {
    return std::isfinite(rules.gun_interval_seconds) &&
        std::isfinite(rules.gun_mount_pitch_degrees) &&
        std::isfinite(rules.gun_spread_degrees) &&
        std::isfinite(rules.gun_gravity_mps2) &&
        std::isfinite(rules.standard_reload_seconds) &&
        std::isfinite(rules.missile_range2_meters) &&
        std::isfinite(rules.missile_gravity_mps2) &&
        std::isfinite(rules.missile_turn_rate_degrees_per_second) &&
        std::isfinite(rules.special_reload_seconds) &&
        rules.gun_interval_seconds >= 0.0f &&
        rules.gun_mount_pitch_degrees >= -180.0f &&
        rules.gun_mount_pitch_degrees <= 180.0f &&
        rules.gun_spread_degrees >= 0.0f && rules.gun_gravity_mps2 >= 0.0f &&
        rules.standard_reload_seconds >= 0.0f &&
        rules.missile_range2_meters >= 0.0f &&
        rules.missile_gravity_mps2 >= 0.0f &&
        rules.missile_turn_rate_degrees_per_second >= 0.0f &&
        rules.special_reload_seconds >= 0.0f &&
        (rules.standard_pylon_mask & ~0x03u) == 0 &&
        (rules.special_pylon_mask & ~0x0fu) == 0;
}

bool valid_weapon_state(const AircraftWeaponRules& rules,
                        const AircraftWeaponState& state) noexcept {
    const bool valid_slot = state.selected == AircraftWeaponSlot::Standard ||
                            state.selected == AircraftWeaponSlot::Special;
    const bool valid_status =
        state.status == AircraftWeaponStatus::StandardReady ||
        state.status == AircraftWeaponStatus::SpecialReady ||
        state.status == AircraftWeaponStatus::StandardEmpty ||
        state.status == AircraftWeaponStatus::SpecialEmpty ||
        state.status == AircraftWeaponStatus::GunEmpty;
    const bool rules_have_unlimited_gun =
        rules.gun_ammunition == unlimited_ammunition_sentinel;
    const bool valid_gun_ammunition =
        state.unlimited_gun_ammunition == rules_have_unlimited_gun &&
        (state.unlimited_gun_ammunition
            ? state.gun_ammunition == unlimited_ammunition_sentinel
            : state.gun_ammunition <= rules.gun_ammunition);
    return valid_slot && valid_status &&
        state.next_standard_pylon < 2 && state.next_special_pylon < 4 &&
        valid_gun_ammunition &&
        state.missile_ammunition <= rules.missile_ammunition &&
        state.special_ammunition <= rules.special_ammunition;
}

} // namespace

bool decode_aircraft_weapon_rules(
    std::span<const std::uint8_t> loadout_slot,
    std::span<const std::uint8_t> gun_slot,
    std::span<const std::uint8_t> missile_slot,
    AircraftWeaponRules& rules, std::string& error) {
    // The .wxb loadout record is 0x30 bytes in retail data; fields consumed
    // below end at 0x21. Gun and missile .wpb records are 0x50 bytes.
    if (loadout_slot.size() < 0x22 || gun_slot.size() < 0x50 ||
        missile_slot.size() < 0x50) {
        error = "aircraft weapon slots are truncated";
        return false;
    }

    AircraftWeaponRules decoded;
    if (!read_f32(loadout_slot, 0x04, decoded.gun_mount_pitch_degrees) ||
        !read_u16(loadout_slot, 0x1c, decoded.gun_ammunition) ||
        !read_f32(gun_slot, 0x34, decoded.gun_interval_seconds) ||
        !read_f32(gun_slot, 0x40, decoded.gun_spread_degrees) ||
        !read_f32(gun_slot, 0x44, decoded.gun_gravity_mps2) ||
        !read_u16(gun_slot, 0x48, decoded.gun_muzzle_speed_kmh) ||
        !read_f32(missile_slot, 0x24, decoded.missile_range2_meters) ||
        !read_f32(missile_slot, 0x34, decoded.standard_reload_seconds) ||
        !read_f32(missile_slot, 0x3c,
                  decoded.missile_turn_rate_degrees_per_second) ||
        !read_f32(missile_slot, 0x44, decoded.missile_gravity_mps2) ||
        !read_u16(missile_slot, 0x48, decoded.missile_launch_speed_kmh)) {
        error = "aircraft weapon slots contain truncated or non-finite fields";
        return false;
    }
    decoded.missile_ammunition = loadout_slot[0x1e];
    decoded.special_ammunition = loadout_slot[0x1f];
    decoded.special_weapon_id = loadout_slot[0x21];
    decoded.special_reload_seconds = decoded.standard_reload_seconds;
    decoded.seeker_half_angle_degrees = missile_slot[0x4c];

    if (!(decoded.gun_interval_seconds > 0.0f) ||
        decoded.gun_mount_pitch_degrees < -180.0f ||
        decoded.gun_mount_pitch_degrees > 180.0f ||
        !(decoded.missile_range2_meters > 0.0f) ||
        decoded.gun_spread_degrees < 0.0f ||
        decoded.gun_gravity_mps2 < 0.0f ||
        decoded.standard_reload_seconds < 0.0f ||
        decoded.missile_gravity_mps2 < 0.0f ||
        decoded.missile_turn_rate_degrees_per_second < 0.0f ||
        decoded.seeker_half_angle_degrees > 180) {
        error = "aircraft weapon slots contain invalid ammunition or weapon parameters";
        return false;
    }
    rules = decoded;
    error.clear();
    return true;
}

bool initialize_aircraft_weapons(const AircraftWeaponRules& rules,
                                 AircraftWeaponState& state) noexcept {
    if (!valid_weapon_rules(rules)) return false;

    AircraftWeaponState initialized{};
    initialized.gun_ammunition = rules.gun_ammunition;
    initialized.missile_ammunition = rules.missile_ammunition;
    initialized.special_ammunition = rules.special_ammunition;
    initialized.unlimited_gun_ammunition =
        rules.gun_ammunition == unlimited_ammunition_sentinel;
    initialized.status = rules.missile_ammunition != 0
        ? AircraftWeaponStatus::StandardReady
        : AircraftWeaponStatus::StandardEmpty;
    state = initialized;
    return true;
}

AircraftWeaponTick advance_aircraft_weapons(
    const AircraftWeaponRules& rules, const AircraftWeaponInput& input,
    AircraftWeaponState& state) noexcept {
    AircraftWeaponTick output{};
    if (!valid_weapon_rules(rules) || !valid_weapon_state(rules, state))
        return output;
    if (state.gun_cooldown_ticks != 0) --state.gun_cooldown_ticks;
    reduce_cooldowns(state.standard_pylon_cooldowns);
    reduce_cooldowns(state.special_pylon_cooldowns);

    if (input.switch_weapon_pressed) {
        if (state.selected == AircraftWeaponSlot::Special) {
            state.selected = AircraftWeaponSlot::Standard;
            state.status = state.missile_ammunition != 0
                ? AircraftWeaponStatus::StandardReady
                : AircraftWeaponStatus::StandardEmpty;
            output.switched_weapon = true;
        } else if (state.special_ammunition != 0) {
            state.selected = AircraftWeaponSlot::Special;
            state.status = AircraftWeaponStatus::SpecialReady;
            output.switched_weapon = true;
        } else {
            state.status = AircraftWeaponStatus::SpecialEmpty;
        }
    }

    if (input.fire_selected_pressed &&
        state.selected == AircraftWeaponSlot::Standard &&
        state.missile_ammunition != 0) {
        const int pylon = find_ready_pylon(rules.standard_pylon_mask,
            state.next_standard_pylon, state.standard_pylon_cooldowns);
        if (pylon >= 0) {
            push_shot(output, AircraftShotKind::StandardMissile,
                      static_cast<std::uint8_t>(pylon));
            --state.missile_ammunition;
            state.standard_pylon_cooldowns[static_cast<std::size_t>(pylon)] =
                ticks_for_seconds(rules.standard_reload_seconds);
            state.next_standard_pylon = static_cast<std::uint8_t>(1 - pylon);
            state.status = state.missile_ammunition != 0
                ? AircraftWeaponStatus::StandardReady
                : AircraftWeaponStatus::StandardEmpty;
        }
    } else if (input.fire_selected_pressed &&
               state.selected == AircraftWeaponSlot::Special &&
               state.special_ammunition != 0) {
        if (rules.special_weapon_id == 8 || rules.special_weapon_id == 0x13) {
            // Retail gives these special weapons a distinct salvo/burst path;
            // do not pretend the ordinary one-pylon sequence reproduces it.
            output.special_sequence_unavailable = true;
        } else {
            const int pylon = find_ready_pylon(rules.special_pylon_mask,
                state.next_special_pylon, state.special_pylon_cooldowns);
            if (pylon >= 0) {
                push_shot(output, AircraftShotKind::Special,
                          static_cast<std::uint8_t>(pylon));
                --state.special_ammunition;
                state.special_pylon_cooldowns[static_cast<std::size_t>(pylon)] =
                    ticks_for_seconds(rules.special_reload_seconds);
                state.next_special_pylon = static_cast<std::uint8_t>(
                    (static_cast<unsigned>(pylon) + 1u) % 4u);
                if (state.special_ammunition == 0)
                    state.status = AircraftWeaponStatus::SpecialEmpty;
            }
        }
    }

    if (input.fire_gun_held && state.gun_ammunition != 0 &&
        state.gun_cooldown_ticks == 0) {
        push_shot(output, AircraftShotKind::Gun, 0);
        if (!state.unlimited_gun_ammunition) --state.gun_ammunition;
        state.gun_cooldown_ticks = std::max<std::uint32_t>(
            1, ticks_for_seconds(rules.gun_interval_seconds));
        if (state.gun_ammunition == 0)
            state.status = AircraftWeaponStatus::GunEmpty;
    }
    return output;
}

} // namespace ac5
