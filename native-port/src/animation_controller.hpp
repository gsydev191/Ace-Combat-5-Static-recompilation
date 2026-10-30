#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace ac5 {

inline constexpr std::size_t aircraft_animation_channel_count = 64;
using AircraftAnimationChannels =
    std::array<float, aircraft_animation_channel_count>;

struct AircraftAnimationRules {
    // Subfile 6: twenty little-endian float32 values. The consumed indices
    // below retain the order used by the recovered per-aircraft controller.
    std::array<float, 20> raw{};
    bool available = false;
};

struct AircraftAnimationInput {
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    float slip = 0.0f;
    float controller_override = 0.0f;
    float throttle_lever = 0.0f;
    double speed_kmh = 0.0;
    double speed_units_per_tick = 0.0;
    bool gear_down = false;
    std::uint32_t model_flags = 0;
    std::uint32_t controller_flags = 0;
};

struct AircraftGearController {
    // 0 up, 1 extending, 2 down, 3 retracting.
    std::uint8_t state = 0;
    std::uint32_t capabilities = 0;
    [[nodiscard]] bool visible() const noexcept { return state != 0; }
};

[[nodiscard]] bool decode_aircraft_animation_rules(
    std::span<const std::uint8_t> bytes, AircraftAnimationRules& rules,
    std::string& error);
[[nodiscard]] bool load_aircraft_animation_rules(
    const std::string& path, AircraftAnimationRules& rules, std::string& error);
[[nodiscard]] AircraftAnimationChannels aircraft_animation_targets(
    const AircraftAnimationRules& rules, const AircraftAnimationInput& input) noexcept;
void advance_aircraft_animation_channels(AircraftAnimationChannels& channels,
                                         const AircraftAnimationChannels& targets) noexcept;
void advance_aircraft_animation_tick(AircraftAnimationChannels& channels,
                                     const AircraftAnimationRules& rules,
                                     const AircraftAnimationInput& input) noexcept;
void request_aircraft_gear(AircraftGearController& gear, bool down) noexcept;
void advance_aircraft_retail_gear(AircraftGearController& gear,
                                  AircraftAnimationChannels& channels) noexcept;

} // namespace ac5
