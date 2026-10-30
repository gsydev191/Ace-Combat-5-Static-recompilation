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
    float throttle_lever = 0.0f;
    double speed_kmh = 0.0;
    bool gear_down = false;
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

} // namespace ac5
