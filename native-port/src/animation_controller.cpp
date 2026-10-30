#include "animation_controller.hpp"

#include "utf8_path.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <fstream>
#include <new>
#include <stdexcept>
#include <vector>

namespace ac5 {
namespace {

bool read_u32(std::span<const std::uint8_t> bytes, std::size_t offset,
              std::uint32_t& value) noexcept {
    if (offset > bytes.size() || bytes.size() - offset < 4) return false;
    value = static_cast<std::uint32_t>(bytes[offset]) |
            (static_cast<std::uint32_t>(bytes[offset + 1]) << 8) |
            (static_cast<std::uint32_t>(bytes[offset + 2]) << 16) |
            (static_cast<std::uint32_t>(bytes[offset + 3]) << 24);
    return true;
}

double reverse_speed_curve(double speed, double full_below,
                           double none_above) noexcept {
    if (!(none_above > full_below)) return 0.0;
    if (speed <= full_below) return 1.0;
    if (speed >= none_above) return 0.0;
    return (none_above - speed) / (none_above - full_below);
}

} // namespace

bool decode_aircraft_animation_rules(std::span<const std::uint8_t> bytes,
                                     AircraftAnimationRules& rules,
                                     std::string& error) {
    if (bytes.size() != 20 * sizeof(float)) {
        error = "aircraft animation config must contain exactly 20 float32 values";
        return false;
    }
    AircraftAnimationRules decoded;
    for (std::size_t i = 0; i < decoded.raw.size(); ++i) {
        std::uint32_t bits = 0;
        if (!read_u32(bytes, i * sizeof(float), bits)) {
            error = "aircraft animation config is truncated";
            return false;
        }
        decoded.raw[i] = std::bit_cast<float>(bits);
        if (!std::isfinite(decoded.raw[i])) {
            error = "aircraft animation config contains a non-finite value";
            return false;
        }
    }
    const auto& f = decoded.raw;
    if (f[3] < f[2] || f[4] < 0.0f || f[4] > 1.0f ||
        f[6] < f[5] || f[12] < f[11] || f[14] < f[13] || f[16] < f[15]) {
        error = "aircraft animation config has invalid curve bounds";
        return false;
    }
    decoded.available = true;
    rules = decoded;
    return true;
}

bool load_aircraft_animation_rules(const std::string& path,
                                   AircraftAnimationRules& rules,
                                   std::string& error) {
    std::ifstream input(path_from_utf8(path), std::ios::binary);
    if (!input) {
        error = "cannot open aircraft animation config: " + path;
        return false;
    }
    std::vector<std::uint8_t> bytes;
    try {
        bytes.resize(20 * sizeof(float));
    } catch (const std::bad_alloc&) {
        error = "aircraft animation config allocation failed: " + path;
        return false;
    } catch (const std::length_error&) {
        error = "aircraft animation config exceeds the platform limit: " + path;
        return false;
    }
    if (!input.read(reinterpret_cast<char*>(bytes.data()),
                    static_cast<std::streamsize>(bytes.size())) ||
        input.peek() != std::char_traits<char>::eof()) {
        error = "aircraft animation config is truncated or has trailing data: " + path;
        return false;
    }
    return decode_aircraft_animation_rules(bytes, rules, error);
}

AircraftAnimationChannels aircraft_animation_targets(
    const AircraftAnimationRules& rules, const AircraftAnimationInput& input) noexcept {
    AircraftAnimationChannels channels{};
    channels.fill(50.0f);
    const double speed = std::max(0.0, input.speed_kmh);
    const auto& config = rules.raw;
    const double authority = rules.available && config[3] > config[2]
        ? (speed <= config[2] ? 1.0
           : std::max(static_cast<double>(config[4]),
               1.0 - (1.0 - config[4]) * (speed - config[2]) /
                         (config[3] - config[2])))
        : 1.0;
    const auto flap_curve = [speed](double full_below, double none_above) {
        if (!(none_above > full_below)) return 0.0;
        if (speed <= full_below) return 1.0;
        if (speed >= none_above) return 0.0;
        return (none_above - speed) / (none_above - full_below);
    };
    const double flap = rules.available ? flap_curve(config[5], config[6]) : 0.0;
    const double secondary_flap = rules.available
        ? flap_curve(config[11], config[12]) : 0.0;
    const double channel16 = rules.available
        ? reverse_speed_curve(speed, config[9], config[10]) : 0.0;
    const double channel20 = rules.available
        ? reverse_speed_curve(speed, config[15], config[16]) : 0.0;
    const double sweep = rules.available && config[14] > config[13]
        ? std::clamp((speed - config[13]) / (config[14] - config[13]), 0.0, 1.0)
        : 0.0;
    const double lever = std::clamp(static_cast<double>(input.throttle_lever), -1.0, 1.0);
    const double brake = std::clamp((-0.6 - lever) / 0.4, 0.0, 1.0);
    const double dry = lever >= 0.9 ? 1.0 : lever >= -0.6 ? (lever + 0.6) / 1.5 : 0.0;
    const double afterburner = lever >= 0.9 ? (lever - 0.9) / 0.1 : 0.0;
    const auto set = [&channels](std::size_t index, double value) {
        channels[index] = static_cast<float>(std::clamp(value, 1.0, 100.0));
    };
    set(1, 50.0 - 50.0 * input.pitch * authority);
    set(3, 50.0 + 50.0 * input.yaw);
    set(5, 50.0 - 50.0 * input.roll);
    set(6, 50.0 + 50.0 * input.roll);
    set(7, 50.0 - 50.0 * flap);
    set(8, 50.0 + 25.0 * input.pitch * authority);
    set(12, 50.0 - 50.0 * input.roll);
    set(13, 50.0 + 50.0 * input.roll);
    set(14, 50.0 * dry + 50.0 * afterburner);
    set(15, 50.0 + 50.0 * brake);
    set(16, 50.0 + 50.0 * channel16);
    set(17, 50.0 + 50.0 * secondary_flap);
    set(18, 50.0 + 50.0 * sweep);
    set(20, 50.0 + 50.0 * channel20);
    for (std::size_t channel = 25; channel <= 27; ++channel)
        set(channel, input.gear_down ? 50.0 : 100.0);
    for (std::size_t channel = 0x21; channel <= 0x27; ++channel)
        set(channel, input.gear_down ? 50.0 : 100.0);
    return channels;
}

void advance_aircraft_animation_channels(AircraftAnimationChannels& channels,
                                         const AircraftAnimationChannels& targets) noexcept {
    for (std::size_t i = 0; i < channels.size(); ++i) {
        const float delta = targets[i] - channels[i];
        const bool gear = (i >= 25 && i <= 27) || (i >= 0x21 && i <= 0x27);
        const bool flap = i == 7 || i == 17;
        const float step = gear ? 50.0f / 6.0f
            : flap ? std::min(std::abs(delta) * 0.2f, 0.5f)
                   : std::abs(delta) * 0.2f;
        channels[i] = std::clamp(channels[i] + std::clamp(delta, -step, step),
                                 1.0f, 100.0f);
    }
}

} // namespace ac5
