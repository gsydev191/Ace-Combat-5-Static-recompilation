#include "animation_controller.hpp"

#include "utf8_path.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <fstream>
#include <new>
#include <initializer_list>
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

float retail_float(double value) noexcept {
    const float rounded = static_cast<float>(value);
    return std::abs(static_cast<double>(rounded)) > std::abs(value)
        ? std::nextafter(rounded, 0.0f) : rounded;
}

float gear_channel(AircraftAnimationChannels& channels, std::size_t index,
                   float target, float step) noexcept {
    float& value = channels[index];
    const float difference = retail_float(static_cast<double>(value) - target);
    if (difference > step) value = retail_float(static_cast<double>(value) - step);
    else if (difference < -step) value = retail_float(static_cast<double>(value) + step);
    else value = target;
    return value;
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
        f[6] < f[5] || f[8] < f[7] || f[10] < f[9] ||
        f[12] < f[11] || f[14] < f[13] || f[16] < f[15]) {
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
    const double canard_curve = rules.available
        ? reverse_speed_curve(speed, config[7], config[8]) : 0.0;
    const double channel16 = rules.available
        ? reverse_speed_curve(speed, config[9], config[10]) : 0.0;
    const double channel20 = rules.available
        ? reverse_speed_curve(speed, config[15], config[16]) : 0.0;
    const double channel16_value = (input.model_flags & 0x4000u) != 0
        ? std::max(channel16, static_cast<double>(input.controller_override))
        : channel16;
    const double channel17_value = (input.controller_flags & 0x1000u) != 0
        ? std::max(secondary_flap,
                   static_cast<double>(input.controller_override))
        : secondary_flap;
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
    if ((input.controller_flags & 0x1u) != 0) {
        double surface1 = -static_cast<double>(input.pitch);
        double surface2 = surface1;
        if ((input.model_flags & 0x2u) != 0) {
            surface1 -= input.yaw;
            surface2 = static_cast<double>(input.yaw) - input.pitch;
        } else if ((input.model_flags & 0x1u) != 0) {
            surface1 -= input.roll;
            surface2 = static_cast<double>(input.roll) - input.pitch;
        } else if ((input.model_flags & 0x80u) != 0) {
            surface1 -= static_cast<double>(input.yaw) + input.roll;
            surface2 = static_cast<double>(input.yaw) - input.pitch + input.roll;
        } else if ((input.model_flags & 0x200u) != 0) {
            surface1 -= input.slip;
            surface2 = surface1;
        }
        set(1, 50.0 + 50.0 *
            std::clamp(surface1, -1.0, 1.0) * authority);
        set(2, 50.0 + 50.0 *
            std::clamp(surface2, -1.0, 1.0) * authority);
    } else {
        const double surface1 = (input.controller_flags & 0x200000u) != 0
            ? static_cast<double>(input.slip) - input.pitch
            : -static_cast<double>(input.pitch);
        set(1, 50.0 + 50.0 *
            std::clamp(surface1, -1.0, 1.0) * authority);
    }
    set(3, 50.0 + 50.0 * input.yaw);
    // These branches follow the recovered CPlAnimCtrl flag priority. Native
    // pitch/roll/slip feed its matching controller channels at +4/+8/+1c;
    // model flags themselves come directly from ACM header offset +4.
    double roll5 = -static_cast<double>(input.roll);
    double roll6 = static_cast<double>(input.roll);
    // Retail multiplies its km/h threshold by 0.0237037018 before comparing
    // against speed-units-per-tick. speed_kmh is already converted back to
    // km/h by the native flight adapter, so this comparison is equivalent.
    const bool low_speed_400 = (input.model_flags & 0x400u) != 0 &&
        rules.available && speed < config[13];
    if (low_speed_400) {
        roll5 = -flap;
        roll6 = 0.0;
    } else if ((input.model_flags & 0x10u) != 0) {
        roll5 = -flap - input.roll;
        roll6 = input.roll - flap;
    } else if ((input.model_flags & 0x20u) != 0) {
        roll5 = -static_cast<double>(input.pitch) - input.roll;
        roll6 = static_cast<double>(input.roll) - input.pitch;
    } else if ((input.model_flags & 0x40u) != 0) {
        roll5 = static_cast<double>(input.slip) - input.roll;
        roll6 = static_cast<double>(input.slip) + input.roll;
    }
    set(5, 50.0 + 50.0 * std::clamp(roll5, -1.0, 1.0) * authority);
    set(6, 50.0 + 50.0 * std::clamp(roll6, -1.0, 1.0) * authority);
    set(7, 50.0 - 50.0 * flap);
    if ((input.controller_flags & 0x2u) != 0) {
        const double canard_left = std::clamp(
            -static_cast<double>(input.pitch) - input.roll, -1.0, 1.0) *
            canard_curve;
        const double canard_right = std::clamp(
            static_cast<double>(input.roll) - input.pitch, -1.0, 1.0) *
            canard_curve;
        set(8, 50.0 + 50.0 * canard_left);
        set(9, 50.0 + 50.0 * canard_right);
    } else {
        set(8, 50.0 - 50.0 * input.pitch * canard_curve);
    }
    const bool channel10_slip_override =
        (input.controller_flags & 0x1000u) != 0 &&
        (input.model_flags & 0x100u) != 0 && input.slip > 0.0f;
    if (channel10_slip_override) {
        set(10, 100.0);
    } else if ((input.controller_flags & 0x8u) != 0) {
        const double surface10 = std::clamp(
            -static_cast<double>(input.pitch) - input.roll, -1.0, 1.0);
        const double surface11 = std::clamp(
            static_cast<double>(input.roll) - input.pitch, -1.0, 1.0);
        set(10, 50.0 + 50.0 * surface10 * authority);
        set(11, 50.0 + 50.0 * surface11 * authority);
    } else {
        set(10, 50.0 + 50.0 * input.pitch * authority);
    }
    set(12, 50.0 - 50.0 * input.roll);
    set(13, 50.0 + 50.0 * input.roll);
    set(14, 50.0 * dry + 50.0 * afterburner);
    set(15, 50.0 + 50.0 * brake);
    set(16, 50.0 + 50.0 * channel16_value);
    set(17, 50.0 + 50.0 * channel17_value);
    set(18, 50.0 + 50.0 * sweep);
    if ((input.model_flags & 0x10000u) != 0) {
        set(20, 50.0 + 50.0 * channel20);
    } else if ((input.model_flags & 0x20000u) != 0) {
        set(20, 50.0 + 50.0 * input.controller_override);
    }
    if ((input.controller_flags & 0x1000u) != 0)
        set(24, 100.0);
    for (std::size_t channel = 25; channel <= 27; ++channel)
        set(channel, input.gear_down ? 50.0 : 100.0);
    return channels;
}

void advance_aircraft_animation_channels(AircraftAnimationChannels& channels,
                                         const AircraftAnimationChannels& targets) noexcept {
    for (std::size_t i = 0; i < channels.size(); ++i) {
        const float delta = targets[i] - channels[i];
        const bool hangar_gear = i >= 25 && i <= 27;
        const bool retail_gear = i >= 33 && i <= 39;
        const bool flap = i == 7 || i == 17;
        if (retail_gear) continue;
        const float step = hangar_gear ? 50.0f / 6.0f
            : flap ? std::min(std::abs(delta) * 0.2f, 0.5f)
                   : std::abs(delta) * 0.2f;
        channels[i] = std::clamp(channels[i] + std::clamp(delta, -step, step),
                                 1.0f, 100.0f);
    }
}

void request_aircraft_gear(AircraftGearController& gear, bool down) noexcept {
    if (down && (gear.state == 0 || gear.state == 3)) gear.state = 1;
    else if (!down && (gear.state == 1 || gear.state == 2)) gear.state = 3;
}

void advance_aircraft_retail_gear(AircraftGearController& gear,
                                  AircraftAnimationChannels& channels) noexcept {
    constexpr float slow = 0.1666666567325592f;
    int completed = 0;
    if (gear.state == 1) {
        for (std::size_t i : {33u, 34u})
            completed += gear_channel(channels, i, 50.0f, slow) == 50.0f;
        if ((gear.capabilities & 0x300u) == 0x100u)
            completed += gear_channel(channels, 39, 100.0f, slow) == 100.0f;
        else ++completed;
        for (std::size_t i : {35u, 37u, 36u})
            completed += gear_channel(channels, i, 50.0f, slow) == 50.0f;
        if (completed >= 6) gear.state = 2;
    } else if (gear.state == 3) {
        completed = gear_channel(channels, 38, 50.0f, 2.0f) == 50.0f ? 1 : 0;
        if (completed != 0) {
            for (std::size_t i : {35u, 37u, 36u}) {
                if (channels[i] > 50.0f) gear_channel(channels, i, 50.0f, 1.0f);
                else ++completed;
            }
        }
        if (completed >= 4) {
            for (std::size_t i : {33u, 34u})
                completed += gear_channel(channels, i, 0.0f, slow) == 0.0f;
            completed += gear_channel(channels, 39, 50.0f, slow) == 50.0f;
            for (std::size_t i : {35u, 37u, 36u})
                completed += gear_channel(channels, i, 0.0f, slow) == 0.0f;
            if (completed >= 10) gear.state = 0;
        }
    }
}

} // namespace ac5
