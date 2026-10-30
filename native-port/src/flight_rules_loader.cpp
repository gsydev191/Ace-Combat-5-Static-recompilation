#include "flight_rules_loader.hpp"

#include "utf8_path.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <fstream>
#include <new>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ac5 {
namespace {

bool read_u16(std::span<const std::uint8_t> bytes, std::size_t offset,
              std::uint16_t& value) {
    if (offset > bytes.size() || bytes.size() - offset < 2) return false;
    value = static_cast<std::uint16_t>(bytes[offset]) |
            static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[offset + 1]) << 8);
    return true;
}

bool read_u32(std::span<const std::uint8_t> bytes, std::size_t offset,
              std::uint32_t& value) {
    if (offset > bytes.size() || bytes.size() - offset < 4) return false;
    value = static_cast<std::uint32_t>(bytes[offset]) |
            (static_cast<std::uint32_t>(bytes[offset + 1]) << 8) |
            (static_cast<std::uint32_t>(bytes[offset + 2]) << 16) |
            (static_cast<std::uint32_t>(bytes[offset + 3]) << 24);
    return true;
}

bool read_f32(std::span<const std::uint8_t> bytes, std::size_t offset,
              double& value) {
    std::uint32_t bits = 0;
    if (!read_u32(bytes, offset, bits)) return false;
    const float decoded = std::bit_cast<float>(bits);
    if (!std::isfinite(decoded)) return false;
    value = decoded;
    return true;
}

bool decode_axis(std::span<const std::uint8_t> bytes, std::size_t data_offset,
                 double final_gain, double penultimate_gain,
                 FlightAxisRules& axis) {
    std::uint16_t points[6]{};
    for (std::size_t i = 0; i < 6; ++i)
        if (!read_u16(bytes, data_offset + i * 2, points[i])) return false;
    for (std::size_t i = 0; i < 8; ++i) {
        if (!read_f32(bytes, data_offset + 0x0c + i * 4,
                      axis.rates_degrees_per_second[i])) return false;
    }
    axis.low_input_gain_percent = penultimate_gain;
    axis.full_input_gain_percent = final_gain;
    return true;
}

} // namespace

bool decode_aircraft_flight_rules(std::span<const std::uint8_t> bytes,
                                  AircraftFlightRules& rules,
                                  std::string& error) {
    if (bytes.size() != 400) {
        error = "aircraft flight parameter file must be exactly 400 bytes";
        return false;
    }
    std::uint32_t wrapper_a = 0, wrapper_b = 0, outer_count = 0, outer_offset = 0;
    std::uint32_t section_count = 0;
    if (!read_u32(bytes, 0, wrapper_a) || !read_u32(bytes, 4, wrapper_b) ||
        wrapper_a != 16 || wrapper_b != 32 || !read_u32(bytes, 0x20, outer_count) ||
        !read_u32(bytes, 0x24, outer_offset) || outer_count != 1 || outer_offset != 0x10 ||
        !read_u32(bytes, 0x40, section_count) || section_count != 4) {
        error = "aircraft flight parameter bundle header is invalid";
        return false;
    }

    constexpr std::array<std::uint32_t, 4> expected_offsets{0x20, 0x60, 0xb0, 0x100};
    std::array<std::size_t, 4> data_offsets{};
    std::array<std::size_t, 4> block_ends{};
    for (std::size_t i = 0; i < expected_offsets.size(); ++i) {
        std::uint32_t relative = 0, block_header = 0;
        if (!read_u32(bytes, 0x44 + i * 4, relative) || relative != expected_offsets[i]) {
            error = "aircraft flight parameter section offsets are unsupported";
            return false;
        }
        const std::size_t block = 0x40 + relative;
        const std::size_t next_block = i + 1 < expected_offsets.size()
            ? 0x40 + expected_offsets[i + 1] : bytes.size();
        if (!read_u32(bytes, block, block_header) || block_header != 0x10 ||
            block + 0x10 > next_block) {
            error = "aircraft flight parameter section header is invalid";
            return false;
        }
        data_offsets[i] = block + 0x10;
        block_ends[i] = next_block;
    }

    AircraftFlightRules decoded;
    const std::size_t speed = data_offsets[0];
    if (block_ends[0] - speed < 0x22 ||
        !read_f32(bytes, speed + 0x00, decoded.speed.spool_up_gain) ||
        !read_f32(bytes, speed + 0x04, decoded.speed.spool_down_gain) ||
        !read_f32(bytes, speed + 0x08, decoded.speed.climb_penalty_percent) ||
        !read_f32(bytes, speed + 0x10, decoded.speed.inverted_sink_mps)) {
        error = "aircraft speed parameter block is truncated or invalid";
        return false;
    }
    std::uint16_t speed_values[7]{};
    for (std::size_t i = 0; i < 7; ++i) {
        if (!read_u16(bytes, speed + 0x14 + i * 2, speed_values[i])) {
            error = "aircraft speed table is truncated";
            return false;
        }
    }
    decoded.speed.cruise_kmh = speed_values[0];
    decoded.speed.maximum_kmh = speed_values[1];
    decoded.speed.minimum_kmh = speed_values[2];
    decoded.speed.stall_kmh = speed_values[3];
    decoded.speed.full_control_kmh = speed_values[4];
    decoded.speed.dive_limit_kmh = speed_values[5];
    decoded.speed.dive_extra_kmh = speed_values[6];

    const std::array<std::size_t, 3> axis_indices{1, 2, 3};
    std::array<FlightAxisRules*, 3> axes{&decoded.pitch, &decoded.roll, &decoded.yaw};
    for (std::size_t i = 0; i < axes.size(); ++i) {
        const std::size_t section = axis_indices[i];
        const std::size_t data = data_offsets[section];
        const std::size_t data_size = block_ends[section] - data;
        if (data_size < 0x2c) {
            error = "aircraft axis parameter block is truncated";
            return false;
        }
        std::array<std::uint16_t, 6> points{};
        for (std::size_t point = 0; point < points.size(); ++point) {
            if (!read_u16(bytes, data + point * 2, points[point])) {
                error = "aircraft axis speed breakpoints are truncated";
                return false;
            }
        }
        axes[i]->breakpoints_kmh[0] = decoded.speed.full_control_kmh;
        for (std::size_t point = 0; point < points.size(); ++point)
            axes[i]->breakpoints_kmh[point + 1] = points[point];
        axes[i]->breakpoints_kmh[7] = decoded.speed.dive_limit_kmh;

        double gain_a = 0.0, gain_b = 0.0;
        const std::size_t gain_a_offset = i == 0 ? 0x34 : (i == 1 ? 0x2c : 0x30);
        const std::size_t gain_b_offset = i == 0 ? 0x38 : (i == 1 ? 0x30 : 0x34);
        if (data_size < gain_b_offset + 4 ||
            !read_f32(bytes, data + gain_a_offset, gain_a) ||
            !read_f32(bytes, data + gain_b_offset, gain_b) ||
            !decode_axis(bytes, data, gain_a, gain_b, *axes[i])) {
            error = "aircraft axis rate table is truncated or invalid";
            return false;
        }
    }

    if (!read_f32(bytes, data_offsets[1] + 0x30, decoded.trim_pitch_gain) ||
        !read_f32(bytes, data_offsets[3] + 0x2c, decoded.trim_yaw_gain) ||
        !read_f32(bytes, data_offsets[3] + 0x30, decoded.stall_drop_percent)) {
        error = "aircraft trim parameters are truncated or invalid";
        return false;
    }
    rules = std::move(decoded);
    return true;
}

bool load_aircraft_flight_rules(const std::string& path,
                                AircraftFlightRules& rules,
                                std::string& error) {
    std::ifstream input(path_from_utf8(path), std::ios::binary);
    if (!input) {
        error = "cannot open aircraft flight parameter file: " + path;
        return false;
    }
    std::vector<std::uint8_t> bytes;
    try {
        bytes.resize(400);
    } catch (const std::bad_alloc&) {
        error = "aircraft flight parameter allocation failed: " + path;
        return false;
    } catch (const std::length_error&) {
        error = "aircraft flight parameter exceeds the platform limit: " + path;
        return false;
    }
    if (!input.read(reinterpret_cast<char*>(bytes.data()), 400) ||
        input.peek() != std::char_traits<char>::eof()) {
        error = "aircraft flight parameter file is truncated or has trailing data: " + path;
        return false;
    }
    return decode_aircraft_flight_rules(bytes, rules, error);
}

} // namespace ac5
