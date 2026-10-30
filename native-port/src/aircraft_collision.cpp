#include "aircraft_collision.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <initializer_list>
#include <span>
#include <vector>

namespace ac5 {
namespace {

std::uint32_t read_u32_le(std::span<const std::uint8_t> bytes,
                          std::size_t offset) noexcept {
    return static_cast<std::uint32_t>(bytes[offset]) |
        (static_cast<std::uint32_t>(bytes[offset + 1]) << 8) |
        (static_cast<std::uint32_t>(bytes[offset + 2]) << 16) |
        (static_cast<std::uint32_t>(bytes[offset + 3]) << 24);
}

float read_f32_le(std::span<const std::uint8_t> bytes,
                  std::size_t offset) noexcept {
    const std::uint32_t bits = read_u32_le(bytes, offset);
    return std::bit_cast<float>(bits);
}

bool parse_bundle(std::span<const std::uint8_t> bytes,
                  std::vector<std::span<const std::uint8_t>>& slots) {
    if (bytes.size() < 4) return false;
    const std::uint32_t count = read_u32_le(bytes, 0);
    if (count == 0 || count > 4096 ||
        static_cast<std::size_t>(count) > (bytes.size() - 4) / 4)
        return false;
    const std::size_t table_end = 4 + static_cast<std::size_t>(count) * 4;
    std::vector<std::uint32_t> offsets;
    offsets.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        const std::uint32_t offset = read_u32_le(bytes, 4 + index * 4);
        if (offset != 0 && (offset < table_end || offset > bytes.size()))
            return false;
        if (offset != 0) offsets.push_back(offset);
    }
    std::sort(offsets.begin(), offsets.end());
    offsets.erase(std::unique(offsets.begin(), offsets.end()), offsets.end());

    slots.clear();
    slots.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        const std::uint32_t offset = read_u32_le(bytes, 4 + index * 4);
        if (offset == 0) {
            slots.emplace_back();
            continue;
        }
        const auto next = std::upper_bound(offsets.begin(), offsets.end(), offset);
        const std::size_t end = next == offsets.end() ? bytes.size() : *next;
        if (end < offset) return false;
        slots.push_back(bytes.subspan(offset, end - offset));
    }
    return true;
}

} // namespace

std::optional<std::size_t> count_airframe_surface_hits(
    const std::array<double, 3>& position,
    const std::array<std::array<double, 3>, 3>& body_axes,
    const std::array<double, 6>& extents,
    const std::function<bool(double, double, double&)>& sample_surface) {
    if (!sample_surface ||
        !std::all_of(position.begin(), position.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::all_of(extents.begin(), extents.end(),
                     [](double value) { return std::isfinite(value); }))
        return std::nullopt;
    for (const auto& axis : body_axes)
        if (!std::all_of(axis.begin(), axis.end(),
                         [](double value) { return std::isfinite(value); }))
            return std::nullopt;

    std::size_t hits = 0;
    for (std::size_t axis_index = 0; axis_index < body_axes.size(); ++axis_index) {
        for (const double extent : {extents[axis_index * 2],
                                    extents[axis_index * 2 + 1]}) {
            const double x = position[0] + extent * body_axes[axis_index][0];
            const double y = position[1] + extent * body_axes[axis_index][1];
            const double z = position[2] + extent * body_axes[axis_index][2];
            double surface_height = 0.0;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
                !sample_surface(x, z, surface_height) ||
                !std::isfinite(surface_height))
                return std::nullopt;
            if (y <= surface_height) ++hits;
        }
    }
    return hits;
}

bool load_ac5_aircraft_hit_extents(
    const DataArchive& archive, std::size_t mission_entry_member,
    std::array<double, 6>& extents, std::string& error) {
    std::vector<std::uint8_t> member;
    if (!archive.read_member(mission_entry_member, member, error)) return false;
    std::vector<std::span<const std::uint8_t>> root_slots;
    if (!parse_bundle(member, root_slots) || root_slots.size() <= 1 ||
        root_slots[1].empty()) {
        error = "aircraft mission entry has no valid AHM bundle slot";
        return false;
    }

    std::vector<std::span<const std::uint8_t>> ahm_slots;
    if (!parse_bundle(root_slots[1], ahm_slots)) {
        error = "aircraft mission entry AHM slot is not a valid bundle";
        return false;
    }
    for (const auto payload : ahm_slots) {
        if (payload.size() < 64 || payload[0] != 'A' || payload[1] != 'H' ||
            payload[2] != 'M' || payload[3] != 0)
            continue;
        const std::size_t values_offset = payload.size() - 32 + 2 * 4;
        std::array<double, 6> decoded{};
        bool valid = true;
        for (std::size_t index = 0; index < decoded.size(); ++index) {
            const float value = read_f32_le(payload, values_offset + index * 4);
            valid = valid && std::isfinite(value) && std::abs(value) < 1.0e5f;
            decoded[index] = value;
        }
        if (valid) {
            extents = decoded;
            error.clear();
            return true;
        }
    }
    error = "aircraft mission entry has no valid AHM collision extents";
    return false;
}

} // namespace ac5
