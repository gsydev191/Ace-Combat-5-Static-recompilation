#include "mission_units.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <utility>

namespace ac5 {
namespace {

constexpr std::size_t maximum_table_entries = 4096;
constexpr std::size_t maximum_order_count = 65536;
constexpr std::size_t maximum_script_nodes = 1'000'000;
constexpr std::size_t maximum_members = 65'536;

bool range_fits(std::span<const std::uint8_t> data, std::size_t offset,
                std::size_t size) noexcept {
    return offset <= data.size() && size <= data.size() - offset;
}

bool read_u16(std::span<const std::uint8_t> data, std::size_t offset,
              std::uint16_t& value) noexcept {
    if (!range_fits(data, offset, 2)) return false;
    value = static_cast<std::uint16_t>(data[offset]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[offset + 1]) << 8);
    return true;
}

bool read_s16(std::span<const std::uint8_t> data, std::size_t offset,
              std::int16_t& value) noexcept {
    std::uint16_t raw = 0;
    if (!read_u16(data, offset, raw)) return false;
    value = std::bit_cast<std::int16_t>(raw);
    return true;
}

bool read_u32(std::span<const std::uint8_t> data, std::size_t offset,
              std::uint32_t& value) noexcept {
    if (!range_fits(data, offset, 4)) return false;
    value = static_cast<std::uint32_t>(data[offset]) |
        (static_cast<std::uint32_t>(data[offset + 1]) << 8) |
        (static_cast<std::uint32_t>(data[offset + 2]) << 16) |
        (static_cast<std::uint32_t>(data[offset + 3]) << 24);
    return true;
}

bool read_f32(std::span<const std::uint8_t> data, std::size_t offset,
              float& value) noexcept {
    std::uint32_t raw = 0;
    if (!read_u32(data, offset, raw)) return false;
    value = std::bit_cast<float>(raw);
    return true;
}

bool extract_sparse_slot_zero(std::span<const std::uint8_t> member,
                              std::span<const std::uint8_t>& mmb,
                              std::string& error) {
    std::uint32_t count = 0;
    if (!read_u32(member, 0, count) || count == 0 || count > maximum_table_entries ||
        !range_fits(member, 4, static_cast<std::size_t>(count) * 4)) {
        error = "mission pack sparse bundle header is invalid";
        return false;
    }
    std::vector<std::uint32_t> offsets;
    offsets.reserve(count);
    std::uint32_t slot_zero = 0;
    const std::size_t table_end = 4 + static_cast<std::size_t>(count) * 4;
    for (std::size_t index = 0; index < count; ++index) {
        std::uint32_t offset = 0;
        if (!read_u32(member, 4 + index * 4, offset) ||
            (offset != 0 && (offset < table_end || offset >= member.size()))) {
            error = "mission pack sparse bundle contains an invalid slot offset";
            return false;
        }
        if (index == 0) slot_zero = offset;
        if (offset != 0) offsets.push_back(offset);
    }
    if (slot_zero == 0) {
        error = "mission pack has no MMB in sparse bundle slot 0";
        return false;
    }
    std::sort(offsets.begin(), offsets.end());
    const auto next = std::upper_bound(offsets.begin(), offsets.end(), slot_zero);
    const std::size_t end = next == offsets.end() ? member.size() : *next;
    if (end <= slot_zero) {
        error = "mission pack MMB slot has an invalid extent";
        return false;
    }
    mmb = member.subspan(slot_zero, end - slot_zero);
    return true;
}

bool extract_wrapped_bundle_slot_zero(std::span<const std::uint8_t> block,
                                      std::span<const std::uint8_t>& slot,
                                      std::string& error) {
    std::size_t base = 0;
    std::uint32_t wrapper_first = 0, wrapper_second = 0;
    if (read_u32(block, 0, wrapper_first) &&
        read_u32(block, 4, wrapper_second) &&
        wrapper_first == 16 && wrapper_second == 32) {
        base = 32;
    }
    std::uint32_t count = 0;
    if (!read_u32(block, base, count) || count == 0 ||
        count > maximum_table_entries ||
        !range_fits(block, base + 4, static_cast<std::size_t>(count) * 4)) {
        error = "mission entry bundle header is invalid";
        return false;
    }
    const std::size_t table_end = base + 4 + static_cast<std::size_t>(count) * 4;
    std::vector<std::uint32_t> offsets;
    offsets.reserve(count);
    std::uint32_t first_offset = 0;
    for (std::size_t index = 0; index < count; ++index) {
        std::uint32_t relative = 0;
        if (!read_u32(block, base + 4 + index * 4, relative) ||
            (relative != 0 &&
             (relative < 4 + count * 4 ||
              !range_fits(block, base + relative, 1)))) {
            error = "mission entry bundle contains an invalid slot offset";
            return false;
        }
        if (index == 0) first_offset = relative;
        if (relative != 0) offsets.push_back(relative);
    }
    if (first_offset == 0) {
        error = "mission entry bundle has no unit-table slot 0";
        return false;
    }
    std::sort(offsets.begin(), offsets.end());
    const auto next = std::upper_bound(offsets.begin(), offsets.end(), first_offset);
    const std::size_t start = base + first_offset;
    const std::size_t end = next == offsets.end() ? block.size() : base + *next;
    if (end <= start || end > block.size() || start < table_end) {
        error = "mission entry unit-table slot has an invalid extent";
        return false;
    }
    slot = block.subspan(start, end - start);
    return true;
}

bool add_offset(std::size_t base, std::uint32_t relative,
                std::size_t& absolute) noexcept {
    if (relative > std::numeric_limits<std::size_t>::max() - base) return false;
    absolute = base + relative;
    return true;
}

bool read_string(std::span<const std::uint8_t> data, std::size_t offset,
                 std::size_t length, std::string& value) {
    if (!range_fits(data, offset, length)) return false;
    const auto begin = data.begin() + static_cast<std::ptrdiff_t>(offset);
    const auto end = begin + static_cast<std::ptrdiff_t>(length);
    const auto terminator = std::find(begin, end, std::uint8_t{0});
    value.assign(reinterpret_cast<const char*>(data.data() + offset),
                 static_cast<std::size_t>(terminator - begin));
    return true;
}

bool parse_mission_path(std::span<const std::uint8_t> data, std::size_t node,
                        std::optional<Ac5MissionPath>& path,
                        std::string& error) {
    std::size_t header = 0;
    if (!add_offset(node, 0xa0, header) || !range_fits(data, header, 0x30)) {
        error = "mission keyframed path header is missing or truncated";
        return false;
    }
    std::uint32_t count = 0;
    Ac5MissionPath decoded;
    if (!read_u32(data, header, count) || count == 0 || count > maximum_order_count ||
        !read_f32(data, header + 4, decoded.tangent) ||
        !read_f32(data, header + 8, decoded.z_start) ||
        !read_f32(data, header + 12, decoded.z_margin) ||
        !range_fits(data, header + 0x24, static_cast<std::size_t>(count) * 4)) {
        error = "mission keyframed path header or offset table is invalid";
        return false;
    }
    const std::size_t keys = header + 0x30;
    decoded.keys.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        std::uint32_t relative = 0;
        std::size_t key = 0;
        Ac5MissionPathKey decoded_key;
        if (!read_u32(data, header + 0x24 + index * 4, relative) ||
            !add_offset(keys, relative, key) || !range_fits(data, key, 0x18) ||
            !read_f32(data, key, decoded_key.position[0]) ||
            !read_f32(data, key + 4, decoded_key.position[1]) ||
            !read_f32(data, key + 8, decoded_key.position[2]) ||
            !read_f32(data, key + 0x0c, decoded_key.attitude[0]) ||
            !read_f32(data, key + 0x10, decoded_key.attitude[1]) ||
            !read_f32(data, key + 0x14, decoded_key.attitude[2])) {
            error = "mission keyframed path key is outside the MMB block";
            return false;
        }
        if (!std::all_of(decoded_key.position.begin(), decoded_key.position.end(),
                         [](float value) { return std::isfinite(value); }) ||
            !std::all_of(decoded_key.attitude.begin(), decoded_key.attitude.end(),
                         [](float value) { return std::isfinite(value); }) ||
            (index != 0 && decoded.keys.back().position[2] <=
                               decoded_key.position[2])) {
            error = "mission keyframed path has invalid or unordered keys";
            return false;
        }
        decoded.keys.push_back(decoded_key);
    }
    if (!std::isfinite(decoded.tangent) || !std::isfinite(decoded.z_start) ||
        !std::isfinite(decoded.z_margin)) {
        error = "mission keyframed path header contains non-finite values";
        return false;
    }
    path = std::move(decoded);
    return true;
}

bool bundle_children(std::span<const std::uint8_t> data, std::size_t node,
                     std::vector<std::size_t>& children,
                     std::size_t& remaining_script_nodes,
                     std::string& error) {
    std::uint32_t header_relative = 0, table_relative = 0;
    if (!read_u32(data, node, header_relative) ||
        !read_u32(data, node + 4, table_relative)) {
        error = "mission script node header is truncated";
        return false;
    }
    if (header_relative == 0 || table_relative == 0) {
        children.clear();
        return true;
    }
    std::size_t header = 0, table = 0;
    if (!add_offset(node, header_relative, header) ||
        !add_offset(node, table_relative, table) ||
        !range_fits(data, header, 1) || !range_fits(data, table, 4)) {
        error = "mission script node points outside the MMB block";
        return false;
    }
    const std::size_t count = data[header];
    if (count > maximum_order_count || count > remaining_script_nodes ||
        count > (std::numeric_limits<std::size_t>::max() - 4) / 4 ||
        !range_fits(data, table, 4 + count * 4)) {
        error = "mission script child table is truncated or oversized";
        return false;
    }
    remaining_script_nodes -= count;
    std::vector<std::size_t> decoded;
    decoded.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        std::uint32_t relative = 0;
        if (!read_u32(data, table + 4 + i * 4, relative)) {
            error = "mission script child offset is outside the MMB block";
            return false;
        }
        std::size_t child = 0;
        if (!add_offset(table, relative, child) || child >= data.size()) {
            error = "mission script child offset is outside the MMB block";
            return false;
        }
        decoded.push_back(child);
    }
    children = std::move(decoded);
    return true;
}

bool sparse_bundle_slots(std::span<const std::uint8_t> data, std::size_t table,
                         std::vector<std::size_t>& slots) {
    std::uint32_t count = 0;
    if (!read_u32(data, table, count) || count == 0 ||
        count > maximum_table_entries ||
        !range_fits(data, table + 4, static_cast<std::size_t>(count) * 4))
        return false;
    std::vector<std::size_t> decoded;
    decoded.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        std::uint32_t relative = 0;
        if (!read_u32(data, table + 4 + index * 4, relative)) return false;
        if (relative == 0) continue;
        std::size_t slot = 0;
        if (relative < 4 + count * 4 ||
            !add_offset(table, relative, slot) || slot >= data.size()) return false;
        decoded.push_back(slot);
    }
    slots = std::move(decoded);
    return true;
}

bool parse_order(std::span<const std::uint8_t> data, std::size_t node,
                 Ac5MissionOrder& order, std::string& error) {
    std::uint32_t header_relative = 0;
    std::size_t header = 0;
    if (!read_u32(data, node, header_relative) || header_relative == 0 ||
        !add_offset(node, header_relative, header) || !range_fits(data, header, 1) ||
        !range_fits(data, node + 0x40, order.raw_words.size() * 4)) {
        error = "mission order payload is truncated";
        return false;
    }
    Ac5MissionOrder decoded;
    decoded.source_offset = node;
    decoded.type = data[header];
    for (std::size_t i = 0; i < decoded.raw_words.size(); ++i) {
        if (!read_u32(data, node + 0x40 + i * 4, decoded.raw_words[i])) {
            error = "mission order payload is truncated";
            return false;
        }
        decoded.float_words[i] = std::bit_cast<float>(decoded.raw_words[i]);
    }
    if (decoded.type == 0) {
        decoded.has_world_position = true;
        decoded.radius = decoded.float_words[4];
        decoded.world_position = {decoded.float_words[5], decoded.float_words[6],
                                  decoded.float_words[7]};
        decoded.distance = decoded.float_words[9];
        decoded.heading_degrees = decoded.float_words[11];
    } else if (decoded.type == 2) {
        decoded.has_world_position = true;
        decoded.radius = decoded.float_words[1];
        decoded.world_position = {decoded.float_words[2], decoded.float_words[3],
                                  decoded.float_words[4]};
        decoded.distance = decoded.float_words[8];
    }
    if (decoded.type == 0 || decoded.type == 1) {
        const std::size_t radius_word = decoded.type == 0 ? 12 : 4;
        const std::size_t value_word = decoded.type == 0 ? 13 : 5;
        const std::size_t kind_word = decoded.type == 0 ? 14 : 6;
        decoded.has_trigger = true;
        decoded.trigger_seconds = decoded.float_words[0];
        decoded.trigger_position = {decoded.float_words[1],
                                    decoded.float_words[2],
                                    decoded.float_words[3]};
        decoded.trigger_radius = decoded.float_words[radius_word];
        decoded.trigger_value = std::bit_cast<std::int16_t>(
            static_cast<std::uint16_t>(decoded.raw_words[value_word]));
        decoded.trigger_state = static_cast<std::uint16_t>(
            decoded.raw_words[value_word] >> 16);
        const std::uint32_t packed = decoded.raw_words[kind_word];
        decoded.trigger_kind = static_cast<std::uint8_t>(packed);
        decoded.trigger_phase = static_cast<std::uint8_t>(packed >> 8);
        if (decoded.type == 0) {
            decoded.trigger_group = static_cast<std::uint8_t>(packed >> 16);
            decoded.trigger_member = static_cast<std::uint8_t>(packed >> 24);
        } else {
            decoded.trigger_movement_mode = static_cast<std::uint8_t>(packed >> 16);
            decoded.trigger_group = static_cast<std::uint8_t>(packed >> 24);
            decoded.trigger_member = static_cast<std::uint8_t>(
                decoded.raw_words[kind_word + 1]);
        }
        if (!std::isfinite(decoded.trigger_seconds) ||
            !std::all_of(decoded.trigger_position.begin(), decoded.trigger_position.end(),
                         [](float value) { return std::isfinite(value); }) ||
            !std::isfinite(decoded.trigger_radius)) {
            error = "mission order trigger contains non-finite values";
            return false;
        }
    }
    if (decoded.type == 2 &&
        static_cast<std::uint8_t>(decoded.raw_words[14] >> 24) == 8 &&
        !parse_mission_path(data, node, decoded.path, error)) return false;
    if (decoded.has_world_position &&
        (!std::all_of(decoded.world_position.begin(), decoded.world_position.end(),
                      [](float value) { return std::isfinite(value); }) ||
         !std::isfinite(decoded.radius) || !std::isfinite(decoded.distance) ||
         !std::isfinite(decoded.heading_degrees))) {
        error = "mission spawn or waypoint contains non-finite coordinates";
        return false;
    }
    order = decoded;
    return true;
}

bool parse_script_set(std::span<const std::uint8_t> data, std::size_t set_node,
                      std::vector<std::vector<Ac5MissionOrder>>& acts,
                      std::size_t& remaining_script_nodes,
                      bool require_count_match, std::string& error) {
    std::uint32_t header_relative = 0, table_relative = 0, table_count = 0;
    std::size_t header = 0, table = 0;
    if (!read_u32(data, set_node, header_relative) || header_relative == 0 ||
        !read_u32(data, set_node + 4, table_relative) || table_relative == 0 ||
        !add_offset(set_node, header_relative, header) ||
        !add_offset(set_node, table_relative, table) ||
        !range_fits(data, header, 1) || !read_u32(data, table, table_count) ||
        (require_count_match && data[header] != table_count) ||
        table_count > maximum_order_count) {
        error = "mission set node header and child table disagree";
        return false;
    }
    std::vector<std::size_t> act_nodes;
    if (!bundle_children(data, set_node, act_nodes, remaining_script_nodes,
                         error)) return false;
    std::vector<std::vector<Ac5MissionOrder>> decoded_acts;
    decoded_acts.reserve(act_nodes.size());
    for (const std::size_t act_node : act_nodes) {
        std::vector<std::size_t> order_nodes;
        if (!bundle_children(data, act_node, order_nodes,
                             remaining_script_nodes, error)) return false;
        std::vector<Ac5MissionOrder> orders;
        orders.reserve(order_nodes.size());
        for (const std::size_t order_node : order_nodes) {
            Ac5MissionOrder order;
            if (!parse_order(data, order_node, order, error)) return false;
            if (order.type > 6) {
                error = "mission set traversal reached a non-order child";
                return false;
            }
            orders.push_back(order);
        }
        decoded_acts.push_back(std::move(orders));
    }
    acts = std::move(decoded_acts);
    return true;
}

bool parse_scripts(std::span<const std::uint8_t> data, std::size_t unit_child,
                   Ac5MissionUnit& output, std::size_t& remaining_script_nodes,
                   std::string& error) {
    std::uint32_t root_relative = 0;
    std::size_t member_table = 0;
    if (!read_u32(data, unit_child + 0x44, root_relative) ||
        !add_offset(unit_child + 0x40, root_relative, member_table) ||
        member_table >= data.size()) {
        error = "mission unit root set pointer is invalid";
        return false;
    }
    std::vector<std::size_t> act_nodes;
    if (!bundle_children(data, member_table, act_nodes, remaining_script_nodes,
                         error)) return false;
    std::vector<std::vector<Ac5MissionOrder>> acts;
    acts.reserve(act_nodes.size());
    for (const std::size_t act_node : act_nodes) {
        std::vector<std::size_t> order_nodes;
        if (!bundle_children(data, act_node, order_nodes,
                             remaining_script_nodes, error)) return false;
        std::vector<Ac5MissionOrder> orders;
        orders.reserve(order_nodes.size());
        for (const std::size_t order_node : order_nodes) {
            Ac5MissionOrder order;
            if (!parse_order(data, order_node, order, error)) return false;
            if (order.type > 6) {
                error = "mission set traversal reached a non-order child";
                return false;
            }
            orders.push_back(std::move(order));
        }
        acts.push_back(std::move(orders));
    }
    output.sets.clear();
    output.sets.push_back(std::move(acts));
    return true;
}

bool parse_members(std::span<const std::uint8_t> data, std::size_t child,
                   Ac5MissionUnit& output, std::size_t& remaining_members,
                   std::string& error) {
    if (!range_fits(data, child, 0x44)) {
        error = "mission unit member table is truncated";
        return false;
    }
    const std::size_t member_count = data[child + 0x2c];
    std::uint32_t table_count = 0;
    if (member_count > remaining_members ||
        !read_u32(data, child + 0x40, table_count) ||
        table_count != member_count + 1 || table_count > maximum_table_entries ||
        !range_fits(data, child + 0x44, static_cast<std::size_t>(table_count) * 4)) {
        error = "mission unit member table count or offsets are invalid";
        return false;
    }
    remaining_members -= member_count;
    std::vector<Ac5MissionMember> members;
    members.reserve(member_count);
    for (std::size_t i = 0; i < member_count; ++i) {
        std::uint32_t relative = 0;
        if (!read_u32(data, child + 0x44 + (i + 1) * 4, relative)) {
            error = "mission unit member offset is truncated";
            return false;
        }
        std::size_t member = 0;
        if (!add_offset(child + 0x44, relative, member) ||
            !range_fits(data, member, 0xbau)) {
            error = "mission unit member record is outside the MMB block";
            return false;
        }
        Ac5MissionMember decoded;
        decoded.source_offset = member;
        std::uint32_t flags = 0;
        if (!read_f32(data, member + 0x0c, decoded.formation_offset[0]) ||
            !read_f32(data, member + 0x10, decoded.formation_offset[1]) ||
            !read_f32(data, member + 0x14, decoded.formation_offset[2]) ||
            !read_u32(data, member + 0x24, flags) ||
            !read_string(data, member + 0x4c, 0x20, decoded.object_name) ||
            !read_string(data, member + 0x6c, 0x10, decoded.callsign) ||
            !read_f32(data, member + 0x8c, decoded.hit_radius) ||
            !read_u16(data, member + 0x9c, decoded.kill_points) ||
            !read_s16(data, member + 0x9e, decoded.model_index) ||
            !read_u16(data, member + 0xac, decoded.object_record_id)) {
            error = "mission unit member fields are truncated";
            return false;
        }
        decoded.flags = flags;
        decoded.hit_points = data[member + 0xae];
        decoded.hit_type = data[member + 0xaf];
        decoded.object_class = data[member + 0xb5];
        decoded.movement_kind = data[member + 0xb9];
        if (!std::all_of(decoded.formation_offset.begin(), decoded.formation_offset.end(),
                         [](float value) { return std::isfinite(value); }) ||
            !std::isfinite(decoded.hit_radius)) {
            error = "mission unit member has invalid numeric values";
            return false;
        }
        members.push_back(std::move(decoded));
    }
    output.members = std::move(members);
    return true;
}

void append_member_script_sets(std::span<const std::uint8_t> data,
                               Ac5MissionUnit& unit,
                               std::size_t& remaining_script_nodes) {
    std::unordered_set<std::size_t> seen_orders;
    for (const auto& set : unit.sets)
        for (const auto& act : set)
            for (const Ac5MissionOrder& order : act)
                seen_orders.insert(order.source_offset);

    for (const Ac5MissionMember& member : unit.members) {
        if (member.source_offset > data.size() ||
            data.size() - member.source_offset < 0x2c) continue;
        std::vector<std::size_t> candidates;
        if (!sparse_bundle_slots(data, member.source_offset + 0x2c,
                                 candidates)) continue;
        for (const std::size_t candidate : candidates) {
            std::vector<std::vector<Ac5MissionOrder>> acts;
            std::string ignored_error;
            if (!parse_script_set(data, candidate, acts,
                                  remaining_script_nodes, true,
                                  ignored_error)) continue;
            bool found_new_order = false;
            for (auto& act : acts) {
                std::erase_if(act, [&](const Ac5MissionOrder& order) {
                    if (!seen_orders.insert(order.source_offset).second) return true;
                    found_new_order = true;
                    return false;
                });
            }
            if (found_new_order) unit.sets.push_back(std::move(acts));
        }
    }
}

bool parse_unit(std::span<const std::uint8_t> data, std::size_t record,
                std::size_t index, Ac5MissionUnit& output,
                std::size_t& remaining_members,
                std::size_t& remaining_script_nodes,
                std::string& error) {
    std::uint32_t header_relative = 0, child_relative = 0;
    std::size_t header = 0;
    if (!read_u32(data, record, header_relative) ||
        !add_offset(record, header_relative, header) ||
        !range_fits(data, header, 0x0e) ||
        !read_u32(data, record + 4, child_relative)) {
        error = "mission unit header is truncated";
        return false;
    }
    Ac5MissionUnit decoded;
    decoded.index = index;
    decoded.side = data[header + 0x0d];
    if (child_relative != 0) {
        std::size_t child = 0;
        if (!add_offset(record, child_relative, child) ||
            !parse_members(data, child, decoded, remaining_members, error) ||
            !parse_scripts(data, child, decoded, remaining_script_nodes,
                           error)) return false;
        append_member_script_sets(data, decoded, remaining_script_nodes);
    }
    output = std::move(decoded);
    return true;
}

} // namespace

bool evaluate_ac5_mission_trigger(
    const Ac5MissionOrder& order, Ac5MissionTriggerMode mode,
    const Ac5MissionTriggerContext& context) {
    if (!order.has_trigger || !std::isfinite(context.ticks_per_second) ||
        context.ticks_per_second <= 0.0f) return false;

    switch (order.trigger_kind) {
    case 0: {
        if (context.phase != order.trigger_phase ||
            !std::isfinite(order.trigger_seconds)) return false;
        const float trigger_frame = order.trigger_seconds * context.ticks_per_second;
        const double precise_trigger_frame = static_cast<double>(trigger_frame);
        if (!std::isfinite(trigger_frame) ||
            precise_trigger_frame < static_cast<double>(std::numeric_limits<int>::min()) ||
            precise_trigger_frame > static_cast<double>(std::numeric_limits<int>::max()))
            return false;
        const int at_frame = static_cast<int>(trigger_frame);
        const double frame = static_cast<double>(context.frame);
        return mode == Ac5MissionTriggerMode::spawn_gate
            ? frame >= static_cast<double>(at_frame)
            : frame == static_cast<double>(at_frame);
    }
    case 1: {
        if (!context.player_active || !std::isfinite(order.trigger_radius) ||
            order.trigger_radius < 0.0f) return false;
        float distance_squared = 0.0f;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const float delta = context.player_position[axis] - order.trigger_position[axis];
            if (!std::isfinite(delta)) return false;
            distance_squared += delta * delta;
        }
        return distance_squared <= order.trigger_radius * order.trigger_radius;
    }
    case 2:
        return context.target_is_active &&
            !context.target_is_active(order.trigger_group, order.trigger_member);
    case 3: {
        const auto state = context.states.find(order.trigger_state);
        const std::int32_t value = state == context.states.end() ? 0 : state->second;
        return value == order.trigger_value;
    }
    default:
        return false;
    }
}

bool resolve_ac5_mission_spawn_transform(
    const Ac5MissionOrder& order, const Ac5MissionSpawnContext& context,
    Ac5MissionSpawnTransform& transform, std::string& error) {
    constexpr double pi = 3.14159265358979323846;
    constexpr double degrees_to_radians = pi / 180.0;
    error.clear();
    if (order.type != 0 || !order.has_world_position) {
        error = "mission spawn transform requires a decoded type-0 order";
        return false;
    }
    const std::uint32_t placement = order.raw_words[15];
    const std::uint8_t placement_kind = static_cast<std::uint8_t>(placement);
    if (placement_kind > 2) {
        error = "mission spawn placement mode is not supported";
        return false;
    }
    const float pitch_degrees = order.float_words[10];
    const float yaw_degrees = order.float_words[11];
    if (!std::isfinite(pitch_degrees) || !std::isfinite(yaw_degrees) ||
        !std::isfinite(context.model_altitude_offset)) {
        error = "mission spawn transform contains non-finite orientation or altitude";
        return false;
    }

    std::array<double, 4> position{
        order.world_position[0], order.world_position[1],
        order.world_position[2], 1.0};
    double target_yaw = 0.0;
    if (placement_kind == 1) {
        const std::uint8_t target_group =
            static_cast<std::uint8_t>(placement >> 16);
        if (target_group == 0xff) {
            position[0] += -300000.0;
            position[1] += 4096.0;
            position[2] += -300000.0;
        } else {
            if (!context.relative_target_available ||
                !std::all_of(context.relative_target_position.begin(),
                             context.relative_target_position.end(),
                             [](double value) { return std::isfinite(value); })) {
                error = "target-relative mission spawn requires a live target position";
                return false;
            }
            for (std::size_t axis = 0; axis < 3; ++axis)
                position[axis] += context.relative_target_position[axis];
        }
    } else if (placement_kind == 2) {
        const std::uint8_t target_group =
            static_cast<std::uint8_t>(placement >> 16);
        if (target_group == 0xff) {
            position = {-300000.0, 4096.0, -300000.0, 1.0};
        } else {
            if (!context.relative_target_available ||
                !std::isfinite(context.relative_target_yaw) ||
                !std::all_of(context.relative_target_position.begin(),
                             context.relative_target_position.end(),
                             [](double value) { return std::isfinite(value); })) {
                error = "rotated mission spawn requires a live target position and yaw";
                return false;
            }
            for (std::size_t axis = 0; axis < 3; ++axis)
                position[axis] = context.relative_target_position[axis];
            target_yaw = context.relative_target_yaw;
        }
        const double rotation = target_yaw +
            static_cast<double>(order.float_words[8]) * degrees_to_radians;
        const double distance = static_cast<double>(order.float_words[9]);
        if (!std::isfinite(rotation) || !std::isfinite(distance)) {
            error = "rotated mission spawn offset contains non-finite values";
            return false;
        }
        position[0] -= std::sin(rotation) * distance;
        position[1] += order.float_words[6];
        position[2] -= std::cos(rotation) * distance;
    }

    double y = position[1];
    if ((placement & 0x00000100u) != 0u) {
        if (!context.terrain_height_available ||
            !std::isfinite(context.terrain_height_at_spawn)) {
            error = "terrain-relative mission spawn requires a terrain height";
            return false;
        }
        y += context.terrain_height_at_spawn;
    }
    y += context.model_altitude_offset;
    position[1] = y;
    Ac5MissionSpawnTransform decoded;
    decoded.position = position;
    decoded.attitude = {static_cast<double>(pitch_degrees) * degrees_to_radians,
                        target_yaw + static_cast<double>(yaw_degrees) * degrees_to_radians,
                        0.0};
    if (decoded.attitude[1] > pi)
        decoded.attitude[1] -= 2.0 * pi;
    else if (decoded.attitude[1] <= -pi)
        decoded.attitude[1] += 2.0 * pi;
    if (!std::all_of(decoded.position.begin(), decoded.position.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::all_of(decoded.attitude.begin(), decoded.attitude.end(),
                     [](double value) { return std::isfinite(value); })) {
        error = "mission spawn transform exceeds the native numeric range";
        return false;
    }
    transform = decoded;
    return true;
}

std::vector<std::size_t> eligible_ac5_mission_acts(
    std::span<const std::vector<Ac5MissionOrder>> acts,
    const Ac5MissionTriggerContext& context) {
    std::vector<std::size_t> eligible;
    for (std::size_t index = 0; index < acts.size(); ++index) {
        if (acts[index].empty()) continue;
        const Ac5MissionOrder& first = acts[index].front();
        Ac5MissionTriggerMode mode;
        if (first.type == 0) {
            mode = Ac5MissionTriggerMode::spawn_gate;
        } else if (first.type == 1) {
            mode = Ac5MissionTriggerMode::act_trigger;
        } else {
            continue;
        }
        if (evaluate_ac5_mission_trigger(first, mode, context))
            eligible.push_back(index);
    }
    return eligible;
}

std::optional<Ac5MissionActStart> select_ac5_mission_act(
    std::span<const std::vector<Ac5MissionOrder>> acts,
    Ac5MissionUnitRuntime& runtime,
    const Ac5MissionTriggerContext& context) {
    std::optional<Ac5MissionActStart> selected;
    const bool already_spawned = runtime.current_act >= 0;
    const auto start = [&](std::size_t index, bool triggered, bool initial,
                           std::uint8_t movement_mode) {
        runtime.current_act = static_cast<std::int32_t>(index);
        runtime.order = {};
        ++runtime.order_serial;
        selected = Ac5MissionActStart{index, triggered, initial, movement_mode};
    };

    for (std::size_t index = 0; index < acts.size(); ++index) {
        if (index == static_cast<std::size_t>(runtime.current_act) ||
            acts[index].empty()) continue;
        const Ac5MissionOrder& first = acts[index].front();
        if (!first.has_trigger) continue;
        if (first.type == 1) {
            if (evaluate_ac5_mission_trigger(first,
                    Ac5MissionTriggerMode::act_trigger, context))
                start(index, true, false, first.trigger_movement_mode);
            continue;
        }
        if (first.type != 0) continue;
        const bool state_trigger = first.trigger_kind == 3;
        const std::uint64_t key = state_trigger
            ? (std::uint64_t{1} << 40) | index
            : (static_cast<std::uint64_t>(context.phase) << 20) | index;
        if (runtime.fired_spawn_triggers.contains(key) ||
            !evaluate_ac5_mission_trigger(first,
                    Ac5MissionTriggerMode::spawn_gate, context)) continue;
        runtime.fired_spawn_triggers.insert(key);
        if (!already_spawned && !selected) {
            start(index, false, true, 0);
            for (std::size_t spent = index + 1; spent < acts.size(); ++spent) {
                if (acts[spent].empty()) continue;
                const Ac5MissionOrder& other = acts[spent].front();
                if (other.type == 0 && other.has_trigger && other.trigger_kind != 3 &&
                    other.trigger_phase == context.phase)
                    runtime.fired_spawn_triggers.insert(
                        (static_cast<std::uint64_t>(context.phase) << 20) | spent);
            }
        } else {
            start(index, false, false, 0);
        }
    }
    return selected;
}

Ac5MissionOrderStep step_ac5_mission_order(
    std::span<const Ac5MissionOrder> orders, Ac5MissionOrderRuntime& runtime,
    std::unordered_map<std::uint16_t, std::int32_t>& states,
    const Ac5MissionOrderContext& context) {
    if (runtime.order_index >= orders.size()) return Ac5MissionOrderStep::act_complete;
    const Ac5MissionOrder& order = orders[runtime.order_index];
    const auto enter_order = [&](std::size_t index) {
        runtime.order_index = index;
        runtime.elapsed_order_ticks = 0;
        ++runtime.order_serial;
        return index < orders.size()
            ? Ac5MissionOrderStep::advanced : Ac5MissionOrderStep::act_complete;
    };
    const auto advance_order = [&]() { return enter_order(runtime.order_index + 1); };
    // Act selection reports the corresponding spawn/removal event to the
    // caller; consuming these first records then advances the script cursor.
    if (order.type == 0 || order.type == 1) return advance_order();
    if (order.type == 4) {
        return advance_order();
    }
    if (order.type == 2) {
        const std::uint32_t flags = order.raw_words[13] >> 16;
        const std::uint32_t follow_data = order.raw_words[14];
        const auto kind = static_cast<std::uint8_t>(follow_data);
        const auto group = static_cast<std::uint8_t>(follow_data >> 8);
        const auto mode = static_cast<std::uint8_t>(follow_data >> 24);
        bool complete = false;
        if ((flags & (1u << 3)) != 0) {
            if (!std::isfinite(order.float_words[0]) ||
                !std::isfinite(context.ticks_per_second) ||
                context.ticks_per_second <= 0.0f)
                return Ac5MissionOrderStep::unsupported;
            complete = static_cast<float>(runtime.elapsed_order_ticks) /
                context.ticks_per_second >= order.float_words[0];
            if (!complete) ++runtime.elapsed_order_ticks;
        }
        const auto state_id = static_cast<std::uint16_t>(order.raw_words[12]);
        if (!complete && (flags & (1u << 4)) != 0 && state_id != 0xffff) {
            const auto expected = std::bit_cast<std::int16_t>(
                static_cast<std::uint16_t>(order.raw_words[12] >> 16));
            const auto state = states.find(state_id);
            const std::int32_t current = state == states.end() ? 0 : state->second;
            complete = current == expected;
        }
        if (!complete && mode == 8) {
            if (!order.path) return Ac5MissionOrderStep::unsupported;
            complete = context.keyframed_path_complete;
        }
        if (!complete && (kind == 0 || kind == 1)) {
            std::array<double, 4> target{};
            bool has_target = true;
            if (kind == 0) {
                if (!(order.float_words[1] > 0.0f)) has_target = false;
                target = {order.world_position[0],
                          order.world_position[1] + context.model_altitude_offset,
                          order.world_position[2], 0.0};
                if ((flags & 1u) != 0) {
                    if (!context.terrain_height_available ||
                        !std::isfinite(context.terrain_height_at_waypoint))
                        return Ac5MissionOrderStep::unsupported;
                    target[1] += context.terrain_height_at_waypoint;
                }
            } else if (group == 0xff) {
                target = {-300000.0, 4096.0, -300000.0, 1.0};
            } else {
                if (!context.follow_target_available)
                    return Ac5MissionOrderStep::unsupported;
                if (!context.follow_target_active) {
                    complete = true;
                    has_target = false;
                } else {
                    target = context.follow_target_position;
                    target[3] = 1.0;
                }
            }
            if (!complete && has_target) {
                const double radius = order.float_words[1];
                if (!std::isfinite(radius) || radius < 0.0 ||
                    !std::isfinite(context.model_altitude_offset))
                    return Ac5MissionOrderStep::unsupported;
                double distance_squared = 0.0;
                for (std::size_t axis = 0; axis < 4; ++axis) {
                    if (!std::isfinite(target[axis]) ||
                        !std::isfinite(context.waypoint_unit_position[axis]))
                        return Ac5MissionOrderStep::unsupported;
                    const double delta = target[axis] -
                        context.waypoint_unit_position[axis];
                    distance_squared += delta * delta;
                }
                complete = distance_squared < radius * radius;
            }
        }
        if (!complete && (flags & (1u << 6)) != 0 && mode == 7)
            complete = context.below_ground_after_ballistic_movement;
        if (!complete) return Ac5MissionOrderStep::waiting;
        return advance_order();
    }
    if (order.type == 3) {
        const float seconds = order.float_words[0];
        const float distance = order.float_words[1];
        const std::uint32_t value_and_state = order.raw_words[2];
        const auto expected = std::bit_cast<std::int16_t>(
            static_cast<std::uint16_t>(value_and_state));
        const auto state_id = static_cast<std::uint16_t>(value_and_state >> 16);
        const auto mode = static_cast<std::uint8_t>(order.raw_words[3]);
        bool complete = false;
        switch (mode) {
        case 0:
            break;
        case 1:
            if (!std::isfinite(seconds) || !std::isfinite(context.ticks_per_second) ||
                context.ticks_per_second <= 0.0f) return Ac5MissionOrderStep::unsupported;
            complete = static_cast<float>(runtime.elapsed_order_ticks) /
                context.ticks_per_second >= seconds;
            if (!complete) ++runtime.elapsed_order_ticks;
            break;
        case 2:
            if (!std::isfinite(distance) || distance < 0.0f)
                return Ac5MissionOrderStep::unsupported;
            complete = true;
            for (const Ac5MissionStopTarget& target : context.stop_targets) {
                const bool unit_is_friendly = context.unit_side < 2;
                const bool target_is_friendly = target.side < 2;
                if (!target.active || unit_is_friendly == target_is_friendly) continue;
                double distance_squared = 0.0;
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    if (!std::isfinite(target.position[axis]) ||
                        !std::isfinite(context.unit_position[axis]))
                        return Ac5MissionOrderStep::unsupported;
                    const double delta = static_cast<double>(target.position[axis]) -
                        static_cast<double>(context.unit_position[axis]);
                    distance_squared += delta * delta;
                }
                if (distance_squared < static_cast<double>(distance) * distance) {
                    complete = false;
                    break;
                }
            }
            break;
        case 3: {
            const auto state = states.find(state_id);
            const std::int32_t current = state == states.end() ? 0 : state->second;
            complete = current == expected;
            break;
        }
        default:
            return Ac5MissionOrderStep::unsupported;
        }
        if (!complete || runtime.order_index + 1 >= orders.size())
            return Ac5MissionOrderStep::waiting;
        return advance_order();
    }
    if (order.type == 6) {
        const std::uint32_t state_and_value = order.raw_words[0];
        const auto state_id = static_cast<std::uint16_t>(state_and_value);
        const std::int32_t value = std::bit_cast<std::int16_t>(
            static_cast<std::uint16_t>(state_and_value >> 16));
        const auto mode = static_cast<std::uint8_t>(order.raw_words[1]);
        const Ac5MissionOrderStep result = advance_order();
        if (mode == 0) {
            states[state_id] = value;
        } else if (mode == 1) {
            const auto found = states.find(state_id);
            const std::uint32_t previous = found == states.end()
                ? 0u : std::bit_cast<std::uint32_t>(found->second);
            const std::uint32_t sum = previous + static_cast<std::uint32_t>(value);
            states[state_id] = std::bit_cast<std::int32_t>(sum);
        }
        return result;
    }
    if (order.type != 5) return Ac5MissionOrderStep::unsupported;

    const std::uint32_t state_and_value = order.raw_words[0];
    const auto state_id = static_cast<std::uint16_t>(state_and_value);
    const auto expected = std::bit_cast<std::int16_t>(
        static_cast<std::uint16_t>(state_and_value >> 16));
    const std::uint32_t control = order.raw_words[1];
    const auto mode = static_cast<std::uint8_t>(control);
    const auto repeat = static_cast<std::uint8_t>(control >> 8);
    const auto offset = std::bit_cast<std::int8_t>(
        static_cast<std::uint8_t>(control >> 16));
    const std::size_t next = runtime.order_index + 1;
    const auto loop_target = [&]() {
        const auto target = static_cast<std::int64_t>(runtime.order_index) +
            static_cast<std::int64_t>(offset) + 1;
        return target >= 0 && static_cast<std::uint64_t>(target) < orders.size()
            ? static_cast<std::size_t>(target) : next;
    };
    const auto state = states.find(state_id);
    const std::int32_t current = state == states.end() ? 0 : state->second;
    if (mode == 1) {
        if (current == expected) return advance_order();
        return enter_order(loop_target());
    } else if (repeat == 0) {
        return advance_order();
    } else if (runtime.counted_loop_marker != runtime.order_index) {
        runtime.counted_loop_marker = runtime.order_index;
        runtime.counted_loop_remaining = static_cast<std::uint8_t>(repeat - 1);
        return enter_order(loop_target());
    } else if (runtime.counted_loop_remaining != 0) {
        --runtime.counted_loop_remaining;
        return enter_order(loop_target());
    } else {
        runtime.counted_loop_marker.reset();
        return advance_order();
    }
    return Ac5MissionOrderStep::unsupported;
}

bool decode_ac5_mission_units(std::span<const std::uint8_t> mmb,
                              std::vector<Ac5MissionUnit>& units,
                              std::string& error) {
    error.clear();
    std::size_t table = 0;
    std::uint32_t wrapper_first = 0, wrapper_second = 0;
    if (read_u32(mmb, 0, wrapper_first) && read_u32(mmb, 4, wrapper_second) &&
        wrapper_first == 16 && wrapper_second == 32) table = 32;
    std::uint32_t count = 0;
    if (!read_u32(mmb, table, count) || count == 0 ||
        count > maximum_table_entries ||
        !range_fits(mmb, table + 4, static_cast<std::size_t>(count) * 4)) {
        error = "mission unit bundle header is invalid";
        return false;
    }
    const std::size_t table_end = table + 4 + static_cast<std::size_t>(count) * 4;
    std::vector<std::uint32_t> offsets;
    offsets.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        std::uint32_t relative = 0;
        if (!read_u32(mmb, table + 4 + index * 4, relative) ||
            (relative != 0 &&
             (relative < 4 + count * 4 ||
              !range_fits(mmb, table + relative, 1)))) {
            error = "mission unit slot offset is outside the bundle";
            return false;
        }
        offsets.push_back(relative);
    }
    std::vector<Ac5MissionUnit> decoded;
    decoded.reserve(count);
    std::size_t remaining_members = maximum_members;
    std::size_t remaining_script_nodes = maximum_script_nodes;
    for (std::size_t index = 0; index < count; ++index) {
        const std::uint32_t relative = offsets[index];
        if (relative == 0) {
            Ac5MissionUnit empty;
            empty.index = index;
            decoded.push_back(std::move(empty));
            continue;
        }
        const std::size_t record = table + relative;
        const auto next = std::find_if(
            offsets.begin() + static_cast<std::ptrdiff_t>(index + 1), offsets.end(),
            [](std::uint32_t offset) { return offset != 0; });
        const std::size_t record_end = next == offsets.end()
            ? mmb.size() : table + *next;
        if (record < table_end || record_end <= record ||
            record_end > mmb.size() || !range_fits(mmb, record, 8)) {
            error = "mission unit record is outside the MMB block";
            return false;
        }
        Ac5MissionUnit unit;
        if (!parse_unit(mmb, record, index, unit, remaining_members,
                        remaining_script_nodes, error)) return false;
        decoded.push_back(std::move(unit));
    }
    units = std::move(decoded);
    return true;
}

bool load_ac5_mission_units(const DataArchive& archive, std::size_t pack_member,
                            std::vector<Ac5MissionUnit>& units,
                            std::string& error) {
    std::vector<std::uint8_t> member;
    if (!archive.read_member(pack_member, member, error)) return false;
    std::span<const std::uint8_t> mission_entry;
    std::span<const std::uint8_t> unit_table;
    return extract_sparse_slot_zero(member, mission_entry, error) &&
        extract_wrapped_bundle_slot_zero(mission_entry, unit_table, error) &&
        decode_ac5_mission_units(unit_table, units, error);
}

} // namespace ac5
