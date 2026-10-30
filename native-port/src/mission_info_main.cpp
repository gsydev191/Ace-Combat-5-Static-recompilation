#include "command_line.hpp"
#include "aircraft_collision.hpp"
#include "mission_spawn_preview.hpp"
#include "mission_terrain.hpp"
#include "mission_units.hpp"
#include "utf8_path.hpp"

#include <cstdio>
#include <charconv>
#include <array>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    std::vector<std::string> arguments;
    std::string error;
    if (!ac5::command_line_arguments_utf8(argc, argv, arguments, error)) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 2;
    }
    const bool archive_mode = arguments.size() == 5 && arguments[1] == "--archive";
    const bool hit_extents_mode = arguments.size() == 5 &&
                                  arguments[1] == "--hit-extents";
    const bool preview_mode = arguments.size() == 6 &&
                              arguments[1] == "--spawn-preview";
    const bool raw_mode = arguments.size() == 2;
    if ((!raw_mode && !archive_mode && !hit_extents_mode && !preview_mode) || arguments[1] == "--help" ||
        arguments[1] == "-h") {
        std::fprintf(arguments.size() == 2 ? stdout : stderr,
                     "usage: ac5_mission_info <decompressed-mission.mmb>\n"
                     "       ac5_mission_info --archive <DATA.TBL> <DATA.PAC> <pack-member-index>\n"
                     "       ac5_mission_info --hit-extents <DATA.TBL> <DATA.PAC> <aircraft-entry-index>\n"
                     "       ac5_mission_info --spawn-preview <DATA.TBL> <DATA.PAC> <terrain-member-index> <pack-member-index>\n");
        return arguments.size() == 2 ? 0 : 2;
    }

    std::vector<ac5::Ac5MissionUnit> units;
    if (hit_extents_mode) {
        std::size_t member_index = 0;
        const std::string& text = arguments[4];
        const auto parsed = std::from_chars(text.data(), text.data() + text.size(),
                                            member_index);
        if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
            std::fprintf(stderr, "Invalid aircraft mission-entry member index\n");
            return 2;
        }
        ac5::DataArchive archive;
        std::array<double, 6> extents{};
        if (!archive.open(path_from_utf8(arguments[2]),
                          path_from_utf8(arguments[3]), error) ||
            !ac5::load_ac5_aircraft_hit_extents(archive, member_index,
                                                extents, error)) {
            std::fprintf(stderr, "Aircraft hit extent load failed: %s\n",
                         error.c_str());
            return 1;
        }
        std::printf("hit_extent %.3f %.3f %.3f %.3f %.3f %.3f\n",
                    extents[0], extents[1], extents[2], extents[3],
                    extents[4], extents[5]);
        return 0;
    }
    if (preview_mode) {
        const auto parse_member = [&](std::size_t argument_index,
                                      std::size_t& member_index) {
            const std::string& text = arguments[argument_index];
            const auto parsed = std::from_chars(text.data(), text.data() + text.size(),
                                                member_index);
            return parsed.ec == std::errc{} &&
                   parsed.ptr == text.data() + text.size();
        };
        std::size_t terrain_member = 0;
        std::size_t pack_member = 0;
        if (!parse_member(4, terrain_member) || !parse_member(5, pack_member)) {
            std::fprintf(stderr, "Invalid DATA.PAC member index\n");
            return 2;
        }
        ac5::DataArchive archive;
        std::vector<ac5::ArchiveFile> terrain_files;
        ac5::Ac5MissionTerrain terrain;
        if (!archive.open(path_from_utf8(arguments[2]), path_from_utf8(arguments[3]),
                          error) ||
            !archive.split_member(terrain_member, terrain_files, error) ||
            !ac5::decode_ac5_mission_terrain(terrain_files, terrain, error) ||
            !ac5::load_ac5_mission_units(archive, pack_member, units, error)) {
            std::fprintf(stderr, "Mission preview load failed: %s\n", error.c_str());
            return 1;
        }
        const ac5::Ac5MissionSpawnPreview preview =
            ac5::build_ac5_mission_spawn_preview(units, terrain);
        std::printf("resolved spawn markers: %zu, unresolved orders: %zu\n",
                    preview.spawns.size(), preview.unresolved_orders);
        for (const ac5::Ac5MissionSpawnPreviewItem& spawn : preview.spawns) {
            std::printf("unit %zu side %u source 0x%zx order %.3f %.3f %.3f position %.3f %.3f %.3f attitude %.6f %.6f %.6f\n",
                        spawn.unit_index, static_cast<unsigned>(spawn.side),
                        spawn.source_offset,
                        spawn.order_position[0], spawn.order_position[1],
                        spawn.order_position[2],
                        spawn.transform.position[0], spawn.transform.position[1],
                        spawn.transform.position[2], spawn.transform.attitude[0],
                        spawn.transform.attitude[1], spawn.transform.attitude[2]);
        }
        return 0;
    }
    if (archive_mode) {
        std::size_t member_index = 0;
        const std::string& text = arguments[4];
        const auto result = std::from_chars(text.data(), text.data() + text.size(),
                                            member_index);
        if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
            std::fprintf(stderr, "Invalid DATA.PAC pack-member index: %s\n",
                         text.c_str());
            return 2;
        }
        ac5::DataArchive archive;
        if (!archive.open(path_from_utf8(arguments[2]), path_from_utf8(arguments[3]),
                          error) ||
            !ac5::load_ac5_mission_units(archive, member_index, units, error)) {
            std::fprintf(stderr, "Mission unit load failed: %s\n", error.c_str());
            return 1;
        }
    } else {
        const std::filesystem::path path = path_from_utf8(arguments[1]);
        std::ifstream input(path, std::ios::binary | std::ios::ate);
        if (!input) {
            std::fprintf(stderr, "Cannot open mission MMB: %s\n", arguments[1].c_str());
            return 1;
        }
        const std::streampos end = input.tellg();
        if (end <= 0 || static_cast<std::uintmax_t>(end) > 256u * 1024u * 1024u ||
            static_cast<std::uintmax_t>(end) >
                static_cast<std::uintmax_t>(std::numeric_limits<std::streamsize>::max())) {
            std::fprintf(stderr, "Mission MMB size is outside the supported range\n");
            return 1;
        }
        std::vector<std::uint8_t> bytes(static_cast<std::size_t>(end));
        input.seekg(0, std::ios::beg);
        if (!input.read(reinterpret_cast<char*>(bytes.data()),
                        static_cast<std::streamsize>(bytes.size()))) {
            std::fprintf(stderr, "Cannot read the complete mission MMB\n");
            return 1;
        }
        if (!ac5::decode_ac5_mission_units(bytes, units, error)) {
            std::fprintf(stderr, "Mission unit decode failed: %s\n", error.c_str());
            return 1;
        }
    }
    if (!error.empty()) {
        std::fprintf(stderr, "Mission unit decode failed: %s\n", error.c_str());
        return 1;
    }
    std::size_t members = 0, orders = 0, spawns = 0, waypoints = 0;
    for (const ac5::Ac5MissionUnit& unit : units) {
        members += unit.members.size();
        for (const auto& set : unit.sets)
            for (const auto& act : set)
                for (const ac5::Ac5MissionOrder& order : act) {
                    ++orders;
                    spawns += order.type == 0;
                    waypoints += order.type == 2;
                }
    }
    std::printf("units: %zu, members: %zu, orders: %zu, spawn orders: %zu, waypoints: %zu\n",
                units.size(), members, orders, spawns, waypoints);
    for (const ac5::Ac5MissionUnit& unit : units) {
        std::size_t unit_acts = 0, unit_orders = 0;
        for (const auto& set : unit.sets) {
            unit_acts += set.size();
            for (const auto& act : set) unit_orders += act.size();
        }
        std::printf("unit %zu side %u members %zu sets %zu acts %zu orders %zu\n",
                    unit.index, static_cast<unsigned>(unit.side), unit.members.size(),
                    unit.sets.size(), unit_acts, unit_orders);
        for (const ac5::Ac5MissionMember& member : unit.members) {
            std::printf("  %-24s source 0x%zx side %u class %u model %d hp %u radius %.1f offset %.1f %.1f %.1f\n",
                        member.object_name.c_str(), member.source_offset,
                        static_cast<unsigned>(unit.side),
                        static_cast<unsigned>(member.object_class), member.model_index,
                        static_cast<unsigned>(member.hit_points), member.hit_radius,
                        member.formation_offset[0], member.formation_offset[1],
                        member.formation_offset[2]);
        }
    }
    return 0;
}
