#include "mission_spawn_preview.hpp"

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

namespace ac5 {
namespace {

struct SpawnCandidate {
    const Ac5MissionOrder* order = nullptr;
    std::size_t unit_index = 0;
    std::uint8_t side = 0;
    bool primary_for_unit = false;
    std::optional<Ac5MissionSpawnTransform> transform;
};

bool resolve_preview_spawn(const Ac5MissionOrder& spawn,
                           Ac5MissionSpawnContext context,
                           const Ac5MissionTerrain& terrain,
                           Ac5MissionSpawnTransform& result) {
    const bool terrain_relative =
        (spawn.raw_words[15] & 0x00000100u) != 0u;
    if (terrain_relative) {
        context.terrain_height_available = true;
        context.terrain_height_at_spawn = 0.0;
    }
    std::string error;
    if (!resolve_ac5_mission_spawn_transform(spawn, context, result, error))
        return false;
    if (!terrain_relative) return true;
    float terrain_height = 0.0f;
    if (!sample_ac5_mission_terrain_height(
            terrain, result.position[0], result.position[2], terrain_height,
            error)) return false;
    context.terrain_height_at_spawn = terrain_height;
    return resolve_ac5_mission_spawn_transform(spawn, context, result, error);
}

} // namespace

Ac5MissionSpawnPreview build_ac5_mission_spawn_preview(
    std::span<const Ac5MissionUnit> units, const Ac5MissionTerrain& terrain) {
    std::vector<SpawnCandidate> candidates;
    std::vector<const Ac5MissionUnit*> units_by_index(units.size());
    std::vector<std::optional<Ac5MissionSpawnTransform>> group_transforms(units.size());
    std::vector<bool> has_primary_candidate(units.size());

    for (const Ac5MissionUnit& unit : units) {
        if (unit.index < units_by_index.size()) units_by_index[unit.index] = &unit;
        for (const auto& set : unit.sets) {
            for (const auto& act : set) {
                if (act.empty() || act.front().type != 0 ||
                    !act.front().has_world_position) continue;
                const bool primary = unit.index < has_primary_candidate.size() &&
                    !has_primary_candidate[unit.index];
                if (primary) has_primary_candidate[unit.index] = true;
                candidates.push_back({&act.front(), unit.index, unit.side,
                                      primary, std::nullopt});
            }
        }
    }

    bool made_progress = true;
    while (made_progress) {
        made_progress = false;
        for (SpawnCandidate& candidate : candidates) {
            if (candidate.transform.has_value()) continue;
            const Ac5MissionOrder& spawn = *candidate.order;
            Ac5MissionSpawnContext context;
            const std::uint32_t placement = spawn.raw_words[15];
            const std::uint8_t placement_kind =
                static_cast<std::uint8_t>(placement);
            const std::uint8_t target_group =
                static_cast<std::uint8_t>(placement >> 16);
            const std::uint8_t target_member =
                static_cast<std::uint8_t>(placement >> 24);
            if ((placement_kind == 1 || placement_kind == 2) &&
                target_group != 0xff) {
                if (target_group >= group_transforms.size() ||
                    !group_transforms[target_group].has_value()) continue;
                const Ac5MissionUnit* target_unit = units_by_index[target_group];
                if (target_unit == nullptr) continue;
                if (target_member != 0xff &&
                    (target_member != 0 || target_unit->members.empty() ||
                     !std::all_of(
                         target_unit->members.front().formation_offset.begin(),
                         target_unit->members.front().formation_offset.end(),
                         [](float value) { return value == 0.0f; }))) continue;
                const Ac5MissionSpawnTransform& target =
                    *group_transforms[target_group];
                context.relative_target_available = true;
                context.relative_target_position = target.position;
                context.relative_target_yaw = target.attitude[1];
            }

            Ac5MissionSpawnTransform resolved;
            if (!resolve_preview_spawn(spawn, context, terrain, resolved)) continue;
            candidate.transform = resolved;
            if (candidate.primary_for_unit &&
                candidate.unit_index < group_transforms.size() &&
                !group_transforms[candidate.unit_index].has_value())
                group_transforms[candidate.unit_index] = resolved;
            made_progress = true;
        }
    }

    Ac5MissionSpawnPreview preview;
    preview.spawns.reserve(candidates.size());
    for (const SpawnCandidate& candidate : candidates) {
        if (!candidate.transform.has_value()) {
            ++preview.unresolved_orders;
            continue;
        }
        preview.spawns.push_back({candidate.unit_index,
                                  candidate.order->source_offset, candidate.side,
                                  candidate.order->world_position,
                                  *candidate.transform});
    }
    return preview;
}

} // namespace ac5
