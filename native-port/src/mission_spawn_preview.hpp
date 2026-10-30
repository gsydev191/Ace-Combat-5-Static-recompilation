#pragma once

#include "mission_terrain.hpp"
#include "mission_units.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace ac5 {

struct Ac5MissionSpawnPreviewItem {
    std::size_t unit_index = 0;
    std::size_t source_offset = 0;
    std::uint8_t side = 0;
    std::array<float, 3> order_position{};
    Ac5MissionSpawnTransform transform;
};

struct Ac5MissionSpawnPreview {
    std::vector<Ac5MissionSpawnPreviewItem> spawns;
    std::size_t unresolved_orders = 0;
};

// Resolve the initial type-0 spawn orders in static mission data. Group
// references are followed through resolved initial spawns; actor references
// require member zero with a decoded zero formation offset. Other live state
// and member formation transforms are not available in this preview.
[[nodiscard]] Ac5MissionSpawnPreview build_ac5_mission_spawn_preview(
    std::span<const Ac5MissionUnit> units, const Ac5MissionTerrain& terrain);

} // namespace ac5
