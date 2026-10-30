#pragma once

#include "scene.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace ac5 {

struct Ac5MapTextureSet {
    TextureData atlas;
    std::uint32_t atlas_columns = 0;
    std::uint32_t tile_size = 64;
    std::uint32_t atlas_tile_extent = 0;
    std::uint32_t atlas_gutter = 0;
    std::uint32_t tile_count = 0;
    std::uint32_t map_width = 0;
    std::uint32_t map_height = 0;
    // Original .mti words in row-major order. The low 10 bits index the atlas;
    // 0x400/0x800 affect the generated height query, and the remaining flags
    // are preserved because their meanings are unresolved.
    std::vector<std::uint16_t> map_entries;
};

// Decode the indexed tile map from a mission's extracted .mti/.mtd/.mtc files.
// Atlas tiles are packed row-major; map entries retain their source flags.
[[nodiscard]] bool decode_ac5_map_texture_set(
    std::span<const std::uint8_t> mti, std::span<const std::uint8_t> mtd,
    std::span<const std::uint8_t> mtc, Ac5MapTextureSet& output,
    std::string& error);
[[nodiscard]] bool load_ac5_map_texture_set(
    const std::string& mti_path, const std::string& mtd_path,
    const std::string& mtc_path, Ac5MapTextureSet& output,
    std::string& error);

// Build a flat, indexed mesh that displays the .mti tile layout with atlas UVs.
// Cell dimensions and origin are presentation units, not recovered game units.
[[nodiscard]] bool build_ac5_map_texture_mesh(
    const Ac5MapTextureSet& map, Vec3 origin, float cell_spacing_x,
    float cell_spacing_z, MeshData& mesh, std::string& error);

} // namespace ac5
