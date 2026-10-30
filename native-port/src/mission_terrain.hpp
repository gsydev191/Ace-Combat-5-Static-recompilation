#pragma once

#include "data_archive.hpp"
#include "map_texture.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace ac5 {

struct Ac5MissionTerrain {
    static constexpr std::uint32_t height_width = 513;
    static constexpr std::uint32_t tile_width = 256;
    static constexpr float height_spacing_world_units = 1024.0f;
    static constexpr float height_origin_xz_world_units = -262144.0f;
    static constexpr float tile_spacing_world_units = 2048.0f;

    // Row-major mission heights in generated world units (stored byte * 80).
    std::vector<float> elevations;
    // Original selector grid and block pool are retained for direct
    // translation of generated per-block query addressing, including the
    // 0x400 path's even-coordinate sample selection.
    std::vector<std::uint8_t> height_block_selectors;
    std::vector<std::uint8_t> height_block_pool;
    // Row-major terrain tints decoded from `.mia`/`.mii`/`.mid`; channels are
    // expanded from the GS 0..128 full-scale range to RGBA8.
    std::vector<std::array<std::uint8_t, 4>> vertex_colours_rgba;
    // Indexed land texture, including tile-index words and the decoded atlas.
    Ac5MapTextureSet surface;
};

struct Ac5MissionTerrainMeshHit {
    bool hit = false;
    double fraction = 0.0;
    double normal_y = 0.0;
    int kind = 0;
    std::array<double, 3> point{};
};

// Decode the map's `.mha` block selector and `.mhd` 33-by-33 byte blocks into
// the generated 513-by-513 world-height grid.
[[nodiscard]] bool decode_ac5_mha_mhd_heightfield(
    std::span<const std::uint8_t> mha, std::span<const std::uint8_t> mhd,
    std::vector<float>& elevations, std::string& error);
[[nodiscard]] bool load_ac5_mha_mhd_heightfield(
    const std::string& mha_path, const std::string& mhd_path,
    std::vector<float>& elevations, std::string& error);

// Decode terrain layers from the decompressed DATA.PAC inner files returned
// by DataArchive::split_member. Files 0-8 supply height, color, and tile layers.
// Heights and vertex colors assemble overlapping 33x33 blocks; tile indices
// assemble independent 16x16 blocks.
[[nodiscard]] bool decode_ac5_mission_terrain(
    std::span<const ArchiveFile> files, Ac5MissionTerrain& output,
    std::string& error);

// Query the generated 0x0011BB88 regular and 0x400 even-sample terrain paths,
// plus the 0x800 zero-height class. Uses the source's 512-cell map and its
// first/last 32-cell handling for coordinates outside the map. Output is
// unchanged on error.
[[nodiscard]] bool sample_ac5_mission_terrain_height(
    const Ac5MissionTerrain& terrain, double world_x, double world_z,
    float& world_y, std::string& error);

// Trace against the same checkerboard triangles used to render mission
// terrain. `kind == 2` marks the zero-height tile class used for water. Output
// is unchanged on error.
[[nodiscard]] bool trace_ac5_mission_terrain_mesh(
    const Ac5MissionTerrain& terrain, const std::array<double, 3>& from,
    const std::array<double, 3>& to, Ac5MissionTerrainMeshHit& hit,
    std::string& error);

} // namespace ac5
