#pragma once

#include "scene.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace ac5 {

struct Ac5MissionTerrain;

struct Ac5TerrainPatchVertex {
    Vec3 position;
    float clip_w = 0.0f;
};

// Apply the generated ten-record terrain patch's five camera-depth edge
// morphs. Record 9 duplicates record 1 for the final neighbor pair.
[[nodiscard]] bool morph_ac5_terrain_patch(
    std::span<Ac5TerrainPatchVertex> patch, float clip_w_min,
    float clip_w_max, std::string& error);

// Decode the signed 32-bit height lanes used by the generated terrain vertex
// path. Retail world height is the signed integer value multiplied by 80.
[[nodiscard]] bool decode_ac5_terrain_height_lanes(
    std::span<const std::uint32_t> lane_bits, std::vector<float>& elevations,
    std::string& error);

// Build a regular terrain mesh from row-major world-space elevation samples.
// The resulting triangle winding and vertex normals face +Y. Spacing is the
// horizontal distance between adjacent samples; heights are not rescaled.
[[nodiscard]] bool build_heightfield_mesh(
    std::span<const float> elevations, std::uint32_t width,
    std::uint32_t height, Vec3 origin, float spacing_x, float spacing_z,
    MeshData& mesh, std::string& error);

// Build a lower-density full-field mesh while preserving the source bounds.
// A stride of one is equivalent to build_heightfield_mesh.
[[nodiscard]] bool build_heightfield_mesh_lod(
    std::span<const float> elevations, std::uint32_t width,
    std::uint32_t height, Vec3 origin, float spacing_x, float spacing_z,
    std::uint32_t sample_stride, MeshData& mesh, std::string& error);

// Build mission-scale terrain with atlas UVs. Each two-by-two group of
// height cells uses its mission tile, with per-tile vertices at boundaries.
[[nodiscard]] bool build_ac5_mission_terrain_mesh(
    const Ac5MissionTerrain& terrain, MeshData& mesh, std::string& error);

// Query the piecewise-planar surface using the same cell diagonal as the
// generated mesh. Returns false for invalid input or a position outside the
// finite heightfield bounds; output is unchanged on failure.
[[nodiscard]] bool sample_heightfield_surface(
    std::span<const float> elevations, std::uint32_t width,
    std::uint32_t height, Vec3 origin, float spacing_x, float spacing_z,
    float world_x, float world_z, float& world_y, std::string& error);

} // namespace ac5
