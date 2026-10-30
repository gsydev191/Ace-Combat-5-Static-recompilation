#include "heightfield_mesh.hpp"

#include "mission_terrain.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <new>
#include <stdexcept>
#include <utility>

namespace ac5 {
namespace {

bool finite(Vec3 value) noexcept {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

float checked_float(double value) noexcept {
    if (!std::isfinite(value) ||
        std::abs(value) > std::numeric_limits<float>::max())
        return std::numeric_limits<float>::quiet_NaN();
    return static_cast<float>(value);
}

} // namespace

bool morph_ac5_terrain_patch(std::span<Ac5TerrainPatchVertex> patch,
                             float clip_w_min, float clip_w_max,
                             std::string& error) {
    error.clear();
    if (patch.size() != 10 || !std::isfinite(clip_w_min) ||
        !std::isfinite(clip_w_max) || clip_w_max <= clip_w_min) {
        error = "AC5 terrain patch requires ten records and a finite increasing clip range";
        return false;
    }
    for (const Ac5TerrainPatchVertex& vertex : patch) {
        if (!finite(vertex.position) || !std::isfinite(vertex.clip_w)) {
            error = "AC5 terrain patch contains a non-finite position or clip value";
            return false;
        }
    }

    constexpr std::uint32_t edge_indices[5] = {0, 2, 4, 6, 8};
    constexpr std::uint32_t neighbor_indices[5][2] = {
        {1, 5}, {3, 1}, {5, 3}, {7, 5}, {9, 7}};
    std::array<float, 5> morphed_heights{};
    std::array<bool, 5> should_morph{};
    const float clip_range = clip_w_max - clip_w_min;
    if (!std::isfinite(clip_range) || clip_range <= 0.0f) {
        error = "AC5 terrain patch clip range is not representable";
        return false;
    }
    for (std::size_t edge = 0; edge < 5; ++edge) {
        const Ac5TerrainPatchVertex& vertex = patch[edge_indices[edge]];
        const float distance_from_near = vertex.clip_w - clip_w_min;
        if (std::signbit(distance_from_near)) continue;

        const float neighbor_sum =
            patch[neighbor_indices[edge][0]].position.y +
            patch[neighbor_indices[edge][1]].position.y;
        const float average = neighbor_sum * 0.5f;
        float height = vertex.position.y;
        if (!std::signbit(vertex.clip_w - clip_w_max)) {
            height = average;
        } else {
            const float weight = distance_from_near / clip_range;
            height = height + (average - height) * weight;
        }
        if (!std::isfinite(height)) {
            error = "AC5 terrain patch morph produced a non-finite height";
            return false;
        }
        morphed_heights[edge] = height;
        should_morph[edge] = true;
    }
    for (std::size_t edge = 0; edge < 5; ++edge)
        if (should_morph[edge]) patch[edge_indices[edge]].position.y = morphed_heights[edge];
    return true;
}

bool decode_ac5_terrain_height_lanes(std::span<const std::uint32_t> lane_bits,
                                    std::vector<float>& elevations,
                                    std::string& error) {
    error.clear();
    std::vector<float> decoded;
    try {
        decoded.reserve(lane_bits.size());
        for (const std::uint32_t bits : lane_bits) {
            const std::int32_t sample = std::bit_cast<std::int32_t>(bits);
            // Match the generated path's int32 -> float conversion followed
            // by a single-precision multiply, including its rounding order.
            const float height = static_cast<float>(sample) * 80.0f;
            if (!std::isfinite(height)) {
                error = "AC5 terrain height lane exceeds the native float range";
                return false;
            }
            decoded.push_back(height);
        }
    } catch (const std::bad_alloc&) {
        error = "AC5 terrain height allocation failed";
        return false;
    } catch (const std::length_error&) {
        error = "AC5 terrain height samples exceed the platform limit";
        return false;
    }
    elevations = std::move(decoded);
    return true;
}

bool build_heightfield_mesh(std::span<const float> elevations,
                            std::uint32_t width, std::uint32_t height,
                            Vec3 origin, float spacing_x, float spacing_z,
                            MeshData& mesh, std::string& error) {
    error.clear();
    if (width < 2 || height < 2 || !finite(origin) ||
        !std::isfinite(spacing_x) || !std::isfinite(spacing_z) ||
        spacing_x <= 0.0f || spacing_z <= 0.0f) {
        error = "heightfield dimensions, origin, or spacing are invalid";
        return false;
    }

    const std::uint64_t vertex_count =
        static_cast<std::uint64_t>(width) * height;
    const std::uint64_t cell_count =
        static_cast<std::uint64_t>(width - 1) * (height - 1);
    if (vertex_count > std::numeric_limits<std::uint32_t>::max() ||
        elevations.size() != vertex_count ||
        cell_count > std::numeric_limits<std::uint32_t>::max() / 6u) {
        error = "heightfield sample or mesh count exceeds native limits";
        return false;
    }

    MeshData result;
    try {
        result.vertices.resize(static_cast<std::size_t>(vertex_count));
        result.indices.reserve(static_cast<std::size_t>(cell_count * 6u));
    } catch (const std::bad_alloc&) {
        error = "heightfield mesh allocation failed";
        return false;
    } catch (const std::length_error&) {
        error = "heightfield mesh exceeds the platform limit";
        return false;
    }

    for (std::uint32_t z = 0; z < height; ++z) {
        for (std::uint32_t x = 0; x < width; ++x) {
            const std::size_t index = static_cast<std::size_t>(z) * width + x;
            const float elevation = elevations[index];
            if (!std::isfinite(elevation)) {
                error = "heightfield contains a non-finite elevation";
                return false;
            }
            const double world_x = static_cast<double>(origin.x) +
                                   static_cast<double>(x) * spacing_x;
            const double world_y = static_cast<double>(origin.y) + elevation;
            const double world_z = static_cast<double>(origin.z) +
                                   static_cast<double>(z) * spacing_z;
            const float px = checked_float(world_x);
            const float py = checked_float(world_y);
            const float pz = checked_float(world_z);
            if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
                error = "heightfield coordinates exceed the native float range";
                return false;
            }

            const std::uint32_t left = x == 0 ? x : x - 1;
            const std::uint32_t right = x + 1 < width ? x + 1 : x;
            const std::uint32_t up = z == 0 ? z : z - 1;
            const std::uint32_t down = z + 1 < height ? z + 1 : z;
            const double dx = static_cast<double>(right - left) * spacing_x;
            const double dz = static_cast<double>(down - up) * spacing_z;
            const double dhdx = dx > 0.0
                ? (static_cast<double>(elevations[static_cast<std::size_t>(z) * width + right]) -
                   elevations[static_cast<std::size_t>(z) * width + left]) / dx
                : 0.0;
            const double dhdz = dz > 0.0
                ? (static_cast<double>(elevations[static_cast<std::size_t>(down) * width + x]) -
                   elevations[static_cast<std::size_t>(up) * width + x]) / dz
                : 0.0;
            const double nx = -dhdx;
            const double ny = 1.0;
            const double nz = -dhdz;
            const double normal_length = std::sqrt(nx * nx + ny * ny + nz * nz);
            Vertex vertex{};
            vertex.position = {px, py, pz};
            vertex.color = {1.0f, 1.0f, 1.0f};
            vertex.uv = {static_cast<float>(x) / static_cast<float>(width - 1),
                         static_cast<float>(z) / static_cast<float>(height - 1)};
            vertex.normal = {static_cast<float>(nx / normal_length),
                             static_cast<float>(ny / normal_length),
                             static_cast<float>(nz / normal_length)};
            result.vertices[index] = vertex;
        }
    }

    for (std::uint32_t z = 0; z + 1 < height; ++z) {
        for (std::uint32_t x = 0; x + 1 < width; ++x) {
            const std::uint32_t top_left = z * width + x;
            const std::uint32_t top_right = top_left + 1;
            const std::uint32_t bottom_left = top_left + width;
            const std::uint32_t bottom_right = bottom_left + 1;
            result.indices.insert(result.indices.end(), {
                top_left, bottom_left, top_right,
                top_right, bottom_left, bottom_right});
        }
    }
    mesh = std::move(result);
    return true;
}

bool build_heightfield_mesh_lod(std::span<const float> elevations,
                                std::uint32_t width, std::uint32_t height,
                                Vec3 origin, float spacing_x, float spacing_z,
                                std::uint32_t sample_stride, MeshData& mesh,
                                std::string& error) {
    error.clear();
    if (sample_stride == 0 || width < 2 || height < 2 ||
        elevations.size() != static_cast<std::uint64_t>(width) * height) {
        error = "heightfield LOD dimensions, samples, or stride are invalid";
        return false;
    }
    if (sample_stride == 1)
        return build_heightfield_mesh(elevations, width, height, origin,
                                      spacing_x, spacing_z, mesh, error);

    const auto reduced_dimension = [sample_stride](std::uint32_t dimension) {
        return static_cast<std::uint32_t>(
            (static_cast<std::uint64_t>(dimension - 1) + sample_stride - 1) /
                sample_stride + 1);
    };
    const std::uint32_t reduced_width = reduced_dimension(width);
    const std::uint32_t reduced_height = reduced_dimension(height);
    const std::uint64_t reduced_count =
        static_cast<std::uint64_t>(reduced_width) * reduced_height;
    if (reduced_count > std::numeric_limits<std::uint32_t>::max()) {
        error = "heightfield LOD sample count exceeds native limits";
        return false;
    }

    std::vector<float> reduced;
    try {
        reduced.resize(static_cast<std::size_t>(reduced_count));
    } catch (const std::bad_alloc&) {
        error = "heightfield LOD allocation failed";
        return false;
    } catch (const std::length_error&) {
        error = "heightfield LOD exceeds the platform limit";
        return false;
    }
    for (std::uint32_t z = 0; z < reduced_height; ++z) {
        const std::uint32_t source_z = static_cast<std::uint32_t>(std::llround(
            static_cast<double>(z) * (height - 1) / (reduced_height - 1)));
        for (std::uint32_t x = 0; x < reduced_width; ++x) {
            const std::uint32_t source_x = static_cast<std::uint32_t>(std::llround(
                static_cast<double>(x) * (width - 1) / (reduced_width - 1)));
            reduced[static_cast<std::size_t>(z) * reduced_width + x] =
                elevations[static_cast<std::size_t>(source_z) * width + source_x];
        }
    }

    const double lod_spacing_x = static_cast<double>(spacing_x) *
                                 (width - 1) / (reduced_width - 1);
    const double lod_spacing_z = static_cast<double>(spacing_z) *
                                 (height - 1) / (reduced_height - 1);
    const float output_spacing_x = checked_float(lod_spacing_x);
    const float output_spacing_z = checked_float(lod_spacing_z);
    if (!std::isfinite(output_spacing_x) || !std::isfinite(output_spacing_z)) {
        error = "heightfield LOD spacing exceeds the native float range";
        return false;
    }
    return build_heightfield_mesh(reduced, reduced_width, reduced_height, origin,
                                  output_spacing_x, output_spacing_z, mesh, error);
}

bool build_ac5_mission_terrain_mesh(const Ac5MissionTerrain& terrain,
                                    MeshData& mesh, std::string& error) {
    error.clear();
    constexpr std::uint32_t height_side = Ac5MissionTerrain::height_width;
    constexpr std::uint32_t tile_side = Ac5MissionTerrain::tile_width;
    const Ac5MapTextureSet& surface = terrain.surface;
    const std::uint64_t expected_heights =
        static_cast<std::uint64_t>(height_side) * height_side;
    const std::uint64_t expected_tiles =
        static_cast<std::uint64_t>(tile_side) * tile_side;
    const std::uint64_t expected_vertices = expected_tiles * 9u;
    const std::uint64_t expected_indices = expected_tiles * 24u;
    if (terrain.elevations.size() != expected_heights ||
        (!terrain.vertex_colours_rgba.empty() &&
         terrain.vertex_colours_rgba.size() != expected_heights) ||
        surface.map_width != tile_side || surface.map_height != tile_side ||
        surface.map_entries.size() != expected_tiles ||
        surface.atlas_columns == 0 || surface.tile_count == 0 ||
        surface.tile_size == 0 || surface.atlas_gutter >
            (std::numeric_limits<std::uint32_t>::max() - surface.tile_size) / 2u ||
        surface.atlas_tile_extent != surface.tile_size + surface.atlas_gutter * 2u ||
        static_cast<std::uint64_t>(surface.atlas.width) !=
            static_cast<std::uint64_t>(surface.atlas_columns) *
                surface.atlas_tile_extent ||
        surface.atlas.height == 0 ||
        surface.atlas.rgba.size() != static_cast<std::uint64_t>(surface.atlas.width) *
                                         surface.atlas.height * 4u ||
        expected_vertices > std::numeric_limits<std::uint32_t>::max()) {
        error = "AC5 mission terrain dimensions or atlas layout are invalid";
        return false;
    }

    MeshData result;
    try {
        result.vertices.resize(static_cast<std::size_t>(expected_vertices));
        result.indices.reserve(static_cast<std::size_t>(expected_indices));
    } catch (const std::bad_alloc&) {
        error = "AC5 mission terrain mesh allocation failed";
        return false;
    } catch (const std::length_error&) {
        error = "AC5 mission terrain mesh exceeds the platform limit";
        return false;
    }

    const auto height_at = [&](std::uint32_t x, std::uint32_t z) {
        return terrain.elevations[static_cast<std::size_t>(z) * height_side + x];
    };
    for (std::uint32_t tile_z = 0; tile_z < tile_side; ++tile_z) {
        for (std::uint32_t tile_x = 0; tile_x < tile_side; ++tile_x) {
            const std::size_t tile_index =
                static_cast<std::size_t>(tile_z) * tile_side + tile_x;
            const std::uint16_t tile_entry = surface.map_entries[tile_index];
            const std::uint32_t tile = tile_entry & 0x03FFu;
            const bool zero_height_tile = (tile_entry & 0x0800u) != 0u;
            const std::uint32_t atlas_x = tile % surface.atlas_columns;
            const std::uint32_t atlas_y = tile / surface.atlas_columns;
            const std::uint64_t pixel_x =
                static_cast<std::uint64_t>(atlas_x) * surface.atlas_tile_extent;
            const std::uint64_t pixel_y =
                static_cast<std::uint64_t>(atlas_y) * surface.atlas_tile_extent;
            if (tile >= surface.tile_count ||
                pixel_x + surface.atlas_tile_extent > surface.atlas.width ||
                pixel_y + surface.atlas_tile_extent > surface.atlas.height) {
                error = "AC5 mission tile references an atlas cell outside its image";
                return false;
            }

            const std::uint32_t base_x = tile_x * 2u;
            const std::uint32_t base_z = tile_z * 2u;
            const std::uint32_t base_vertex =
                static_cast<std::uint32_t>(tile_index * 9u);
            for (std::uint32_t local_z = 0; local_z < 3; ++local_z) {
                for (std::uint32_t local_x = 0; local_x < 3; ++local_x) {
                    const std::uint32_t x = base_x + local_x;
                    const std::uint32_t z = base_z + local_z;
                    const std::uint32_t left = x == 0 ? x : x - 1;
                    const std::uint32_t right = x + 1 < height_side ? x + 1 : x;
                    const std::uint32_t up = z == 0 ? z : z - 1;
                    const std::uint32_t down = z + 1 < height_side ? z + 1 : z;
                    const double dx = static_cast<double>(right - left) *
                                      Ac5MissionTerrain::height_spacing_world_units;
                    const double dz = static_cast<double>(down - up) *
                                      Ac5MissionTerrain::height_spacing_world_units;
                    const double dhdx = dx > 0.0 && !zero_height_tile
                        ? (static_cast<double>(height_at(right, z)) - height_at(left, z)) / dx
                        : 0.0;
                    const double dhdz = dz > 0.0 && !zero_height_tile
                        ? (static_cast<double>(height_at(x, down)) - height_at(x, up)) / dz
                        : 0.0;
                    const double nx = -dhdx, ny = 1.0, nz = -dhdz;
                    const double normal_length = std::sqrt(nx * nx + ny * ny + nz * nz);
                    const float world_x = checked_float(
                        Ac5MissionTerrain::height_origin_xz_world_units +
                        static_cast<double>(x) *
                            Ac5MissionTerrain::height_spacing_world_units);
                    const float world_z = checked_float(
                        Ac5MissionTerrain::height_origin_xz_world_units +
                        static_cast<double>(z) *
                            Ac5MissionTerrain::height_spacing_world_units);
                    const float world_y = zero_height_tile
                        ? 0.0f : checked_float(height_at(x, z));
                    const float atlas_u = static_cast<float>(
                        (static_cast<double>(pixel_x + surface.atlas_gutter) + 0.5 +
                         static_cast<double>(local_x) *
                             (surface.tile_size - 1u) * 0.5) /
                        surface.atlas.width);
                    const float atlas_v = static_cast<float>(
                        (static_cast<double>(pixel_y + surface.atlas_gutter) + 0.5 +
                         static_cast<double>(local_z) *
                             (surface.tile_size - 1u) * 0.5) /
                        surface.atlas.height);
                    if (!std::isfinite(world_x) || !std::isfinite(world_y) ||
                        !std::isfinite(world_z) || !std::isfinite(normal_length) ||
                        normal_length <= 0.0 || !std::isfinite(atlas_u) ||
                        !std::isfinite(atlas_v)) {
                        error = "AC5 mission terrain contains an invalid height or coordinate";
                        return false;
                    }
                    Vertex vertex{};
                    vertex.position = {world_x, world_y, world_z};
                    vertex.color = {1.0f, 1.0f, 1.0f};
                    if (!terrain.vertex_colours_rgba.empty()) {
                        const auto& colour = terrain.vertex_colours_rgba[
                            static_cast<std::size_t>(z) * height_side + x];
                        vertex.color = {
                            static_cast<float>(colour[0]) / 255.0f,
                            static_cast<float>(colour[1]) / 255.0f,
                            static_cast<float>(colour[2]) / 255.0f};
                        vertex.alpha = static_cast<float>(colour[3]) / 255.0f;
                    }
                    vertex.uv = {atlas_u, atlas_v};
                    vertex.normal = {static_cast<float>(nx / normal_length),
                                     static_cast<float>(ny / normal_length),
                                     static_cast<float>(nz / normal_length)};
                    result.vertices[base_vertex + local_z * 3u + local_x] = vertex;
                }
            }

            for (std::uint32_t cell_z = 0; cell_z < 2; ++cell_z) {
                for (std::uint32_t cell_x = 0; cell_x < 2; ++cell_x) {
                    const std::uint32_t top_left =
                        base_vertex + cell_z * 3u + cell_x;
                    const std::uint32_t top_right = top_left + 1u;
                    const std::uint32_t bottom_left = top_left + 3u;
                    const std::uint32_t bottom_right = bottom_left + 1u;
                    const std::uint32_t global_x = base_x + cell_x;
                    const std::uint32_t global_z = base_z + cell_z;
                    if (((global_x ^ global_z) & 1u) == 0u) {
                        // The generated query uses the TL-to-BR diagonal on
                        // even cells and selects its triangle from u <= v.
                        result.indices.insert(result.indices.end(), {
                            top_left, bottom_left, bottom_right,
                            top_left, bottom_right, top_right});
                    } else {
                        result.indices.insert(result.indices.end(), {
                            top_left, bottom_left, top_right,
                            top_right, bottom_left, bottom_right});
                    }
                }
            }
        }
    }
    mesh = std::move(result);
    return true;
}

bool sample_heightfield_surface(std::span<const float> elevations,
                                std::uint32_t width, std::uint32_t height,
                                Vec3 origin, float spacing_x, float spacing_z,
                                float world_x, float world_z, float& world_y,
                                std::string& error) {
    error.clear();
    const std::uint64_t vertex_count =
        static_cast<std::uint64_t>(width) * height;
    if (width < 2 || height < 2 || vertex_count != elevations.size() ||
        !finite(origin) || !std::isfinite(spacing_x) || spacing_x <= 0.0f ||
        !std::isfinite(spacing_z) || spacing_z <= 0.0f ||
        !std::isfinite(world_x) || !std::isfinite(world_z)) {
        error = "heightfield query dimensions, coordinates, or spacing are invalid";
        return false;
    }
    const double grid_x = (static_cast<double>(world_x) - origin.x) / spacing_x;
    const double grid_z = (static_cast<double>(world_z) - origin.z) / spacing_z;
    if (!std::isfinite(grid_x) || !std::isfinite(grid_z) || grid_x < 0.0 ||
        grid_z < 0.0 || grid_x > static_cast<double>(width - 1) ||
        grid_z > static_cast<double>(height - 1)) {
        error = "heightfield query position is outside the terrain bounds";
        return false;
    }

    const std::uint32_t cell_x = std::min(
        static_cast<std::uint32_t>(grid_x), width - 2);
    const std::uint32_t cell_z = std::min(
        static_cast<std::uint32_t>(grid_z), height - 2);
    const double u = grid_x - cell_x;
    const double v = grid_z - cell_z;
    const std::size_t top_left_index =
        static_cast<std::size_t>(cell_z) * width + cell_x;
    const double top_left = elevations[top_left_index];
    const double top_right = elevations[top_left_index + 1];
    const double bottom_left = elevations[top_left_index + width];
    const double bottom_right = elevations[top_left_index + width + 1];
    if (!std::isfinite(top_left) || !std::isfinite(top_right) ||
        !std::isfinite(bottom_left) || !std::isfinite(bottom_right)) {
        error = "heightfield query cell contains a non-finite elevation";
        return false;
    }

    const double elevation = u + v <= 1.0
        ? top_left + u * (top_right - top_left) + v * (bottom_left - top_left)
        : top_right * (1.0 - v) + bottom_left * (1.0 - u) +
              bottom_right * (u + v - 1.0);
    const float result = checked_float(static_cast<double>(origin.y) + elevation);
    if (!std::isfinite(result)) {
        error = "heightfield query result exceeds the native float range";
        return false;
    }
    world_y = result;
    return true;
}

} // namespace ac5
