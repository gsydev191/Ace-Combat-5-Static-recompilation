#include "mission_terrain.hpp"

#include "utf8_path.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <new>
#include <stdexcept>
#include <utility>

namespace ac5 {
namespace {

constexpr std::uint32_t blocks_per_side = 16;
constexpr std::uint32_t block_side = 33;
constexpr std::size_t height_block_bytes = block_side * block_side;
constexpr std::uint32_t tile_block_side = 16;
constexpr std::size_t tile_block_bytes = tile_block_side * tile_block_side * 2u;
constexpr std::size_t selector_grid_bytes = blocks_per_side * blocks_per_side;
constexpr std::size_t max_height_pool_bytes = 256u * height_block_bytes;

bool read_height_file(const std::string& path, std::vector<std::uint8_t>& bytes,
                      std::string& error) {
    std::ifstream input(path_from_utf8(path), std::ios::binary | std::ios::ate);
    if (!input) {
        error = "cannot open AC5 terrain file: " + path;
        return false;
    }
    const std::streampos end = input.tellg();
    if (end < 0 || static_cast<std::uintmax_t>(end) > max_height_pool_bytes ||
        static_cast<std::uintmax_t>(end) >
            static_cast<std::uintmax_t>(std::numeric_limits<std::streamsize>::max())) {
        error = "AC5 terrain file exceeds the supported size: " + path;
        return false;
    }
    try {
        bytes.resize(static_cast<std::size_t>(end));
    } catch (const std::bad_alloc&) {
        error = "AC5 terrain file allocation failed";
        return false;
    } catch (const std::length_error&) {
        error = "AC5 terrain file exceeds the platform limit";
        return false;
    }
    input.seekg(0, std::ios::beg);
    if (!bytes.empty() && !input.read(reinterpret_cast<char*>(bytes.data()),
                                      static_cast<std::streamsize>(bytes.size()))) {
        error = "cannot read complete AC5 terrain file: " + path;
        return false;
    }
    return true;
}

std::uint16_t read_u16_le(const std::uint8_t* data) noexcept {
    return static_cast<std::uint16_t>(data[0]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8);
}

const std::vector<std::uint8_t>* find_file(
    std::span<const ArchiveFile> files, std::size_t index) noexcept {
    const std::vector<std::uint8_t>* found = nullptr;
    for (const ArchiveFile& file : files) {
        if (file.index != index) continue;
        if (found) return nullptr;
        found = &file.contents;
    }
    return found;
}

} // namespace

bool decode_ac5_mha_mhd_heightfield(std::span<const std::uint8_t> mha,
                                    std::span<const std::uint8_t> mhd,
                                    std::vector<float>& elevations,
                                    std::string& error) {
    error.clear();
    if (mha.size() != selector_grid_bytes) {
        error = "AC5 MHA selector grid must contain 256 block indices";
        return false;
    }
    const std::uint8_t max_block = *std::max_element(mha.begin(), mha.end());
    const std::size_t required_bytes =
        (static_cast<std::size_t>(max_block) + 1u) * height_block_bytes;
    if (mhd.size() < required_bytes ||
        mhd.size() - required_bytes > 14u) {
        error = "AC5 MHD must contain the selected 33-by-33 height blocks";
        return false;
    }

    std::vector<float> decoded;
    try {
        decoded.resize(static_cast<std::size_t>(Ac5MissionTerrain::height_width) *
                       Ac5MissionTerrain::height_width);
        for (std::uint32_t block_z = 0; block_z < blocks_per_side; ++block_z) {
            for (std::uint32_t block_x = 0; block_x < blocks_per_side; ++block_x) {
                const std::size_t grid_index =
                    static_cast<std::size_t>(block_z) * blocks_per_side + block_x;
                const std::size_t source_block =
                    static_cast<std::size_t>(mha[grid_index]) * height_block_bytes;
                for (std::uint32_t row = 0; row < block_side; ++row) {
                    const std::size_t source = source_block + row * block_side;
                    const std::size_t destination =
                        static_cast<std::size_t>(block_z * 32u + row) *
                            Ac5MissionTerrain::height_width + block_x * 32u;
                    for (std::uint32_t column = 0; column < block_side; ++column)
                        decoded[destination + column] =
                            static_cast<float>(mhd[source + column]) * 80.0f;
                }
            }
        }
    } catch (const std::bad_alloc&) {
        error = "AC5 heightfield allocation failed";
        return false;
    } catch (const std::length_error&) {
        error = "AC5 heightfield exceeds the platform limit";
        return false;
    }
    elevations = std::move(decoded);
    return true;
}

bool load_ac5_mha_mhd_heightfield(const std::string& mha_path,
                                  const std::string& mhd_path,
                                  std::vector<float>& elevations,
                                  std::string& error) {
    std::vector<std::uint8_t> mha;
    std::vector<std::uint8_t> mhd;
    if (!read_height_file(mha_path, mha, error) ||
        !read_height_file(mhd_path, mhd, error))
        return false;
    return decode_ac5_mha_mhd_heightfield(mha, mhd, elevations, error);
}

bool decode_ac5_mission_terrain(std::span<const ArchiveFile> files,
                                Ac5MissionTerrain& output,
                                std::string& error) {
    error.clear();
    const auto* height_grid = find_file(files, 0);
    const auto* height_blocks = find_file(files, 1);
    const auto* colour_grid = find_file(files, 2);
    const auto* colour_blocks = find_file(files, 3);
    const auto* colour_palette = find_file(files, 4);
    const auto* tile_grid = find_file(files, 5);
    const auto* tile_blocks = find_file(files, 6);
    const auto* tile_pool = find_file(files, 7);
    const auto* palette = find_file(files, 8);
    if (!height_grid || !height_blocks || !colour_grid || !colour_blocks ||
        !colour_palette || !tile_grid || !tile_blocks || !tile_pool || !palette) {
        error = "mission member is missing a required terrain subfile";
        return false;
    }
    if (height_grid->size() != selector_grid_bytes ||
        colour_grid->size() != selector_grid_bytes ||
        tile_grid->size() != blocks_per_side * blocks_per_side ||
        tile_blocks->size() % tile_block_bytes != 0u ||
        colour_palette->empty() || colour_palette->size() % 4u != 0u) {
        error = "mission terrain block grids have invalid dimensions";
        return false;
    }

    const std::uint8_t max_tile_block =
        *std::max_element(tile_grid->begin(), tile_grid->end());
    const std::uint8_t max_colour_block =
        *std::max_element(colour_grid->begin(), colour_grid->end());
    const std::uint64_t tile_bytes_required =
        (static_cast<std::uint64_t>(max_tile_block) + 1u) * tile_block_bytes;
    const std::uint64_t colour_bytes_required =
        (static_cast<std::uint64_t>(max_colour_block) + 1u) * height_block_bytes;
    if (tile_bytes_required > tile_blocks->size()) {
        error = "mission terrain grid references a missing block";
        return false;
    }
    if (colour_bytes_required > colour_blocks->size()) {
        error = "mission terrain colour grid references a missing block";
        return false;
    }

    Ac5MissionTerrain decoded;
    if (!decode_ac5_mha_mhd_heightfield(*height_grid, *height_blocks,
                                        decoded.elevations, error))
        return false;
    std::vector<std::uint8_t> map_entries;
    try {
        decoded.height_block_selectors = *height_grid;
        decoded.height_block_pool = *height_blocks;
        decoded.vertex_colours_rgba.resize(
            static_cast<std::size_t>(Ac5MissionTerrain::height_width) *
            Ac5MissionTerrain::height_width);
        map_entries.resize(static_cast<std::size_t>(Ac5MissionTerrain::tile_width) *
                           Ac5MissionTerrain::tile_width * 2u);
        for (std::uint32_t block_z = 0; block_z < blocks_per_side; ++block_z) {
            for (std::uint32_t block_x = 0; block_x < blocks_per_side; ++block_x) {
                const std::size_t grid_index =
                    static_cast<std::size_t>(block_z) * blocks_per_side + block_x;
                const std::size_t colour_offset =
                    static_cast<std::size_t>((*colour_grid)[grid_index]) * height_block_bytes;
                for (std::uint32_t row = 0; row < block_side; ++row) {
                    const std::size_t source = colour_offset + row * block_side;
                    const std::size_t destination =
                        static_cast<std::size_t>(block_z * 32u + row) *
                            Ac5MissionTerrain::height_width + block_x * 32u;
                    for (std::uint32_t column = 0; column < block_side; ++column) {
                        const std::uint8_t palette_index = (*colour_blocks)[source + column];
                        const std::size_t palette_offset =
                            static_cast<std::size_t>(palette_index) * 4u;
                        if (palette_offset + 4u > colour_palette->size()) {
                            error = "mission terrain colour index exceeds its palette";
                            return false;
                        }
                        std::array<std::uint8_t, 4> colour{};
                        for (std::size_t channel = 0; channel < colour.size(); ++channel)
                            colour[channel] = static_cast<std::uint8_t>(std::min(
                                static_cast<unsigned>((*colour_palette)[palette_offset + channel]) * 2u,
                                255u));
                        decoded.vertex_colours_rgba[destination + column] = colour;
                    }
                }

                const std::size_t tile_offset =
                    static_cast<std::size_t>((*tile_grid)[grid_index]) * tile_block_bytes;
                for (std::uint32_t row = 0; row < tile_block_side; ++row) {
                    const std::size_t source = tile_offset + row * tile_block_side * 2u;
                    const std::size_t destination =
                        (static_cast<std::size_t>(block_z * tile_block_side + row) *
                             Ac5MissionTerrain::tile_width +
                         block_x * tile_block_side) * 2u;
                    for (std::uint32_t column = 0; column < tile_block_side; ++column) {
                        const std::uint16_t entry = read_u16_le(
                            tile_blocks->data() + source + column * 2u);
                        const std::size_t output_offset = destination + column * 2u;
                        map_entries[output_offset] = static_cast<std::uint8_t>(entry);
                        map_entries[output_offset + 1u] =
                            static_cast<std::uint8_t>(entry >> 8);
                    }
                }
            }
        }
    } catch (const std::bad_alloc&) {
        error = "mission terrain allocation failed";
        return false;
    } catch (const std::length_error&) {
        error = "mission terrain exceeds the platform limit";
        return false;
    }

    if (!decode_ac5_map_texture_set(map_entries, *tile_pool, *palette,
                                    decoded.surface, error))
        return false;
    output = std::move(decoded);
    return true;
}

bool sample_ac5_mission_terrain_height(const Ac5MissionTerrain& terrain,
                                       double world_x, double world_z,
                                       float& world_y, std::string& error) {
    error.clear();
    constexpr std::size_t height_block_bytes = 33u * 33u;
    constexpr std::size_t selector_count = 16u * 16u;
    constexpr std::size_t expected_tiles =
        static_cast<std::size_t>(Ac5MissionTerrain::tile_width) *
        Ac5MissionTerrain::tile_width;
    if (terrain.height_block_selectors.size() != selector_count ||
        terrain.height_block_pool.empty() ||
        terrain.surface.map_width != Ac5MissionTerrain::tile_width ||
        terrain.surface.map_height != Ac5MissionTerrain::tile_width ||
        terrain.surface.map_entries.size() != expected_tiles ||
        !std::isfinite(world_x) || !std::isfinite(world_z)) {
        error = "AC5 mission terrain query input or decoded dimensions are invalid";
        return false;
    }

    // The generated EE routine takes single-precision x/z. Inside the map,
    // the integer terrain cell is trunc((coordinate + 262144) / 1024). Its
    // two out-of-range branches retain the edge 32-cell band and wrap within
    // that band; they do not wrap the whole 512-cell terrain.
    const float x = static_cast<float>(world_x);
    const float z = static_cast<float>(world_z);
    const float x_offset = x - Ac5MissionTerrain::height_origin_xz_world_units;
    const float z_offset = z - Ac5MissionTerrain::height_origin_xz_world_units;
    constexpr float cells_per_world_unit = 1.0f / 1024.0f;
    const auto get_cell = [](float coordinate, float offset,
                             std::uint32_t& cell) {
        const float scaled = offset * cells_per_world_unit;
        constexpr float int32_upper_exclusive = 2147483648.0f;
        if (!std::isfinite(scaled) || scaled < -int32_upper_exclusive ||
            scaled >= int32_upper_exclusive) return false;
        if (coordinate >= 262144.0f) {
            cell = static_cast<std::uint32_t>(
                (static_cast<std::int32_t>(scaled) & 31) + 480);
        } else if (coordinate < -262144.0f) {
            cell = static_cast<std::uint32_t>(static_cast<std::int32_t>(scaled) & 31);
        } else {
            cell = static_cast<std::uint32_t>(
                static_cast<std::int32_t>(offset) >> 10);
        }
        return true;
    };
    std::uint32_t cell_x = 0, cell_z = 0;
    if (!std::isfinite(x) || !std::isfinite(z) ||
        !get_cell(x, x_offset, cell_x) || !get_cell(z, z_offset, cell_z)) {
        error = "AC5 mission terrain query position exceeds the grid range";
        return false;
    }
    const std::uint32_t local_x = cell_x & 0x1fu;
    const std::uint32_t local_z = cell_z & 0x1fu;
    const std::size_t block_index =
        static_cast<std::size_t>(cell_z >> 5) * 16u + (cell_x >> 5);
    const std::uint8_t height_block = terrain.height_block_selectors[block_index];
    const std::size_t height_block_offset =
        static_cast<std::size_t>(height_block) * height_block_bytes;
    const std::size_t tile_x = cell_x >> 1;
    const std::size_t tile_z = cell_z >> 1;
    const std::uint16_t tile_entry = terrain.surface.map_entries[
        tile_z * Ac5MissionTerrain::tile_width + tile_x];
    if ((tile_entry & 0x0800u) != 0u) {
        world_y = 0.0f;
        return true;
    }
    if (height_block_offset > terrain.height_block_pool.size() ||
        height_block_bytes > terrain.height_block_pool.size() - height_block_offset) {
        error = "AC5 mission terrain height selector references a missing block";
        return false;
    }

    const std::size_t sample_base = height_block_offset;
    const auto at = [&](std::uint32_t sample_x, std::uint32_t sample_z) {
        return static_cast<float>(terrain.height_block_pool[
            sample_base + static_cast<std::size_t>(sample_z) * 33u + sample_x]);
    };
    float height = 0.0f;
    if ((tile_entry & 0x0400u) != 0u) {
        // The recovered flag selects a two-cell (2048-unit) triangle and
        // reads only even height vertices. Its addressing differs from the
        // regular one-cell branch.
        const float dx = x_offset - static_cast<float>(cell_x & 0x1feu) * 1024.0f;
        const float dz = z_offset - static_cast<float>(cell_z & 0x1feu) * 1024.0f;
        const std::uint32_t sx = cell_x & 0x1eu;
        const std::uint32_t sz = cell_z & 0x1eu;
        const float u = dx * (1.0f / 2048.0f);
        const float v = dz * (1.0f / 2048.0f);
        if (dx - dz < 0.0f) {
            const float bottom_left = at(sx, sz + 2u);
            height = (at(sx + 2u, sz + 2u) - bottom_left) * u + bottom_left +
                (at(sx, sz) - bottom_left) * (1.0f - v);
        } else {
            const float top_left = at(sx, sz);
            const float top_right = at(sx + 2u, sz);
            height = (top_right - top_left) * u + top_left +
                (at(sx + 2u, sz + 2u) - top_right) * v;
        }
    } else {
        const float dx = x_offset - static_cast<float>(cell_x) * 1024.0f;
        const float dz = z_offset - static_cast<float>(cell_z) * 1024.0f;
        const float u = dx * (1.0f / 1024.0f);
        const float v = dz * (1.0f / 1024.0f);
        const float top_left = at(local_x, local_z);
        if (((local_x ^ local_z) & 1u) != 0u) {
            if (dx + dz > 1024.0f) {
                const float bottom_left = at(local_x, local_z + 1u);
                const float bottom_right = at(local_x + 1u, local_z + 1u);
                const float top_right = at(local_x + 1u, local_z);
                height = (bottom_right - bottom_left) * u + bottom_left +
                    (top_right - bottom_right) * (1.0f - v);
            } else {
                height = (at(local_x + 1u, local_z) - top_left) * u + top_left +
                    (at(local_x, local_z + 1u) - top_left) * v;
            }
        } else if (dx - dz < 0.0f) {
            const float bottom_left = at(local_x, local_z + 1u);
            height = (at(local_x + 1u, local_z + 1u) - bottom_left) * u +
                bottom_left + (top_left - bottom_left) * (1.0f - v);
        } else {
            const float top_right = at(local_x + 1u, local_z);
            height = (top_right - top_left) * u + top_left +
                (at(local_x + 1u, local_z + 1u) - top_right) * v;
        }
    }
    if (!std::isfinite(height)) {
        error = "AC5 mission terrain query produced a non-finite height";
        return false;
    }
    world_y = height * 80.0f;
    return true;
}

bool trace_ac5_mission_terrain_mesh(
    const Ac5MissionTerrain& terrain, const std::array<double, 3>& from,
    const std::array<double, 3>& to, Ac5MissionTerrainMeshHit& hit,
    std::string& error) {
    error.clear();
    Ac5MissionTerrainMeshHit staged_hit;
    constexpr std::uint32_t side = Ac5MissionTerrain::height_width;
    constexpr double origin = Ac5MissionTerrain::height_origin_xz_world_units;
    constexpr double spacing = Ac5MissionTerrain::height_spacing_world_units;
    const std::size_t expected_samples = static_cast<std::size_t>(side) * side;
    const std::size_t expected_tiles =
        static_cast<std::size_t>(Ac5MissionTerrain::tile_width) *
        Ac5MissionTerrain::tile_width;
    if (terrain.elevations.size() != expected_samples ||
        terrain.surface.map_entries.size() != expected_tiles) {
        error = "AC5 mission terrain mesh dimensions are invalid";
        return false;
    }
    for (double value : from)
        if (!std::isfinite(value)) {
            error = "AC5 mission terrain trace start is non-finite";
            return false;
        }
    for (double value : to)
        if (!std::isfinite(value)) {
            error = "AC5 mission terrain trace end is non-finite";
            return false;
        }

    const double half = spacing * (side - 1u) * 0.5;
    const double min_x = std::max(-half, std::min(from[0], to[0]));
    const double max_x = std::min(half, std::max(from[0], to[0]));
    const double min_z = std::max(-half, std::min(from[2], to[2]));
    const double max_z = std::min(half, std::max(from[2], to[2]));
    if (min_x > max_x || min_z > max_z) {
        hit = staged_hit;
        return true;
    }
    const auto cell_index = [half](double coordinate) {
        const double grid = std::clamp((coordinate + half) / spacing,
                                       0.0, static_cast<double>(side - 1u));
        return std::min(static_cast<std::uint32_t>(std::floor(grid)), side - 2u);
    };
    const std::uint32_t x_begin = cell_index(min_x);
    const std::uint32_t x_end = cell_index(max_x);
    const std::uint32_t z_begin = cell_index(min_z);
    const std::uint32_t z_end = cell_index(max_z);
    const auto vertex = [&](std::uint32_t x, std::uint32_t z) {
        const std::size_t tile = static_cast<std::size_t>(z >> 1u) *
            Ac5MissionTerrain::tile_width + (x >> 1u);
        const bool zero_height =
            (terrain.surface.map_entries[tile] & 0x0800u) != 0u;
        const double y = zero_height ? 0.0 : terrain.elevations[
            static_cast<std::size_t>(z) * side + x];
        return std::array<double, 3>{origin + x * spacing, y,
                                     origin + z * spacing};
    };
    const std::array<double, 3> direction{
        to[0] - from[0], to[1] - from[1], to[2] - from[2]};
    for (double value : direction)
        if (!std::isfinite(value)) {
            error = "AC5 mission terrain trace segment exceeds the numeric range";
            return false;
        }
    const auto subtract = [](const auto& lhs, const auto& rhs) {
        return std::array<double, 3>{lhs[0] - rhs[0], lhs[1] - rhs[1],
                                     lhs[2] - rhs[2]};
    };
    const auto cross = [](const auto& lhs, const auto& rhs) {
        return std::array<double, 3>{
            lhs[1] * rhs[2] - lhs[2] * rhs[1],
            lhs[2] * rhs[0] - lhs[0] * rhs[2],
            lhs[0] * rhs[1] - lhs[1] * rhs[0]};
    };
    const auto dot = [](const auto& lhs, const auto& rhs) {
        return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
    };
    const auto triangle = [&](const auto& a, const auto& b, const auto& c) {
        const auto edge1 = subtract(b, a);
        const auto edge2 = subtract(c, a);
        const auto h = cross(direction, edge2);
        const double determinant = dot(edge1, h);
        if (std::abs(determinant) < 1e-12) return;
        const auto offset = subtract(from, a);
        const double u = dot(offset, h) / determinant;
        if (u < -1e-10 || u > 1.0 + 1e-10) return;
        const auto q = cross(offset, edge1);
        const double v = dot(direction, q) / determinant;
        if (v < -1e-10 || u + v > 1.0 + 1e-10) return;
        const double fraction = dot(edge2, q) / determinant;
        if (fraction < 0.0 || fraction > 1.0 ||
            (staged_hit.hit && fraction >= staged_hit.fraction)) return;
        const auto normal = cross(edge1, edge2);
        const double normal_length = std::sqrt(dot(normal, normal));
        if (!(normal_length > 0.0) || !std::isfinite(normal_length)) return;
        staged_hit.hit = true;
        staged_hit.fraction = fraction;
        staged_hit.normal_y = normal[1] / normal_length;
        for (std::size_t axis = 0; axis < 3; ++axis)
            staged_hit.point[axis] = from[axis] + fraction * direction[axis];
        const std::uint32_t hit_x = cell_index(staged_hit.point[0]);
        const std::uint32_t hit_z = cell_index(staged_hit.point[2]);
        const std::size_t tile = static_cast<std::size_t>(hit_z >> 1u) *
            Ac5MissionTerrain::tile_width + (hit_x >> 1u);
        staged_hit.kind = (terrain.surface.map_entries[tile] & 0x0800u) != 0u ? 2 : 0;
    };

    for (std::uint32_t z = z_begin; z <= z_end; ++z) {
        for (std::uint32_t x = x_begin; x <= x_end; ++x) {
            const auto a = vertex(x, z);
            const auto b = vertex(x + 1u, z);
            const auto c = vertex(x, z + 1u);
            const auto d = vertex(x + 1u, z + 1u);
            if (!std::isfinite(a[1]) || !std::isfinite(b[1]) ||
                !std::isfinite(c[1]) || !std::isfinite(d[1])) {
                error = "AC5 mission terrain trace encountered a non-finite vertex";
                return false;
            }
            if (((x ^ z) & 1u) == 0u) {
                triangle(a, c, d);
                triangle(a, d, b);
            } else {
                triangle(a, c, b);
                triangle(b, c, d);
            }
        }
    }
    hit = staged_hit;
    return true;
}

} // namespace ac5
