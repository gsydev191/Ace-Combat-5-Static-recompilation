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
    constexpr std::uint32_t height_cells = Ac5MissionTerrain::height_width - 1u;
    constexpr std::uint32_t tile_last = Ac5MissionTerrain::tile_width - 1u;
    const std::uint64_t expected_heights =
        static_cast<std::uint64_t>(Ac5MissionTerrain::height_width) *
        Ac5MissionTerrain::height_width;
    const std::uint64_t expected_tiles =
        static_cast<std::uint64_t>(Ac5MissionTerrain::tile_width) *
        Ac5MissionTerrain::tile_width;
    if (terrain.elevations.size() != expected_heights ||
        terrain.surface.map_width != Ac5MissionTerrain::tile_width ||
        terrain.surface.map_height != Ac5MissionTerrain::tile_width ||
        terrain.surface.map_entries.size() != expected_tiles ||
        !std::isfinite(world_x) || !std::isfinite(world_z)) {
        error = "AC5 mission terrain query input or decoded dimensions are invalid";
        return false;
    }

    const double unwrapped_grid_x =
        (world_x - Ac5MissionTerrain::height_origin_xz_world_units) /
        Ac5MissionTerrain::height_spacing_world_units;
    const double unwrapped_grid_z =
        (world_z - Ac5MissionTerrain::height_origin_xz_world_units) /
        Ac5MissionTerrain::height_spacing_world_units;
    if (!std::isfinite(unwrapped_grid_x) || !std::isfinite(unwrapped_grid_z)) {
        error = "AC5 mission terrain query position exceeds the grid range";
        return false;
    }
    // The generated reader wraps cell indices across its 512-cell terrain
    // domain; retain the fractional position within the wrapped cell.
    const auto wrap_grid_coordinate = [](double coordinate) {
        constexpr double period =
            static_cast<double>(Ac5MissionTerrain::height_width - 1u);
        double wrapped = std::fmod(coordinate, period);
        if (wrapped < 0.0) wrapped += period;
        if (wrapped >= period) wrapped = 0.0;
        return wrapped;
    };
    const double grid_x = wrap_grid_coordinate(unwrapped_grid_x);
    const double grid_z = wrap_grid_coordinate(unwrapped_grid_z);

    const std::uint32_t tile_x = std::min(
        static_cast<std::uint32_t>(grid_x / 2.0), tile_last);
    const std::uint32_t tile_z = std::min(
        static_cast<std::uint32_t>(grid_z / 2.0), tile_last);
    const std::uint16_t tile_entry = terrain.surface.map_entries[
        static_cast<std::size_t>(tile_z) * Ac5MissionTerrain::tile_width + tile_x];
    if ((tile_entry & 0x0800u) != 0u) {
        world_y = 0.0f;
        return true;
    }

    const bool finer_half_cell_path = (tile_entry & 0x0400u) != 0u;
    if (finer_half_cell_path) {
        error = "AC5 0x400 terrain query still requires raw-pool address translation";
        return false;
    }

    const std::uint32_t cell_x = std::min(
        static_cast<std::uint32_t>(grid_x), height_cells - 1u);
    const std::uint32_t cell_z = std::min(
        static_cast<std::uint32_t>(grid_z), height_cells - 1u);
    const double u = grid_x - cell_x;
    const double v = grid_z - cell_z;
    const auto at = [&](std::uint32_t x, std::uint32_t z) {
        return static_cast<double>(terrain.elevations[
            static_cast<std::size_t>(z) * Ac5MissionTerrain::height_width + x]);
    };
    const double top_left = at(cell_x, cell_z);
    const double top_right = at(cell_x + 1u, cell_z);
    const double bottom_left = at(cell_x, cell_z + 1u);
    const double bottom_right = at(cell_x + 1u, cell_z + 1u);
    if (!std::isfinite(top_left) || !std::isfinite(top_right) ||
        !std::isfinite(bottom_left) || !std::isfinite(bottom_right)) {
        error = "AC5 mission terrain query cell contains a non-finite height";
        return false;
    }

    double result;
    if (((cell_x ^ cell_z) & 1u) == 0u) {
        result = u >= v
            ? top_left + u * (top_right - top_left) +
                  v * (bottom_right - top_right)
            : top_left + v * (bottom_left - top_left) +
                  u * (bottom_right - bottom_left);
    } else if (u + v <= 1.0) {
        result = top_left + u * (top_right - top_left) +
                 v * (bottom_left - top_left);
    } else {
        result = top_right * (1.0 - v) + bottom_left * (1.0 - u) +
                 bottom_right * (u + v - 1.0);
    }
    if (!std::isfinite(result) ||
        std::abs(result) > std::numeric_limits<float>::max()) {
        error = "AC5 mission terrain query height exceeds the native float range";
        return false;
    }
    world_y = static_cast<float>(result);
    return true;
}

} // namespace ac5
