#include "map_texture.hpp"

#include "gs_image.hpp"
#include "utf8_path.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <new>
#include <stdexcept>
#include <string>
#include <utility>

namespace ac5 {
namespace {

constexpr std::uint32_t tile_size = 64;
constexpr std::uint32_t atlas_columns = 32;
constexpr std::uint32_t atlas_gutter = 4;
constexpr std::uint32_t atlas_tile_extent = tile_size + atlas_gutter * 2u;
constexpr std::uint32_t max_tiles = 1024;
constexpr std::size_t mtd_header_size = 32;
constexpr std::size_t max_map_file_bytes = 16u * 1024u * 1024u;
constexpr std::size_t tile_bytes[4] = {4096, 1024, 256, 64};

bool read_map_file(const std::string& path, std::vector<std::uint8_t>& bytes,
                   std::string& error) {
    std::ifstream input(path_from_utf8(path), std::ios::binary | std::ios::ate);
    if (!input) {
        error = "cannot open AC5 map texture file: " + path;
        return false;
    }
    const std::streampos end = input.tellg();
    if (end < 0 || static_cast<std::uintmax_t>(end) > max_map_file_bytes ||
        static_cast<std::uintmax_t>(end) >
            static_cast<std::uintmax_t>(std::numeric_limits<std::streamsize>::max())) {
        error = "AC5 map texture file size is outside the supported range: " + path;
        return false;
    }
    try {
        bytes.resize(static_cast<std::size_t>(end));
    } catch (const std::bad_alloc&) {
        error = "AC5 map texture file allocation failed";
        return false;
    } catch (const std::length_error&) {
        error = "AC5 map texture file exceeds the platform limit";
        return false;
    }
    input.seekg(0, std::ios::beg);
    if (!bytes.empty() && !input.read(reinterpret_cast<char*>(bytes.data()),
                                      static_cast<std::streamsize>(bytes.size()))) {
        error = "cannot read complete AC5 map texture file: " + path;
        return false;
    }
    return true;
}

bool read_u32_le(std::span<const std::uint8_t> data, std::size_t offset,
                 std::uint32_t& value) noexcept {
    if (offset > data.size() || data.size() - offset < 4) return false;
    value = static_cast<std::uint32_t>(data[offset]) |
            (static_cast<std::uint32_t>(data[offset + 1]) << 8) |
            (static_cast<std::uint32_t>(data[offset + 2]) << 16) |
            (static_cast<std::uint32_t>(data[offset + 3]) << 24);
    return true;
}

std::uint32_t read_u16_le(std::span<const std::uint8_t> data,
                          std::size_t offset) noexcept {
    return static_cast<std::uint32_t>(data[offset]) |
           (static_cast<std::uint32_t>(data[offset + 1]) << 8);
}

float checked_float(double value) noexcept {
    if (!std::isfinite(value) ||
        std::abs(value) > std::numeric_limits<float>::max())
        return std::numeric_limits<float>::quiet_NaN();
    return static_cast<float>(value);
}

} // namespace

bool decode_ac5_map_texture_set(std::span<const std::uint8_t> mti,
                                std::span<const std::uint8_t> mtd,
                                std::span<const std::uint8_t> mtc,
                                Ac5MapTextureSet& output,
                                std::string& error) {
    error.clear();
    std::uint32_t tile_count = 0;
    std::uint32_t tile_data_offset = 0;
    std::uint32_t level1_offset = 0;
    std::uint32_t level2_offset = 0;
    std::uint32_t level3_offset = 0;
    if (mti.empty() || mti.size() % 2u != 0u) {
        error = "AC5 MTI size must contain complete 16-bit entries";
        return false;
    }
    if (mtd.size() < mtd_header_size ||
        !read_u32_le(mtd, 0, tile_count) ||
        !read_u32_le(mtd, 4, tile_data_offset) ||
        !read_u32_le(mtd, 8, level1_offset) ||
        !read_u32_le(mtd, 12, level2_offset) ||
        !read_u32_le(mtd, 16, level3_offset) ||
        tile_count == 0 || tile_count > max_tiles) {
        error = "AC5 MTD tile header is invalid or unsupported";
        return false;
    }
    const std::uint64_t expected_level1 = mtd_header_size +
        static_cast<std::uint64_t>(tile_count) * tile_bytes[0];
    const std::uint64_t expected_level2 = expected_level1 +
        static_cast<std::uint64_t>(tile_count) * tile_bytes[1];
    const std::uint64_t expected_level3 = expected_level2 +
        static_cast<std::uint64_t>(tile_count) * tile_bytes[2];
    const std::uint64_t expected_size = expected_level3 +
        static_cast<std::uint64_t>(tile_count) * tile_bytes[3];
    if (tile_data_offset != mtd_header_size ||
        level1_offset != expected_level1 || level2_offset != expected_level2 ||
        level3_offset != expected_level3 || expected_size > mtd.size()) {
        error = "AC5 MTD mip offsets or payload size do not match the indexed tile layout";
        return false;
    }
    if (mtc.size() != 256u * 4u) {
        error = "AC5 MTC palette must contain 256 RGBA32 entries";
        return false;
    }

    const std::size_t entry_count = mti.size() / 2u;
    if (entry_count > std::numeric_limits<std::uint32_t>::max()) {
        error = "AC5 MTI contains more entries than native dimensions support";
        return false;
    }
    std::uint32_t decoded_map_width = 64;
    const std::uint64_t square_side = static_cast<std::uint64_t>(
        std::sqrt(static_cast<double>(entry_count)));
    if (square_side * square_side == entry_count && square_side >= 2u &&
        square_side <= std::numeric_limits<std::uint32_t>::max())
        decoded_map_width = static_cast<std::uint32_t>(square_side);
    if (entry_count % decoded_map_width != 0u) {
        error = "AC5 MTI size does not form complete rows";
        return false;
    }
    Ac5MapTextureSet decoded;
    decoded.map_width = decoded_map_width;
    decoded.map_height = static_cast<std::uint32_t>(entry_count / decoded_map_width);
    decoded.atlas_columns = atlas_columns;
    decoded.tile_size = tile_size;
    decoded.atlas_gutter = atlas_gutter;
    decoded.atlas_tile_extent = atlas_tile_extent;
    decoded.tile_count = tile_count;
    const std::uint32_t atlas_rows =
        (tile_count + atlas_columns - 1u) / atlas_columns;
    decoded.atlas.width = atlas_columns * atlas_tile_extent;
    decoded.atlas.height = atlas_rows * atlas_tile_extent;
    const std::uint64_t atlas_bytes =
        static_cast<std::uint64_t>(decoded.atlas.width) * decoded.atlas.height * 4u;
    if (atlas_bytes > std::numeric_limits<std::size_t>::max()) {
        error = "AC5 MTD atlas dimensions exceed native limits";
        return false;
    }

    try {
        decoded.atlas.rgba.resize(static_cast<std::size_t>(atlas_bytes));
        decoded.map_entries.resize(entry_count);
        const PaletteView palette{mtc, 0, PixelFormat::Rgba32};
        for (std::size_t i = 0; i < entry_count; ++i) {
            const std::uint16_t entry = static_cast<std::uint16_t>(read_u16_le(mti, i * 2u));
            decoded.map_entries[i] = entry;
            if ((entry & 0x03FFu) >= tile_count) {
                error = "AC5 MTI references a tile outside the MTD tile table";
                return false;
            }
        }

        for (std::uint32_t tile = 0; tile < tile_count; ++tile) {
            const std::uint64_t tile_offset = mtd_header_size +
                static_cast<std::uint64_t>(tile) * tile_bytes[0];
            const TiledImageView image{
                mtd, static_cast<std::uint32_t>(tile_offset), tile_size,
                PixelFormat::Index8};
            TextureData pixels;
            if (!decode_tiled_image(image, &palette, tile_size, tile_size,
                                    pixels, error))
                return false;

            const std::uint32_t atlas_x = (tile % atlas_columns) * atlas_tile_extent;
            const std::uint32_t atlas_y = (tile / atlas_columns) * atlas_tile_extent;
            for (std::uint32_t y = 0; y < atlas_tile_extent; ++y) {
                const std::uint32_t source_y = std::clamp(
                    y, atlas_gutter, atlas_gutter + tile_size - 1u) - atlas_gutter;
                for (std::uint32_t x = 0; x < atlas_tile_extent; ++x) {
                    const std::uint32_t source_x = std::clamp(
                        x, atlas_gutter, atlas_gutter + tile_size - 1u) - atlas_gutter;
                    const std::size_t source =
                        (static_cast<std::size_t>(source_y) * tile_size + source_x) * 4u;
                    const std::size_t destination =
                        (static_cast<std::size_t>(atlas_y + y) * decoded.atlas.width +
                         atlas_x + x) * 4u;
                    std::copy_n(pixels.rgba.begin() +
                                    static_cast<std::ptrdiff_t>(source),
                                4u, decoded.atlas.rgba.begin() +
                                    static_cast<std::ptrdiff_t>(destination));
                }
            }
        }
    } catch (const std::bad_alloc&) {
        error = "AC5 map texture allocation failed";
        return false;
    } catch (const std::length_error&) {
        error = "AC5 map texture exceeds the platform limit";
        return false;
    }

    output = std::move(decoded);
    return true;
}

bool load_ac5_map_texture_set(const std::string& mti_path,
                              const std::string& mtd_path,
                              const std::string& mtc_path,
                              Ac5MapTextureSet& output,
                              std::string& error) {
    error.clear();
    std::vector<std::uint8_t> mti;
    std::vector<std::uint8_t> mtd;
    std::vector<std::uint8_t> mtc;
    if (!read_map_file(mti_path, mti, error) ||
        !read_map_file(mtd_path, mtd, error) ||
        !read_map_file(mtc_path, mtc, error))
        return false;
    return decode_ac5_map_texture_set(mti, mtd, mtc, output, error);
}

bool build_ac5_map_texture_mesh(const Ac5MapTextureSet& map, Vec3 origin,
                                float cell_spacing_x, float cell_spacing_z,
                                MeshData& mesh, std::string& error) {
    error.clear();
    const std::uint64_t cell_count =
        static_cast<std::uint64_t>(map.map_width) * map.map_height;
    if (map.map_width == 0 || map.map_height == 0 ||
        map.map_entries.size() != cell_count || map.atlas_columns == 0 ||
        map.tile_size == 0 || !std::isfinite(origin.x) ||
        !std::isfinite(origin.y) || !std::isfinite(origin.z) ||
        !std::isfinite(cell_spacing_x) || !std::isfinite(cell_spacing_z) ||
        cell_spacing_x <= 0.0f || cell_spacing_z <= 0.0f ||
        map.atlas.width == 0 || map.atlas.height == 0 || map.tile_count == 0 ||
        map.atlas_gutter > (std::numeric_limits<std::uint32_t>::max() -
                            map.tile_size) / 2u ||
        map.atlas_tile_extent != map.tile_size + map.atlas_gutter * 2u ||
        static_cast<std::uint64_t>(map.atlas.width) !=
            static_cast<std::uint64_t>(map.atlas_columns) * map.atlas_tile_extent ||
        map.atlas.width % map.atlas_tile_extent != 0 ||
        map.atlas.height % map.atlas_tile_extent != 0 ||
        static_cast<std::uint64_t>(map.atlas.width) * map.atlas.height >
            std::numeric_limits<std::size_t>::max() / 4u ||
        map.atlas.rgba.size() != static_cast<std::uint64_t>(map.atlas.width) *
                                     map.atlas.height * 4u ||
        map.tile_count > static_cast<std::uint64_t>(map.atlas.width /
                            map.atlas_tile_extent) * (map.atlas.height /
                            map.atlas_tile_extent) ||
        cell_count > std::numeric_limits<std::uint32_t>::max() / 6u) {
        error = "AC5 map texture mesh inputs are invalid or exceed native limits";
        return false;
    }

    const std::uint64_t vertex_count = cell_count * 4u;
    if (vertex_count > std::numeric_limits<std::uint32_t>::max()) {
        error = "AC5 map texture mesh vertex count exceeds native limits";
        return false;
    }
    MeshData result;
    try {
        result.vertices.resize(static_cast<std::size_t>(vertex_count));
        result.indices.reserve(static_cast<std::size_t>(cell_count * 6u));
    } catch (const std::bad_alloc&) {
        error = "AC5 map texture mesh allocation failed";
        return false;
    } catch (const std::length_error&) {
        error = "AC5 map texture mesh exceeds the platform limit";
        return false;
    }

    for (std::uint32_t z = 0; z < map.map_height; ++z) {
        for (std::uint32_t x = 0; x < map.map_width; ++x) {
            const std::size_t cell = static_cast<std::size_t>(z) * map.map_width + x;
            const std::uint32_t tile = map.map_entries[cell] & 0x03FFu;
            if (tile >= map.tile_count) {
                error = "AC5 map texture entry exceeds the decoded tile count";
                return false;
            }
            const std::uint32_t tile_x = tile % map.atlas_columns;
            const std::uint32_t tile_y = tile / map.atlas_columns;
            if (tile_x >= map.atlas_columns ||
                tile_y >= map.atlas.height / map.atlas_tile_extent) {
                error = "AC5 map texture entry exceeds the atlas dimensions";
                return false;
            }
            const std::uint32_t tile_origin_x = tile_x * map.atlas_tile_extent;
            const std::uint32_t tile_origin_y = tile_y * map.atlas_tile_extent;
            const double u0 = (static_cast<double>(tile_origin_x + map.atlas_gutter) + 0.5) /
                              map.atlas.width;
            const double u1 = (static_cast<double>(tile_origin_x + map.atlas_gutter +
                                                   map.tile_size) - 0.5) /
                              map.atlas.width;
            const double v0 = (static_cast<double>(tile_origin_y + map.atlas_gutter) + 0.5) /
                              map.atlas.height;
            const double v1 = (static_cast<double>(tile_origin_y + map.atlas_gutter +
                                                   map.tile_size) - 0.5) /
                              map.atlas.height;
            const double left = static_cast<double>(origin.x) +
                                static_cast<double>(x) * cell_spacing_x;
            const double right = left + cell_spacing_x;
            const double top = static_cast<double>(origin.z) +
                               static_cast<double>(z) * cell_spacing_z;
            const double bottom = top + cell_spacing_z;
            const std::array<Vertex, 4> vertices{{
                {{checked_float(left), origin.y, checked_float(top)},
                 {1.0f, 1.0f, 1.0f},
                 {static_cast<float>(u0), static_cast<float>(v0)},
                 {0.0f, 1.0f, 0.0f}},
                {{checked_float(left), origin.y, checked_float(bottom)},
                 {1.0f, 1.0f, 1.0f},
                 {static_cast<float>(u0), static_cast<float>(v1)},
                 {0.0f, 1.0f, 0.0f}},
                {{checked_float(right), origin.y, checked_float(top)},
                 {1.0f, 1.0f, 1.0f},
                 {static_cast<float>(u1), static_cast<float>(v0)},
                 {0.0f, 1.0f, 0.0f}},
                {{checked_float(right), origin.y, checked_float(bottom)},
                 {1.0f, 1.0f, 1.0f},
                 {static_cast<float>(u1), static_cast<float>(v1)},
                 {0.0f, 1.0f, 0.0f}}
            }};
            for (const Vertex& vertex : vertices) {
                if (!std::isfinite(vertex.position.x) ||
                    !std::isfinite(vertex.position.z)) {
                    error = "AC5 map texture mesh coordinates exceed the native float range";
                    return false;
                }
            }
            const std::uint32_t base = static_cast<std::uint32_t>(cell * 4u);
            std::copy(vertices.begin(), vertices.end(), result.vertices.begin() + base);
            result.indices.insert(result.indices.end(), {
                base, base + 1u, base + 2u,
                base + 2u, base + 1u, base + 3u});
        }
    }

    mesh = std::move(result);
    return true;
}

} // namespace ac5
