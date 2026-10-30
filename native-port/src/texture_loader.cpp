#include "texture_loader.hpp"

#include "utf8_path.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <new>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace ac5 {

namespace {

constexpr std::size_t max_ppm_token_length = 32;

bool next_token(std::istream& input, std::string& token) {
    token.clear();
    char c = 0;
    while (input.get(c)) {
        if (std::isspace(static_cast<unsigned char>(c))) continue;
        if (c == '#') {
            input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }
        token.push_back(c);
        break;
    }
    while (input.get(c)) {
        if (std::isspace(static_cast<unsigned char>(c))) {
            if (c == '\r' && input.peek() == '\n') input.get();
            return true;
        }
        if (c == '#') {
            input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            return true;
        }
        if (token.size() == max_ppm_token_length) return false;
        token.push_back(c);
    }
    return !token.empty();
}

bool parse_u32(const std::string& token, std::uint32_t& value) {
    const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value);
    return !token.empty() && parsed.ec == std::errc{} &&
           parsed.ptr == token.data() + token.size();
}

std::uint8_t scale_sample(std::uint32_t sample, std::uint32_t maximum) {
    return static_cast<std::uint8_t>(
        (static_cast<std::uint64_t>(sample) * 255u + maximum / 2u) / maximum);
}

std::uint16_t read_u16_le(const std::uint8_t* data) {
    return static_cast<std::uint16_t>(data[0]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8);
}

bool decode_gim_impl(std::span<const std::uint8_t> bytes, TextureData& texture,
                     std::string& error, bool force_opaque) {
    if (bytes.size() < 0x20 || bytes[0] != 'G' || bytes[1] != 'I' ||
        bytes[2] != 'M' || bytes[3] != 0) {
        error = "texture is not a supported GIM image";
        return false;
    }
    const std::uint32_t format = (static_cast<std::uint32_t>(bytes[0x10]) << 24) |
        (static_cast<std::uint32_t>(bytes[0x11]) << 16) |
        (static_cast<std::uint32_t>(bytes[0x12]) << 8) | bytes[0x13];
    std::uint32_t bits_per_pixel = 0;
    if (format == 0x13 || format == 0x113 || format == 0x1b ||
        (format >= 0x200 && format <= 0x5ff && (format & 0xffu) == 0x13u))
        bits_per_pixel = 8;
    else if (format == 0x14 || format == 0x114 ||
             (format >= 0x200 && format <= 0x5ff && (format & 0xffu) == 0x14u))
        bits_per_pixel = 4;
    else {
        error = "GIM uses an unsupported or palette-only format marker";
        return false;
    }

    const std::uint32_t width = read_u16_le(bytes.data() + 0x1c);
    const std::uint32_t height = read_u16_le(bytes.data() + 0x1e);
    if (width == 0 || height == 0) {
        error = "GIM dimensions are zero";
        return false;
    }
    constexpr std::uint64_t max_rgba_bytes = 256ull * 1024ull * 1024ull;
    const std::uint64_t pixel_count = static_cast<std::uint64_t>(width) * height;
    if (pixel_count > max_rgba_bytes / 4u) {
        error = "GIM decoded image exceeds the 256 MiB limit";
        return false;
    }
    const std::size_t palette_entries = bits_per_pixel == 4 ? 16u : 256u;
    const std::size_t palette_bytes = palette_entries * 4u;
    const std::size_t packed_bytes = static_cast<std::size_t>(
        bits_per_pixel == 4 ? (pixel_count + 1u) / 2u : pixel_count);

    std::size_t pixels_offset = 0x30u + palette_bytes;
    std::size_t palette_offset = 0x30u;
    constexpr std::size_t livery_data_offset = 0x20u;
    if (packed_bytes <= bytes.size() - std::min(bytes.size(), livery_data_offset) &&
        livery_data_offset + packed_bytes + 0x10u + palette_bytes <= bytes.size() &&
        livery_data_offset + packed_bytes + 0x0cu + 4u <= bytes.size() &&
        bytes[livery_data_offset + packed_bytes + 0x0cu] == 0x10 &&
        bytes[livery_data_offset + packed_bytes + 0x0du] == 0 &&
        bytes[livery_data_offset + packed_bytes + 0x0eu] == 0x10 &&
        bytes[livery_data_offset + packed_bytes + 0x0fu] == 0) {
        pixels_offset = livery_data_offset;
        palette_offset = livery_data_offset + packed_bytes + 0x10u;
    }
    if (pixels_offset > bytes.size() || packed_bytes > bytes.size() - pixels_offset ||
        palette_offset > bytes.size() || palette_bytes > bytes.size() - palette_offset) {
        error = "GIM palette or indexed pixel payload is truncated";
        return false;
    }

    std::array<std::uint8_t, 256> palette_order{};
    for (std::size_t i = 0; i < palette_entries; ++i) palette_order[i] = static_cast<std::uint8_t>(i);
    if (bits_per_pixel == 8) {
        for (std::size_t group = 0; group < 256; group += 32) {
            for (std::size_t i = 0; i < 8; ++i) {
                palette_order[group + 8 + i] = static_cast<std::uint8_t>(group + 16 + i);
                palette_order[group + 16 + i] = static_cast<std::uint8_t>(group + 8 + i);
            }
        }
    }

    TextureData decoded;
    decoded.width = width;
    decoded.height = height;
    try {
        decoded.rgba.resize(static_cast<std::size_t>(pixel_count) * 4u);
    } catch (const std::bad_alloc&) {
        error = "GIM texture allocation failed";
        return false;
    } catch (const std::length_error&) {
        error = "GIM texture exceeds the platform's vector limit";
        return false;
    }
    for (std::uint64_t pixel = 0; pixel < pixel_count; ++pixel) {
        std::uint8_t index = 0;
        if (bits_per_pixel == 8) {
            index = bytes[pixels_offset + static_cast<std::size_t>(pixel)];
        } else {
            const std::uint8_t packed = bytes[pixels_offset + static_cast<std::size_t>(pixel / 2u)];
            index = static_cast<std::uint8_t>((pixel & 1u) == 0 ? packed >> 4 : packed & 0x0fu);
        }
        const std::size_t palette_index = palette_order[index];
        const std::size_t source = palette_offset + palette_index * 4u;
        const std::size_t target = static_cast<std::size_t>(pixel) * 4u;
        decoded.rgba[target] = bytes[source];
        decoded.rgba[target + 1] = bytes[source + 1];
        decoded.rgba[target + 2] = bytes[source + 2];
        decoded.rgba[target + 3] = force_opaque ? 255 : bytes[source + 3];
    }
    texture = std::move(decoded);
    return true;
}

bool load_tga(const std::string& path, TextureData& texture, std::string& error) {
    std::ifstream input(path_from_utf8(path), std::ios::binary);
    if (!input) {
        error = "cannot open TGA texture: " + path;
        return false;
    }
    std::array<std::uint8_t, 18> header{};
    if (!input.read(reinterpret_cast<char*>(header.data()), header.size())) {
        error = "TGA header is truncated: " + path;
        return false;
    }
    const std::uint8_t image_type = header[2];
    const std::uint32_t width = read_u16_le(header.data() + 12);
    const std::uint32_t height = read_u16_le(header.data() + 14);
    const std::uint8_t depth = header[16];
    const std::uint8_t descriptor = header[17];
    const std::uint8_t attribute_bits = descriptor & 0x0Fu;
    if (header[1] != 0 || (image_type != 2 && image_type != 10) ||
        (depth != 24 && depth != 32) || width == 0 || height == 0 ||
        (descriptor & 0xC0u) != 0 || attribute_bits > 8 ||
        (depth == 24 && attribute_bits != 0)) {
        error = "TGA must be non-interleaved 24-bit or 32-bit true-color: " + path;
        return false;
    }
    constexpr std::uint64_t max_rgba_bytes = 256ull * 1024ull * 1024ull;
    const std::uint64_t pixel_count = static_cast<std::uint64_t>(width) * height;
    if (pixel_count > max_rgba_bytes / 4u) {
        error = "TGA texture exceeds the 256 MiB decoded-size limit: " + path;
        return false;
    }
    if (header[0] != 0) {
        input.ignore(header[0]);
        if (!input) {
            error = "TGA image identifier is truncated: " + path;
            return false;
        }
    }

    TextureData decoded;
    decoded.width = width;
    decoded.height = height;
    try {
        decoded.rgba.resize(static_cast<std::size_t>(pixel_count * 4u));
    } catch (const std::bad_alloc&) {
        error = "TGA texture allocation failed: " + path;
        return false;
    } catch (const std::length_error&) {
        error = "TGA texture exceeds the platform's vector limit: " + path;
        return false;
    }
    const std::size_t pixel_bytes = depth / 8u;
    const bool top_origin = (descriptor & 0x20u) != 0;
    const bool right_origin = (descriptor & 0x10u) != 0;
    std::uint64_t output_pixel = 0;
    const auto store_pixel = [&](const std::array<std::uint8_t, 4>& bgra) {
        const std::uint32_t source_x = static_cast<std::uint32_t>(output_pixel % width);
        const std::uint32_t source_y = static_cast<std::uint32_t>(output_pixel / width);
        const std::uint32_t x = right_origin ? width - 1u - source_x : source_x;
        const std::uint32_t y = top_origin ? source_y : height - 1u - source_y;
        const std::size_t target = (static_cast<std::size_t>(y) * width + x) * 4u;
        decoded.rgba[target] = bgra[2];
        decoded.rgba[target + 1] = bgra[1];
        decoded.rgba[target + 2] = bgra[0];
        decoded.rgba[target + 3] = depth == 32 && attribute_bits != 0 ? bgra[3] : 255;
        ++output_pixel;
    };
    const auto read_pixel = [&](std::array<std::uint8_t, 4>& pixel) {
        pixel = {0, 0, 0, 255};
        return static_cast<bool>(input.read(reinterpret_cast<char*>(pixel.data()),
                                            static_cast<std::streamsize>(pixel_bytes)));
    };
    while (output_pixel < pixel_count) {
        std::uint64_t packet_size = 1;
        bool repeated = false;
        if (image_type == 10) {
            const int packet_header = input.get();
            if (packet_header == std::char_traits<char>::eof()) {
                error = "TGA RLE packet stream is truncated: " + path;
                return false;
            }
            repeated = (packet_header & 0x80) != 0;
            packet_size = static_cast<std::uint64_t>(packet_header & 0x7F) + 1u;
        }
        if (packet_size > pixel_count - output_pixel) {
            error = "TGA RLE packet exceeds the declared image dimensions: " + path;
            return false;
        }
        if (repeated) {
            std::array<std::uint8_t, 4> pixel{};
            if (!read_pixel(pixel)) {
                error = "TGA RLE pixel is truncated: " + path;
                return false;
            }
            for (std::uint64_t i = 0; i < packet_size; ++i) store_pixel(pixel);
        } else {
            for (std::uint64_t i = 0; i < packet_size; ++i) {
                std::array<std::uint8_t, 4> pixel{};
                if (!read_pixel(pixel)) {
                    error = "TGA pixel data is truncated: " + path;
                    return false;
                }
                store_pixel(pixel);
            }
        }
    }
    texture = std::move(decoded);
    return true;
}

} // namespace

bool decode_gim(std::span<const std::uint8_t> bytes, TextureData& texture,
                std::string& error, bool force_opaque) {
    error.clear();
    return decode_gim_impl(bytes, texture, error, force_opaque);
}

bool decode_gim_indexed_with_palette(std::span<const std::uint8_t> bytes,
                                    std::span<const std::uint8_t> rgba_palette,
                                    TextureData& texture, std::string& error,
                                    bool force_opaque) {
    error.clear();
    if (bytes.size() < 0x20 || bytes[0] != 'G' || bytes[1] != 'I' ||
        bytes[2] != 'M' || bytes[3] != 0 || rgba_palette.size() < 256u * 4u) {
        error = "indexed GIM or shared RGBA palette is invalid";
        return false;
    }
    const std::uint32_t format = (static_cast<std::uint32_t>(bytes[0x10]) << 24) |
        (static_cast<std::uint32_t>(bytes[0x11]) << 16) |
        (static_cast<std::uint32_t>(bytes[0x12]) << 8) | bytes[0x13];
    if (format != 0x13 && format != 0x113 && format != 0x1b &&
        !(format >= 0x200 && format <= 0x5ff && (format & 0xffu) == 0x13u)) {
        error = "PM texture is not an 8-bit indexed GIM";
        return false;
    }
    const std::uint32_t width = read_u16_le(bytes.data() + 0x1c);
    const std::uint32_t height = read_u16_le(bytes.data() + 0x1e);
    const std::uint64_t pixels = static_cast<std::uint64_t>(width) * height;
    if (width == 0 || height == 0 || pixels > (256ull * 1024ull * 1024ull) / 4u ||
        bytes.size() - 0x20u < pixels) {
        error = "PM indexed GIM dimensions or pixel payload are invalid";
        return false;
    }
    TextureData decoded;
    decoded.width = width;
    decoded.height = height;
    try {
        decoded.rgba.resize(static_cast<std::size_t>(pixels) * 4u);
    } catch (const std::bad_alloc&) {
        error = "PM texture allocation failed";
        return false;
    } catch (const std::length_error&) {
        error = "PM texture exceeds the platform's vector limit";
        return false;
    }
    for (std::size_t i = 0; i < static_cast<std::size_t>(pixels); ++i) {
        const std::uint8_t index = bytes[0x20u + i];
        const std::size_t group = index & 0xe0u;
        const std::size_t within = index & 0x1fu;
        const std::size_t palette_index = group +
            (within >= 8 && within < 16 ? within + 8 :
             within >= 16 && within < 24 ? within - 8 : within);
        const std::size_t source = palette_index * 4u;
        const std::size_t target = i * 4u;
        decoded.rgba[target] = rgba_palette[source];
        decoded.rgba[target + 1] = rgba_palette[source + 1];
        decoded.rgba[target + 2] = rgba_palette[source + 2];
        decoded.rgba[target + 3] = force_opaque ? 255 : rgba_palette[source + 3];
    }
    texture = std::move(decoded);
    return true;
}

bool load_gim(const std::string& path, TextureData& texture, std::string& error) {
    error.clear();
    std::ifstream input(path_from_utf8(path), std::ios::binary);
    if (!input) {
        error = "cannot open GIM texture: " + path;
        return false;
    }
    input.seekg(0, std::ios::end);
    const std::streamoff length = input.tellg();
    if (length <= 0 || static_cast<std::uint64_t>(length) > 512ull * 1024ull * 1024ull) {
        error = "GIM file is empty or exceeds the 512 MiB input limit: " + path;
        return false;
    }
    input.seekg(0, std::ios::beg);
    std::vector<std::uint8_t> bytes;
    try {
        bytes.resize(static_cast<std::size_t>(length));
    } catch (const std::bad_alloc&) {
        error = "GIM file allocation failed: " + path;
        return false;
    } catch (const std::length_error&) {
        error = "GIM file exceeds the platform's vector limit: " + path;
        return false;
    }
    if (!input.read(reinterpret_cast<char*>(bytes.data()), length)) {
        error = "GIM file read failed: " + path;
        return false;
    }
    if (!decode_gim(bytes, texture, error)) {
        error += ": " + path;
        return false;
    }
    return true;
}

bool load_ppm(const std::string& path, TextureData& texture, std::string& error) {
    error.clear();
    std::ifstream input(path_from_utf8(path), std::ios::binary);
    if (!input) {
        error = "cannot open PPM texture: " + path;
        return false;
    }

    std::string token;
    if (!next_token(input, token) || (token != "P3" && token != "P6")) {
        error = "texture must be an ASCII (P3) or binary (P6) PPM: " + path;
        return false;
    }
    const bool binary = token == "P6";
    std::uint32_t width = 0, height = 0, maximum = 0;
    if (!next_token(input, token) || !parse_u32(token, width) ||
        !next_token(input, token) || !parse_u32(token, height) ||
        !next_token(input, token) || !parse_u32(token, maximum) ||
        width == 0 || height == 0 || maximum == 0 || maximum > 65535) {
        error = "PPM dimensions or channel range are invalid: " + path;
        return false;
    }
    constexpr std::uint64_t max_rgba_bytes = 256ull * 1024ull * 1024ull;
    const std::uint64_t pixel_count = static_cast<std::uint64_t>(width) * height;
    if (pixel_count > max_rgba_bytes / 4u) {
        error = "PPM texture exceeds the 256 MiB decoded-size limit: " + path;
        return false;
    }

    TextureData decoded;
    decoded.width = width;
    decoded.height = height;
    try {
        decoded.rgba.resize(static_cast<std::size_t>(pixel_count * 4u));
    } catch (const std::bad_alloc&) {
        error = "PPM texture allocation failed: " + path;
        return false;
    } catch (const std::length_error&) {
        error = "PPM texture exceeds the platform's vector limit: " + path;
        return false;
    }
    if (binary) {
        const std::size_t sample_bytes = maximum < 256 ? 1u : 2u;
        constexpr std::size_t chunk_bytes = 64u * 1024u;
        std::array<std::uint8_t, chunk_bytes> chunk{};
        const std::uint64_t sample_count = pixel_count * 3u;
        const std::size_t samples_per_chunk = chunk.size() / sample_bytes;
        for (std::uint64_t sample_offset = 0; sample_offset < sample_count;) {
            const std::size_t count = static_cast<std::size_t>(std::min<std::uint64_t>(
                samples_per_chunk, sample_count - sample_offset));
            const std::size_t byte_count = count * sample_bytes;
            if (!input.read(reinterpret_cast<char*>(chunk.data()),
                            static_cast<std::streamsize>(byte_count))) {
                error = "PPM pixel data is truncated: " + path;
                return false;
            }
            for (std::size_t i = 0; i < count; ++i) {
                const std::size_t sample_at = i * sample_bytes;
                const std::uint32_t sample = sample_bytes == 1
                    ? chunk[sample_at]
                    : (static_cast<std::uint32_t>(chunk[sample_at]) << 8) |
                          chunk[sample_at + 1];
                if (sample > maximum) {
                    error = "PPM pixel sample exceeds maxval: " + path;
                    return false;
                }
                const std::uint64_t sample_index = sample_offset + i;
                const std::size_t pixel = static_cast<std::size_t>(sample_index / 3u);
                const std::size_t channel = static_cast<std::size_t>(sample_index % 3u);
                decoded.rgba[pixel * 4u + channel] = scale_sample(sample, maximum);
                if (channel == 2) decoded.rgba[pixel * 4u + 3] = 255;
            }
            sample_offset += count;
        }
    } else {
        for (std::size_t target = 0; target < decoded.rgba.size(); target += 4) {
            for (std::size_t channel = 0; channel < 3; ++channel) {
                std::uint32_t value = 0;
                if (!next_token(input, token) || !parse_u32(token, value) || value > maximum) {
                    error = "PPM pixel data is invalid or truncated: " + path;
                    return false;
                }
                decoded.rgba[target + channel] = scale_sample(value, maximum);
            }
            decoded.rgba[target + 3] = 255;
        }
    }
    texture = std::move(decoded);
    return true;
}

bool load_texture_image(const std::string& path, TextureData& texture,
                        std::string& error) {
    error.clear();
    std::string extension = path_from_utf8(path).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (extension == ".ppm" || extension == ".pnm")
        return load_ppm(path, texture, error);
    if (extension == ".tga") return load_tga(path, texture, error);
    if (extension == ".gim") return load_gim(path, texture, error);
    error = "unsupported texture image extension: " + extension;
    return false;
}

} // namespace ac5
