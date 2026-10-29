#pragma once

#include "scene.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace ac5 {

// Pixel storage formats used by the game's texture and framebuffer images.
// The values are the format codes stored in the asset headers, so they can be
// read straight from data files.
enum class PixelFormat : std::uint8_t {
    Rgba32 = 0,
    Rgb24 = 1,
    Rgba16 = 2,
    Rgba16S = 10,
    Index8 = 19,
    Index4 = 20,
};

[[nodiscard]] constexpr unsigned bits_per_pixel(PixelFormat f) noexcept {
    switch (f) {
    case PixelFormat::Rgba16:
    case PixelFormat::Rgba16S: return 16;
    case PixelFormat::Index8: return 8;
    case PixelFormat::Index4: return 4;
    default: return 32;
    }
}

[[nodiscard]] bool is_known_pixel_format(std::uint32_t code) noexcept;

// The tiled pixel layout used by the original image memory: pages of 8 KiB
// split into 256-byte blocks, with a fixed pixel order inside each block.
// `base` is a byte offset into `memory`; `stride` is the image row width in
// pixels (a multiple of the page width). Returned values are raw: a palette
// index for the indexed formats, otherwise the packed pixel value.
struct TiledImageView {
    std::span<const std::uint8_t> memory;
    std::uint32_t base = 0;
    std::uint32_t stride = 64;
    PixelFormat format = PixelFormat::Rgba32;
};

// Byte/bit position of a pixel in tiled memory.
[[nodiscard]] std::uint64_t tiled_bit_offset(PixelFormat f, std::uint32_t base,
                                             std::uint32_t stride, std::uint32_t x,
                                             std::uint32_t y) noexcept;

// Smallest tiled-memory size (bytes, from `base`) that holds a w x h image.
[[nodiscard]] std::size_t tiled_extent(PixelFormat f, std::uint32_t stride,
                                       std::uint32_t w, std::uint32_t h) noexcept;

[[nodiscard]] std::uint32_t tiled_read(const TiledImageView& v, std::uint32_t x, std::uint32_t y);
void tiled_write(std::span<std::uint8_t> memory, std::uint32_t base, std::uint32_t stride,
                 PixelFormat f, std::uint32_t x, std::uint32_t y, std::uint32_t value);

// Palette: 256 (or 16) entries in tiled 32-bit or 16-bit memory.
struct PaletteView {
    std::span<const std::uint8_t> memory;
    std::uint32_t base = 0;
    PixelFormat format = PixelFormat::Rgba32; // Rgba32 or Rgba16/Rgba16S
};

// Converts a packed pixel to RGBA8. Alpha follows the original convention
// where 0x80 is fully opaque (32-bit alpha is scaled to 0..255).
[[nodiscard]] std::uint32_t pixel_to_rgba8(std::uint32_t raw, PixelFormat f) noexcept;

// Palette entry in image memory order: the 8-bit palette stores entries with
// bits 3 and 4 of the index exchanged.
[[nodiscard]] std::uint32_t palette_entry(const PaletteView& pal, PixelFormat indexed,
                                          std::uint32_t index);

// Decodes an image into linear RGBA8. For indexed formats `pal` is required.
// Returns false (and sets error) if the memory is too small or inputs invalid.
bool decode_tiled_image(const TiledImageView& image, const PaletteView* pal, std::uint32_t w,
                        std::uint32_t h, TextureData& out, std::string& error);

} // namespace ac5
