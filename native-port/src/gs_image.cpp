#include "gs_image.hpp"

#include <array>
#include <limits>
#include <string>

namespace ac5 {
namespace {

// Block tables: which 256-byte block of a page holds each block-grid cell.
constexpr std::uint8_t blk32[4][8] = {{0, 1, 4, 5, 16, 17, 20, 21},
                                      {2, 3, 6, 7, 18, 19, 22, 23},
                                      {8, 9, 12, 13, 24, 25, 28, 29},
                                      {10, 11, 14, 15, 26, 27, 30, 31}};
constexpr std::uint8_t blk16[8][4] = {{0, 2, 8, 10},   {1, 3, 9, 11},   {4, 6, 12, 14},
                                      {5, 7, 13, 15},  {16, 18, 24, 26}, {17, 19, 25, 27},
                                      {20, 22, 28, 30}, {21, 23, 29, 31}};
constexpr std::uint8_t blk16s[8][4] = {{0, 2, 16, 18},  {1, 3, 17, 19},   {8, 10, 24, 26},
                                       {9, 11, 25, 27}, {4, 6, 20, 22},   {5, 7, 21, 23},
                                       {12, 14, 28, 30}, {13, 15, 29, 31}};

struct Geom {
    unsigned pw, ph;     // page size in pixels
    unsigned bw, bh;     // block size in pixels
    unsigned bits;
    unsigned tabcols;
    const std::uint8_t* tab;
    int cls; // 0:32 1:16 2:8 3:4
};

Geom geom_for(PixelFormat f) {
    switch (f) {
    case PixelFormat::Index4: return {128, 128, 32, 16, 4, 4, &blk16[0][0], 3};
    case PixelFormat::Index8: return {128, 64, 16, 16, 8, 8, &blk32[0][0], 2};
    case PixelFormat::Rgba16: return {64, 64, 16, 8, 16, 4, &blk16[0][0], 1};
    case PixelFormat::Rgba16S: return {64, 64, 16, 8, 16, 4, &blk16s[0][0], 1};
    default: return {64, 32, 8, 8, 32, 8, &blk32[0][0], 0};
    }
}

unsigned swizzle_bit(int cls, unsigned x, unsigned y) {
    const unsigned colh = cls >= 2 ? 4u : 2u;
    const unsigned colnum = y / colh, cy = y % colh;
    unsigned w = (x & 1u) | (((x >> 1) & 3u) << 2) | ((cy & 1u) << 1);
    if (colh == 4u && (((cy >> 1) & 1u) ^ (colnum & 1u))) w ^= 8u;
    unsigned sub = 0, bits = 32;
    switch (cls) {
    case 1: bits = 16; sub = (x >> 3) & 1u; break;
    case 2: bits = 8; sub = ((cy >> 1) & 1u) | (((x >> 3) & 1u) << 1); break;
    case 3: bits = 4; sub = (((x >> 3) & 3u) << 1) | ((cy >> 1) & 1u); break;
    default: break;
    }
    return colnum * 512u + w * 32u + sub * bits;
}

// In-block bit offset for each pixel of a block, per format class.
const std::vector<std::uint16_t>& block_offsets(int cls) {
    static const std::array<std::vector<std::uint16_t>, 4> tables = [] {
        std::array<std::vector<std::uint16_t>, 4> t;
        const unsigned bw[4] = {8, 16, 16, 32}, bh[4] = {8, 8, 16, 16};
        for (int c = 0; c < 4; ++c) {
            t[c].resize(bw[c] * bh[c]);
            for (unsigned y = 0; y < bh[c]; ++y)
                for (unsigned x = 0; x < bw[c]; ++x)
                    t[c][y * bw[c] + x] = static_cast<std::uint16_t>(swizzle_bit(c, x, y));
        }
        return t;
    }();
    return tables[static_cast<std::size_t>(cls)];
}

unsigned log2u(unsigned v) {
    unsigned n = 0;
    while ((1u << n) < v) ++n;
    return n;
}

} // namespace

bool is_known_pixel_format(std::uint32_t code) noexcept {
    switch (code) {
    case 0: case 1: case 2: case 10: case 19: case 20: return true;
    default: return false;
    }
}

std::uint64_t tiled_bit_offset(PixelFormat f, std::uint32_t base, std::uint32_t stride,
                               std::uint32_t x, std::uint32_t y) noexcept {
    const Geom g = geom_for(f);
    unsigned ppr = stride / g.pw;
    if (!ppr) ppr = 1;
    const unsigned page = (y / g.ph) * ppr + (x / g.pw);
    const unsigned block =
        g.tab[(((y % g.ph) / g.bh) * g.tabcols) + ((x % g.pw) / g.bw)];
    const unsigned inblk =
        block_offsets(g.cls)[((y % g.bh) << log2u(g.bw)) + (x % g.bw)];
    return static_cast<std::uint64_t>(base) * 8u + static_cast<std::uint64_t>(page) * 65536u +
           static_cast<std::uint64_t>(block) * 2048u + inblk;
}

std::size_t tiled_extent(PixelFormat f, std::uint32_t stride, std::uint32_t w,
                         std::uint32_t h) noexcept {
    if (!w || !h) return 0;
    const Geom g = geom_for(f);
    std::uint64_t ppr = stride / g.pw;
    if (!ppr) ppr = 1;
    const std::uint64_t pages = static_cast<std::uint64_t>((h - 1) / g.ph) * ppr +
                                (w - 1) / g.pw + 1;
    if (pages > std::numeric_limits<std::size_t>::max() / 8192u)
        return std::numeric_limits<std::size_t>::max();
    return static_cast<std::size_t>(pages * 8192u);
}

std::uint32_t tiled_read(const TiledImageView& v, std::uint32_t x, std::uint32_t y) {
    const std::uint64_t bit = tiled_bit_offset(v.format, v.base, v.stride, x, y);
    const std::size_t off = static_cast<std::size_t>(bit >> 3);
    const auto& m = v.memory;
    switch (bits_per_pixel(v.format)) {
    case 4:
        if (off >= m.size()) return 0;
        return (bit & 4u) ? (m[off] >> 4) : (m[off] & 0xFu);
    case 8: return off < m.size() ? m[off] : 0u;
    case 16:
        if (off + 2 > m.size()) return 0;
        return static_cast<std::uint32_t>(m[off]) | (static_cast<std::uint32_t>(m[off + 1]) << 8);
    default:
        if (off + 4 > m.size()) return 0;
        return static_cast<std::uint32_t>(m[off]) | (static_cast<std::uint32_t>(m[off + 1]) << 8) |
               (static_cast<std::uint32_t>(m[off + 2]) << 16) |
               (static_cast<std::uint32_t>(m[off + 3]) << 24);
    }
}

void tiled_write(std::span<std::uint8_t> m, std::uint32_t base, std::uint32_t stride,
                 PixelFormat f, std::uint32_t x, std::uint32_t y, std::uint32_t value) {
    const std::uint64_t bit = tiled_bit_offset(f, base, stride, x, y);
    const std::size_t off = static_cast<std::size_t>(bit >> 3);
    switch (bits_per_pixel(f)) {
    case 4:
        if (off >= m.size()) return;
        m[off] = (bit & 4u) ? static_cast<std::uint8_t>((m[off] & 0x0Fu) | ((value & 0xFu) << 4))
                            : static_cast<std::uint8_t>((m[off] & 0xF0u) | (value & 0xFu));
        return;
    case 8:
        if (off < m.size()) m[off] = static_cast<std::uint8_t>(value);
        return;
    case 16:
        if (off + 2 > m.size()) return;
        m[off] = static_cast<std::uint8_t>(value);
        m[off + 1] = static_cast<std::uint8_t>(value >> 8);
        return;
    default:
        if (off + 4 > m.size()) return;
        for (int i = 0; i < 4; ++i) m[off + i] = static_cast<std::uint8_t>(value >> (8 * i));
        return;
    }
}

std::uint32_t pixel_to_rgba8(std::uint32_t raw, PixelFormat f) noexcept {
    switch (f) {
    case PixelFormat::Rgba32: {
        std::uint32_t a = (raw >> 24) & 0xFFu;
        a = a >= 128u ? 255u : a * 2u;
        return (raw & 0x00FFFFFFu) | (a << 24);
    }
    case PixelFormat::Rgb24: return (raw & 0x00FFFFFFu) | 0xFF000000u;
    case PixelFormat::Rgba16:
    case PixelFormat::Rgba16S: {
        const std::uint32_t r = (raw & 0x1Fu) << 3, g = ((raw >> 5) & 0x1Fu) << 3,
                            b = ((raw >> 10) & 0x1Fu) << 3, a = (raw >> 15) & 1u ? 255u : 0u;
        return r | (g << 8) | (b << 16) | (a << 24);
    }
    default: return raw | 0xFF000000u;
    }
}

std::uint32_t palette_entry(const PaletteView& pal, PixelFormat indexed, std::uint32_t index) {
    std::uint32_t e = index;
    if (indexed == PixelFormat::Index4) {
        // 16-entry palettes sit in rows of 8 entries.
        e &= 15u;
    } else {
        e &= 255u;
        e = (e & ~0x18u) | ((e & 0x08u) << 1) | ((e & 0x10u) >> 1);
    }
    const bool is16 = pal.format == PixelFormat::Rgba16 || pal.format == PixelFormat::Rgba16S;
    const unsigned cx = indexed == PixelFormat::Index4 ? e % 8u : e % 16u;
    const unsigned cy = indexed == PixelFormat::Index4 ? e / 8u : e / 16u;
    TiledImageView v{pal.memory, pal.base, 64,
                     is16 ? pal.format : PixelFormat::Rgba32};
    return pixel_to_rgba8(tiled_read(v, cx, cy), v.format);
}

bool decode_tiled_image(const TiledImageView& image, const PaletteView* pal, std::uint32_t w,
                        std::uint32_t h, TextureData& out, std::string& error) {
    if (!is_known_pixel_format(static_cast<std::uint32_t>(image.format))) {
        error = "unsupported image pixel format";
        return false;
    }
    if (!w || !h || w > 4096 || h > 4096) {
        error = "image dimensions out of range";
        return false;
    }
    const bool indexed =
        image.format == PixelFormat::Index8 || image.format == PixelFormat::Index4;
    if (indexed && !pal) {
        error = "indexed image requires a palette";
        return false;
    }
    if (indexed && !is_known_pixel_format(static_cast<std::uint32_t>(pal->format))) {
        error = "unsupported palette pixel format";
        return false;
    }
    if (indexed && pal->format != PixelFormat::Rgba32 &&
        pal->format != PixelFormat::Rgba16 && pal->format != PixelFormat::Rgba16S) {
        error = "palette must use a 16-bit or 32-bit RGBA format";
        return false;
    }
    if (indexed) {
        std::uint64_t palette_end_bit = 0;
        const std::uint32_t entry_count = image.format == PixelFormat::Index4 ? 16u : 256u;
        for (std::uint32_t i = 0; i < entry_count; ++i) {
            std::uint32_t entry = i;
            if (image.format == PixelFormat::Index8)
                entry = (entry & ~0x18u) | ((entry & 0x08u) << 1) | ((entry & 0x10u) >> 1);
            const std::uint32_t cx = image.format == PixelFormat::Index4 ? entry % 8u : entry % 16u;
            const std::uint32_t cy = image.format == PixelFormat::Index4 ? entry / 8u : entry / 16u;
            const std::uint64_t end_bit = tiled_bit_offset(pal->format, pal->base, 64, cx, cy) +
                                          bits_per_pixel(pal->format);
            palette_end_bit = std::max(palette_end_bit, end_bit);
        }
        const std::uint64_t palette_end_byte = (palette_end_bit + 7u) / 8u;
        if (palette_end_byte > pal->memory.size()) {
            error = "palette memory is too small for its indexed image";
            return false;
        }
    }
    if (image.stride == 0 || image.stride % 64u != 0) {
        error = "stride must be a multiple of 64 pixels";
        return false;
    }
    const std::size_t base = image.base;
    const std::size_t extent = tiled_extent(image.format, image.stride, w, h);
    if (base > image.memory.size() || extent > image.memory.size() - base) {
        error = "image memory is too small";
        return false;
    }
    TextureData t;
    t.width = w;
    t.height = h;
    t.rgba.resize(static_cast<std::size_t>(w) * h * 4);
    for (std::uint32_t y = 0; y < h; ++y)
        for (std::uint32_t x = 0; x < w; ++x) {
            const std::uint32_t raw = tiled_read(image, x, y);
            const std::uint32_t c = indexed ? palette_entry(*pal, image.format, raw)
                                            : pixel_to_rgba8(raw, image.format);
            std::uint8_t* o = &t.rgba[(static_cast<std::size_t>(y) * w + x) * 4];
            o[0] = static_cast<std::uint8_t>(c);
            o[1] = static_cast<std::uint8_t>(c >> 8);
            o[2] = static_cast<std::uint8_t>(c >> 16);
            o[3] = static_cast<std::uint8_t>(c >> 24);
        }
    out = std::move(t);
    return true;
}

} // namespace ac5
