#include "adpcm.hpp"
#include "gs_image.hpp"
#include "model_shading.hpp"

#include <cmath>
#include <cstdio>
#include <random>
#include <string>

static int failures = 0;
#define CHECK(c)                                                              \
    do {                                                                      \
        if (!(c)) {                                                           \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); \
            ++failures;                                                       \
        }                                                                     \
    } while (0)

using namespace ac5;

static void test_tiled_roundtrip(PixelFormat f, std::uint32_t w, std::uint32_t h,
                                 std::uint32_t stride) {
    const std::size_t size = tiled_extent(f, stride, w, h) + 4096;
    std::vector<std::uint8_t> mem(size, 0);
    std::mt19937 rng(7);
    const unsigned bits = bits_per_pixel(f);
    const std::uint32_t mask = bits == 32 ? 0xFFFFFFFFu : (1u << bits) - 1u;
    std::vector<std::uint32_t> ref(static_cast<std::size_t>(w) * h);
    for (auto& v : ref) v = static_cast<std::uint32_t>(rng()) & mask;
    for (std::uint32_t y = 0; y < h; ++y)
        for (std::uint32_t x = 0; x < w; ++x)
            tiled_write(mem, 256, stride, f, x, y, ref[y * w + x]);
    TiledImageView v{mem, 256, stride, f};
    bool ok = true;
    for (std::uint32_t y = 0; y < h; ++y)
        for (std::uint32_t x = 0; x < w; ++x)
            ok &= tiled_read(v, x, y) == ref[y * w + x];
    CHECK(ok);
}

// Every pixel of a tiled image must map to a distinct bit range.
static void test_tiled_bijective(PixelFormat f, std::uint32_t w, std::uint32_t h) {
    std::vector<std::uint8_t> seen(tiled_extent(f, 128, w, h) * 8 / bits_per_pixel(f), 0);
    bool ok = true;
    for (std::uint32_t y = 0; y < h; ++y)
        for (std::uint32_t x = 0; x < w; ++x) {
            const auto bit = tiled_bit_offset(f, 0, 128, x, y);
            const auto slot = bit / bits_per_pixel(f);
            if (slot >= seen.size() || seen[slot]) { ok = false; continue; }
            seen[slot] = 1;
        }
    CHECK(ok);
}

static void test_known_swizzle() {
    // First 32-bit page: pixel order within block 0 (hand-verified against
    // the column pattern: x pairs interleave in 4-pixel columns).
    CHECK(tiled_bit_offset(PixelFormat::Rgba32, 0, 64, 0, 0) == 0);
    CHECK(tiled_bit_offset(PixelFormat::Rgba32, 0, 64, 1, 0) == 32);
    CHECK(tiled_bit_offset(PixelFormat::Rgba32, 0, 64, 0, 1) == 64);
    CHECK(tiled_bit_offset(PixelFormat::Rgba32, 0, 64, 2, 0) == 128);
    CHECK(tiled_bit_offset(PixelFormat::Rgba32, 0, 64, 8, 0) == 2048); // next block (id 1)
    CHECK(tiled_bit_offset(PixelFormat::Rgba32, 0, 64, 0, 8) == 2048 * 2); // block id 2
    CHECK(tiled_bit_offset(PixelFormat::Rgba32, 0, 64, 0, 32) == 65536);   // next page row
}

static void test_palette_decode() {
    const std::uint32_t w = 16, h = 16;
    std::vector<std::uint8_t> img(tiled_extent(PixelFormat::Index8, 64, w, h));
    std::vector<std::uint8_t> clut(8192);
    for (std::uint32_t i = 0; i < 256; ++i) {
        std::uint32_t e = (i & ~0x18u) | ((i & 8u) << 1) | ((i & 16u) >> 1);
        tiled_write(clut, 0, 64, PixelFormat::Rgba32, e % 16, e / 16, i | (i << 8) | 0x80000000u);
    }
    for (std::uint32_t y = 0; y < h; ++y)
        for (std::uint32_t x = 0; x < w; ++x)
            tiled_write(img, 0, 64, PixelFormat::Index8, x, y, y * 16 + x);
    TextureData t;
    std::string err;
    CHECK(decode_tiled_image({img, 0, 64, PixelFormat::Index8},
                             &(const PaletteView&)PaletteView{clut, 0, PixelFormat::Rgba32}, w, h,
                             t, err));
    bool ok = t.width == w;
    for (std::uint32_t i = 0; i < 256 && ok; ++i)
        ok = t.rgba[i * 4] == i && t.rgba[i * 4 + 1] == i && t.rgba[i * 4 + 3] == 255;
    CHECK(ok);
    CHECK(!decode_tiled_image({img, 0, 64, PixelFormat::Index8}, nullptr, w, h, t, err));
    CHECK(!decode_tiled_image({std::span<const std::uint8_t>(img).first(16), 0, 64,
                               PixelFormat::Rgba32}, nullptr, w, h, t, err));
}

static void test_adpcm() {
    std::uint8_t blk[16] = {0};
    AdpcmState st;
    std::int16_t out[28];
    adpcm_decode_block(blk, out, st);
    for (auto s : out) CHECK(s == 0);
    // Shift 12, filter 0, nibble 7 -> +7<<12>>12 = 7; nibble 8 -> -8.
    blk[0] = 0x0C;
    blk[2] = 0x87;
    adpcm_decode_block(blk, out, st);
    CHECK(out[0] == 7 && out[1] == -8);
    // Out-of-range filter falls back to 0; shift > 12 mutes.
    st = {};
    blk[0] = 0x0F | 0x90;
    adpcm_decode_block(blk, out, st);
    CHECK(out[0] == 0);
    // Stream: loop start, then loop end+repeat.
    std::vector<std::uint8_t> s(48, 0);
    s[16 * 0 + 1] = adpcm_flag_loop_start;
    s[16 * 2 + 1] = adpcm_flag_loop_end | adpcm_flag_loop_repeat;
    auto r = adpcm_decode(s);
    CHECK(r.samples.size() == 84 && r.loop_start == 0 && r.looping && r.ended);
    s.push_back(1); // trailing partial block ignored
    CHECK(adpcm_decode(s).samples.size() == 84);
}

static void test_shading() {
    ModelShadeParams p;
    for (int i = 0; i < 4; ++i) p.clip[i][i] = 1.0f;
    p.clip[2][3] = 1.0f;       // w = z
    p.clip[3][3] = 0.0f;
    p.fog_bias = 10.0f;
    p.fog_scale = 2.0f;
    p.depth_scale = 4.0f;
    ModelVertexIn in;
    in.position = {2.0f, 4.0f, 2.0f, 1.0f};
    in.color = {10, 20, 30, 128};
    in.uv = {0.5f, 1.0f};
    auto o = shade_model_vertex(ModelShadeMode::Plain, p, in);
    CHECK(o.screen[0] == 1.0f && o.screen[1] == 2.0f);
    CHECK(o.screen[2] == 4.0f * 2.0f * 0.5f);
    CHECK(o.screen[3] == 14.0f);
    CHECK(o.st[0] == 0.25f && o.st[2] == 0.5f);
    CHECK(o.rgba[1] == 20);
    p.aircraft = true;
    CHECK(shade_model_vertex(ModelShadeMode::Plain, p, in).screen[2] == 16.0f);

    // Lit: light 0 along +z, white; ambient 0.25 on all channels.
    p.light_dir = {};
    p.light_dir[2][0] = 1.0f;
    p.light_col = {};
    p.light_col[0] = {0.5f, 0.5f, 0.5f, 0.0f};
    p.light_col[3] = {0.25f, 0.25f, 0.25f, 0.0f};
    in.normal = {0, 0, 4096};
    o = shade_model_vertex(ModelShadeMode::Lit, p, in);
    // l0 = 1^3*2 = 2 -> 2*0.5 + 0.25 = 1.25
    CHECK(o.rgba[0] == 12 && o.rgba[1] == 25 && o.rgba[2] == 37 && o.rgba[3] == 128);
    in.normal = {0, 0, -4096}; // facing away: only ambient
    o = shade_model_vertex(ModelShadeMode::Lit, p, in);
    CHECK(o.rgba[0] == 2 && o.rgba[1] == 5 && o.rgba[2] == 7);
}

int main() {
    for (auto f : {PixelFormat::Rgba32, PixelFormat::Rgba16, PixelFormat::Rgba16S,
                   PixelFormat::Index8, PixelFormat::Index4}) {
        test_tiled_roundtrip(f, 200, 150, 256);
        test_tiled_bijective(f, 128, 128);
    }
    test_known_swizzle();
    test_palette_decode();
    test_adpcm();
    test_shading();
    if (failures) {
        std::fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    std::puts("all native tests passed");
    return 0;
}
