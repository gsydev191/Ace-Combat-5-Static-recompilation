#include "adpcm.hpp"

namespace ac5 {
namespace {

constexpr int filter0[5] = {0, 60, 115, 98, 122};
constexpr int filter1[5] = {0, 0, -52, -55, -60};

std::int16_t clamp16(int v) noexcept {
    return static_cast<std::int16_t>(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
}

} // namespace

std::uint8_t adpcm_decode_block(std::span<const std::uint8_t, adpcm_block_bytes> block,
                                std::int16_t* out, AdpcmState& st) noexcept {
    const unsigned shift = block[0] & 0x0Fu;
    unsigned filter = (block[0] >> 4) & 0x0Fu;
    if (filter > 4) filter = 0;
    int p1 = st.s1, p2 = st.s2;
    for (std::size_t i = 0; i < adpcm_block_samples; ++i) {
        const int nib = (block[2 + (i >> 1)] >> ((i & 1) ? 4 : 0)) & 0x0F;
        int s = static_cast<std::int16_t>(static_cast<std::uint16_t>(nib << 12));
        s = shift > 12 ? 0 : (s >> shift);
        const int v = s + ((p1 * filter0[filter] + p2 * filter1[filter]) >> 6);
        out[i] = clamp16(v);
        p2 = p1;
        p1 = out[i];
    }
    st.s1 = p1;
    st.s2 = p2;
    return block[1];
}

AdpcmStream adpcm_decode(std::span<const std::uint8_t> data) {
    AdpcmStream out;
    AdpcmState st;
    std::int16_t buf[adpcm_block_samples];
    for (std::size_t off = 0; off + adpcm_block_bytes <= data.size(); off += adpcm_block_bytes) {
        const std::uint8_t flags = adpcm_decode_block(
            std::span<const std::uint8_t, adpcm_block_bytes>(data.data() + off, adpcm_block_bytes),
            buf, st);
        if ((flags & adpcm_flag_loop_start) && out.loop_start == static_cast<std::size_t>(-1))
            out.loop_start = out.samples.size();
        out.samples.insert(out.samples.end(), buf, buf + adpcm_block_samples);
        if (flags & adpcm_flag_loop_end) {
            out.ended = true;
            out.looping = (flags & adpcm_flag_loop_repeat) != 0;
            break;
        }
    }
    return out;
}

} // namespace ac5
