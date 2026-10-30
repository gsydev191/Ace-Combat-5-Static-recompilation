#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace ac5 {

// 16-byte block ADPCM used by the game's sound banks and streamed music:
// a shift/filter byte, a flags byte (loop end/repeat/start), and 28 samples
// packed as 4-bit nibbles.
inline constexpr std::size_t adpcm_block_bytes = 16;
inline constexpr std::size_t adpcm_block_samples = 28;

inline constexpr std::uint8_t adpcm_flag_loop_end = 0x01;
inline constexpr std::uint8_t adpcm_flag_loop_repeat = 0x02;
inline constexpr std::uint8_t adpcm_flag_loop_start = 0x04;

struct AdpcmState {
    int s1 = 0; // previous sample
    int s2 = 0; // sample before that
};

// Decodes one block. Returns the flags byte.
std::uint8_t adpcm_decode_block(std::span<const std::uint8_t, adpcm_block_bytes> block,
                                std::int16_t* out28, AdpcmState& state) noexcept;

struct AdpcmStream {
    std::vector<std::int16_t> samples;
    std::size_t loop_start = static_cast<std::size_t>(-1); // sample index, or -1
    bool looping = false;                                   // loop-repeat seen
    bool ended = false;                                     // loop-end block seen
};

// Decodes blocks until a loop-end flag or the end of `data`. Trailing bytes
// that do not form a whole block are ignored.
AdpcmStream adpcm_decode(std::span<const std::uint8_t> data);

} // namespace ac5
