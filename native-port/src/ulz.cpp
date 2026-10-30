#include "ulz.hpp"

#include <algorithm>
#include <new>
#include <stdexcept>
#include <utility>

namespace ac5 {

namespace {

constexpr std::uint32_t ulz_magic = 0x1A7A6C55u;
constexpr std::size_t ulz_header_size = 16;

std::uint32_t read_u32_le(std::span<const std::uint8_t> data, std::size_t offset) {
    return static_cast<std::uint32_t>(data[offset]) |
           (static_cast<std::uint32_t>(data[offset + 1]) << 8) |
           (static_cast<std::uint32_t>(data[offset + 2]) << 16) |
           (static_cast<std::uint32_t>(data[offset + 3]) << 24);
}

} // namespace

bool UlzDecoder::begin(std::span<const std::uint8_t> packed, std::string& error) {
    packed_ = packed;
    output_.clear();
    output_size_ = control_ = literal_ = token_ = 0;
    distance_mask_ = shift_ = 0;
    initialized_ = false;
    finished_ = false;
    failed_ = false;
    if (packed.size() < ulz_header_size || read_u32_le(packed, 0) != ulz_magic) {
        error = "invalid ULZ header or magic";
        failed_ = true;
        return false;
    }
    const std::uint32_t word1 = read_u32_le(packed, 4);
    const std::uint32_t word2 = read_u32_le(packed, 8);
    output_size_ = word1 & 0x00FFFFFFu;
    // The generated MIPS uses variable shifts, which mask the count to 5 bits.
    shift_ = (word2 >> 24) & 31u;
    literal_ = word2 & 0x00FFFFFFu;
    token_ = read_u32_le(packed, 12) & 0x00FFFFFFu;
    control_ = ulz_header_size;
    if (literal_ < ulz_header_size || token_ < ulz_header_size ||
        literal_ > packed.size() || token_ > packed.size()) {
        error = "ULZ stream has invalid data offsets";
        failed_ = true;
        return false;
    }

    try {
        output_.reserve(output_size_);
    } catch (const std::bad_alloc&) {
        error = "ULZ output allocation failed";
        failed_ = true;
        return false;
    } catch (const std::length_error&) {
        error = "ULZ output size exceeds the platform's vector limit";
        failed_ = true;
        return false;
    }
    distance_mask_ = (1u << shift_) - 1u;
    initialized_ = true;
    finished_ = output_size_ == 0;
    return true;
}

bool UlzDecoder::step(std::size_t max_control_words, std::string& error) {
    if (failed_) {
        error = "ULZ decoder is in a failed state; call begin to reset it";
        return false;
    }
    if (!initialized_) {
        error = "ULZ decoder has not been initialized";
        return false;
    }
    if (finished_) return true;
    const auto fail = [this, &error](const char* message) {
        error = message;
        failed_ = true;
        return false;
    };

    for (std::size_t words = 0; words < max_control_words && output_.size() < output_size_; ++words) {
        if (control_ > packed_.size() || packed_.size() - control_ < 4) {
            return fail("ULZ control stream ends before the declared output size");
        }
        std::uint32_t control_word = read_u32_le(packed_, control_);
        control_ += 4;
        for (int bit = 0; bit < 32 && output_.size() < output_size_; ++bit) {
            if (control_word & 0x80000000u) {
                if (literal_ >= packed_.size()) {
                    return fail("ULZ literal stream ends before the declared output size");
                }
                output_.push_back(packed_[literal_++]);
            } else {
                if (token_ > packed_.size() || packed_.size() - token_ < 2) {
                    return fail("ULZ token stream ends before the declared output size");
                }
                const std::uint32_t encoded = static_cast<std::uint32_t>(packed_[token_]) |
                                              (static_cast<std::uint32_t>(packed_[token_ + 1]) << 8);
                token_ += 2;
                const std::size_t distance = (encoded & distance_mask_) + 1u;
                const std::size_t length = (encoded >> shift_) + 3u;
                if (distance > output_.size()) {
                    return fail("ULZ back-reference points before the output buffer");
                }
                const std::size_t count = std::min(length, output_size_ - output_.size());
                for (std::size_t i = 0; i < count; ++i)
                    output_.push_back(output_[output_.size() - distance]);
            }
            control_word <<= 1;
        }
    }
    finished_ = output_.size() == output_size_;
    return true;
}

bool UlzDecoder::take_output(std::vector<std::uint8_t>& output, std::string& error) {
    if (failed_) {
        error = "ULZ output is unavailable after decoder failure";
        return false;
    }
    if (!finished_) {
        error = "ULZ output is not complete";
        return false;
    }
    output = std::move(output_);
    packed_ = {};
    output_size_ = control_ = literal_ = token_ = 0;
    distance_mask_ = shift_ = 0;
    initialized_ = false;
    finished_ = false;
    failed_ = false;
    return true;
}

bool decode_ulz(std::span<const std::uint8_t> packed,
                std::vector<std::uint8_t>& decoded, std::string& error) {
    UlzDecoder decoder;
    if (!decoder.begin(packed, error)) return false;
    while (!decoder.finished()) {
        if (!decoder.step(2500, error)) return false;
    }
    return decoder.take_output(decoded, error);
}

} // namespace ac5
