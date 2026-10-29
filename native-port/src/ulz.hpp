#pragma once

#include <cstdint>
#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace ac5 {

class UlzDecoder {
public:
    UlzDecoder() = default;
    UlzDecoder(const UlzDecoder&) = delete;
    UlzDecoder& operator=(const UlzDecoder&) = delete;
    UlzDecoder(UlzDecoder&&) noexcept = default;
    UlzDecoder& operator=(UlzDecoder&&) noexcept = default;

    // The packed span is non-owning and must remain valid until decoding finishes.
    bool begin(std::span<const std::uint8_t> packed, std::string& error);
    bool step(std::size_t max_control_words, std::string& error);
    bool finished() const { return finished_; }
    bool failed() const { return failed_; }
    const std::vector<std::uint8_t>& output() const { return output_; }
    bool take_output(std::vector<std::uint8_t>& output, std::string& error);

private:
    std::span<const std::uint8_t> packed_;
    std::vector<std::uint8_t> output_;
    std::size_t output_size_ = 0;
    std::size_t control_ = 0;
    std::size_t literal_ = 0;
    std::size_t token_ = 0;
    std::uint32_t distance_mask_ = 0;
    std::uint32_t shift_ = 0;
    bool initialized_ = false;
    bool finished_ = false;
    bool failed_ = false;
};

bool decode_ulz(std::span<const std::uint8_t> packed,
                std::vector<std::uint8_t>& decoded, std::string& error);

} // namespace ac5
