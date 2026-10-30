#pragma once

#include "scene.hpp"

#include <cstdint>
#include <span>
#include <string>

namespace ac5 {

bool load_ppm(const std::string& path, TextureData& texture, std::string& error);
bool load_gim(const std::string& path, TextureData& texture, std::string& error);
bool decode_gim(std::span<const std::uint8_t> bytes, TextureData& texture,
                std::string& error, bool force_opaque = false);
bool decode_gim_indexed_with_palette(std::span<const std::uint8_t> bytes,
                                    std::span<const std::uint8_t> rgba_palette,
                                    TextureData& texture, std::string& error,
                                    bool force_opaque = true);
bool load_texture_image(const std::string& path, TextureData& texture, std::string& error);

} // namespace ac5
