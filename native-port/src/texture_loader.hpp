#pragma once

#include "scene.hpp"

#include <string>

namespace ac5 {

bool load_ppm(const std::string& path, TextureData& texture, std::string& error);
bool load_texture_image(const std::string& path, TextureData& texture, std::string& error);

} // namespace ac5
