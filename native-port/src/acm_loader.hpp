#pragma once

#include "scene.hpp"

#include <string>

namespace ac5 {

// Load the first ACM mesh from a standalone block or an AC5 .mpl/.bin model
// bundle into one flattened, indexed native mesh. Part animation and material
// textures are not included in this static geometry import.
bool load_acm_model(const std::string& path, MeshData& mesh, std::string& error);
bool load_acm_bundle_texture(const std::string& path, TextureData& texture,
                             bool& found, std::string& error);

} // namespace ac5
