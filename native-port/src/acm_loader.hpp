#pragma once

#include "scene.hpp"

#include <string>

namespace ac5 {

// Load a standalone ACM block or the first supported mesh from an AC5
// .mpl/.bin bundle into one flattened, indexed native mesh. Nested LOD bundles
// also include their parent-level companion ACM meshes. Part animation and
// material textures are not included in this static geometry import.
bool load_acm_model(const std::string& path, MeshData& mesh, std::string& error);
bool load_acm_bundle_texture(const std::string& path, TextureData& texture,
                             bool& found, std::string& error);

} // namespace ac5
