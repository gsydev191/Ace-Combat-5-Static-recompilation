#pragma once

#include "scene.hpp"

#include <string>
#include <vector>

namespace ac5 {

struct AcmLodLevel {
    float maximum_distance = 0.0f;
    MeshData mesh;
    TextureData texture;
    bool has_texture = false;
};

struct AcmLodBundle {
    std::vector<AcmLodLevel> levels;
};

// Load all five ACM meshes and their distance thresholds/textures from an AC5
// nested LOD bundle. Returns found=false for ordinary single-mesh bundles.
bool load_acm_lod_bundle(const std::string& path, AcmLodBundle& lods,
                         bool& found, std::string& error);

// Load a standalone ACM block or the first supported mesh from an AC5
// .mpl/.bin bundle into one flattened, indexed native mesh. Nested LOD bundles
// also include their parent-level companion ACM meshes. Part animation and
// material textures are not included in this static geometry import.
bool load_acm_model(const std::string& path, MeshData& mesh, std::string& error);
bool load_acm_bundle_texture(const std::string& path, TextureData& texture,
                             bool& found, std::string& error);

} // namespace ac5
