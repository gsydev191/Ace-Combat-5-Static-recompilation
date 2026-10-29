#pragma once

#include "scene.hpp"

#include <string>

namespace ac5 {

bool load_obj(const std::string& path, MeshData& mesh, std::string& error);
bool load_obj_scene(const std::string& path, Scene& scene, std::string& error,
                    bool load_material_textures = true, TransformId transform = 0);

} // namespace ac5
