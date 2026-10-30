#pragma once

#include "projectile_system.hpp"
#include "scene.hpp"

namespace ac5 {

// Adds one fixed-topology dynamic mesh and an identity-space scene draw.
[[nodiscard]] MeshId add_projectile_mesh(Scene& scene);

// Updates projectile vertex positions, normals, and colors without changing
// the mesh's uploaded vertex or index counts.
void update_projectile_mesh(Scene& scene, MeshId mesh,
                            const AircraftProjectileSystem& projectiles);

} // namespace ac5
