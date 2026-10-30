#include "projectile_mesh.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <span>

namespace ac5 {
namespace {

MeshData make_projectile_mesh() {
    MeshData mesh;
    mesh.vertices.resize(projectile_capacity * 6);
    mesh.indices.reserve(projectile_capacity * 24);
    constexpr std::array<std::uint32_t, 24> octahedron_indices{
        0, 2, 4, 0, 4, 3, 0, 3, 5, 0, 5, 2,
        1, 4, 2, 1, 3, 4, 1, 5, 3, 1, 2, 5};
    for (std::size_t i = 0; i < projectile_capacity; ++i) {
        for (std::size_t vertex_index = 0; vertex_index < 6; ++vertex_index) {
            Vertex& vertex = mesh.vertices[i * 6 + vertex_index];
            vertex.position = {};
            vertex.color = {1.0f, 0.78f, 0.25f};
            vertex.uv = {0.0f, 0.0f};
            vertex.normal = {0.0f, 1.0f, 0.0f};
            vertex.alpha = 1.0f;
            vertex.light_response = 1.0f;
            vertex.environment_uv = {0.0f, 0.0f};
            vertex.environment_color = {0.0f, 0.0f, 0.0f};
            vertex.environment_alpha = 0.0f;
        }
        for (std::uint32_t index : octahedron_indices)
            mesh.indices.push_back(static_cast<std::uint32_t>(i * 6 + index));
    }
    return mesh;
}

Vec3 add(Vec3 a, Vec3 b) noexcept {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 subtract(Vec3 a, Vec3 b) noexcept {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 scale(Vec3 value, float amount) noexcept {
    return {value.x * amount, value.y * amount, value.z * amount};
}

Vec3 cross(Vec3 a, Vec3 b) noexcept {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

Vec3 normalize(Vec3 value, Vec3 fallback) noexcept {
    const float length = std::sqrt(value.x * value.x + value.y * value.y +
                                   value.z * value.z);
    if (!std::isfinite(length) || length <= 1.0e-6f) return fallback;
    return scale(value, 1.0f / length);
}

} // namespace

MeshId add_projectile_mesh(Scene& scene) {
    const MeshId mesh = scene.add_dynamic_mesh(make_projectile_mesh());
    const TransformId transform = scene.add_transform(
        Mat4::identity(), no_parent_transform);
    scene.add_draw(mesh, TextureId{0}, Mat4::identity(), transform);
    return mesh;
}

void update_projectile_mesh(Scene& scene, MeshId mesh,
                            const AircraftProjectileSystem& projectiles) {
    std::span<Vertex> vertices = scene.dynamic_vertices(mesh);
    for (std::size_t i = 0; i < projectile_capacity; ++i) {
        const AircraftProjectile& projectile = projectiles.projectiles[i];
        std::array<Vec3, 6> points{};
        Vec3 color{1.0f, 0.78f, 0.25f};
        if (projectile.active) {
            const float length = projectile.kind == AircraftShotKind::Gun ? 0.8f : 2.4f;
            const float radius = projectile.kind == AircraftShotKind::Gun ? 0.12f : 0.42f;
            const Vec3 center{static_cast<float>(projectile.position[0]),
                              static_cast<float>(projectile.position[1]),
                              static_cast<float>(projectile.position[2])};
            const Vec3 forward = normalize(
                {static_cast<float>(projectile.velocity[0]),
                 static_cast<float>(projectile.velocity[1]),
                 static_cast<float>(projectile.velocity[2])},
                {0.0f, 0.0f, -1.0f});
            const Vec3 reference_up = std::abs(forward.y) > 0.98f
                ? Vec3{1.0f, 0.0f, 0.0f} : Vec3{0.0f, 1.0f, 0.0f};
            const Vec3 right = normalize(cross(forward, reference_up),
                                         {1.0f, 0.0f, 0.0f});
            const Vec3 up = cross(right, forward);
            points = {{add(center, scale(up, radius)),
                       subtract(center, scale(up, radius)),
                       subtract(center, scale(right, radius)),
                       add(center, scale(right, radius)),
                       subtract(center, scale(forward, length)),
                       add(center, scale(forward, length))}};
            if (projectile.kind == AircraftShotKind::StandardMissile)
                color = {0.75f, 0.88f, 1.0f};
            else if (projectile.kind == AircraftShotKind::Special)
                color = {1.0f, 0.36f, 0.12f};
        }
        const Vec3 center{static_cast<float>(projectile.position[0]),
                          static_cast<float>(projectile.position[1]),
                          static_cast<float>(projectile.position[2])};
        for (std::size_t vertex_index = 0; vertex_index < points.size(); ++vertex_index) {
            Vertex& vertex = vertices[i * points.size() + vertex_index];
            vertex.position = points[vertex_index];
            vertex.color = color;
            const Vec3 normal = projectile.active
                ? normalize(subtract(points[vertex_index], center), {0.0f, 1.0f, 0.0f})
                : Vec3{0.0f, 1.0f, 0.0f};
            vertex.normal = normal;
        }
    }
}

} // namespace ac5
