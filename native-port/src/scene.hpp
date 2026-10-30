#pragma once

#include "math.hpp"
#include "scene_state_machine.hpp"

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ac5 {

struct Vertex {
    Vec3 position;
    Vec3 color;
    Vec2 uv;
    Vec3 normal{0.0f, 0.0f, 1.0f};
    float alpha = 1.0f;
    float light_response = 1.0f; // zero for colors already lit by model_shading
    Vec2 environment_uv{}; // reflection UV before perspective interpolation
    Vec3 environment_color{}; // normalized modulation color for the environment pass
    float environment_alpha = 0.0f; // normalized blend alpha; zero disables its contribution
};

struct TextureData {
    std::uint32_t width = 1;
    std::uint32_t height = 1;
    std::vector<std::uint8_t> rgba{255, 255, 255, 255};
};

struct SceneLighting {
    Vec3 direction{0.35f, 0.55f, 1.0f};
    float ambient = 0.22f;
};

struct MeshData {
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;
};

using MeshId = std::uint32_t;
using TextureId = std::uint32_t;
using TransformId = std::uint32_t;
inline constexpr TextureId no_environment_texture = std::numeric_limits<TextureId>::max();
inline constexpr TransformId no_parent_transform = std::numeric_limits<TransformId>::max();

struct DrawItem {
    MeshId mesh;
    TextureId texture;
    Mat4 model;
    TransformId transform;
    TextureId environment_texture = no_environment_texture;
    bool visible = true;
};

class Scene {
public:
    const SceneLighting& lighting() const { return lighting_; }

    void set_lighting(SceneLighting lighting) {
        const float length_squared = lighting.direction.x * lighting.direction.x +
                                     lighting.direction.y * lighting.direction.y +
                                     lighting.direction.z * lighting.direction.z;
        if (!std::isfinite(length_squared) || length_squared <= 1.0e-12f ||
            !std::isfinite(lighting.ambient) || lighting.ambient < 0.0f ||
            lighting.ambient > 1.0f)
            throw std::invalid_argument("scene lighting has invalid direction or ambient value");
        lighting_ = lighting;
    }

    MeshId add_mesh(MeshData mesh) {
        return add_mesh(std::move(mesh), false);
    }

    // Dynamic meshes keep fixed vertex and index counts after creation. The
    // renderer allocates one writable vertex buffer per frame in flight.
    MeshId add_dynamic_mesh(MeshData mesh) {
        return add_mesh(std::move(mesh), true);
    }

    [[nodiscard]] bool is_dynamic_mesh(MeshId id) const {
        return dynamic_meshes_.at(id);
    }

    [[nodiscard]] std::span<Vertex> dynamic_vertices(MeshId id) {
        if (!dynamic_meshes_.at(id))
            throw std::invalid_argument("mesh is not marked dynamic");
        return meshes_.at(id).vertices;
    }

    [[nodiscard]] std::span<const Vertex> dynamic_vertices(MeshId id) const {
        if (!dynamic_meshes_.at(id))
            throw std::invalid_argument("mesh is not marked dynamic");
        return meshes_.at(id).vertices;
    }

private:
    MeshId add_mesh(MeshData mesh, bool dynamic) {
        if (meshes_.size() > std::numeric_limits<MeshId>::max())
            throw std::length_error("scene exceeds the mesh ID limit");
        const MeshId id = static_cast<MeshId>(meshes_.size());
        meshes_.reserve(meshes_.size() + 1);
        dynamic_meshes_.reserve(dynamic_meshes_.size() + 1);
        meshes_.push_back(std::move(mesh));
        dynamic_meshes_.push_back(dynamic);
        asset_revision_ = next_asset_revision();
        return id;
    }

public:

    TransformId add_transform(Mat4 transform = Mat4::identity(),
                             TransformId parent = no_parent_transform) {
        if (transforms_.size() >= std::numeric_limits<TransformId>::max())
            throw std::length_error("scene exceeds the transform ID limit");
        if (parent != no_parent_transform && parent >= transforms_.size())
            throw std::out_of_range("transform parent must already exist in the scene");
        const TransformId id = static_cast<TransformId>(transforms_.size());
        transforms_.reserve(transforms_.size() + 1);
        transform_parents_.reserve(transform_parents_.size() + 1);
        transforms_.push_back(transform);
        transform_parents_.push_back(parent);
        return id;
    }

    void set_transform(TransformId id, Mat4 transform) {
        transforms_.at(id) = transform;
    }

    const Mat4& transform(TransformId id) const { return transforms_.at(id); }
    TransformId transform_parent(TransformId id) const { return transform_parents_.at(id); }

    void set_transform_parent(TransformId id, TransformId parent) {
        if (id == 0 || id >= transform_parents_.size())
            throw std::out_of_range("transform parent target is invalid");
        if (parent != no_parent_transform && parent >= id)
            throw std::invalid_argument("transform parent must precede its child");
        transform_parents_[id] = parent;
    }

    std::vector<Mat4> resolved_transforms() const {
        std::vector<Mat4> world;
        world.reserve(transforms_.size());
        for (std::size_t i = 0; i < transforms_.size(); ++i) {
            const TransformId parent = transform_parents_[i];
            world.push_back(parent == no_parent_transform
                                ? transforms_[i]
                                : Mat4::multiply(world[parent], transforms_[i]));
        }
        return world;
    }

    void add_draw(MeshId mesh, Mat4 model = Mat4::identity(), TransformId transform = 0) {
        add_draw(mesh, TextureId{0}, model, transform);
    }

    void add_draw(MeshId mesh, TextureId texture, Mat4 model = Mat4::identity(),
                  TransformId transform = 0) {
        if (mesh >= meshes_.size())
            throw std::out_of_range("draw references a mesh that is not in the scene");
        if (texture >= textures_.size())
            throw std::out_of_range("draw references a texture that is not in the scene");
        if (transform >= transforms_.size())
            throw std::out_of_range("draw references a transform that is not in the scene");
        draws_.push_back({mesh, texture, model, transform});
    }

    void set_model(std::size_t draw_index, Mat4 model) {
        draws_.at(draw_index).model = model;
    }

    void set_draw_mesh(std::size_t draw_index, MeshId mesh) {
        if (mesh >= meshes_.size())
            throw std::out_of_range("draw mesh is not in the scene");
        draws_.at(draw_index).mesh = mesh;
    }

    void set_draw_texture(std::size_t draw_index, TextureId texture) {
        if (texture >= textures_.size())
            throw std::out_of_range("draw texture is not in the scene");
        draws_.at(draw_index).texture = texture;
    }

    void set_draw_visible(std::size_t draw_index, bool visible) {
        draws_.at(draw_index).visible = visible;
    }

    void set_all_models(Mat4 model) {
        for (DrawItem& draw : draws_) draw.model = model;
    }

    void set_all_draw_transforms(TransformId transform) {
        if (transform >= transforms_.size())
            throw std::out_of_range("draw transform is not in the scene");
        for (DrawItem& draw : draws_) draw.transform = transform;
    }

    void set_all_draw_textures(TextureId texture) {
        if (texture >= textures_.size())
            throw std::out_of_range("draw texture is not in the scene");
        for (DrawItem& draw : draws_) draw.texture = texture;
    }

    void set_environment_texture(std::size_t draw_index, TextureId texture) {
        if (texture != no_environment_texture && texture >= textures_.size())
            throw std::out_of_range("environment texture is not in the scene");
        draws_.at(draw_index).environment_texture = texture;
    }

    // Merge a staged asset-only scene. Its transform table must match this
    // scene; texture zero maps to this scene's white texture.
    void append_assets(Scene&& staged) {
        if (staged.transforms_.size() != transforms_.size() ||
            staged.transform_parents_ != transform_parents_)
            throw std::invalid_argument("staged scene has a different transform table");
        for (std::size_t i = 0; i < transforms_.size(); ++i) {
            if (staged.transforms_[i].value != transforms_[i].value)
                throw std::invalid_argument("staged scene transform IDs do not match");
        }
        if (staged.textures_.empty())
            throw std::invalid_argument("staged scene has no default texture");
        for (const DrawItem& draw : staged.draws_) {
            if (draw.mesh >= staged.meshes_.size() ||
                draw.texture >= staged.textures_.size() ||
                (draw.environment_texture != no_environment_texture &&
                 draw.environment_texture >= staged.textures_.size()) ||
                draw.transform >= transforms_.size())
                throw std::invalid_argument("staged scene contains an invalid draw");
        }

        const std::size_t mesh_base = meshes_.size();
        const std::size_t texture_base = textures_.size() - 1;
        const std::uint64_t mesh_count = static_cast<std::uint64_t>(mesh_base) +
                                         staged.meshes_.size();
        const std::uint64_t texture_count = static_cast<std::uint64_t>(texture_base) +
                                            staged.textures_.size();
        if (mesh_count > static_cast<std::uint64_t>(std::numeric_limits<MeshId>::max()) + 1 ||
            texture_count > static_cast<std::uint64_t>(std::numeric_limits<TextureId>::max()) + 1 ||
            staged.meshes_.size() > meshes_.max_size() - mesh_base ||
            staged.textures_.size() - 1 > textures_.max_size() - textures_.size() ||
            staged.draws_.size() > draws_.max_size() - draws_.size())
            throw std::length_error("staged scene exceeds native scene limits");

        // Reserve every destination before moving data, so allocation failure
        // leaves all scene contents unchanged.
        meshes_.reserve(mesh_base + staged.meshes_.size());
        textures_.reserve(textures_.size() + staged.textures_.size() - 1);
        draws_.reserve(draws_.size() + staged.draws_.size());
        for (MeshData& mesh : staged.meshes_) meshes_.push_back(std::move(mesh));
        for (std::size_t i = 1; i < staged.textures_.size(); ++i)
            textures_.push_back(std::move(staged.textures_[i]));
        for (DrawItem draw : staged.draws_) {
            draw.mesh = static_cast<MeshId>(mesh_base + draw.mesh);
            if (draw.texture != 0)
                draw.texture = static_cast<TextureId>(texture_base + draw.texture);
            if (draw.environment_texture != no_environment_texture &&
                draw.environment_texture != 0)
                draw.environment_texture = static_cast<TextureId>(
                    texture_base + draw.environment_texture);
            draws_.push_back(draw);
        }
        if (!staged.meshes_.empty() || staged.textures_.size() > 1)
            asset_revision_ = next_asset_revision();
    }

    TextureId add_texture(TextureData texture) {
        if (textures_.size() > std::numeric_limits<TextureId>::max())
            throw std::length_error("scene exceeds the texture ID limit");
        const auto id = static_cast<TextureId>(textures_.size());
        textures_.push_back(std::move(texture));
        asset_revision_ = next_asset_revision();
        return id;
    }

    void set_texture(TextureData texture) {
        textures_.at(0) = std::move(texture);
        asset_revision_ = next_asset_revision();
    }

    // Drop scene assets while retaining entity transforms for the next state.
    // A renderer that uploaded the old assets will reject drawing until the
    // replacement scene assets have been uploaded.
    void clear_assets() {
        TextureData default_texture;
        meshes_.clear();
        dynamic_meshes_.clear();
        draws_.clear();
        textures_.resize(1);
        textures_[0] = std::move(default_texture);
        asset_revision_ = next_asset_revision();
    }

    std::uint64_t asset_revision() const { return asset_revision_; }

    const std::vector<MeshData>& meshes() const { return meshes_; }
    const std::vector<bool>& dynamic_meshes() const { return dynamic_meshes_; }
    const std::vector<DrawItem>& draws() const { return draws_; }
    const std::vector<TextureData>& textures() const { return textures_; }
    const std::vector<Mat4>& transforms() const { return transforms_; }
    const std::vector<TransformId>& transform_parents() const { return transform_parents_; }

private:
    static std::uint64_t next_asset_revision() {
        static std::atomic<std::uint64_t> next{1};
        std::uint64_t revision = next.fetch_add(1, std::memory_order_relaxed);
        if (revision == 0) revision = next.fetch_add(1, std::memory_order_relaxed);
        return revision;
    }

    std::vector<MeshData> meshes_;
    std::vector<bool> dynamic_meshes_;
    std::vector<DrawItem> draws_;
    std::vector<TextureData> textures_{TextureData{}};
    SceneLighting lighting_{};
    std::vector<Mat4> transforms_{Mat4::identity()};
    std::vector<TransformId> transform_parents_{no_parent_transform};
    std::uint64_t asset_revision_ = next_asset_revision();
};

} // namespace ac5
