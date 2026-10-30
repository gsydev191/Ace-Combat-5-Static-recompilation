#pragma once

#include "scene.hpp"

#include <cstdint>
#include <limits>
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

struct AcmModelPart {
    static constexpr std::uint32_t no_parent =
        std::numeric_limits<std::uint32_t>::max();
    std::uint32_t parent = no_parent;
    std::uint8_t kind = 0;
    std::uint32_t translation_track = no_parent;
    std::uint32_t rotation_track = no_parent;
    std::uint32_t shape_track = no_parent;
    Vec3 position{};
    Vec3 rotation{};
    MeshData mesh;
    struct ShapeVariant {
        std::int32_t chain_index = -1;
        MeshData mesh;
    };
    std::vector<ShapeVariant> shape_variants;
};

struct AcmAnimationKey {
    Vec3 value{};
    std::int32_t frame = 0;
    std::int32_t shape_index = -1;
};

struct AcmAnimationTrack {
    std::uint32_t channel = 0;
    std::uint32_t end_frame = 0;
    std::vector<AcmAnimationKey> keys;
};

struct AcmPartModel {
    // Second ACM header word, used by the retail aircraft animation controller
    // to select alternate control-surface mappings.
    std::uint32_t model_flags = 0;
    std::vector<AcmModelPart> parts;
    std::vector<AcmAnimationTrack> translation_tracks;
    std::vector<AcmAnimationTrack> rotation_tracks;
    std::vector<AcmAnimationTrack> shape_tracks;
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
// Preserve the ACM pre-order part tree, local meshes, and translation/rotation/
// shape track tables instead of baking all part transforms into one static mesh.
bool load_acm_part_model(const std::string& path, AcmPartModel& model,
                         std::string& error);
[[nodiscard]] bool sample_acm_animation_track(
    const AcmAnimationTrack& track, double frame, Vec3& value) noexcept;
bool load_acm_bundle_texture(const std::string& path, TextureData& texture,
                             bool& found, std::string& error);

} // namespace ac5
