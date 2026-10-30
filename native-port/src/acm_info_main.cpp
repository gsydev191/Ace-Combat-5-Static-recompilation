#include "acm_loader.hpp"
#include "command_line.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    std::vector<std::string> arguments;
    std::string error;
    if (!ac5::command_line_arguments_utf8(argc, argv, arguments, error)) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 2;
    }
    const bool parts_mode = arguments.size() == 3 && arguments[1] == "--parts";
    const bool simple_mode = arguments.size() == 2;
    const bool help = (simple_mode &&
                       (arguments[1] == "--help" || arguments[1] == "-h")) ||
                      (parts_mode &&
                       (arguments[2] == "--help" || arguments[2] == "-h"));
    if (help || (!simple_mode && !parts_mode)) {
        std::fprintf(help ? stdout : stderr,
                     "usage: ac5_model_info [--parts] <model.acm|aircraft.mpl|model.bin>\n");
        return help ? 0 : 2;
    }
    const std::size_t path_index = parts_mode ? 2 : 1;
    if (parts_mode) {
        ac5::AcmPartModel model;
        if (!ac5::load_acm_part_model(arguments[path_index], model, error)) {
            std::fprintf(stderr, "Part model load failed: %s\n", error.c_str());
            return 1;
        }
        std::size_t vertices = 0, triangles = 0, geometry_parts = 0;
        std::size_t translation_tracks = 0, rotation_tracks = 0, shape_tracks = 0;
        std::size_t translation_keys = 0, rotation_keys = 0, shape_keys = 0;
        for (const auto& track : model.translation_tracks)
            translation_keys += track.keys.size();
        for (const auto& track : model.rotation_tracks)
            rotation_keys += track.keys.size();
        for (const auto& track : model.shape_tracks)
            shape_keys += track.keys.size();
        for (const ac5::AcmModelPart& part : model.parts) {
            vertices += part.mesh.vertices.size();
            triangles += part.mesh.indices.size() / 3;
            geometry_parts += !part.mesh.indices.empty();
            translation_tracks += part.translation_track != ac5::AcmModelPart::no_parent;
            rotation_tracks += part.rotation_track != ac5::AcmModelPart::no_parent;
            shape_tracks += part.shape_track != ac5::AcmModelPart::no_parent;
        }
        std::printf("parts: %zu\ngeometry parts: %zu\nvertices: %zu\ntriangles: %zu\n"
                    "part track references: translation %zu, rotation %zu, shape %zu\n"
                    "decoded tracks: translation %zu (%zu keys), rotation %zu (%zu keys), "
                    "shape %zu (%zu keys)\n",
                    model.parts.size(), geometry_parts, vertices, triangles,
                    translation_tracks, rotation_tracks, shape_tracks,
                    model.translation_tracks.size(), translation_keys,
                    model.rotation_tracks.size(), rotation_keys,
                    model.shape_tracks.size(), shape_keys);
        for (std::size_t i = 0; i < model.parts.size(); ++i) {
            const ac5::AcmModelPart& part = model.parts[i];
            if (part.mesh.indices.empty()) continue;
            std::printf("  part %zu kind %u parent ", i, part.kind);
            if (part.parent == ac5::AcmModelPart::no_parent) std::printf("root");
            else std::printf("%u", part.parent);
            std::printf(": %zu triangles\n", part.mesh.indices.size() / 3);
        }
        return 0;
    }

    ac5::MeshData mesh;
    if (!ac5::load_acm_model(arguments[path_index], mesh, error)) {
        std::fprintf(stderr, "Model load failed: %s\n", error.c_str());
        return 1;
    }
    ac5::Vec3 minimum{std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::max()};
    ac5::Vec3 maximum{std::numeric_limits<float>::lowest(),
                      std::numeric_limits<float>::lowest(),
                      std::numeric_limits<float>::lowest()};
    for (const ac5::Vertex& vertex : mesh.vertices) {
        minimum.x = std::min(minimum.x, vertex.position.x);
        minimum.y = std::min(minimum.y, vertex.position.y);
        minimum.z = std::min(minimum.z, vertex.position.z);
        maximum.x = std::max(maximum.x, vertex.position.x);
        maximum.y = std::max(maximum.y, vertex.position.y);
        maximum.z = std::max(maximum.z, vertex.position.z);
    }
    std::printf("vertices: %zu\ntriangles: %zu\n"
                "bounds: [%.3f, %.3f, %.3f] to [%.3f, %.3f, %.3f]\n",
                mesh.vertices.size(), mesh.indices.size() / 3,
                minimum.x, minimum.y, minimum.z, maximum.x, maximum.y, maximum.z);
    ac5::TextureData texture;
    bool has_texture = false;
    if (ac5::load_acm_bundle_texture(arguments[path_index], texture, has_texture, error) && has_texture)
        std::printf("bundled texture: %ux%u\n", texture.width, texture.height);
    ac5::AcmLodBundle lods;
    bool has_lods = false;
    if (!ac5::load_acm_lod_bundle(arguments[path_index], lods, has_lods, error)) {
        std::fprintf(stderr, "LOD import failed: %s\n", error.c_str());
        return 1;
    }
    if (has_lods) {
        std::printf("LOD levels: %zu\n", lods.levels.size());
        for (std::size_t i = 0; i < lods.levels.size(); ++i) {
            const auto& level = lods.levels[i];
            std::printf("  %zu: threshold %.3f, %zu vertices, %zu triangles",
                        i, level.maximum_distance, level.mesh.vertices.size(),
                        level.mesh.indices.size() / 3);
            if (level.has_texture)
                std::printf(", texture %ux%u", level.texture.width, level.texture.height);
            std::printf("\n");
        }
    }
    return 0;
}
