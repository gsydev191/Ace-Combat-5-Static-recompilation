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
    if (arguments.size() != 2 || arguments[1] == "--help" || arguments[1] == "-h") {
        std::fprintf(arguments.size() == 2 ? stdout : stderr,
                     "usage: ac5_model_info <model.acm|aircraft.mpl|model.bin>\n");
        return arguments.size() == 2 ? 0 : 2;
    }

    ac5::MeshData mesh;
    if (!ac5::load_acm_model(arguments[1], mesh, error)) {
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
    if (ac5::load_acm_bundle_texture(arguments[1], texture, has_texture, error) && has_texture)
        std::printf("bundled texture: %ux%u\n", texture.width, texture.height);
    ac5::AcmLodBundle lods;
    bool has_lods = false;
    if (!ac5::load_acm_lod_bundle(arguments[1], lods, has_lods, error)) {
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
