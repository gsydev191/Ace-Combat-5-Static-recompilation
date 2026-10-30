#include "acm_loader.hpp"
#include "command_line.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <numeric>
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
    const bool ground_markers_mode =
        (arguments.size() == 3 || arguments.size() == 4) &&
        arguments[1] == "--ground-markers";
    const bool weapons_mode = arguments.size() == 3 && arguments[1] == "--weapons";
    const bool textures_mode = arguments.size() == 3 && arguments[1] == "--textures";
    const bool pm_textures_mode = arguments.size() == 3 && arguments[1] == "--pm-textures";
    const bool pm_materials_mode = arguments.size() == 4 && arguments[1] == "--pm-materials";
    const bool simple_mode = arguments.size() == 2;
    const bool help = (simple_mode &&
                       (arguments[1] == "--help" || arguments[1] == "-h")) ||
                      ((parts_mode || ground_markers_mode || weapons_mode || textures_mode || pm_textures_mode ||
                        pm_materials_mode) &&
                       (arguments[2] == "--help" || arguments[2] == "-h"));
    if (help || (!simple_mode && !parts_mode && !ground_markers_mode && !weapons_mode && !textures_mode &&
                 !pm_textures_mode && !pm_materials_mode)) {
        std::fprintf(help ? stdout : stderr,
                     "usage: ac5_model_info [--parts|--ground-markers|--weapons|--textures|--pm-textures] <model.acm|aircraft.mpl|model.bin>\n"
                     "       ac5_model_info --ground-markers <model.acm|aircraft.mpl|model.bin> [gear-channel]\n"
                     "       ac5_model_info --pm-materials <model.acm|model.bin> <pmtex.bin>\n");
        return help ? 0 : 2;
    }
    const std::size_t path_index =
        (parts_mode || ground_markers_mode || weapons_mode || textures_mode || pm_textures_mode) ? 2 : 1;
    if (ground_markers_mode) {
        ac5::AcmPartModel model;
        if (!ac5::load_acm_part_model(arguments[path_index], model, error)) {
            std::fprintf(stderr, "Part model load failed: %s\n", error.c_str());
            return 1;
        }
        std::array<float, 64> channels{};
        channels.fill(50.0f);
        if (arguments.size() == 4) {
            float channel_value = 0.0f;
            const std::string& text = arguments[3];
            const auto parsed = std::from_chars(
                text.data(), text.data() + text.size(), channel_value);
            if (parsed.ec != std::errc{} ||
                parsed.ptr != text.data() + text.size() ||
                !std::isfinite(channel_value) || channel_value < 0.0f ||
                channel_value > 100.0f) {
                std::fprintf(stderr, "Ground marker gear channel must be in [0, 100]\n");
                return 2;
            }
            for (std::size_t channel = 35; channel <= 37; ++channel)
                channels[channel] = channel_value;
        }
        const auto markers = ac5::sample_acm_ground_markers(model, channels);
        if (!markers) {
            std::fprintf(stderr, "Model does not provide valid I/K/J ground markers\n");
            return 1;
        }
        constexpr std::array<char, 3> labels{'I', 'K', 'J'};
        for (std::size_t marker = 0; marker < markers->size(); ++marker)
            std::printf("%c [%.6f %.6f %.6f]\n", labels[marker],
                        (*markers)[marker][0], (*markers)[marker][1],
                        (*markers)[marker][2]);
        return 0;
    }
    if (textures_mode || pm_textures_mode) {
        std::vector<ac5::AcmBundleTexture> textures;
        const bool loaded = pm_textures_mode
            ? ac5::load_pm_texture_bundle(arguments[path_index], textures, error)
            : ac5::load_acm_bundle_textures(arguments[path_index], textures, error);
        if (!loaded) {
            std::fprintf(stderr, "Texture bundle load failed: %s\n", error.c_str());
            return 1;
        }
        std::printf("%s textures: %zu\n", pm_textures_mode ? "PM full-resolution" : "indexed", textures.size());
        for (const ac5::AcmBundleTexture& item : textures) {
            std::printf("  slots");
            for (const std::uint32_t slot : item.bundle_slots)
                std::printf(" / %u", slot);
            std::printf(": %ux%u\n", item.texture.width, item.texture.height);
        }
        return 0;
    }
    if (pm_materials_mode) {
        ac5::AcmPartModel model;
        std::vector<ac5::AcmBundleTexture> textures;
        if (!ac5::load_acm_part_model(arguments[2], model, error)) {
            std::fprintf(stderr, "Part model load failed: %s\n", error.c_str());
            return 1;
        }
        if (!ac5::load_pm_texture_bundle(arguments[3], textures, error)) {
            std::fprintf(stderr, "PM material texture load failed: %s\n", error.c_str());
            return 1;
        }
        std::map<std::uint32_t, std::size_t> uses;
        std::size_t invalid = 0;
        for (const ac5::AcmModelPart& part : model.parts) {
            if (part.mesh.indices.empty()) continue;
            const std::uint32_t slot = part.source_words[3];
            ++uses[slot];
            invalid += std::none_of(textures.begin(), textures.end(),
                [slot](const ac5::AcmBundleTexture& texture) {
                    return texture.bundle_slots.size() == 2 &&
                           texture.bundle_slots[1] == slot;
                });
        }
        std::printf("geometry parts: %zu\nfull-resolution textures: %zu\n",
                    std::accumulate(uses.begin(), uses.end(), std::size_t{0},
                        [](std::size_t total, const auto& use) { return total + use.second; }),
                    textures.size());
        for (const auto& [slot, count] : uses)
            std::printf("  material slot %u: %zu part(s)%s\n", slot, count,
                std::any_of(textures.begin(), textures.end(),
                    [slot](const ac5::AcmBundleTexture& texture) {
                        return texture.bundle_slots.size() == 2 &&
                               texture.bundle_slots[1] == slot;
                    }) ? "" : " [MISSING TEXTURE]");
        std::printf("unresolved material references: %zu\n", invalid);
        return invalid == 0 ? 0 : 1;
    }
    if (weapons_mode) {
        ac5::AircraftWeaponRules rules;
        bool found = false;
        if (!ac5::load_acm_weapon_rules(arguments[path_index], rules, found, error)) {
            std::fprintf(stderr, "Weapon data load failed: %s\n", error.c_str());
            return 1;
        }
        if (!found) {
            std::fprintf(stderr, "No supported aircraft weapon bundle found.\n");
            return 1;
        }
        std::printf("gun ammo: %u%s\nmissile ammo: %u\nspecial ammo: %u (weapon id %u)\n"
                    "gun interval: %.4f s, mount pitch: %.3f deg, spread: %.3f deg, gravity: %.3f m/s^2, muzzle speed: %u km/h\n"
                    "standard missile reload: %.3f s, range2: %.1f m, turn rate: %.3f deg/s, gravity record: %.3f m/s^2, launch speed: %u km/h, seeker half-angle: %u deg\n",
                    rules.gun_ammunition,
                    rules.gun_ammunition == ac5::unlimited_ammunition_sentinel
                        ? " (unlimited)" : "",
                    rules.missile_ammunition, rules.special_ammunition,
                    rules.special_weapon_id, rules.gun_interval_seconds,
                    rules.gun_mount_pitch_degrees, rules.gun_spread_degrees,
                    rules.gun_gravity_mps2,
                    rules.gun_muzzle_speed_kmh, rules.standard_reload_seconds,
                    rules.missile_range2_meters,
                    rules.missile_turn_rate_degrees_per_second,
                    rules.missile_gravity_mps2, rules.missile_launch_speed_kmh,
                    rules.seeker_half_angle_degrees);
        return 0;
    }
    if (parts_mode) {
        ac5::AcmPartModel model;
        if (!ac5::load_acm_part_model(arguments[path_index], model, error)) {
            std::fprintf(stderr, "Part model load failed: %s\n", error.c_str());
            return 1;
        }
        std::size_t vertices = 0, triangles = 0, geometry_parts = 0;
        std::size_t shape_variants = 0;
        std::size_t translation_tracks = 0, rotation_tracks = 0, shape_tracks = 0;
        std::size_t translation_keys = 0, rotation_keys = 0, shape_keys = 0;
        std::size_t continuous_shape_parts = 0;
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
            shape_variants += part.shape_variants.size();
            translation_tracks += part.translation_track != ac5::AcmModelPart::no_parent;
            rotation_tracks += part.rotation_track != ac5::AcmModelPart::no_parent;
            shape_tracks += part.shape_track != ac5::AcmModelPart::no_parent;
            if (part.shape_track != ac5::AcmModelPart::no_parent &&
                !part.shape_variants.empty()) {
                bool compatible = true;
                for (const auto& variant : part.shape_variants)
                    compatible = compatible &&
                        variant.mesh.vertices.size() == part.mesh.vertices.size() &&
                        variant.mesh.indices == part.mesh.indices;
                const auto& track = model.shape_tracks[part.shape_track];
                for (const auto& key : track.keys) {
                    compatible = compatible && std::any_of(
                        part.shape_variants.begin(), part.shape_variants.end(),
                        [&key](const auto& variant) {
                            return variant.chain_index == key.shape_index;
                        });
                }
                continuous_shape_parts += compatible;
            }
        }
        std::printf("parts: %zu\ngeometry parts: %zu\nvertices: %zu\ntriangles: %zu\n"
                    "part track references: translation %zu, rotation %zu, shape %zu\n"
                    "shape variants: %zu, continuous-compatible parts: %zu\n"
                    "decoded tracks: translation %zu (%zu keys), rotation %zu (%zu keys), "
                    "shape %zu (%zu keys)\n",
                    model.parts.size(), geometry_parts, vertices, triangles,
                    translation_tracks, rotation_tracks, shape_tracks,
                    shape_variants, continuous_shape_parts,
                    model.translation_tracks.size(), translation_keys,
                    model.rotation_tracks.size(), rotation_keys,
                    model.shape_tracks.size(), shape_keys);
        for (std::size_t i = 0; i < model.parts.size(); ++i) {
            const ac5::AcmModelPart& part = model.parts[i];
            std::printf("  part %zu kind %u", i, part.kind);
            if (part.attachment_code != '\0')
                std::printf(" attachment '%c'", part.attachment_code);
            std::printf(" parent ");
            if (part.parent == ac5::AcmModelPart::no_parent) std::printf("root");
            else std::printf("%u", part.parent);
            std::printf(" chain %u source [", part.source_words[2]);
            for (std::size_t word = 0; word < part.source_words.size(); ++word)
                std::printf("%s%08x", word == 0 ? "" : " ", part.source_words[word]);
            std::printf("]: %zu triangles, local position [%.4f %.4f %.4f], "
                        "rotation [%.4f %.4f %.4f]\n",
                        part.mesh.indices.size() / 3,
                        part.position.x, part.position.y, part.position.z,
                        part.rotation.x, part.rotation.y, part.rotation.z);
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
