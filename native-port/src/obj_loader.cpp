#include "obj_loader.hpp"

#include "texture_loader.hpp"
#include "utf8_path.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ac5 {

namespace {

struct ObjCorner {
    std::uint32_t position = 0;
    std::uint32_t uv = 0;
    std::uint32_t normal = 0;
    bool has_uv = false;
    bool has_normal = false;
};

struct VertexKey {
    std::uint32_t position;
    std::uint32_t uv;
    std::uint32_t normal;
    std::uint32_t smoothing_group;
    std::size_t flat_face;

    bool operator==(const VertexKey&) const = default;
};

struct ObjSubmesh {
    std::string material;
    MeshData mesh;
    std::vector<VertexKey> generated_normal_keys;
};

struct ObjMaterial {
    Vec3 diffuse{1.0f, 1.0f, 1.0f};
    float opacity = 1.0f;
    std::filesystem::path diffuse_map;
};

struct ParsedObj {
    std::vector<ObjSubmesh> submeshes;
    std::vector<std::string> material_libraries;
};

struct VertexKeyHash {
    std::size_t operator()(const VertexKey& key) const noexcept {
        std::size_t hash = key.position;
        hash ^= static_cast<std::size_t>(key.uv) + 0x9e3779b9u + (hash << 6) + (hash >> 2);
        hash ^= static_cast<std::size_t>(key.normal) + 0x9e3779b9u + (hash << 6) + (hash >> 2);
        hash ^= static_cast<std::size_t>(key.smoothing_group) + 0x9e3779b9u + (hash << 6) + (hash >> 2);
        hash ^= key.flat_face + 0x9e3779b9u + (hash << 6) + (hash >> 2);
        return hash;
    }
};

Vec3 normalized(Vec3 value) {
    const float length = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
    if (!std::isfinite(length) || length <= 1.0e-12f) return {};
    return {value.x / length, value.y / length, value.z / length};
}

Vec3 face_normal_vector(const std::vector<ObjCorner>& face,
                        const std::vector<Vertex>& positions) {
    Vec3 normal{};
    for (std::size_t i = 0; i < face.size(); ++i) {
        const Vec3 a = positions[face[i].position].position;
        const Vec3 b = positions[face[(i + 1) % face.size()].position].position;
        normal.x += (a.y - b.y) * (a.z + b.z);
        normal.y += (a.z - b.z) * (a.x + b.x);
        normal.z += (a.x - b.x) * (a.y + b.y);
    }
    return normal;
}

double cross_2d(const std::array<double, 2>& a, const std::array<double, 2>& b,
                const std::array<double, 2>& c) {
    return (b[0] - a[0]) * (c[1] - a[1]) -
           (b[1] - a[1]) * (c[0] - a[0]);
}

bool triangulate_face(const std::vector<ObjCorner>& face,
                      const std::vector<Vertex>& positions, Vec3 normal,
                      std::vector<std::array<std::size_t, 3>>& triangles) {
    const float ax = std::abs(normal.x), ay = std::abs(normal.y), az = std::abs(normal.z);
    const int dropped_axis = ax >= ay && ax >= az ? 0 : ay >= az ? 1 : 2;
    std::vector<std::array<double, 2>> projected;
    projected.reserve(face.size());
    double coordinate_scale = 1.0;
    for (const ObjCorner& corner : face) {
        const Vec3 p = positions[corner.position].position;
        std::array<double, 2> point{};
        if (dropped_axis == 0) point = {p.y, p.z};
        else if (dropped_axis == 1) point = {p.x, p.z};
        else point = {p.x, p.y};
        coordinate_scale = std::max({coordinate_scale, std::abs(point[0]), std::abs(point[1])});
        projected.push_back(point);
    }

    double signed_area = 0.0;
    for (std::size_t i = 0; i < projected.size(); ++i) {
        const auto& a = projected[i];
        const auto& b = projected[(i + 1) % projected.size()];
        signed_area += a[0] * b[1] - b[0] * a[1];
    }
    const double epsilon = coordinate_scale * coordinate_scale * 1.0e-12;
    if (!std::isfinite(signed_area) || std::abs(signed_area) <= epsilon) return false;
    const double winding = signed_area > 0.0 ? 1.0 : -1.0;

    std::vector<std::size_t> remaining(face.size());
    for (std::size_t i = 0; i < remaining.size(); ++i) remaining[i] = i;
    std::vector<std::array<std::size_t, 3>> result;
    result.reserve(face.size() - 2);
    while (remaining.size() > 3) {
        bool clipped = false;
        for (std::size_t i = 0; i < remaining.size(); ++i) {
            const std::size_t previous = remaining[(i + remaining.size() - 1) % remaining.size()];
            const std::size_t current = remaining[i];
            const std::size_t next = remaining[(i + 1) % remaining.size()];
            if (winding * cross_2d(projected[previous], projected[current],
                                   projected[next]) <= epsilon) continue;

            bool contains_corner = false;
            for (std::size_t candidate : remaining) {
                if (candidate == previous || candidate == current || candidate == next) continue;
                const double a = winding * cross_2d(projected[previous], projected[current],
                                                    projected[candidate]);
                const double b = winding * cross_2d(projected[current], projected[next],
                                                    projected[candidate]);
                const double c = winding * cross_2d(projected[next], projected[previous],
                                                    projected[candidate]);
                if (a > epsilon && b > epsilon && c > epsilon) {
                    contains_corner = true;
                    break;
                }
            }
            if (contains_corner) continue;

            result.push_back({previous, current, next});
            remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
            break;
        }
        if (!clipped) return false;
    }
    if (winding * cross_2d(projected[remaining[0]], projected[remaining[1]],
                           projected[remaining[2]]) <= epsilon) return false;
    result.push_back({remaining[0], remaining[1], remaining[2]});
    triangles = std::move(result);
    return true;
}

bool parse_obj_index(std::string_view token, std::size_t element_count,
                     std::uint32_t& index) {
    int parsed = 0;
    const auto result = std::from_chars(token.data(), token.data() + token.size(), parsed);
    if (token.empty() || result.ec != std::errc{} ||
        result.ptr != token.data() + token.size() || parsed == 0) return false;
    const std::int64_t resolved = parsed > 0
        ? static_cast<std::int64_t>(parsed) - 1
        : static_cast<std::int64_t>(element_count) + parsed;
    if (resolved < 0 || static_cast<std::uint64_t>(resolved) >= element_count ||
        resolved > std::numeric_limits<std::uint32_t>::max()) return false;
    index = static_cast<std::uint32_t>(resolved);
    return true;
}

bool parse_float(std::string_view token, float& value) {
    const auto parsed = std::from_chars(token.data(), token.data() + token.size(), value);
    return !token.empty() && parsed.ec == std::errc{} &&
           parsed.ptr == token.data() + token.size() && std::isfinite(value);
}

bool parse_face_corner(std::string_view token, std::size_t position_count,
                       std::size_t uv_count, std::size_t normal_count,
                       ObjCorner& corner) {
    const std::size_t first_slash = token.find('/');
    if (!parse_obj_index(token.substr(0, first_slash), position_count, corner.position))
        return false;
    if (first_slash == std::string_view::npos) return true;
    const std::size_t second_slash = token.find('/', first_slash + 1);
    if (second_slash == std::string_view::npos) {
        const std::string_view uv_token = token.substr(first_slash + 1);
        return !uv_token.empty() &&
               parse_obj_index(uv_token, uv_count, corner.uv) &&
               (corner.has_uv = true);
    }
    if (token.find('/', second_slash + 1) != std::string_view::npos) return false;
    const std::string_view uv_token = token.substr(first_slash + 1,
                                                   second_slash - first_slash - 1);
    const std::string_view normal_token = token.substr(second_slash + 1);
    if (normal_token.empty() ||
        !parse_obj_index(normal_token, normal_count, corner.normal)) return false;
    corner.has_normal = true;
    if (!uv_token.empty()) {
        if (!parse_obj_index(uv_token, uv_count, corner.uv)) return false;
        corner.has_uv = true;
    }
    return true;
}

bool parse_obj(const std::string& path, ParsedObj& output, std::string& error) {
    std::ifstream file(path_from_utf8(path));
    if (!file) {
        error = "cannot open OBJ file: " + path;
        return false;
    }

    ParsedObj parsed;
    parsed.submeshes.emplace_back();
    std::vector<Vertex> positions;
    std::vector<Vec2> texcoords;
    std::vector<Vec3> normals;
    std::unordered_map<VertexKey, std::uint32_t, VertexKeyHash> vertex_lookup;
    std::unordered_map<VertexKey, Vec3, VertexKeyHash> generated_normal_sums;
    std::size_t current_submesh = 0;
    std::size_t face_number = 0;
    std::uint32_t smoothing_group = 0;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(file, line)) {
        ++line_number;
        std::istringstream row(line);
        std::string record;
        row >> record;
        if (record.empty() || record[0] == '#') continue;
        if (record == "v") {
            Vertex vertex{{}, {0.78f, 0.82f, 0.88f}, {}};
            if (!(row >> vertex.position.x >> vertex.position.y >> vertex.position.z)) {
                error = path + ":" + std::to_string(line_number) + ": invalid vertex";
                return false;
            }
            std::vector<float> extra;
            std::string token;
            while (row >> token) {
                if (token[0] == '#') break;
                float value = 0.0f;
                const auto parsed_value = std::from_chars(
                    token.data(), token.data() + token.size(), value);
                if (parsed_value.ec != std::errc{} ||
                    parsed_value.ptr != token.data() + token.size()) {
                    error = path + ":" + std::to_string(line_number) + ": invalid vertex field";
                    return false;
                }
                extra.push_back(value);
            }
            if (extra.size() == 2 || extra.size() > 4) {
                error = path + ":" + std::to_string(line_number) + ": invalid vertex component count";
                return false;
            }
            if (!std::isfinite(vertex.position.x) || !std::isfinite(vertex.position.y) ||
                !std::isfinite(vertex.position.z) ||
                !std::all_of(extra.begin(), extra.end(),
                             [](float value) { return std::isfinite(value); })) {
                error = path + ":" + std::to_string(line_number) + ": non-finite vertex field";
                return false;
            }
            if (extra.size() == 1 || extra.size() == 4) {
                const float weight = extra[0];
                if (weight == 0.0f) {
                    error = path + ":" + std::to_string(line_number) + ": zero vertex weight";
                    return false;
                }
                vertex.position.x /= weight;
                vertex.position.y /= weight;
                vertex.position.z /= weight;
                if (!std::isfinite(vertex.position.x) || !std::isfinite(vertex.position.y) ||
                    !std::isfinite(vertex.position.z)) {
                    error = path + ":" + std::to_string(line_number) + ": invalid weighted vertex";
                    return false;
                }
            }
            if (extra.size() == 3 || extra.size() == 4) {
                const std::size_t color_start = extra.size() == 4 ? 1 : 0;
                vertex.color = {extra[color_start], extra[color_start + 1],
                                extra[color_start + 2]};
                vertex.color.x = std::clamp(vertex.color.x, 0.0f, 1.0f);
                vertex.color.y = std::clamp(vertex.color.y, 0.0f, 1.0f);
                vertex.color.z = std::clamp(vertex.color.z, 0.0f, 1.0f);
            }
            if (positions.size() >= std::numeric_limits<std::uint32_t>::max()) {
                error = path + ": mesh exceeds the 32-bit vertex limit";
                return false;
            }
            positions.push_back(vertex);
        } else if (record == "vt") {
            Vec2 uv{};
            if (!(row >> uv.x >> uv.y) || !std::isfinite(uv.x) || !std::isfinite(uv.y)) {
                error = path + ":" + std::to_string(line_number) + ": invalid texture coordinate";
                return false;
            }
            if (texcoords.size() >= std::numeric_limits<std::uint32_t>::max()) {
                error = path + ": texture coordinate count exceeds the 32-bit index limit";
                return false;
            }
            texcoords.push_back(uv);
        } else if (record == "vn") {
            Vec3 normal{};
            if (!(row >> normal.x >> normal.y >> normal.z) ||
                !std::isfinite(normal.x) || !std::isfinite(normal.y) ||
                !std::isfinite(normal.z)) {
                error = path + ":" + std::to_string(line_number) + ": invalid normal";
                return false;
            }
            if (normals.size() >= std::numeric_limits<std::uint32_t>::max()) {
                error = path + ": normal count exceeds the 32-bit index limit";
                return false;
            }
            normals.push_back(normal);
        } else if (record == "s") {
            std::string group;
            if (!(row >> group)) {
                error = path + ":" + std::to_string(line_number) + ": missing smoothing group";
                return false;
            }
            if (group == "off" || group == "0") {
                smoothing_group = 0;
            } else if (group == "on") {
                smoothing_group = 1;
            } else {
                std::uint32_t parsed_group = 0;
                const auto parsed_value = std::from_chars(
                    group.data(), group.data() + group.size(), parsed_group);
                if (parsed_value.ec != std::errc{} ||
                    parsed_value.ptr != group.data() + group.size() || parsed_group == 0) {
                    error = path + ":" + std::to_string(line_number) +
                            ": invalid smoothing group";
                    return false;
                }
                smoothing_group = parsed_group;
            }
        } else if (record == "mtllib") {
            std::string library;
            while (row >> library) {
                if (!library.empty() && library[0] == '#') break;
                parsed.material_libraries.push_back(library);
            }
        } else if (record == "usemtl") {
            std::string material;
            if (!(row >> material)) {
                error = path + ":" + std::to_string(line_number) + ": missing material name";
                return false;
            }
            ObjSubmesh& current = parsed.submeshes[current_submesh];
            if (!current.mesh.indices.empty()) {
                parsed.submeshes.emplace_back();
                current_submesh = parsed.submeshes.size() - 1;
                vertex_lookup.clear();
            }
            parsed.submeshes[current_submesh].material = std::move(material);
        } else if (record == "f") {
            ObjSubmesh& submesh = parsed.submeshes[current_submesh];
            std::vector<ObjCorner> face;
            std::string token;
            while (row >> token) {
                if (!token.empty() && token[0] == '#') break;
                ObjCorner corner;
                if (!parse_face_corner(token, positions.size(), texcoords.size(),
                                       normals.size(), corner)) {
                    error = path + ":" + std::to_string(line_number) + ": invalid face index";
                    return false;
                }
                face.push_back(corner);
            }
            if (face.size() < 3) {
                error = path + ":" + std::to_string(line_number) + ": face has fewer than 3 vertices";
                return false;
            }
            const Vec3 weighted_normal = face_normal_vector(face, positions);
            const Vec3 generated_normal = normalized(weighted_normal);
            if (generated_normal.x == 0.0f && generated_normal.y == 0.0f &&
                generated_normal.z == 0.0f) {
                error = path + ":" + std::to_string(line_number) + ": degenerate face";
                return false;
            }
            std::vector<std::array<std::size_t, 3>> triangles;
            if (!triangulate_face(face, positions, generated_normal, triangles)) {
                error = path + ":" + std::to_string(line_number) +
                        ": face cannot be triangulated (degenerate or invalid polygon)";
                return false;
            }
            std::vector<std::uint32_t> face_indices;
            face_indices.reserve(face.size());
            for (const ObjCorner& corner : face) {
                const std::uint32_t normal_group = corner.has_normal ? 0 : smoothing_group;
                const std::size_t flat_face = corner.has_normal || smoothing_group != 0
                    ? 0 : face_number;
                const VertexKey generated_key{corner.position, UINT32_MAX, UINT32_MAX,
                                              normal_group, flat_face};
                if (!corner.has_normal) {
                    Vec3& sum = generated_normal_sums[generated_key];
                    sum.x += weighted_normal.x;
                    sum.y += weighted_normal.y;
                    sum.z += weighted_normal.z;
                }
                const VertexKey key{corner.position,
                                    corner.has_uv ? corner.uv : UINT32_MAX,
                                    corner.has_normal ? corner.normal : UINT32_MAX,
                                    normal_group, flat_face};
                auto found = vertex_lookup.find(key);
                if (found == vertex_lookup.end()) {
                    if (submesh.mesh.vertices.size() >=
                        std::numeric_limits<std::uint32_t>::max()) {
                        error = path + ": mesh exceeds the 32-bit vertex limit";
                        return false;
                    }
                    Vertex vertex = positions[corner.position];
                    if (corner.has_uv) vertex.uv = texcoords[corner.uv];
                    vertex.normal = corner.has_normal
                        ? normalized(normals[corner.normal]) : generated_normal;
                    if (vertex.normal.x == 0.0f && vertex.normal.y == 0.0f &&
                        vertex.normal.z == 0.0f) {
                        error = path + ":" + std::to_string(line_number) + ": zero vertex normal";
                        return false;
                    }
                    const auto index = static_cast<std::uint32_t>(submesh.mesh.vertices.size());
                    submesh.mesh.vertices.push_back(vertex);
                    submesh.generated_normal_keys.push_back(corner.has_normal
                        ? VertexKey{corner.position, UINT32_MAX, corner.normal, 0, 0}
                        : generated_key);
                    found = vertex_lookup.emplace(key, index).first;
                }
                face_indices.push_back(found->second);
            }
            for (const auto& triangle : triangles) {
                submesh.mesh.indices.push_back(face_indices[triangle[0]]);
                submesh.mesh.indices.push_back(face_indices[triangle[1]]);
                submesh.mesh.indices.push_back(face_indices[triangle[2]]);
            }
            ++face_number;
        }
    }
    if (file.bad()) {
        error = "error while reading OBJ file: " + path;
        return false;
    }
    std::erase_if(parsed.submeshes, [](const ObjSubmesh& submesh) {
        return submesh.mesh.indices.empty();
    });
    if (parsed.submeshes.empty()) {
        error = "OBJ contains no triangle faces: " + path;
        return false;
    }
    for (ObjSubmesh& submesh : parsed.submeshes) {
        for (std::size_t i = 0; i < submesh.mesh.vertices.size(); ++i) {
            const VertexKey& key = submesh.generated_normal_keys[i];
            if (key.normal != UINT32_MAX) continue;
            const auto sum = generated_normal_sums.find(key);
            if (sum != generated_normal_sums.end()) {
                const Vec3 normal = normalized(sum->second);
                if (normal.x != 0.0f || normal.y != 0.0f || normal.z != 0.0f)
                    submesh.mesh.vertices[i].normal = normal;
            }
        }
    }
    output = std::move(parsed);
    return true;
}

bool load_materials(const std::filesystem::path& obj_path, const ParsedObj& obj,
                    bool load_textures,
                    std::unordered_map<std::string, ObjMaterial>& materials,
                    std::string& error) {
    for (const std::string& library_name : obj.material_libraries) {
        const std::filesystem::path library_path = obj_path.parent_path() /
                                                   path_from_utf8(library_name);
        std::ifstream library(library_path);
        if (!library) {
            if (!load_textures) continue;
            error = "cannot open OBJ material library: " + path_to_utf8(library_path);
            return false;
        }
        std::string material;
        std::string line;
        while (std::getline(library, line)) {
            const auto comment = line.find('#');
            if (comment != std::string::npos) line.resize(comment);
            std::istringstream row(line);
            std::string record;
            row >> record;
            if (record == "newmtl") {
                if (!(row >> material)) {
                    error = path_to_utf8(library_path) + ": invalid material declaration";
                    return false;
                }
                materials.try_emplace(material);
            } else if (record == "Kd" && !material.empty()) {
                Vec3 diffuse{};
                std::string red, green, blue, extra;
                if (!(row >> red >> green >> blue) || (row >> extra) ||
                    !parse_float(red, diffuse.x) || !parse_float(green, diffuse.y) ||
                    !parse_float(blue, diffuse.z)) {
                    error = path_to_utf8(library_path) + ": invalid Kd diffuse color";
                    return false;
                }
                diffuse.x = std::clamp(diffuse.x, 0.0f, 1.0f);
                diffuse.y = std::clamp(diffuse.y, 0.0f, 1.0f);
                diffuse.z = std::clamp(diffuse.z, 0.0f, 1.0f);
                materials[material].diffuse = diffuse;
            } else if ((record == "d" || record == "Tr") && !material.empty()) {
                std::string token, extra;
                float value = 0.0f;
                if (!(row >> token) || (row >> extra) || !parse_float(token, value)) {
                    error = path_to_utf8(library_path) + ": invalid " + record + " opacity";
                    return false;
                }
                const float opacity = record == "d" ? value : 1.0f - value;
                materials[material].opacity = std::clamp(opacity, 0.0f, 1.0f);
            } else if (record == "map_Kd" && !material.empty() && load_textures) {
                std::string texture_name;
                std::getline(row, texture_name);
                const auto comment = texture_name.find('#');
                if (comment != std::string::npos) texture_name.resize(comment);
                const auto first = texture_name.find_first_not_of(" \t");
                if (first == std::string::npos || texture_name[first] == '-') {
                    error = path_to_utf8(library_path) + ": unsupported or missing map_Kd path";
                    return false;
                }
                const auto last = texture_name.find_last_not_of(" \t\r");
                texture_name = texture_name.substr(first, last - first + 1);
                const std::filesystem::path texture_path = library_path.parent_path() /
                                                           path_from_utf8(texture_name);
                materials[material].diffuse_map = texture_path.lexically_normal();
            }
        }
        if (library.bad()) {
            error = "error while reading OBJ material library: " + path_to_utf8(library_path);
            return false;
        }
    }
    return true;
}

} // namespace

bool load_obj(const std::string& path, MeshData& mesh, std::string& error) {
    ParsedObj parsed;
    if (!parse_obj(path, parsed, error)) return false;
    MeshData combined;
    for (const ObjSubmesh& submesh : parsed.submeshes) {
        if (combined.vertices.size() > std::numeric_limits<std::uint32_t>::max() -
                                           submesh.mesh.vertices.size()) {
            error = "combined OBJ exceeds the 32-bit vertex limit: " + path;
            return false;
        }
        const auto base = static_cast<std::uint32_t>(combined.vertices.size());
        combined.vertices.insert(combined.vertices.end(), submesh.mesh.vertices.begin(),
                                 submesh.mesh.vertices.end());
        for (std::uint32_t index : submesh.mesh.indices) {
            if (index > std::numeric_limits<std::uint32_t>::max() - base) {
                error = "combined OBJ index overflows 32-bit range: " + path;
                return false;
            }
            combined.indices.push_back(base + index);
        }
    }
    mesh = std::move(combined);
    return true;
}

bool load_obj_scene(const std::string& path, Scene& scene, std::string& error,
                    bool load_material_textures, TransformId transform) {
    ParsedObj parsed;
    if (!parse_obj(path, parsed, error)) return false;
    const std::filesystem::path obj_path = path_from_utf8(path);
    std::unordered_map<std::string, ObjMaterial> materials;
    if (!load_materials(obj_path, parsed, load_material_textures, materials, error)) return false;

    Scene loaded;
    for (std::size_t i = 0; i < scene.transforms().size(); ++i) {
        if (i == 0) loaded.set_transform(0, scene.transform(0));
        else loaded.add_transform(scene.transform(static_cast<TransformId>(i)),
                                  scene.transform_parent(static_cast<TransformId>(i)));
    }
    if (transform >= loaded.transforms().size()) {
        error = "OBJ scene transform is not in the destination scene: " + path;
        return false;
    }
    std::unordered_map<std::string, TextureId> texture_ids;
    for (ObjSubmesh& submesh : parsed.submeshes) {
        TextureId texture_id = 0;
        const auto material = materials.find(submesh.material);
        if (material != materials.end() && !material->second.diffuse_map.empty()) {
            const std::string key = path_to_utf8(material->second.diffuse_map.lexically_normal());
            const auto existing = texture_ids.find(key);
            if (existing != texture_ids.end()) {
                texture_id = existing->second;
            } else {
                TextureData texture;
                const std::string texture_path = path_to_utf8(material->second.diffuse_map);
                if (!load_texture_image(texture_path, texture, error)) return false;
                texture_id = loaded.add_texture(std::move(texture));
                texture_ids.emplace(key, texture_id);
            }
        }
        if (material != materials.end()) {
            for (Vertex& vertex : submesh.mesh.vertices) {
                vertex.color.x *= material->second.diffuse.x;
                vertex.color.y *= material->second.diffuse.y;
                vertex.color.z *= material->second.diffuse.z;
                vertex.alpha *= material->second.opacity;
            }
        }
        const MeshId mesh_id = loaded.add_mesh(std::move(submesh.mesh));
        loaded.add_draw(mesh_id, texture_id, Mat4::identity(), transform);
    }
    scene.append_assets(std::move(loaded));
    return true;
}

} // namespace ac5
