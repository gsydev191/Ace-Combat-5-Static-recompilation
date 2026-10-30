#include "acm_loader.hpp"

#include "utf8_path.hpp"
#include "texture_loader.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <new>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ac5 {
namespace {

constexpr std::size_t acm_header_size = 0x50;
constexpr std::size_t part_record_size = 0x40;
constexpr std::uint32_t no_link = std::numeric_limits<std::uint32_t>::max();
constexpr std::size_t max_model_file_size = 128u * 1024u * 1024u;
constexpr std::uint32_t max_parts_or_chains = 4096;

bool read_u32(std::span<const std::uint8_t> bytes, std::size_t offset,
              std::uint32_t& value) {
    if (offset > bytes.size() || bytes.size() - offset < 4) return false;
    value = static_cast<std::uint32_t>(bytes[offset]) |
            (static_cast<std::uint32_t>(bytes[offset + 1]) << 8) |
            (static_cast<std::uint32_t>(bytes[offset + 2]) << 16) |
            (static_cast<std::uint32_t>(bytes[offset + 3]) << 24);
    return true;
}

bool read_i32(std::span<const std::uint8_t> bytes, std::size_t offset,
              std::int32_t& value) {
    std::uint32_t raw = 0;
    if (!read_u32(bytes, offset, raw)) return false;
    value = std::bit_cast<std::int32_t>(raw);
    return true;
}

bool read_i16(std::span<const std::uint8_t> bytes, std::size_t offset,
              std::int16_t& value) {
    if (offset > bytes.size() || bytes.size() - offset < 2) return false;
    const std::uint16_t raw = static_cast<std::uint16_t>(bytes[offset]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[offset + 1]) << 8);
    value = std::bit_cast<std::int16_t>(raw);
    return true;
}

bool read_f32(std::span<const std::uint8_t> bytes, std::size_t offset,
              float& value) {
    std::uint32_t raw = 0;
    if (!read_u32(bytes, offset, raw)) return false;
    value = std::bit_cast<float>(raw);
    return std::isfinite(value);
}

bool is_acm(std::span<const std::uint8_t> bytes) {
    return bytes.size() >= 4 && bytes[0] == 'A' && bytes[1] == 'C' &&
           bytes[2] == 'M' && bytes[3] == 0;
}

struct BundleSlot {
    std::uint32_t index = 0;
    std::span<const std::uint8_t> bytes;
};

struct ModelSelection {
    std::span<const std::uint8_t> model;
    std::vector<std::span<const std::uint8_t>> companion_models;
    std::span<const std::uint8_t> texture;
};

struct LodBundleSelection {
    std::span<const std::uint8_t> bundle;
    std::vector<std::span<const std::uint8_t>> companion_models;
};

struct WeaponBundleSelection {
    std::span<const std::uint8_t> loadout;
    std::span<const std::uint8_t> gun;
    std::span<const std::uint8_t> missile;
};

bool is_gim(std::span<const std::uint8_t> bytes) {
    return bytes.size() >= 4 && bytes[0] == 'G' && bytes[1] == 'I' &&
           bytes[2] == 'M' && bytes[3] == 0;
}

bool read_bundle_slots(std::span<const std::uint8_t> file,
                       std::vector<BundleSlot>& slots) {
    std::uint32_t count = 0;
    if (!read_u32(file, 0, count) || count == 0 || count > max_parts_or_chains ||
        file.size() < 4 || count > (file.size() - 4) / 4)
        return false;
    const std::size_t table_end = 4 + static_cast<std::size_t>(count) * 4;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> offsets;
    offsets.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        std::uint32_t offset = 0;
        if (!read_u32(file, 4 + static_cast<std::size_t>(i) * 4, offset)) return false;
        if (offset == 0) continue;
        if (offset < table_end || offset >= file.size()) return false;
        offsets.emplace_back(offset, i);
    }
    std::sort(offsets.begin(), offsets.end());
    slots.clear();
    slots.reserve(offsets.size());
    for (std::size_t i = 0; i < offsets.size();) {
        const std::size_t begin = offsets[i].first;
        std::size_t next = i + 1;
        while (next < offsets.size() && offsets[next].first == begin) ++next;
        const std::size_t end = next < offsets.size() ? offsets[next].first : file.size();
        if (end <= begin) return false;
        for (; i < next; ++i)
            slots.push_back({offsets[i].second, file.subspan(begin, end - begin)});
    }
    return !slots.empty();
}

bool find_model(std::span<const std::uint8_t> file, ModelSelection& selection,
                unsigned depth = 0) {
    if (is_acm(file)) {
        selection.model = file;
        return true;
    }
    if (depth >= 4) return false;
    std::vector<BundleSlot> slots;
    if (!read_bundle_slots(file, slots)) return false;
    std::vector<std::span<const std::uint8_t>> direct_models;
    for (const BundleSlot& slot : slots) {
        if (!is_acm(slot.bytes)) continue;
        const bool already_present = std::any_of(
            direct_models.begin(), direct_models.end(), [&slot](auto model) {
                return model.data() == slot.bytes.data();
            });
        if (!already_present) direct_models.push_back(slot.bytes);
        for (const BundleSlot& sibling : slots) {
            if (sibling.index >= 6 && sibling.index > slot.index &&
                sibling.bytes.size() >= 4 && sibling.bytes[0] == 'G' &&
                sibling.bytes[1] == 'I' && sibling.bytes[2] == 'M' &&
                sibling.bytes[3] == 0) {
                if (selection.texture.empty()) selection.texture = sibling.bytes;
                break;
            }
        }
    }
    for (const BundleSlot& slot : slots) {
        if (is_acm(slot.bytes)) continue;
        ModelSelection nested;
        if (find_model(slot.bytes, nested, depth + 1)) {
            // A nested LOD bundle is accompanied by independent ACM geometry
            // in its parent. Preserve those meshes alongside its LOD0 model.
            nested.companion_models.insert(nested.companion_models.end(),
                                            direct_models.begin(), direct_models.end());
            selection = nested;
            return true;
        }
    }
    if (!direct_models.empty()) {
        selection.model = direct_models.front();
        return true;
    }
    return false;
}

bool find_weapon_bundle(std::span<const std::uint8_t> file,
                        WeaponBundleSelection& selection, unsigned depth = 0) {
    if (depth >= 4) return false;
    std::vector<BundleSlot> slots;
    if (!read_bundle_slots(file, slots)) return false;
    const auto slot_at = [&slots](std::uint32_t index) -> const BundleSlot* {
        const auto found = std::find_if(slots.begin(), slots.end(),
            [index](const BundleSlot& slot) { return slot.index == index; });
        return found == slots.end() ? nullptr : &*found;
    };
    const BundleSlot* loadout = slot_at(7);
    const BundleSlot* gun = slot_at(8);
    const BundleSlot* missile = slot_at(9);
    if (loadout && gun && missile) {
        AircraftWeaponRules validated;
        std::string validation_error;
        if (decode_aircraft_weapon_rules(loadout->bytes, gun->bytes,
                                         missile->bytes, validated,
                                         validation_error)) {
            selection = {loadout->bytes, gun->bytes, missile->bytes};
            return true;
        }
    }
    for (const BundleSlot& slot : slots) {
        if (find_weapon_bundle(slot.bytes, selection, depth + 1)) return true;
    }
    return false;
}

bool collect_bundle_textures(std::span<const std::uint8_t> file,
                             std::vector<std::uint32_t>& slot_path,
                             std::vector<AcmBundleTexture>& textures,
                             std::size_t& decoded_bytes, std::string& error,
                             unsigned depth = 0) {
    if (depth >= 4) return true;
    std::vector<BundleSlot> slots;
    if (!read_bundle_slots(file, slots)) return true;
    for (const BundleSlot& slot : slots) {
        slot_path.push_back(slot.index);
        if (is_gim(slot.bytes)) {
            if (textures.size() >= 64) {
                error = "ACM bundle contains more than 64 indexed textures";
                return false;
            }
            AcmBundleTexture item;
            if (!decode_gim(slot.bytes, item.texture, error, true)) {
                error += " in bundle slot " + std::to_string(slot.index);
                return false;
            }
            const std::size_t bytes = item.texture.rgba.size();
            constexpr std::size_t max_decoded_texture_bytes = 512u * 1024u * 1024u;
            if (bytes > max_decoded_texture_bytes - decoded_bytes) {
                error = "ACM bundle decoded textures exceed the 512 MiB limit";
                return false;
            }
            decoded_bytes += bytes;
            item.bundle_slots = slot_path;
            textures.push_back(std::move(item));
        } else if (slot.bytes.size() >= 4 && !is_acm(slot.bytes)) {
            if (!collect_bundle_textures(slot.bytes, slot_path, textures,
                                         decoded_bytes, error, depth + 1))
                return false;
        }
        slot_path.pop_back();
    }
    return true;
}

bool find_lod_bundle(std::span<const std::uint8_t> file,
                    LodBundleSelection& selection, unsigned depth = 0) {
    if (depth >= 4) return false;
    std::vector<BundleSlot> slots;
    if (!read_bundle_slots(file, slots)) return false;
    const auto slot_at = [&slots](std::uint32_t index) -> const BundleSlot* {
        const auto found = std::find_if(slots.begin(), slots.end(),
            [index](const BundleSlot& slot) { return slot.index == index; });
        return found == slots.end() ? nullptr : &*found;
    };
    const BundleSlot* distances = slot_at(0);
    if (distances && distances->bytes.size() >= 5 * sizeof(float)) {
        std::array<float, 5> thresholds{};
        bool valid = true;
        for (std::size_t i = 0; i < thresholds.size(); ++i) {
            if (!read_f32(distances->bytes, i * sizeof(float), thresholds[i]) ||
                thresholds[i] < 0.0f || (i != 0 && thresholds[i] < thresholds[i - 1])) {
                valid = false;
                break;
            }
        }
        for (std::uint32_t i = 1; valid && i <= 5; ++i) {
            const BundleSlot* mesh = slot_at(i);
            valid = mesh && is_acm(mesh->bytes);
        }
        if (valid) {
            selection.bundle = file;
            return true;
        }
    }

    std::vector<std::span<const std::uint8_t>> direct_models;
    for (const BundleSlot& slot : slots) {
        if (!is_acm(slot.bytes)) continue;
        const bool duplicate = std::any_of(direct_models.begin(), direct_models.end(),
            [&slot](auto model) { return model.data() == slot.bytes.data(); });
        if (!duplicate) direct_models.push_back(slot.bytes);
    }
    for (const BundleSlot& slot : slots) {
        if (is_acm(slot.bytes)) continue;
        LodBundleSelection nested;
        if (find_lod_bundle(slot.bytes, nested, depth + 1)) {
            nested.companion_models.insert(nested.companion_models.end(),
                                            direct_models.begin(), direct_models.end());
            selection = std::move(nested);
            return true;
        }
    }
    return false;
}

bool append_mesh(MeshData& destination, MeshData&& source, std::string& error) {
    if (destination.vertices.size() > std::numeric_limits<std::uint32_t>::max() ||
        source.vertices.size() > std::numeric_limits<std::uint32_t>::max() -
                                     destination.vertices.size()) {
        error = "combined ACM meshes exceed the native vertex index limit";
        return false;
    }
    const std::uint32_t vertex_offset = static_cast<std::uint32_t>(destination.vertices.size());
    destination.vertices.reserve(destination.vertices.size() + source.vertices.size());
    destination.indices.reserve(destination.indices.size() + source.indices.size());
    destination.vertices.insert(destination.vertices.end(),
                                std::make_move_iterator(source.vertices.begin()),
                                std::make_move_iterator(source.vertices.end()));
    for (const std::uint32_t index : source.indices)
        destination.indices.push_back(vertex_offset + index);
    return true;
}

struct Part {
    std::array<std::uint32_t, 8> words{};
    Vec3 position{};
    Vec3 rotation{};
};

bool parse_animation_tracks(std::span<const std::uint8_t> block,
                           std::uint32_t count, std::uint32_t table_offset,
                           bool shape_track,
                           std::vector<AcmAnimationTrack>& tracks,
                           std::string& error) {
    tracks.clear();
    if (count == 0) return true;
    if (count > max_parts_or_chains || table_offset < acm_header_size ||
        table_offset > block.size() ||
        static_cast<std::uint64_t>(count) * 4u > block.size() - table_offset) {
        error = "ACM animation track offset table has invalid bounds";
        return false;
    }
    const std::uint64_t table_size = static_cast<std::uint64_t>(count) * 4u;
    const std::uint64_t padded_size = (table_size + 15u) & ~std::uint64_t{15u};
    const std::uint64_t tracks_start64 =
        static_cast<std::uint64_t>(table_offset) + padded_size;
    if (tracks_start64 > block.size()) {
        error = "ACM animation track table padding exceeds the block";
        return false;
    }
    const std::size_t tracks_start = static_cast<std::size_t>(tracks_start64);
    tracks.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        std::uint32_t relative = 0;
        if (!read_u32(block, table_offset + static_cast<std::size_t>(i) * 4,
                      relative) || relative > block.size() - tracks_start) {
            error = "ACM animation track offset is outside the block";
            return false;
        }
        const std::size_t at = tracks_start + relative;
        std::uint32_t key_count = 0, end_frame = 0, channel = 0, reserved = 0;
        if (!read_u32(block, at, key_count) ||
            !read_u32(block, at + 4, end_frame) ||
            !read_u32(block, at + 8, channel) ||
            !read_u32(block, at + 12, reserved) || reserved != 0 ||
            key_count > 1024 || key_count > (block.size() - at - 16) / 16) {
            error = "ACM animation track header or key range is invalid";
            return false;
        }
        AcmAnimationTrack track;
        track.channel = channel;
        track.end_frame = end_frame;
        track.keys.reserve(key_count);
        for (std::uint32_t key_index = 0; key_index < key_count; ++key_index) {
            const std::size_t key_at = at + 16 + static_cast<std::size_t>(key_index) * 16;
            AcmAnimationKey key;
            if (shape_track) {
                if (!read_i32(block, key_at, key.shape_index) ||
                    !read_i32(block, key_at + 4, key.frame)) {
                    error = "ACM shape animation key is truncated";
                    return false;
                }
            } else if (!read_f32(block, key_at, key.value.x) ||
                       !read_f32(block, key_at + 4, key.value.y) ||
                       !read_f32(block, key_at + 8, key.value.z) ||
                       !read_i32(block, key_at + 12, key.frame)) {
                error = "ACM transform animation key is truncated or invalid";
                return false;
            }
            if (!track.keys.empty() &&
                key.frame <= track.keys.back().frame) {
                error = "ACM animation key frames are not strictly increasing";
                return false;
            }
            track.keys.push_back(key);
        }
        tracks.push_back(std::move(track));
    }
    return true;
}

struct Unpack {
    std::uint8_t format = 0;
    std::uint32_t count = 0;
    std::size_t data_offset = 0;
};

struct Batch {
    std::uint32_t vertex_count = 0;
    std::array<std::optional<Unpack>, 5> streams{};
};

struct StreamInfo {
    std::uint32_t components;
    std::uint32_t bits;
};

std::optional<StreamInfo> unpack_info(std::uint8_t format) {
    switch (format) {
    case 0x0: return StreamInfo{1, 32};
    case 0x1: return StreamInfo{1, 16};
    case 0x2: return StreamInfo{1, 8};
    case 0x4: return StreamInfo{2, 32};
    case 0x5: return StreamInfo{2, 16};
    case 0x6: return StreamInfo{2, 8};
    case 0x8: return StreamInfo{3, 32};
    case 0x9: return StreamInfo{3, 16};
    case 0xA: return StreamInfo{3, 8};
    case 0xC: return StreamInfo{4, 32};
    case 0xD: return StreamInfo{4, 16};
    case 0xE: return StreamInfo{4, 8};
    case 0xF: return StreamInfo{4, 16};
    default: return std::nullopt;
    }
}

bool finish_batch(const Batch& batch, std::span<const std::uint8_t> block,
                  const Mat4& part_transform, MeshData& mesh, std::string& error) {
    if (batch.vertex_count < 3) return true;
    const std::size_t max_vertex_index = std::numeric_limits<std::uint32_t>::max();
    if (mesh.vertices.size() > max_vertex_index ||
        batch.vertex_count > max_vertex_index - mesh.vertices.size()) {
        error = "ACM mesh exceeds the native vertex index limit";
        return false;
    }
    const auto get = [&batch](std::size_t address) -> const Unpack* {
        return batch.streams[address] ? &*batch.streams[address] : nullptr;
    };
    const Unpack* positions = get(1);
    if (!positions || positions->count != batch.vertex_count ||
        (positions->format != 0x8 && positions->format != 0xC)) {
        error = "ACM VIF batch has no supported position stream";
        return false;
    }
    const Unpack* normals = get(2);
    if (normals && (normals->count != batch.vertex_count ||
                    (normals->format != 0x9 && normals->format != 0xC)))
        normals = nullptr;
    const Unpack* colors = get(3);
    if (colors && (colors->count != batch.vertex_count ||
                   (colors->format != 0xE && colors->format != 0xC)))
        colors = nullptr;
    const Unpack* uvs = get(4);
    if (uvs && (uvs->count != batch.vertex_count ||
                (uvs->format != 0x4 && uvs->format != 0xC)))
        uvs = nullptr;

    const std::size_t vertex_base = mesh.vertices.size();
    const std::size_t index_base = mesh.indices.size();
    mesh.vertices.reserve(vertex_base + batch.vertex_count);
    mesh.indices.reserve(index_base + (batch.vertex_count - 2) * 3u);
    for (std::uint32_t i = 0; i < batch.vertex_count; ++i) {
        Vertex vertex{};
        const std::size_t position_stride = positions->format == 0x8 ? 12 : 16;
        const std::size_t position_at = positions->data_offset + position_stride * i;
        if (!read_f32(block, position_at, vertex.position.x) ||
            !read_f32(block, position_at + 4, vertex.position.y) ||
            !read_f32(block, position_at + 8, vertex.position.z)) {
            error = "ACM position stream contains a non-finite or truncated vector";
            return false;
        }
        if (normals) {
            if (normals->format == 0x9) {
                const std::size_t at = normals->data_offset + static_cast<std::size_t>(i) * 6;
                std::int16_t x = 0, y = 0, z = 0;
                if (!read_i16(block, at, x) || !read_i16(block, at + 2, y) ||
                    !read_i16(block, at + 4, z)) {
                    error = "ACM normal stream is truncated";
                    return false;
                }
                vertex.normal = {static_cast<float>(x) / 4096.0f,
                                 static_cast<float>(y) / 4096.0f,
                                 static_cast<float>(z) / 4096.0f};
            } else {
                const std::size_t at = normals->data_offset + static_cast<std::size_t>(i) * 16;
                if (!read_f32(block, at, vertex.normal.x) ||
                    !read_f32(block, at + 4, vertex.normal.y) ||
                    !read_f32(block, at + 8, vertex.normal.z)) {
                    error = "ACM normal stream contains a non-finite or truncated vector";
                    return false;
                }
            }
        }
        if (colors) {
            std::array<float, 4> channel{};
            if (colors->format == 0xE) {
                const std::size_t at = colors->data_offset + static_cast<std::size_t>(i) * 4;
                if (at > block.size() || block.size() - at < 4) {
                    error = "ACM color stream is truncated";
                    return false;
                }
                for (std::size_t c = 0; c < 4; ++c) channel[c] = block[at + c];
            } else {
                const std::size_t at = colors->data_offset + static_cast<std::size_t>(i) * 16;
                for (std::size_t c = 0; c < 4; ++c) {
                    if (!read_f32(block, at + c * 4, channel[c])) {
                        error = "ACM color stream contains a non-finite or truncated value";
                        return false;
                    }
                }
            }
            vertex.color = {std::clamp(channel[0], 0.0f, 255.0f) / 255.0f,
                            std::clamp(channel[1], 0.0f, 255.0f) / 255.0f,
                            std::clamp(channel[2], 0.0f, 255.0f) / 255.0f};
        }
        if (uvs) {
            const std::size_t stride = uvs->format == 0x4 ? 8 : 16;
            const std::size_t at = uvs->data_offset + stride * i;
            if (!read_f32(block, at, vertex.uv.x) || !read_f32(block, at + 4, vertex.uv.y)) {
                error = "ACM UV stream contains a non-finite or truncated value";
                return false;
            }
            // mesh.vert converts OBJ's top-origin convention. ACM UVs already
            // use row zero at v=0, so compensate here before that shader flip.
            vertex.uv.y = 1.0f - vertex.uv.y;
        }

        const Mat4& transform = part_transform;
        const Vec3 point = vertex.position;
        vertex.position = {
            transform.value[0] * point.x + transform.value[4] * point.y +
                transform.value[8] * point.z + transform.value[12],
            transform.value[1] * point.x + transform.value[5] * point.y +
                transform.value[9] * point.z + transform.value[13],
            transform.value[2] * point.x + transform.value[6] * point.y +
                transform.value[10] * point.z + transform.value[14]};
        const Vec3 normal{
            transform.value[0] * vertex.normal.x + transform.value[4] * vertex.normal.y +
                transform.value[8] * vertex.normal.z,
            transform.value[1] * vertex.normal.x + transform.value[5] * vertex.normal.y +
                transform.value[9] * vertex.normal.z,
            transform.value[2] * vertex.normal.x + transform.value[6] * vertex.normal.y +
                transform.value[10] * vertex.normal.z};
        const float normal_length = std::sqrt(normal.x * normal.x + normal.y * normal.y +
                                              normal.z * normal.z);
        if (!std::isfinite(normal_length) || !std::isfinite(vertex.position.x) ||
            !std::isfinite(vertex.position.y) || !std::isfinite(vertex.position.z)) {
            error = "ACM part transform produced a non-finite vertex";
            return false;
        }
        if (normal_length > 1.0e-12f)
            vertex.normal = {normal.x / normal_length, normal.y / normal_length,
                             normal.z / normal_length};
        mesh.vertices.push_back(vertex);
    }

    const std::uint32_t base = static_cast<std::uint32_t>(vertex_base);
    for (std::uint32_t i = 0; i + 2 < batch.vertex_count; ++i) {
        // Retail vertex normals face opposite the default strip winding.
        if ((i & 1u) == 0)
            mesh.indices.insert(mesh.indices.end(), {base + i, base + i + 2, base + i + 1});
        else
            mesh.indices.insert(mesh.indices.end(), {base + i + 1, base + i + 2, base + i});
    }
    return true;
}

bool parse_vif_chain(std::span<const std::uint8_t> block, std::size_t chain_start,
                     std::size_t chain_end, const Mat4& transform,
                     MeshData& mesh, std::string& error) {
    std::size_t position = chain_start + 8;
    std::optional<Batch> batch;
    while (position < chain_end) {
        std::uint32_t word = 0;
        if (!read_u32(block, position, word) || chain_end - position < 4) {
            error = "ACM VIF command is truncated";
            return false;
        }
        const std::uint8_t command = static_cast<std::uint8_t>(word >> 24);
        const std::uint8_t encoded_count = static_cast<std::uint8_t>(word >> 16);
        const std::uint16_t immediate = static_cast<std::uint16_t>(word);
        if ((command & 0x60u) == 0x60u) {
            const std::uint8_t format = command & 0x0fu;
            const auto info = unpack_info(format);
            if (!info) {
                error = "ACM VIF chain uses an unsupported UNPACK format";
                return false;
            }
            const std::uint32_t count = encoded_count ? encoded_count : 256u;
            const std::uint64_t bit_count = static_cast<std::uint64_t>(info->components) *
                                            info->bits * count;
            const std::uint64_t data_size = ((bit_count + 31u) / 32u) * 4u;
            if (data_size > chain_end - position - 4) {
                error = "ACM VIF UNPACK payload exceeds its DMA chain";
                return false;
            }
            const std::size_t data_offset = position + 4;
            const std::uint16_t address = immediate & 0x03ffu;
            if (address < 5) {
                if (address == 0) {
                    if (batch && !finish_batch(*batch, block, transform, mesh, error))
                        return false;
                    if (format != 0x4 || count != 1 || data_size < 8) {
                        error = "ACM VIF batch header has an unsupported layout";
                        return false;
                    }
                    std::uint32_t vertex_count = 0;
                    if (!read_u32(block, data_offset, vertex_count) || vertex_count > 1'000'000) {
                        error = "ACM VIF batch declares an invalid vertex count";
                        return false;
                    }
                    batch.emplace();
                    batch->vertex_count = vertex_count;
                    batch->streams[0] = Unpack{format, count, data_offset};
                } else if (batch) {
                    batch->streams[address] = Unpack{format, count, data_offset};
                }
            }
            position += 4 + static_cast<std::size_t>(data_size);
            continue;
        }

        std::size_t extra = 0;
        if (command == 0x20) extra = 4;                 // STMASK
        else if (command == 0x30 || command == 0x31) extra = 16; // STROW / STCOL
        else if (command == 0x4a) extra = (encoded_count ? encoded_count : 256u) * 8u;
        else if (command == 0x50 || command == 0x51)
            extra = static_cast<std::size_t>(immediate ? immediate : 65536u) * 16u;
        else if (command == 0x00 || (command >= 0x01 && command <= 0x07) ||
                 command == 0x10 || command == 0x11 || command == 0x13 ||
                 command == 0x14 || command == 0x15 || command == 0x17) {
            // NOP, cycle/state changes, flushes and microprogram call markers.
        } else {
            error = "ACM VIF chain contains an unknown command";
            return false;
        }
        if (extra > chain_end - position - 4) {
            error = "ACM VIF command payload exceeds its DMA chain";
            return false;
        }
        position += 4 + extra;
    }
    if (batch && !finish_batch(*batch, block, transform, mesh, error)) return false;
    return position == chain_end;
}

bool parse_acm(std::span<const std::uint8_t> block, MeshData& mesh,
               std::string& error,
               AcmPartModel* part_model = nullptr) {
    if (!is_acm(block) || block.size() < acm_header_size) {
        error = "ACM header is truncated or has an invalid signature";
        return false;
    }
    std::uint32_t part_count = 0, table_offset = 0, chain_count = 0, table_end = 0;
    if (!read_u32(block, 0x10, part_count) || !read_u32(block, 0x14, table_offset) ||
        !read_u32(block, 0x18, chain_count) || !read_u32(block, 0x1c, table_end) ||
        part_count > max_parts_or_chains || chain_count == 0 ||
        chain_count > max_parts_or_chains || table_offset < acm_header_size ||
        static_cast<std::uint64_t>(table_offset) +
            static_cast<std::uint64_t>(part_count) * part_record_size != table_end ||
        table_end > block.size()) {
        error = "ACM part or chain table has invalid bounds";
        return false;
    }
    const std::uint64_t table_bytes = static_cast<std::uint64_t>(chain_count) * 4u;
    const std::uint64_t padded_table_bytes = std::max<std::uint64_t>(
        16u, (table_bytes + 15u) & ~std::uint64_t{15u});
    const std::uint64_t chains_start64 = static_cast<std::uint64_t>(table_end) + padded_table_bytes;
    if (chains_start64 > block.size() || table_bytes > block.size() - table_end) {
        error = "ACM chain offset table is truncated";
        return false;
    }
    const std::size_t chains_start = static_cast<std::size_t>(chains_start64);

    std::vector<Part> parts(part_count);
    for (std::uint32_t i = 0; i < part_count; ++i) {
        const std::size_t at = table_offset + static_cast<std::size_t>(i) * part_record_size;
        for (std::size_t w = 0; w < 8; ++w)
            read_u32(block, at + w * 4, parts[i].words[w]);
        if (!read_f32(block, at + 0x20, parts[i].position.x) ||
            !read_f32(block, at + 0x24, parts[i].position.y) ||
            !read_f32(block, at + 0x28, parts[i].position.z) ||
            !read_f32(block, at + 0x30, parts[i].rotation.x) ||
            !read_f32(block, at + 0x34, parts[i].rotation.y) ||
            !read_f32(block, at + 0x38, parts[i].rotation.z)) {
            error = "ACM part table contains a non-finite transform";
            return false;
        }
    }

    std::vector<std::uint32_t> parents(part_count, no_link);
    std::vector<std::pair<std::uint32_t, std::uint32_t>> hierarchy;
    hierarchy.reserve(part_count);
    for (std::uint32_t i = 0; i < part_count; ++i) {
        while (!hierarchy.empty() && hierarchy.back().second == 0) hierarchy.pop_back();
        if (i != 0) {
            if (hierarchy.empty()) {
                error = "ACM part child counts do not form a pre-order tree";
                return false;
            }
            parents[i] = hierarchy.back().first;
            --hierarchy.back().second;
        }
        hierarchy.emplace_back(i, parts[i].words[1]);
    }
    while (!hierarchy.empty() && hierarchy.back().second == 0) hierarchy.pop_back();
    if (!hierarchy.empty()) {
        error = "ACM part child counts do not close the pre-order tree";
        return false;
    }

    std::vector<Mat4> world(part_count, Mat4::identity());
    for (std::uint32_t i = 0; i < part_count; ++i) {
        const Mat4 rotate_x = Mat4::rotation_x(parts[i].rotation.x);
        const Mat4 rotate_y = Mat4::rotation_y(parts[i].rotation.y);
        const Mat4 rotate_z = Mat4::rotation_z(parts[i].rotation.z);
        const Mat4 rotate = Mat4::multiply(rotate_z, Mat4::multiply(rotate_y, rotate_x));
        const Mat4 local = Mat4::multiply(Mat4::translation(parts[i].position), rotate);
        world[i] = parents[i] == no_link ? local : Mat4::multiply(world[parents[i]], local);
    }

    AcmPartModel decoded_model;
    if (part_model) {
        if (!read_u32(block, 0x04, decoded_model.model_flags)) {
            error = "ACM model flags are truncated";
            return false;
        }
        decoded_model.parts.resize(part_count);
        for (std::uint32_t i = 0; i < part_count; ++i) {
            AcmModelPart& output = decoded_model.parts[i];
            output.parent = parents[i];
            output.kind = static_cast<std::uint8_t>(parts[i].words[4] & 0xffu);
            if (parts[i].words[2] == no_link)
                output.attachment_code = static_cast<char>(parts[i].words[4] & 0xffu);
            output.translation_track = parts[i].words[5];
            output.rotation_track = parts[i].words[6];
            output.shape_track = parts[i].words[7];
            output.source_words = parts[i].words;
            output.position = parts[i].position;
            output.rotation = parts[i].rotation;
        }
        const auto load_track_table = [&](std::size_t header_offset,
                                          bool shape,
                                          std::vector<AcmAnimationTrack>& tracks) {
            std::uint32_t count = 0, offset = 0;
            if (!read_u32(block, header_offset, count) ||
                !read_u32(block, header_offset + 4, offset)) {
                error = "ACM animation table header is truncated";
                return false;
            }
            return parse_animation_tracks(block, count, offset, shape, tracks, error);
        };
        if (!load_track_table(0x28, false, decoded_model.translation_tracks) ||
            !load_track_table(0x30, false, decoded_model.rotation_tracks) ||
            !load_track_table(0x38, true, decoded_model.shape_tracks)) return false;
        for (const AcmModelPart& part : decoded_model.parts) {
            if ((part.translation_track != AcmModelPart::no_parent &&
                 part.translation_track >= decoded_model.translation_tracks.size()) ||
                (part.rotation_track != AcmModelPart::no_parent &&
                 part.rotation_track >= decoded_model.rotation_tracks.size()) ||
                (part.shape_track != AcmModelPart::no_parent &&
                 part.shape_track >= decoded_model.shape_tracks.size())) {
                error = "ACM part references an animation track outside its table";
                return false;
            }
        }
    }

    std::vector<std::uint32_t> owners(chain_count, no_link);
    bool duplicate_owner = false;
    for (std::uint32_t i = 0; i < part_count; ++i) {
        const std::uint32_t chain = parts[i].words[2];
        if (chain != no_link && chain < chain_count) {
            if (owners[chain] != no_link) duplicate_owner = true;
            owners[chain] = i;
        }
    }
    if (duplicate_owner) {
        error = "ACM chain is assigned to more than one part";
        return false;
    }
    const bool has_owner = std::any_of(owners.begin(), owners.end(),
        [](std::uint32_t owner) { return owner != no_link; });
    if (!has_owner) {
        const bool no_owners = std::all_of(parts.begin(), parts.end(), [chain_count](const Part& part) {
            return part.words[2] == no_link || part.words[2] >= chain_count;
        });
        if (part_count != chain_count || !no_owners) {
            error = "ACM chain-to-part ownership is ambiguous";
            return false;
        }
        for (std::uint32_t i = 0; i < chain_count; ++i) owners[i] = i;
    }

    std::vector<std::vector<std::uint32_t>> shape_targets(chain_count);
    if (part_model) {
        for (std::uint32_t part_index = 0; part_index < part_count; ++part_index) {
            const std::uint32_t track_index =
                decoded_model.parts[part_index].shape_track;
            if (track_index == AcmModelPart::no_parent) continue;
            for (const AcmAnimationKey& key :
                 decoded_model.shape_tracks[track_index].keys) {
                if (key.shape_index < 0 ||
                    static_cast<std::uint32_t>(key.shape_index) >= chain_count) {
                    error = "ACM shape animation key references a missing mesh chain";
                    return false;
                }
                auto& targets = shape_targets[static_cast<std::size_t>(key.shape_index)];
                if (std::find(targets.begin(), targets.end(), part_index) == targets.end())
                    targets.push_back(part_index);
            }
        }
    }

    std::vector<std::uint32_t> chain_offsets(chain_count);
    for (std::uint32_t i = 0; i < chain_count; ++i) {
        if (!read_u32(block, table_end + static_cast<std::size_t>(i) * 4,
                      chain_offsets[i]) ||
            chain_offsets[i] > block.size() - chains_start ||
            (i != 0 && chain_offsets[i] <= chain_offsets[i - 1])) {
            error = "ACM chain offsets are invalid or out of order";
            return false;
        }
    }

    MeshData decoded;
    const Mat4 identity = Mat4::identity();
    for (std::uint32_t i = 0; i < chain_count; ++i) {
        // Unowned chains are decoded only when a shape track references them.
        if (owners[i] == no_link && shape_targets[i].empty()) continue;
        const std::size_t start = chains_start + chain_offsets[i];
        if (start > block.size() || block.size() - start < 16) {
            error = "ACM DMA chain header is truncated";
            return false;
        }
        std::uint32_t tag = 0;
        read_u32(block, start, tag);
        const std::uint32_t qwc = tag & 0xffffu;
        const std::uint32_t dma_id = (tag >> 28) & 7u;
        if (dma_id != 1u && dma_id != 6u && dma_id != 7u) {
            error = "ACM geometry uses an unsupported DMA tag";
            return false;
        }
        const std::uint64_t end64 = static_cast<std::uint64_t>(start) + 16u +
                                    static_cast<std::uint64_t>(qwc) * 16u;
        const std::size_t limit = i + 1 < chain_count
            ? chains_start + chain_offsets[i + 1] : block.size();
        if (end64 > limit || end64 > block.size()) {
            error = "ACM DMA chain payload overlaps the next chain or exceeds the block";
            return false;
        }
        if (qwc != 0 && owners[i] != no_link) {
            const std::uint32_t owner = owners[i];
            const Mat4& transform = part_model ? identity : world[owner];
            MeshData& destination = part_model ? decoded_model.parts[owner].mesh : decoded;
            if (!parse_vif_chain(block, start, static_cast<std::size_t>(end64),
                                 transform, destination, error)) return false;
        }
        if (qwc != 0 && part_model) {
            for (const std::uint32_t target_part : shape_targets[i]) {
                AcmModelPart::ShapeVariant variant;
                variant.chain_index = static_cast<std::int32_t>(i);
                if (!parse_vif_chain(block, start, static_cast<std::size_t>(end64),
                                     identity, variant.mesh, error)) return false;
                if (!variant.mesh.indices.empty())
                    decoded_model.parts[target_part].shape_variants.push_back(
                        std::move(variant));
            }
        }
    }
    const bool has_geometry = part_model
        ? std::any_of(decoded_model.parts.begin(), decoded_model.parts.end(),
              [](const AcmModelPart& part) { return !part.mesh.indices.empty(); })
        : !decoded.indices.empty();
    if (!has_geometry) {
        error = "ACM block contains no supported triangle-strip geometry";
        return false;
    }
    if (part_model) *part_model = std::move(decoded_model);
    else mesh = std::move(decoded);
    return true;
}

bool read_model_file(const std::string& path, std::vector<std::uint8_t>& file,
                     std::string& error) {
    std::ifstream input(path_from_utf8(path), std::ios::binary);
    if (!input) {
        error = "cannot open model bundle: " + path;
        return false;
    }
    input.seekg(0, std::ios::end);
    const std::streamoff length = input.tellg();
    if (length <= 0 || static_cast<std::uint64_t>(length) > max_model_file_size) {
        error = "model bundle is empty or exceeds the 128 MiB size limit: " + path;
        return false;
    }
    input.seekg(0, std::ios::beg);
    try {
        file.resize(static_cast<std::size_t>(length));
    } catch (const std::bad_alloc&) {
        error = "model bundle allocation failed: " + path;
        return false;
    } catch (const std::length_error&) {
        error = "model bundle exceeds the platform's vector limit: " + path;
        return false;
    }
    if (!input.read(reinterpret_cast<char*>(file.data()), length)) {
        error = "model bundle read failed: " + path;
        return false;
    }
    return true;
}

} // namespace

bool load_acm_lod_bundle(const std::string& path, AcmLodBundle& lods,
                         bool& found, std::string& error) {
    error.clear();
    found = false;
    std::vector<std::uint8_t> file;
    if (!read_model_file(path, file, error)) return false;
    try {
        LodBundleSelection selected;
        if (!find_lod_bundle(file, selected)) return true;
        std::vector<BundleSlot> slots;
        if (!read_bundle_slots(selected.bundle, slots)) {
            error = "nested LOD bundle has an invalid slot table: " + path;
            return false;
        }
        const auto slot_at = [&slots](std::uint32_t index) -> const BundleSlot* {
            const auto slot = std::find_if(slots.begin(), slots.end(),
                [index](const BundleSlot& candidate) { return candidate.index == index; });
            return slot == slots.end() ? nullptr : &*slot;
        };

        AcmLodBundle decoded;
        decoded.levels.reserve(5);
        for (std::uint32_t level = 0; level < 5; ++level) {
            const BundleSlot* distance_slot = slot_at(0);
            const BundleSlot* mesh_slot = slot_at(level + 1);
            if (!distance_slot || !mesh_slot || !is_acm(mesh_slot->bytes)) {
                error = "nested LOD bundle is missing a distance or ACM mesh slot: " + path;
                return false;
            }
            AcmLodLevel lod;
            if (!read_f32(distance_slot->bytes, level * sizeof(float), lod.maximum_distance)) {
                error = "nested LOD distance table contains an invalid value: " + path;
                return false;
            }
            if (!parse_acm(mesh_slot->bytes, lod.mesh, error)) return false;
            for (const auto companion : selected.companion_models) {
                MeshData extra;
                if (!parse_acm(companion, extra, error) ||
                    !append_mesh(lod.mesh, std::move(extra), error)) return false;
            }

            const std::uint32_t preferred_texture_slot = 6 + level;
            const BundleSlot* texture_slot = nullptr;
            for (std::uint32_t index = preferred_texture_slot; index < 11; ++index) {
                const BundleSlot* candidate = slot_at(index);
                if (candidate && candidate->bytes.size() >= 4 &&
                    candidate->bytes[0] == 'G' && candidate->bytes[1] == 'I' &&
                    candidate->bytes[2] == 'M' && candidate->bytes[3] == 0) {
                    texture_slot = candidate;
                    break;
                }
            }
            if (!texture_slot) {
                for (std::uint32_t index = preferred_texture_slot; index > 6; --index) {
                    const BundleSlot* candidate = slot_at(index - 1);
                    if (candidate && candidate->bytes.size() >= 4 &&
                        candidate->bytes[0] == 'G' && candidate->bytes[1] == 'I' &&
                        candidate->bytes[2] == 'M' && candidate->bytes[3] == 0) {
                        texture_slot = candidate;
                        break;
                    }
                }
            }
            if (texture_slot) {
                if (!decode_gim(texture_slot->bytes, lod.texture, error, true)) {
                    error += " in LOD " + std::to_string(level) + " of " + path;
                    return false;
                }
                lod.has_texture = true;
            }
            decoded.levels.push_back(std::move(lod));
        }
        lods = std::move(decoded);
        found = true;
        return true;
    } catch (const std::bad_alloc&) {
        error = "ACM LOD bundle allocation failed: " + path;
        return false;
    } catch (const std::length_error&) {
        error = "ACM LOD bundle exceeds the platform's vector limit: " + path;
        return false;
    }
}

bool load_acm_bundle_texture(const std::string& path, TextureData& texture,
                             bool& found, std::string& error) {
    error.clear();
    found = false;
    std::vector<std::uint8_t> file;
    if (!read_model_file(path, file, error)) return false;
    try {
        ModelSelection selected;
        if (!find_model(file, selected) || selected.texture.empty()) return true;
        TextureData decoded;
        if (!decode_gim(selected.texture, decoded, error, true)) {
            error += " in model bundle: " + path;
            return false;
        }
        texture = std::move(decoded);
        found = true;
        return true;
    } catch (const std::bad_alloc&) {
        error = "ACM bundle texture allocation failed: " + path;
        return false;
    } catch (const std::length_error&) {
        error = "ACM bundle texture exceeds the platform's vector limit: " + path;
        return false;
    }
}

bool load_acm_bundle_textures(const std::string& path,
                              std::vector<AcmBundleTexture>& textures,
                              std::string& error) {
    error.clear();
    std::vector<std::uint8_t> file;
    if (!read_model_file(path, file, error)) return false;
    try {
        std::vector<AcmBundleTexture> decoded;
        std::vector<std::uint32_t> slot_path;
        std::size_t decoded_bytes = 0;
        if (!collect_bundle_textures(file, slot_path, decoded, decoded_bytes, error))
            return false;
        textures = std::move(decoded);
        error.clear();
        return true;
    } catch (const std::bad_alloc&) {
        error = "ACM bundle texture allocation failed: " + path;
        return false;
    } catch (const std::length_error&) {
        error = "ACM bundle textures exceed the platform's vector limit: " + path;
        return false;
    }
}

bool load_pm_texture_bundle(const std::string& path,
                           std::vector<AcmBundleTexture>& textures,
                           std::string& error) {
    error.clear();
    try {
        std::vector<std::uint8_t> file;
        if (!read_model_file(path, file, error)) return false;
        std::vector<BundleSlot> root;
        if (!read_bundle_slots(file, root)) {
            error = "PM texture file is not a supported root bundle: " + path;
            return false;
        }
        const auto slot_at = [](const std::vector<BundleSlot>& slots,
                                std::uint32_t index) -> const BundleSlot* {
            const auto it = std::find_if(slots.begin(), slots.end(),
                [index](const BundleSlot& slot) { return slot.index == index; });
            return it == slots.end() ? nullptr : &*it;
        };
        const BundleSlot* palette_slot = slot_at(root, 0);
        const BundleSlot* image_bundle = slot_at(root, 1);
        if (!palette_slot || !is_gim(palette_slot->bytes) ||
            palette_slot->bytes.size() < 0x20u + 256u * 4u || !image_bundle) {
            error = "PM texture bundle is missing its shared palette or full-resolution set";
            return false;
        }
        const auto palette = palette_slot->bytes.subspan(0x20u, 256u * 4u);
        std::vector<BundleSlot> images;
        if (!read_bundle_slots(image_bundle->bytes, images)) {
            error = "PM full-resolution texture slot is not a valid bundle";
            return false;
        }
        std::vector<AcmBundleTexture> decoded;
        std::size_t total_bytes = 0;
        for (const BundleSlot& slot : images) {
            if (!is_gim(slot.bytes)) continue;
            if (decoded.size() >= 256) {
                error = "PM texture set contains more than 256 images";
                return false;
            }
            AcmBundleTexture item;
            if (!decode_gim_indexed_with_palette(slot.bytes, palette,
                                                  item.texture, error, true)) {
                error += " in PM texture slot " + std::to_string(slot.index);
                return false;
            }
            if (item.texture.rgba.size() > 512u * 1024u * 1024u - total_bytes) {
                error = "PM texture set exceeds the 512 MiB decoded-size limit";
                return false;
            }
            total_bytes += item.texture.rgba.size();
            item.bundle_slots = {1u, slot.index};
            decoded.push_back(std::move(item));
        }
        if (decoded.empty()) {
            error = "PM full-resolution texture set contains no indexed images";
            return false;
        }
        textures = std::move(decoded);
        error.clear();
        return true;
    } catch (const std::bad_alloc&) {
        error = "PM texture bundle allocation failed: " + path;
        return false;
    } catch (const std::length_error&) {
        error = "PM texture bundle exceeds the platform's vector limit: " + path;
        return false;
    }
}

bool load_acm_weapon_rules(const std::string& path, AircraftWeaponRules& rules,
                           bool& found, std::string& error) {
    error.clear();
    found = false;
    std::vector<std::uint8_t> file;
    if (!read_model_file(path, file, error)) return false;
    try {
        WeaponBundleSelection selected;
        if (!find_weapon_bundle(file, selected)) return true;
        AircraftWeaponRules decoded;
        if (!decode_aircraft_weapon_rules(selected.loadout, selected.gun,
                                          selected.missile, decoded, error)) {
            error += " in aircraft weapon bundle: " + path;
            return false;
        }
        rules = decoded;
        found = true;
        return true;
    } catch (const std::bad_alloc&) {
        error = "aircraft weapon bundle allocation failed: " + path;
        return false;
    } catch (const std::length_error&) {
        error = "aircraft weapon bundle exceeds the platform's vector limit: " + path;
        return false;
    }
}

bool load_acm_model(const std::string& path, MeshData& mesh, std::string& error) {
    error.clear();
    std::vector<std::uint8_t> file;
    if (!read_model_file(path, file, error)) return false;
    try {
        std::span<const std::uint8_t> block;
        ModelSelection selected;
        if (!find_model(file, selected)) {
            error = "model file contains no ACM block in a supported bundle: " + path;
            return false;
        }
        block = selected.model;
        MeshData decoded;
        if (!parse_acm(block, decoded, error)) return false;
        for (const auto companion : selected.companion_models) {
            MeshData extra;
            if (!parse_acm(companion, extra, error) ||
                !append_mesh(decoded, std::move(extra), error)) return false;
        }
        mesh = std::move(decoded);
        return true;
    } catch (const std::bad_alloc&) {
        error = "ACM model geometry allocation failed: " + path;
        return false;
    } catch (const std::length_error&) {
        error = "ACM model geometry exceeds the platform's vector limit: " + path;
        return false;
    }
}

bool load_acm_part_model(const std::string& path, AcmPartModel& model,
                         std::string& error) {
    error.clear();
    std::vector<std::uint8_t> file;
    if (!read_model_file(path, file, error)) return false;
    try {
        ModelSelection selected;
        if (!find_model(file, selected)) {
            error = "model file contains no ACM block in a supported bundle: " + path;
            return false;
        }
        AcmPartModel decoded;
        MeshData unused;
        if (!parse_acm(selected.model, unused, error, &decoded)) return false;
        for (const auto companion : selected.companion_models) {
            AcmPartModel extra;
            MeshData extra_unused;
            if (!parse_acm(companion, extra_unused, error, &extra)) return false;
            const std::uint32_t base = static_cast<std::uint32_t>(decoded.parts.size());
            if (extra.parts.size() > std::numeric_limits<std::uint32_t>::max() - base) {
                error = "ACM part hierarchy exceeds the native part index limit";
                return false;
            }
            const std::uint32_t translation_base =
                static_cast<std::uint32_t>(decoded.translation_tracks.size());
            const std::uint32_t rotation_base =
                static_cast<std::uint32_t>(decoded.rotation_tracks.size());
            const std::uint32_t shape_base =
                static_cast<std::uint32_t>(decoded.shape_tracks.size());
            for (AcmModelPart& part : extra.parts) {
                if (part.parent != AcmModelPart::no_parent) part.parent += base;
                if (part.translation_track != AcmModelPart::no_parent)
                    part.translation_track += translation_base;
                if (part.rotation_track != AcmModelPart::no_parent)
                    part.rotation_track += rotation_base;
                if (part.shape_track != AcmModelPart::no_parent)
                    part.shape_track += shape_base;
                decoded.parts.push_back(std::move(part));
            }
            decoded.translation_tracks.insert(decoded.translation_tracks.end(),
                std::make_move_iterator(extra.translation_tracks.begin()),
                std::make_move_iterator(extra.translation_tracks.end()));
            decoded.rotation_tracks.insert(decoded.rotation_tracks.end(),
                std::make_move_iterator(extra.rotation_tracks.begin()),
                std::make_move_iterator(extra.rotation_tracks.end()));
            decoded.shape_tracks.insert(decoded.shape_tracks.end(),
                std::make_move_iterator(extra.shape_tracks.begin()),
                std::make_move_iterator(extra.shape_tracks.end()));
        }
        model = std::move(decoded);
        return true;
    } catch (const std::bad_alloc&) {
        error = "ACM part model allocation failed: " + path;
        return false;
    } catch (const std::length_error&) {
        error = "ACM part model exceeds the platform's vector limit: " + path;
        return false;
    }
}

bool sample_acm_animation_track(const AcmAnimationTrack& track, double frame,
                               Vec3& value) noexcept {
    if (track.keys.empty() || !std::isfinite(frame) ||
        track.keys.front().shape_index >= 0) return false;
    const auto& keys = track.keys;
    if (frame <= keys.front().frame) {
        value = keys.front().value;
        return true;
    }
    if (frame >= keys.back().frame) {
        value = keys.back().value;
        return true;
    }
    const auto upper = std::lower_bound(keys.begin(), keys.end(), frame,
        [](const AcmAnimationKey& key, double position) {
            return static_cast<double>(key.frame) < position;
        });
    if (upper == keys.begin() || upper == keys.end()) return false;
    const AcmAnimationKey& high = *upper;
    const AcmAnimationKey& low = *(upper - 1);
    const double fraction = (frame - low.frame) /
                            static_cast<double>(high.frame - low.frame);
    value = {
        static_cast<float>(low.value.x + (high.value.x - low.value.x) * fraction),
        static_cast<float>(low.value.y + (high.value.y - low.value.y) * fraction),
        static_cast<float>(low.value.z + (high.value.z - low.value.z) * fraction)};
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

std::optional<std::array<std::array<double, 3>, 3>> sample_acm_ground_markers(
    const AcmPartModel& model, std::span<const float> channels) {
    if (model.parts.size() > 4096) return std::nullopt;
    constexpr std::array<char, 3> codes{'I', 'K', 'J'};
    std::array<std::uint32_t, 3> marker_parts{
        AcmModelPart::no_parent, AcmModelPart::no_parent,
        AcmModelPart::no_parent};
    for (std::size_t part_index = 0; part_index < model.parts.size(); ++part_index) {
        const char code = model.parts[part_index].attachment_code;
        for (std::size_t marker = 0; marker < codes.size(); ++marker)
            if (code == codes[marker] &&
                marker_parts[marker] == AcmModelPart::no_parent)
                marker_parts[marker] = static_cast<std::uint32_t>(part_index);
    }
    if (std::ranges::any_of(marker_parts, [](std::uint32_t part) {
            return part == AcmModelPart::no_parent;
        })) return std::nullopt;

    const auto local_transform = [&](const AcmModelPart& part)
        -> std::optional<Mat4> {
        Vec3 position = part.position;
        Vec3 rotation = part.rotation;
        Vec3 animated{};
        if (part.translation_track != AcmModelPart::no_parent) {
            if (part.translation_track >= model.translation_tracks.size())
                return std::nullopt;
            const auto& track = model.translation_tracks[part.translation_track];
            if (track.channel >= channels.size() ||
                !sample_acm_animation_track(track, channels[track.channel], animated))
                return std::nullopt;
            position.x += animated.x;
            position.y += animated.y;
            position.z += animated.z;
        }
        if (part.rotation_track != AcmModelPart::no_parent) {
            if (part.rotation_track >= model.rotation_tracks.size())
                return std::nullopt;
            const auto& track = model.rotation_tracks[part.rotation_track];
            if (track.channel >= channels.size() ||
                !sample_acm_animation_track(track, channels[track.channel], animated))
                return std::nullopt;
            rotation.x += animated.x;
            rotation.y += animated.y;
            rotation.z += animated.z;
        }
        return Mat4::multiply(Mat4::translation(position), Mat4::multiply(
            Mat4::rotation_z(rotation.z), Mat4::multiply(
                Mat4::rotation_y(rotation.y), Mat4::rotation_x(rotation.x))));
    };

    std::array<std::array<double, 3>, 3> markers{};
    for (std::size_t marker = 0; marker < marker_parts.size(); ++marker) {
        std::array<std::uint32_t, 4096> ancestry{};
        std::size_t depth = 0;
        std::uint32_t part = marker_parts[marker];
        while (part != AcmModelPart::no_parent) {
            if (part >= model.parts.size() || depth == ancestry.size())
                return std::nullopt;
            ancestry[depth++] = part;
            part = model.parts[part].parent;
        }
        Mat4 world = Mat4::identity();
        while (depth != 0) {
            const auto local = local_transform(model.parts[ancestry[--depth]]);
            if (!local) return std::nullopt;
            world = Mat4::multiply(world, *local);
        }
        markers[marker] = {world.value[12], world.value[13], world.value[14]};
        for (double value : markers[marker])
            if (!std::isfinite(value)) return std::nullopt;
    }
    return markers;
}

} // namespace ac5
