#include "application.hpp"

#include "acm_loader.hpp"
#include "aircraft_collision.hpp"
#include "animation_controller.hpp"
#include "data_archive.hpp"
#include "flight_model.hpp"
#include "flight_rules_loader.hpp"
#include "flight_session.hpp"
#include "heightfield_mesh.hpp"
#include "input.hpp"
#include "map_texture.hpp"
#include "mission_terrain.hpp"
#include "mission_spawn_preview.hpp"
#include "mission_units.hpp"
#include "obj_loader.hpp"
#include "projectile_mesh.hpp"
#include "projectile_system.hpp"
#include "texture_loader.hpp"
#include "weapon_controller.hpp"
#include "vulkan_renderer.hpp"
#include "utf8_path.hpp"

#include <SDL3/SDL.h>

#include <cstdio>
#include <algorithm>
#include <array>
#include <charconv>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <new>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

using namespace ac5;

namespace {

MeshData make_mission_spawn_marker(Vec3 color) {
    constexpr float h = 0.5f;
    MeshData mesh;
    mesh.vertices = {
        {{0.0f, h, 0.0f}, color, {}, {0.0f, 1.0f, 0.0f}},
        {{0.0f, -h, 0.0f}, color, {}, {0.0f, -1.0f, 0.0f}},
        {{-h, 0.0f, 0.0f}, color, {}, {-1.0f, 0.0f, 0.0f}},
        {{h, 0.0f, 0.0f}, color, {}, {1.0f, 0.0f, 0.0f}},
        {{0.0f, 0.0f, -h}, color, {}, {0.0f, 0.0f, -1.0f}},
        {{0.0f, 0.0f, h}, color, {}, {0.0f, 0.0f, 1.0f}},
    };
    mesh.indices = {
        0, 2, 4, 0, 4, 3, 0, 3, 5, 0, 5, 2,
        1, 4, 2, 1, 3, 4, 1, 5, 3, 1, 2, 5,
    };
    return mesh;
}

enum class DemoSceneState : std::uint8_t { Viewing, Exiting };
enum class DemoSceneSubstate : std::uint8_t { Frame };

using DemoStateMachine = SceneStateMachine<DemoSceneState, DemoSceneSubstate>;

struct DemoFrame {
    Scene& scene;
    TransformId model_transform;
    DemoFrame(Scene& scene_ref, TransformId transform)
        : scene(scene_ref), model_transform(transform) {}
    Camera* camera = nullptr;
    Vec3 camera_follow_offset{};
    std::optional<AircraftFlightSession> flight_session;
    std::shared_ptr<const Ac5MissionTerrain> mission_terrain;
    std::optional<std::array<double, 6>> aircraft_hit_extents;
    bool aircraft_crashed = false;
    bool aircraft_on_ground = false;
    bool aircraft_ground_trace_active = false;
    AircraftGroundContactState terrain_ground_contact{};
    std::optional<AircraftWeaponRules> weapon_rules;
    AircraftWeaponState weapon_state{};
    AircraftProjectileSystem projectile_system{};
    std::uint32_t gun_muzzle_index = 0;
    MeshId projectile_mesh = 0;
    bool has_projectile_mesh = false;
    std::uint32_t pending_weapon_switch_presses = 0;
    std::uint32_t pending_primary_fire_presses = 0;
    std::uint32_t pending_gun_fire_presses = 0;
    std::array<AircraftWeaponShot, 24> weapon_shots{};
    std::size_t weapon_shot_count = 0;
    bool unsupported_special_sequence = false;
    AircraftAnimationRules animation_rules{};
    AircraftAnimationChannels animation_channels = [] {
        AircraftAnimationChannels channels{};
        channels.fill(50.0f);
        for (std::size_t channel = 33; channel <= 37; ++channel)
            channels[channel] = 0.0f;
        return channels;
    }();
    bool gear_down = false;
    AircraftGearController gear_controller{};
    std::uint32_t animation_controller_flags = 0;
    std::vector<std::size_t> part_draw_indices;
    std::optional<AcmPartModel> animated_model;
    std::vector<TransformId> part_transforms;
    struct ShapeDraw {
        std::size_t draw_index = 0;
        MeshId mesh = 0;
        bool dynamic = false;
        std::vector<std::pair<std::int32_t, MeshData>> shapes;
        std::vector<std::pair<std::int32_t, MeshId>> static_variants;
    };
    std::vector<ShapeDraw> shape_draws;
    float delta_seconds = 0.0f;
    FlightInput controls{};
    float pitch_angle = 0.0f;
    float roll_angle = 0.0f;
    float yaw_angle = 0.0f;
};

struct DemoLod {
    std::vector<MeshId> meshes;
    std::vector<TextureId> textures;
    std::vector<float> thresholds;
    std::size_t draw_index = 0;
    bool active = false;
    bool world_distance_thresholds = false;
};

std::uint32_t infer_animation_controller_flags(const AcmPartModel& model) {
    // CPlAnimCtrl setup tests these channel slots in its animation table to
    // choose optional surface mappings and gear/nozzle channels.
    std::array<bool, aircraft_animation_channel_count> channels{};
    const auto record_channels = [&channels](
        const std::vector<AcmAnimationTrack>& tracks) {
        for (const AcmAnimationTrack& track : tracks)
            if (track.channel < channels.size()) channels[track.channel] = true;
    };
    record_channels(model.translation_tracks);
    record_channels(model.rotation_tracks);
    record_channels(model.shape_tracks);

    std::uint32_t flags = 0;
    if (channels[2]) flags |= 0x00001u;
    if (channels[9]) flags |= 0x00002u;
    if (channels[4]) flags |= 0x00004u;
    if (channels[11]) flags |= 0x00008u;
    if (channels[25]) flags |= 0x10000u;
    if (channels[26]) flags |= 0x20000u;
    if (channels[27]) flags |= 0x40000u;
    return flags;
}

Mat4 animated_part_local_transform(const AcmPartModel& model,
                                   const AcmModelPart& part,
                                   const AircraftAnimationChannels& channels) noexcept {
    Vec3 position = part.position;
    Vec3 rotation = part.rotation;
    Vec3 animated{};
    if (part.translation_track != AcmModelPart::no_parent) {
        const AcmAnimationTrack& track = model.translation_tracks[part.translation_track];
        if (track.channel < channels.size() &&
            sample_acm_animation_track(track, channels[track.channel], animated)) {
            position.x += animated.x;
            position.y += animated.y;
            position.z += animated.z;
        }
    }
    if (part.rotation_track != AcmModelPart::no_parent) {
        const AcmAnimationTrack& track = model.rotation_tracks[part.rotation_track];
        if (track.channel < channels.size() &&
            sample_acm_animation_track(track, channels[track.channel], animated)) {
            rotation.x += animated.x;
            rotation.y += animated.y;
            rotation.z += animated.z;
        }
    }
    const Mat4 rotate_x = Mat4::rotation_x(rotation.x);
    const Mat4 rotate_y = Mat4::rotation_y(rotation.y);
    const Mat4 rotate_z = Mat4::rotation_z(rotation.z);
    return Mat4::multiply(Mat4::translation(position), Mat4::multiply(
        rotate_z, Mat4::multiply(rotate_y, rotate_x)));
}

bool animated_attachment_world_position(const AcmPartModel& model,
                                        std::size_t part_index,
                                        const AircraftAnimationChannels& channels,
                                        const AircraftFlightState& flight,
                                        std::array<double, 3>& position) noexcept {
    if (part_index >= model.parts.size() || model.parts.size() > 4096) return false;
    std::array<std::uint32_t, 4096> ancestry{};
    std::size_t depth = 0;
    std::uint32_t current = static_cast<std::uint32_t>(part_index);
    while (current != AcmModelPart::no_parent) {
        if (current >= model.parts.size() || depth == ancestry.size()) return false;
        ancestry[depth++] = current;
        current = model.parts[current].parent;
    }
    const Mat4 rotate = Mat4::multiply(
        Mat4::multiply(Mat4::rotation_y(static_cast<float>(flight.euler[1])),
                       Mat4::rotation_x(static_cast<float>(flight.euler[0]))),
        Mat4::rotation_z(static_cast<float>(flight.euler[2])));
    Mat4 world = Mat4::multiply(Mat4::translation({
        static_cast<float>(flight.position[0]), static_cast<float>(flight.position[1]),
        static_cast<float>(flight.position[2])}), rotate);
    while (depth != 0)
        world = Mat4::multiply(world, animated_part_local_transform(
            model, model.parts[ancestry[--depth]], channels));
    position = {world.value[12], world.value[13], world.value[14]};
    return std::isfinite(position[0]) && std::isfinite(position[1]) &&
           std::isfinite(position[2]);
}

void update_aircraft_part_animation(DemoFrame& frame) {
    if (!frame.animated_model ||
        frame.animated_model->parts.size() != frame.part_transforms.size()) return;
    for (std::size_t i = 0; i < frame.animated_model->parts.size(); ++i) {
        const AcmModelPart& part = frame.animated_model->parts[i];
        frame.scene.set_transform(frame.part_transforms[i],
            animated_part_local_transform(*frame.animated_model, part,
                                          frame.animation_channels));
        if (i < frame.part_draw_indices.size() &&
            frame.part_draw_indices[i] != std::numeric_limits<std::size_t>::max()) {
            if (part.kind == 0x20u)
                frame.scene.set_draw_visible(frame.part_draw_indices[i],
                                             !frame.gear_controller.visible());
            else if (part.kind == 0x21u)
                frame.scene.set_draw_visible(frame.part_draw_indices[i],
                                             frame.gear_controller.visible());
        }

        DemoFrame::ShapeDraw& shape_draw = frame.shape_draws[i];
        if (part.shape_track == AcmModelPart::no_parent ||
            (shape_draw.shapes.empty() && shape_draw.static_variants.empty())) continue;
        const AcmAnimationTrack& track =
            frame.animated_model->shape_tracks[part.shape_track];
        if (track.keys.empty() || track.channel >= frame.animation_channels.size()) continue;
        const double channel = frame.animation_channels[track.channel];
        const auto upper = std::lower_bound(track.keys.begin(), track.keys.end(), channel,
            [](const AcmAnimationKey& key, double value) { return key.frame < value; });
        const AcmAnimationKey* low = nullptr;
        const AcmAnimationKey* high = nullptr;
        double fraction = 0.0;
        if (upper == track.keys.begin()) low = high = &*upper;
        else if (upper == track.keys.end()) low = high = &track.keys.back();
        else {
            low = &*(upper - 1);
            high = &*upper;
            fraction = (channel - low->frame) /
                       static_cast<double>(high->frame - low->frame);
        }
        const auto find_shape = [&shape_draw](std::int32_t chain) -> const MeshData* {
            const auto found = std::find_if(shape_draw.shapes.begin(), shape_draw.shapes.end(),
                [chain](const auto& entry) { return entry.first == chain; });
            return found == shape_draw.shapes.end() ? nullptr : &found->second;
        };
        if (shape_draw.dynamic) {
            const MeshData* first = find_shape(low->shape_index);
            const MeshData* second = find_shape(high->shape_index);
            if (!first || !second || first->vertices.size() != second->vertices.size()) continue;
            std::span<Vertex> output = frame.scene.dynamic_vertices(shape_draw.mesh);
            if (output.size() != first->vertices.size()) continue;
            const auto mix = [fraction](float a, float b) {
                return static_cast<float>(a + (b - a) * fraction);
            };
            const auto mix_vec2 = [&mix](Vec2 a, Vec2 b) {
                return Vec2{mix(a.x, b.x), mix(a.y, b.y)};
            };
            const auto mix_vec3 = [&mix](Vec3 a, Vec3 b) {
                return Vec3{mix(a.x, b.x), mix(a.y, b.y), mix(a.z, b.z)};
            };
            for (std::size_t vertex = 0; vertex < output.size(); ++vertex) {
                const Vertex& a = first->vertices[vertex];
                const Vertex& b = second->vertices[vertex];
                Vertex& result = output[vertex];
                result.position = mix_vec3(a.position, b.position);
                result.color = mix_vec3(a.color, b.color);
                result.uv = mix_vec2(a.uv, b.uv);
                result.normal = mix_vec3(a.normal, b.normal);
                result.alpha = mix(a.alpha, b.alpha);
                result.light_response = mix(a.light_response, b.light_response);
                result.environment_uv = mix_vec2(a.environment_uv, b.environment_uv);
                result.environment_color = mix_vec3(a.environment_color, b.environment_color);
                result.environment_alpha = mix(a.environment_alpha, b.environment_alpha);
            }
        } else {
            const AcmAnimationKey* selected = fraction < 0.5 ? low : high;
            const auto variant = std::find_if(shape_draw.static_variants.begin(),
                shape_draw.static_variants.end(), [selected](const auto& entry) {
                    return entry.first == selected->shape_index;
                });
            if (variant != shape_draw.static_variants.end())
                frame.scene.set_draw_mesh(shape_draw.draw_index, variant->second);
        }
    }
}

void advance_demo_animation(DemoFrame& frame, const AircraftFlightState& flight,
                            bool gear_down,
                            AircraftAnimationChannels& channels,
                            AircraftGearController& gear_controller) noexcept {
    AircraftAnimationInput input{};
    input.pitch = frame.controls.pitch;
    input.roll = frame.controls.roll;
    input.yaw = frame.controls.yaw;
    input.throttle_lever = frame.controls.throttle;
    input.speed_kmh = std::max(0.0, flight.speed_units_per_tick * 60.0 /
                                      aircraft_world_units_per_kmh_per_second);
    input.speed_units_per_tick = flight.speed_units_per_tick;
    input.gear_down = gear_down;
    input.model_flags = frame.animated_model ? frame.animated_model->model_flags : 0;
    input.controller_flags = frame.animation_controller_flags;
    advance_aircraft_animation_tick(channels, frame.animation_rules, input);
    request_aircraft_gear(gear_controller, gear_down);
    advance_aircraft_retail_gear(gear_controller, channels);
}

void select_demo_lod(Scene& scene, const DemoLod& lod, const Camera& camera) {
    if (!lod.active || lod.meshes.empty()) return;
    const Vec3 offset{camera.position.x - camera.target.x,
                      camera.position.y - camera.target.y,
                      camera.position.z - camera.target.z};
    const float distance = std::sqrt(offset.x * offset.x + offset.y * offset.y +
                                     offset.z * offset.z);
    constexpr float retail_scale = 627.87f;
    const float scaled_distance = lod.world_distance_thresholds
        ? distance : distance * 512.0f / retail_scale;
    std::size_t level = 0;
    while (level < lod.thresholds.size() &&
           !(scaled_distance < lod.thresholds[level])) ++level;
    if (level >= lod.meshes.size()) {
        scene.set_draw_visible(lod.draw_index, false);
        return;
    }
    scene.set_draw_visible(lod.draw_index, true);
    scene.set_draw_mesh(lod.draw_index, lod.meshes[level]);
    scene.set_draw_texture(lod.draw_index, lod.textures[level]);
}

struct CameraFit {
    float radius = 0.0f;
    float distance = 0.0f;
};

Vec3 transform_direction(const Mat4& transform, Vec3 direction) {
    return {
        transform.value[0] * direction.x + transform.value[4] * direction.y +
            transform.value[8] * direction.z,
        transform.value[1] * direction.x + transform.value[5] * direction.y +
            transform.value[9] * direction.z,
        transform.value[2] * direction.x + transform.value[6] * direction.y +
            transform.value[10] * direction.z};
}

bool fit_camera_to_scene(const Scene& scene, Camera& camera, CameraFit& fit) {
    if (scene.draws().empty()) return false;
    const std::vector<Mat4> world_transforms = scene.resolved_transforms();
    const auto visit_points = [&](auto&& visitor) {
        for (const DrawItem& draw : scene.draws()) {
            if (!draw.visible || draw.mesh >= scene.meshes().size() ||
                draw.transform >= world_transforms.size()) continue;
            const Mat4 model = Mat4::multiply(world_transforms[draw.transform], draw.model);
            for (const Vertex& vertex : scene.meshes()[draw.mesh].vertices) {
                const float x = model.value[0] * vertex.position.x +
                                model.value[4] * vertex.position.y +
                                model.value[8] * vertex.position.z + model.value[12];
                const float y = model.value[1] * vertex.position.x +
                                model.value[5] * vertex.position.y +
                                model.value[9] * vertex.position.z + model.value[13];
                const float z = model.value[2] * vertex.position.x +
                                model.value[6] * vertex.position.y +
                                model.value[10] * vertex.position.z + model.value[14];
                const float w = model.value[3] * vertex.position.x +
                                model.value[7] * vertex.position.y +
                                model.value[11] * vertex.position.z + model.value[15];
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
                    !std::isfinite(w) || std::abs(w) <= 1.0e-8f) continue;
                const Vec3 point{x / w, y / w, z / w};
                if (std::isfinite(point.x) && std::isfinite(point.y) &&
                    std::isfinite(point.z)) visitor(point);
            }
        }
    };
    Vec3 minimum{std::numeric_limits<float>::infinity(),
                 std::numeric_limits<float>::infinity(),
                 std::numeric_limits<float>::infinity()};
    Vec3 maximum{-std::numeric_limits<float>::infinity(),
                 -std::numeric_limits<float>::infinity(),
                 -std::numeric_limits<float>::infinity()};
    std::size_t point_count = 0;
    visit_points([&](Vec3 point) {
        minimum.x = std::min(minimum.x, point.x);
        minimum.y = std::min(minimum.y, point.y);
        minimum.z = std::min(minimum.z, point.z);
        maximum.x = std::max(maximum.x, point.x);
        maximum.y = std::max(maximum.y, point.y);
        maximum.z = std::max(maximum.z, point.z);
        ++point_count;
    });
    if (point_count == 0) return false;
    const Vec3 center{
        static_cast<float>((static_cast<double>(minimum.x) + maximum.x) * 0.5),
        static_cast<float>((static_cast<double>(minimum.y) + maximum.y) * 0.5),
        static_cast<float>((static_cast<double>(minimum.z) + maximum.z) * 0.5)};
    if (!std::isfinite(center.x) || !std::isfinite(center.y) ||
        !std::isfinite(center.z)) return false;
    double radius_squared = 0.0;
    visit_points([&](Vec3 point) {
        const double dx = static_cast<double>(point.x) - center.x;
        const double dy = static_cast<double>(point.y) - center.y;
        const double dz = static_cast<double>(point.z) - center.z;
        radius_squared = std::max(radius_squared, dx * dx + dy * dy + dz * dz);
    });
    fit.radius = static_cast<float>(std::sqrt(radius_squared));
    const float sine = std::sin(camera.vertical_fov_radians * 0.5f);
    if (!std::isfinite(fit.radius) || fit.radius <= 1.0e-5f ||
        !std::isfinite(sine) || sine <= 0.0f) return false;
    fit.distance = fit.radius / sine * 1.2f;
    if (!std::isfinite(fit.distance)) return false;
    camera.target = center;
    camera.position = {center.x, center.y + fit.radius * 0.12f,
                       center.z - fit.distance};
    camera.near_plane = std::max(1.0e-5f, fit.distance - fit.radius * 1.25f);
    camera.far_plane = std::max(camera.near_plane + 1.0f,
                                fit.distance + fit.radius * 1.25f);
    return true;
}

void update_demo_scene(DemoFrame& frame, DemoStateMachine& scene_state) {
    if (frame.flight_session) {
        if (frame.aircraft_crashed) return;
        frame.weapon_shot_count = 0;
        frame.unsupported_special_sequence = false;
        const auto queue_press = [](std::uint32_t& pending) {
            if (pending != std::numeric_limits<std::uint32_t>::max()) ++pending;
        };
        if (frame.weapon_rules && frame.controls.primary_fire.pressed)
            queue_press(frame.pending_primary_fire_presses);
        if (frame.weapon_rules && frame.controls.secondary_fire.pressed)
            queue_press(frame.pending_gun_fire_presses);
        if (frame.weapon_rules && frame.controls.weapon_switch.pressed)
            queue_press(frame.pending_weapon_switch_presses);
        bool staged_gear_down = frame.gear_down;
        if (frame.controls.gear_toggle.pressed) staged_gear_down = !staged_gear_down;
        AircraftFlightSession staged_flight_session = *frame.flight_session;
        bool approach_control_changed = false;
        bool approach_control_enabled = false;
        if (frame.controls.approach_control.pressed) {
            approach_control_enabled =
                (staged_flight_session.state().flags.contact & 0x20u) == 0;
            set_aircraft_approach_control(staged_flight_session.state().flags,
                                          approach_control_enabled);
            approach_control_changed = true;
        }
        AircraftFlightInput flight_input{};
        flight_input.pitch = frame.controls.pitch;
        flight_input.roll = frame.controls.roll;
        flight_input.yaw = frame.controls.yaw;
        flight_input.throttle_lever = frame.controls.throttle;
        flight_input.ticks_per_second = 60;
        AircraftAnimationChannels staged_animation_channels = frame.animation_channels;
        AircraftGearController staged_gear_controller = frame.gear_controller;
        AircraftWeaponState staged_weapon_state = frame.weapon_state;
        AircraftProjectileSystem staged_projectiles = frame.projectile_system;
        std::uint32_t staged_gun_muzzle_index = frame.gun_muzzle_index;
        std::uint32_t staged_weapon_switch_presses =
            frame.pending_weapon_switch_presses;
        std::uint32_t staged_primary_fire_presses =
            frame.pending_primary_fire_presses;
        std::uint32_t staged_gun_fire_presses = frame.pending_gun_fire_presses;
        std::array<AircraftWeaponShot, 24> staged_weapon_shots{};
        std::size_t staged_weapon_shot_count = 0;
        bool staged_unsupported_special_sequence = false;
        bool staged_gameplay_valid = true;
        bool staged_aircraft_crashed = false;
        bool staged_collision_query_failed = false;
        bool staged_on_ground = frame.aircraft_on_ground;
        bool staged_ground_trace_active = false;
        bool staged_ground_trace_evaluated = false;
        AircraftGroundContactState staged_ground_contact =
            frame.terrain_ground_contact;
        std::string collision_error;
        collision_error.reserve(128);
        std::array<double, 3> previous_tick_position =
            staged_flight_session.state().position;
        bool ground_trace_supported = false;
        AircraftGroundTraceModel ground_trace_model{};
        ground_trace_model.gear_state = staged_gear_controller.state;
        ground_trace_model.sample = [&frame, &staged_animation_channels](
            std::optional<double> gear_channel)
                -> std::optional<AircraftGroundMarkers> {
            if (!frame.animated_model) return std::nullopt;
            AircraftAnimationChannels channels = staged_animation_channels;
            if (gear_channel) {
                if (!std::isfinite(*gear_channel)) return std::nullopt;
                for (std::size_t index = 35; index <= 37; ++index)
                    channels[index] = static_cast<float>(*gear_channel);
            }
            const auto model_markers = sample_acm_ground_markers(
                *frame.animated_model, channels);
            if (!model_markers) return std::nullopt;
            AircraftGroundMarkers markers = *model_markers;
            for (auto& marker : markers)
                for (double& coordinate : marker)
                    coordinate *= aircraft_world_units_per_meter;
            return markers;
        };
        // The generated sweep uses the 50-to-100 gear-pose displacement as
        // its probe segment. Static marker rigs produce a zero-length sweep,
        // so keep the native root-height contact fallback for those models.
        ground_trace_model.gear_state = 2;
        const auto gear_down_markers = ground_trace_model.sample(50.0);
        const auto gear_up_markers = ground_trace_model.sample(100.0);
        if (gear_down_markers && gear_up_markers) {
            ground_trace_supported = false;
            for (std::size_t marker = 0; marker < 3; ++marker)
                for (std::size_t axis = 0; axis < 3; ++axis)
                    ground_trace_supported |= std::abs(
                        (*gear_down_markers)[marker][axis] -
                        (*gear_up_markers)[marker][axis]) > 1e-6;
        }
        ground_trace_model.gear_state = staged_gear_controller.state;
        AircraftGroundTraceTerrain ground_trace_terrain{};
        if (frame.mission_terrain) {
            ground_trace_terrain.height = [&frame, &collision_error,
                &staged_gameplay_valid, &staged_collision_query_failed](
                    const std::array<double, 3>& point) {
                Ac5MissionTerrainMeshHit hit{};
                if (!trace_ac5_mission_terrain_mesh(*frame.mission_terrain,
                        {point[0], 524288.0, point[2]},
                        {point[0], -524288.0, point[2]}, hit, collision_error)) {
                    staged_gameplay_valid = false;
                    staged_collision_query_failed = true;
                    return std::numeric_limits<double>::quiet_NaN();
                }
                return hit.hit ? hit.point[1] : 0.0;
            };
            ground_trace_terrain.sweep = [&frame, &collision_error,
                &staged_gameplay_valid, &staged_collision_query_failed](
                    const std::array<double, 3>& end,
                    const std::array<double, 3>& start) {
                Ac5MissionTerrainMeshHit hit{};
                if (!trace_ac5_mission_terrain_mesh(*frame.mission_terrain,
                        end, start, hit, collision_error)) {
                    staged_gameplay_valid = false;
                    staged_collision_query_failed = true;
                    return AircraftGroundSweepHit{};
                }
                return AircraftGroundSweepHit{hit.hit ? 1 : 0, hit.fraction,
                                               hit.normal_y, hit.kind};
            };
            ground_trace_terrain.vertical = [&frame, &collision_error,
                &staged_gameplay_valid, &staged_collision_query_failed](
                    const std::array<double, 3>& upper,
                    const std::array<double, 3>& lower) {
                Ac5MissionTerrainMeshHit hit{};
                if (!trace_ac5_mission_terrain_mesh(*frame.mission_terrain,
                        upper, lower, hit, collision_error)) {
                    staged_gameplay_valid = false;
                    staged_collision_query_failed = true;
                    return AircraftGroundVerticalHit{};
                }
                return AircraftGroundVerticalHit{
                    hit.hit ? 1 : 0, hit.point};
            };
        }
        if (!staged_flight_session.advance_frame_until_contact(
                frame.delta_seconds, flight_input,
                [&frame, &collision_error, ground_trace_supported,
                 &staged_gear_controller, &staged_gameplay_valid,
                 &staged_on_ground, &staged_ground_contact,
                 &staged_collision_query_failed](
                    const AircraftFlightState& tick_state,
                    AircraftFlightInput& tick_input) noexcept {
                    if (!frame.mission_terrain || !frame.aircraft_hit_extents)
                        return true;
                    float root_ground = 0.0f;
                    if (!sample_ac5_mission_terrain_height(
                            *frame.mission_terrain, tick_state.position[0],
                            tick_state.position[2], root_ground, collision_error)) {
                        staged_gameplay_valid = false;
                        staged_collision_query_failed = true;
                        return false;
                    }
                    const double floor_y = std::max(
                        20.0, std::max(0.0, static_cast<double>(root_ground)) + 20.0);
                    const bool use_marker_trace = ground_trace_supported &&
                        staged_gear_controller.state == 2;
                    if (!use_marker_trace)
                        (void)update_aircraft_ground_contact(
                            staged_ground_contact,
                            std::abs(tick_state.position[1] - floor_y) <= 1.0,
                            tick_input.ticks_per_second, true);
                    staged_on_ground =
                        (staged_ground_contact.flags & 0x01u) != 0;
                    tick_input.ground_contact_override = staged_on_ground;
                    return true;
                },
                [&frame, staged_gear_down, &staged_animation_channels,
                 &staged_gear_controller, &staged_weapon_state,
                 &staged_projectiles,
                 &staged_gun_muzzle_index,
                 &staged_weapon_switch_presses, &staged_primary_fire_presses,
                 &staged_gun_fire_presses,
                 &staged_weapon_shots,
                 &staged_weapon_shot_count, &previous_tick_position,
                 &staged_unsupported_special_sequence,
                 ground_trace_supported, &ground_trace_model,
                 &ground_trace_terrain, &staged_ground_trace_active,
                 &staged_ground_trace_evaluated,
                 &staged_gameplay_valid, &staged_aircraft_crashed,
                 &staged_on_ground, &staged_ground_contact,
                 &staged_collision_query_failed,
                 &collision_error](AircraftFlightState& tick_state) noexcept {
                    if (!staged_gameplay_valid) return false;
                    advance_demo_animation(frame, tick_state, staged_gear_down,
                                           staged_animation_channels,
                                           staged_gear_controller);
                    staged_ground_trace_active = false;
                    staged_ground_trace_evaluated = ground_trace_supported &&
                        static_cast<bool>(frame.mission_terrain);
                    if (ground_trace_supported && frame.mission_terrain &&
                        staged_gear_controller.state == 2) {
                        ground_trace_model.gear_state =
                            staged_gear_controller.state;
                        const auto trace = normal_aircraft_ground_trace(
                            tick_state, ground_trace_model, &ground_trace_terrain);
                        if (!trace || !staged_gameplay_valid) {
                            if (collision_error.empty())
                                collision_error = "aircraft ground-marker trace failed";
                            staged_gameplay_valid = false;
                            staged_collision_query_failed = true;
                            return false;
                        }
                        staged_ground_trace_active = true;
                        (void)update_aircraft_ground_contact(
                            staged_ground_contact, trace->hit, 60, true);
                        staged_on_ground =
                            (staged_ground_contact.flags & 0x01u) != 0;
                    }
                    if (frame.mission_terrain && frame.aircraft_hit_extents) {
                        float root_ground = 0.0f;
                        if (!sample_ac5_mission_terrain_height(
                                *frame.mission_terrain, tick_state.position[0],
                                tick_state.position[2], root_ground,
                                collision_error)) {
                            staged_gameplay_valid = false;
                            staged_collision_query_failed = true;
                            return false;
                        }
                        const double floor_y = std::max(
                            20.0, std::max(0.0, static_cast<double>(root_ground)) + 20.0);
                        if (tick_state.position[1] < floor_y) {
                            const auto hits = count_airframe_surface_hits(
                                tick_state.position,
                                aircraft_body_axes(tick_state.euler),
                                *frame.aircraft_hit_extents,
                                [&frame, &collision_error](double x, double z,
                                                           double& height) noexcept {
                                    float sampled_height = 0.0f;
                                    if (!sample_ac5_mission_terrain_height(
                                            *frame.mission_terrain, x, z,
                                            sampled_height, collision_error)) return false;
                                    height = std::max(0.0, static_cast<double>(sampled_height));
                                    return true;
                                });
                            if (!hits.has_value()) {
                                staged_gameplay_valid = false;
                                staged_collision_query_failed = true;
                                return false;
                            }
                            if (*hits != 0 && !staged_on_ground) {
                                staged_aircraft_crashed = true;
                                return false;
                            }
                            if (!staged_ground_trace_active)
                                tick_state.position[1] = floor_y;
                        }
                    }
                    if (frame.weapon_rules) {
                        AircraftProjectileImpacts ignored_impacts{};
                        if (!advance_aircraft_projectiles(staged_projectiles,
                                1.0 / 60.0,
                                std::span<const ProjectileTargetSphere>{},
                                ignored_impacts)) {
                            staged_gameplay_valid = false;
                            return false;
                        }
                        const AircraftWeaponInput weapon_input{
                            .switch_weapon_pressed = staged_weapon_switch_presses != 0,
                            .fire_selected_pressed = staged_primary_fire_presses != 0,
                            .fire_gun_held = frame.controls.secondary_fire.held ||
                                             staged_gun_fire_presses != 0};
                        if (staged_weapon_switch_presses != 0)
                            --staged_weapon_switch_presses;
                        if (staged_primary_fire_presses != 0)
                            --staged_primary_fire_presses;
                        if (staged_gun_fire_presses != 0)
                            --staged_gun_fire_presses;
                        const AircraftWeaponTick output = advance_aircraft_weapons(
                            *frame.weapon_rules, weapon_input, staged_weapon_state);
                        staged_unsupported_special_sequence =
                            staged_unsupported_special_sequence ||
                            output.special_sequence_unavailable;
                        std::array<double, 3> aircraft_velocity{};
                        for (std::size_t axis = 0; axis < aircraft_velocity.size(); ++axis)
                            aircraft_velocity[axis] =
                                (tick_state.position[axis] - previous_tick_position[axis]) * 60.0;
                        for (std::size_t i = 0; i < output.shot_count &&
                             staged_weapon_shot_count < staged_weapon_shots.size(); ++i) {
                            staged_weapon_shots[staged_weapon_shot_count++] = output.shots[i];
                            std::optional<std::array<double, 3>> gun_muzzle_position;
                            if (output.shots[i].kind == AircraftShotKind::Gun) {
                                if (frame.animated_model) {
                                    const std::size_t muzzle_index =
                                        staged_gun_muzzle_index & 1u;
                                    std::size_t matching_point = 0;
                                    for (std::size_t part_index = 0;
                                         part_index < frame.animated_model->parts.size();
                                         ++part_index) {
                                        const AcmModelPart& part =
                                            frame.animated_model->parts[part_index];
                                        if ((part.source_words[4] & 0xffffu) != 0x40u)
                                            continue;
                                        if (matching_point++ != muzzle_index) continue;
                                        std::array<double, 3> position{};
                                        if (animated_attachment_world_position(
                                                *frame.animated_model, part_index,
                                                staged_animation_channels, tick_state,
                                                position))
                                            gun_muzzle_position = position;
                                        break;
                                    }
                                }
                                ++staged_gun_muzzle_index;
                            }
                            if (output.shots[i].kind != AircraftShotKind::Special &&
                                !launch_aircraft_projectile(*frame.weapon_rules,
                                    output.shots[i], tick_state, aircraft_velocity,
                                    staged_projectiles, std::nullopt,
                                    gun_muzzle_position)) {
                                staged_gameplay_valid = false;
                                return false;
                            }
                        }
                    }
                    previous_tick_position = tick_state.position;
                    return true;
                })) {
            if (staged_collision_query_failed) {
                std::fprintf(stderr, "Mission terrain contact query failed: %s\n",
                             collision_error.c_str());
            } else {
                std::fprintf(stderr, "Native flight session rejected its rules or state\n");
            }
            scene_state.request_transition(DemoSceneState::Exiting);
            return;
        }
        if (!staged_gameplay_valid) {
            if (staged_collision_query_failed) {
                std::fprintf(stderr, "Mission terrain collision query failed: %s\n",
                             collision_error.c_str());
                scene_state.request_transition(DemoSceneState::Exiting);
                return;
            }
            std::fprintf(stderr, "Native projectile simulation rejected its state\n");
            scene_state.request_transition(DemoSceneState::Exiting);
            return;
        }
        *frame.flight_session = std::move(staged_flight_session);
        frame.aircraft_crashed = staged_aircraft_crashed;
        frame.aircraft_on_ground = staged_on_ground;
        if (staged_ground_trace_evaluated)
            frame.aircraft_ground_trace_active = staged_ground_trace_active;
        frame.terrain_ground_contact = staged_ground_contact;
        if (staged_aircraft_crashed)
            std::fprintf(stderr, "Aircraft crashed after airframe contact with mission terrain\n");
        if (approach_control_changed)
            std::fprintf(stderr, "Approach control %s\n",
                         approach_control_enabled ? "enabled" : "disabled");
        frame.gear_down = staged_gear_down;
        frame.animation_channels = staged_animation_channels;
        frame.gear_controller = staged_gear_controller;
        frame.weapon_state = staged_weapon_state;
        frame.projectile_system = staged_projectiles;
        frame.gun_muzzle_index = staged_gun_muzzle_index;
        frame.pending_weapon_switch_presses = staged_weapon_switch_presses;
        frame.pending_primary_fire_presses = staged_primary_fire_presses;
        frame.pending_gun_fire_presses = staged_gun_fire_presses;
        frame.weapon_shots = staged_weapon_shots;
        frame.weapon_shot_count = staged_weapon_shot_count;
        frame.unsupported_special_sequence = staged_unsupported_special_sequence;
        const AircraftFlightState& flight_state = frame.flight_session->state();
        const std::array<double, 3>& euler = flight_state.euler;
        const Mat4 rotation = Mat4::multiply(
            Mat4::multiply(Mat4::rotation_y(static_cast<float>(euler[1])),
                           Mat4::rotation_x(static_cast<float>(euler[0]))),
            Mat4::rotation_z(static_cast<float>(euler[2])));
        const Mat4 position = Mat4::translation({
            static_cast<float>(flight_state.position[0]),
            static_cast<float>(flight_state.position[1]),
            static_cast<float>(flight_state.position[2])});
        frame.scene.set_transform(frame.model_transform,
                                  Mat4::multiply(position, rotation));
        if (frame.camera) {
            frame.camera->target = {
                static_cast<float>(flight_state.position[0]),
                static_cast<float>(flight_state.position[1]),
                static_cast<float>(flight_state.position[2])};
            const Vec3 world_offset = transform_direction(rotation,
                                                           frame.camera_follow_offset);
            frame.camera->position = {frame.camera->target.x + world_offset.x,
                                      frame.camera->target.y + world_offset.y,
                                      frame.camera->target.z + world_offset.z};
        }
        update_aircraft_part_animation(frame);
        return;
    }

    constexpr float demo_rotation_rate = 1.2f;
    constexpr float full_turn = 6.28318530717958647692f;
    frame.pitch_angle = std::remainder(
        frame.pitch_angle + frame.controls.pitch * demo_rotation_rate * frame.delta_seconds,
        full_turn);
    frame.roll_angle = std::remainder(
        frame.roll_angle + frame.controls.roll * demo_rotation_rate * frame.delta_seconds,
        full_turn);
    frame.yaw_angle = std::remainder(
        frame.yaw_angle + frame.controls.yaw * demo_rotation_rate * frame.delta_seconds,
        full_turn);
    const Mat4 pitch = Mat4::rotation_x(frame.pitch_angle);
    const Mat4 yaw = Mat4::rotation_y(frame.yaw_angle);
    const Mat4 roll = Mat4::rotation_z(frame.roll_angle);
    frame.scene.set_transform(frame.model_transform,
                              Mat4::multiply(Mat4::multiply(yaw, pitch), roll));
}

} // namespace

int Application::run(int argc, char** argv) {
    std::string model_path;
    std::string heightmap_path;
    std::array<std::string, 3> map_texture_paths{};
    std::array<std::string, 2> mission_archive_paths{};
    std::size_t mission_terrain_member = 0;
    std::optional<std::size_t> aircraft_entry_member;
    std::optional<std::array<double, 3>> aircraft_start_position;
    bool aircraft_gear_down = false;
    std::optional<std::size_t> mission_units_member;
    bool has_mission_terrain_member = false;
    float heightmap_scale = 100.0f;
    float heightmap_spacing = 10.0f;
    Vec3 heightmap_origin{};
    std::string texture_path;
    std::string material_textures_path;
    std::string flight_rules_path;
    std::string weapon_rules_path;
    std::string animation_rules_path;
    const auto usage = [](FILE* stream) {
        std::fprintf(stream,
                     "usage: ac5_native [--model path.obj|model.mpl|model.bin] "
                     "[--heightmap image.ppm|image.tga|image.gim "
                     "[--height-scale units] [--height-spacing units]] "
                     "[--map-textures map.mti map.mtd map.mtc] "
                     "[--mission-terrain DATA.TBL DATA.PAC member-index] "
                     "[--mission-units mission-pack-member-index] "
                     "[--aircraft-entry-index mission-entry-member-index] "
                     "[--aircraft-start x y z] "
                     "[--aircraft-gear-down] "
                     "[--texture image.ppm|image.tga|image.gim] "
                     "[--material-textures aircraft.pmtex] "
                     "[--flight-rules aircraft.pmb] "
                     "[--weapon-rules aircraft-member.bin] "
                     "[--animation-rules aircraft-animation.bin]\n"
                     "       ac5_native --help\n");
    };
    for (int i = 1; i < argc; ++i) {
        const std::string_view argument(argv[i]);
        if (argument == "--help" || argument == "-h") {
            usage(stdout);
            return 0;
        }
        if (argument == "--aircraft-gear-down") {
            aircraft_gear_down = true;
            continue;
        }
        if (argument == "--height-scale" || argument == "--height-spacing") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value after %s\n", argv[i]);
                usage(stderr);
                return 2;
            }
            const std::string_view value(argv[++i]);
            float parsed = 0.0f;
            const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
            if (result.ec != std::errc{} || result.ptr != value.data() + value.size() ||
                !std::isfinite(parsed) || parsed <= 0.0f) {
                std::fprintf(stderr, "%s must be a finite positive number\n", argv[i - 1]);
                return 2;
            }
            if (argument == "--height-scale") heightmap_scale = parsed;
            else heightmap_spacing = parsed;
            continue;
        }
        if (argument == "--map-textures") {
            if (i + 3 >= argc) {
                std::fprintf(stderr, "--map-textures requires .mti, .mtd, and .mtc paths\n");
                usage(stderr);
                return 2;
            }
            for (std::string& path : map_texture_paths) path = argv[++i];
            continue;
        }
        if (argument == "--mission-terrain") {
            if (i + 3 >= argc) {
                std::fprintf(stderr,
                             "--mission-terrain requires DATA.TBL, DATA.PAC, and a member index\n");
                usage(stderr);
                return 2;
            }
            mission_archive_paths[0] = argv[++i];
            mission_archive_paths[1] = argv[++i];
            const std::string_view value(argv[++i]);
            const auto parsed = std::from_chars(value.data(), value.data() + value.size(),
                                                mission_terrain_member);
            if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) {
                std::fprintf(stderr, "mission terrain member index must be an unsigned integer\n");
                return 2;
            }
            has_mission_terrain_member = true;
            continue;
        }
        if (argument == "--mission-units") {
            if (i + 1 >= argc) {
                std::fprintf(stderr,
                             "--mission-units requires a mission-pack member index\n");
                return 2;
            }
            const std::string_view value(argv[++i]);
            std::size_t parsed_member = 0;
            const auto parsed = std::from_chars(value.data(), value.data() + value.size(),
                                                parsed_member);
            if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) {
                std::fprintf(stderr,
                             "mission unit member index must be an unsigned integer\n");
                return 2;
            }
            mission_units_member = parsed_member;
            continue;
        }
        if (argument == "--aircraft-entry-index") {
            if (i + 1 >= argc) {
                std::fprintf(stderr,
                             "--aircraft-entry-index requires a mission-entry member index\n");
                return 2;
            }
            const std::string_view value(argv[++i]);
            std::size_t parsed_member = 0;
            const auto parsed = std::from_chars(value.data(), value.data() + value.size(),
                                                parsed_member);
            if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) {
                std::fprintf(stderr,
                             "aircraft entry member index must be an unsigned integer\n");
                return 2;
            }
            aircraft_entry_member = parsed_member;
            continue;
        }
        if (argument == "--aircraft-start") {
            if (i + 3 >= argc) {
                std::fprintf(stderr, "--aircraft-start requires x, y, and z world coordinates\n");
                return 2;
            }
            std::array<double, 3> position{};
            bool valid = true;
            for (double& coordinate : position) {
                const std::string_view value(argv[++i]);
                const auto parsed = std::from_chars(
                    value.data(), value.data() + value.size(), coordinate);
                valid = valid && parsed.ec == std::errc{} &&
                    parsed.ptr == value.data() + value.size() &&
                    std::isfinite(coordinate);
            }
            if (!valid) {
                std::fprintf(stderr,
                             "aircraft start coordinates must be finite numbers in world units\n");
                return 2;
            }
            aircraft_start_position = position;
            continue;
        }
        if (argument == "--model" || argument == "--heightmap" ||
            argument == "--texture" ||
            argument == "--material-textures" ||
            argument == "--flight-rules" || argument == "--weapon-rules" ||
            argument == "--animation-rules") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing path after %s\n", argv[i]);
                usage(stderr);
                return 2;
            }
            if (argument == "--model") model_path = argv[++i];
            else if (argument == "--heightmap") heightmap_path = argv[++i];
            else if (argument == "--texture") texture_path = argv[++i];
            else if (argument == "--material-textures") material_textures_path = argv[++i];
            else if (argument == "--flight-rules") flight_rules_path = argv[++i];
            else if (argument == "--weapon-rules") weapon_rules_path = argv[++i];
            else animation_rules_path = argv[++i];
            continue;
        }
        std::fprintf(stderr, "unknown argument: %s\n", argv[i]);
        usage(stderr);
        return 2;
    }

    if (!heightmap_path.empty() && !model_path.empty()) {
        std::fprintf(stderr, "--heightmap and --model cannot be combined\n");
        return 2;
    }
    const bool map_texture_mode = !map_texture_paths[0].empty();
    const bool mission_terrain_mode = has_mission_terrain_member;
    if (mission_units_member.has_value() && !mission_terrain_mode) {
        std::fprintf(stderr, "--mission-units requires --mission-terrain\n");
        return 2;
    }
    if (aircraft_entry_member.has_value() &&
        (!mission_terrain_mode || flight_rules_path.empty() || model_path.empty())) {
        std::fprintf(stderr,
                     "--aircraft-entry-index requires --mission-terrain, --model, and --flight-rules\n");
        return 2;
    }
    if (aircraft_start_position.has_value() && flight_rules_path.empty()) {
        std::fprintf(stderr, "--aircraft-start requires --flight-rules\n");
        return 2;
    }
    if (aircraft_gear_down && (model_path.empty() || flight_rules_path.empty())) {
        std::fprintf(stderr,
                     "--aircraft-gear-down requires an aircraft model and --flight-rules\n");
        return 2;
    }
    if (mission_terrain_mode &&
        (!heightmap_path.empty() || map_texture_mode || !texture_path.empty() ||
         !material_textures_path.empty() ||
         (flight_rules_path.empty() != model_path.empty()) ||
         (!flight_rules_path.empty() && !aircraft_entry_member.has_value()))) {
        std::fprintf(stderr,
                     "--mission-terrain can be viewed alone or combined with --model, --flight-rules, and --aircraft-entry-index\n");
        return 2;
    }
    if (map_texture_mode &&
        (!model_path.empty() || !heightmap_path.empty() || !texture_path.empty() ||
         !material_textures_path.empty() || !flight_rules_path.empty() ||
         !weapon_rules_path.empty() || !animation_rules_path.empty())) {
        std::fprintf(stderr,
                     "--map-textures is a standalone map preview and cannot be combined with other asset modes\n");
        return 2;
    }

    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS | SDL_INIT_GAMEPAD)) {
        std::fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    Scene scene;
    if (model_path.empty() && heightmap_path.empty() && !map_texture_mode &&
        !mission_terrain_mode) {
        const char* base = SDL_GetBasePath();
        model_path = base ? std::string(base) + "assets/demo.obj" : "assets/demo.obj";
    }
    const TransformId model_transform = scene.add_transform();
    std::string load_error;
    std::string extension = std::filesystem::path(model_path).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    const bool acm_model = !heightmap_path.empty() ? false :
                           extension == ".mpl" || extension == ".acm" ||
                           extension == ".bin";
    if (!material_textures_path.empty() && (!acm_model || !texture_path.empty())) {
        std::fprintf(stderr, "--material-textures requires an ACM model and cannot be combined with --texture\n");
        SDL_Quit();
        return 2;
    }
    if (!weapon_rules_path.empty() && (!acm_model || flight_rules_path.empty())) {
        std::fprintf(stderr,
                     "--weapon-rules requires an ACM model and --flight-rules\n");
        SDL_Quit();
        return 2;
    }
    std::optional<AircraftFlightRules> flight_rules;
    std::optional<AircraftWeaponRules> weapon_rules;
    std::optional<AircraftAnimationRules> animation_rules;
    std::optional<AcmPartModel> animated_model;
    std::vector<TransformId> part_transforms;
    std::vector<DemoFrame::ShapeDraw> shape_draws;
    std::vector<std::size_t> part_draw_indices;
    AircraftGearController gear_controller{};
    std::uint32_t animation_controller_flags = 0;
    if (acm_model) {
        AircraftWeaponRules decoded_weapon_rules;
        bool found_weapon_rules = false;
        std::string weapon_error;
        const std::string& weapon_source = weapon_rules_path.empty()
            ? model_path : weapon_rules_path;
        if (!load_acm_weapon_rules(weapon_source, decoded_weapon_rules,
                                   found_weapon_rules, weapon_error)) {
            std::fprintf(stderr, "Aircraft weapon data unavailable: %s\n",
                         weapon_error.c_str());
            if (!weapon_rules_path.empty()) {
                SDL_Quit();
                return 1;
            }
        } else if (found_weapon_rules) {
            weapon_rules = decoded_weapon_rules;
        } else if (!weapon_rules_path.empty()) {
            std::fprintf(stderr,
                         "No supported weapon records found in: %s\n",
                         weapon_rules_path.c_str());
            SDL_Quit();
            return 1;
        }
    }
    if (!flight_rules_path.empty()) {
        if (!acm_model) {
            std::fprintf(stderr, "--flight-rules requires an ACM aircraft model\n");
            SDL_Quit();
            return 2;
        }
        flight_rules.emplace();
        if (!load_aircraft_flight_rules(flight_rules_path, *flight_rules, load_error)) {
            std::fprintf(stderr, "Flight rules load failed: %s\n", load_error.c_str());
            SDL_Quit();
            return 1;
        }
    }
    if (!animation_rules_path.empty()) {
        if (!flight_rules) {
            std::fprintf(stderr, "--animation-rules requires --flight-rules\n");
            SDL_Quit();
            return 2;
        }
        animation_rules.emplace();
        if (!load_aircraft_animation_rules(animation_rules_path, *animation_rules,
                                           load_error)) {
            std::fprintf(stderr, "Animation rules load failed: %s\n", load_error.c_str());
            SDL_Quit();
            return 1;
        }
    }
    bool model_loaded = false;
    DemoLod demo_lod;
    std::vector<TextureId> material_texture_ids;
    std::shared_ptr<Ac5MissionTerrain> mission_terrain_data;
    std::optional<std::array<double, 6>> mission_aircraft_hit_extents;
    if (mission_terrain_mode) {
        DataArchive archive;
        std::vector<ArchiveFile> files;
        mission_terrain_data = std::make_shared<Ac5MissionTerrain>();
        Ac5MissionTerrain& mission_terrain = *mission_terrain_data;
        MeshData terrain_mesh;
        const std::filesystem::path table_path =
            path_from_utf8(mission_archive_paths[0]);
        const std::filesystem::path archive_path =
            path_from_utf8(mission_archive_paths[1]);
        if (!archive.open(table_path, archive_path, load_error) ||
            !archive.split_member(mission_terrain_member, files, load_error) ||
            !decode_ac5_mission_terrain(files, mission_terrain, load_error) ||
            !build_ac5_mission_terrain_mesh(mission_terrain, terrain_mesh,
                                            load_error)) {
            std::fprintf(stderr, "Mission terrain load failed: %s\n",
                         load_error.c_str());
            SDL_Quit();
            return 1;
        }
        if (aircraft_entry_member.has_value()) {
            std::array<double, 6> extents{};
            if (!load_ac5_aircraft_hit_extents(archive, *aircraft_entry_member,
                                               extents, load_error)) {
                std::fprintf(stderr, "Aircraft hit extent load failed: %s\n",
                             load_error.c_str());
                SDL_Quit();
                return 1;
            }
            mission_aircraft_hit_extents = extents;
        }
        const MeshId terrain_mesh_id = scene.add_mesh(std::move(terrain_mesh));
        const TextureId terrain_texture_id =
            scene.add_texture(std::move(mission_terrain.surface.atlas));
        const TransformId terrain_transform = scene.add_transform();
        scene.add_draw(terrain_mesh_id, terrain_texture_id, Mat4::identity(),
                       terrain_transform);
        if (mission_units_member.has_value()) {
            std::vector<Ac5MissionUnit> mission_units;
            if (!load_ac5_mission_units(archive, *mission_units_member,
                                        mission_units, load_error)) {
                std::fprintf(stderr, "Mission unit load failed: %s\n",
                             load_error.c_str());
                SDL_Quit();
                return 1;
            }
            constexpr std::array<Vec3, 4> side_colors{{
                {0.15f, 0.85f, 1.0f}, {1.0f, 0.28f, 0.18f},
                {0.3f, 1.0f, 0.35f}, {1.0f, 0.82f, 0.15f},
            }};
            const Ac5MissionSpawnPreview preview =
                build_ac5_mission_spawn_preview(mission_units, mission_terrain);
            std::array<MeshId, side_colors.size()> marker_meshes{};
            std::array<bool, side_colors.size()> marker_mesh_ready{};
            std::size_t visible_spawns = 0;
            std::size_t unresolved_spawns = preview.unresolved_orders;
            for (const Ac5MissionSpawnPreviewItem& candidate : preview.spawns) {
                const auto& position = candidate.transform.position;
                if (std::any_of(position.begin(), position.begin() + 3,
                                [](double value) {
                                    return !std::isfinite(value) ||
                                        std::abs(value) >
                                            std::numeric_limits<float>::max();
                                })) {
                    ++unresolved_spawns;
                    continue;
                }
                const std::size_t side = candidate.side % side_colors.size();
                if (!marker_mesh_ready[side]) {
                    marker_meshes[side] = scene.add_mesh(
                        make_mission_spawn_marker(side_colors[side]));
                    marker_mesh_ready[side] = true;
                }
                Mat4 marker = Mat4::identity();
                constexpr float marker_size = 1024.0f;
                marker.value[0] = marker.value[5] = marker.value[10] =
                    marker_size;
                marker.value[12] = static_cast<float>(position[0]);
                marker.value[13] = static_cast<float>(position[1]);
                marker.value[14] = static_cast<float>(position[2]);
                const TransformId marker_transform =
                    scene.add_transform(marker, terrain_transform);
                scene.add_draw(marker_meshes[side], Mat4::identity(),
                               marker_transform);
                ++visible_spawns;
            }
            std::printf("Mission spawn preview: %zu markers, %zu unresolved orders; "
                        "actor references with unknown formation offsets are omitted.\n",
                        visible_spawns, unresolved_spawns);
        }
        model_loaded = true;
    } else if (map_texture_mode) {
        Ac5MapTextureSet map_texture_set;
        if (!load_ac5_map_texture_set(map_texture_paths[0], map_texture_paths[1],
                                      map_texture_paths[2], map_texture_set,
                                      load_error)) {
            std::fprintf(stderr, "Map texture load failed: %s\n", load_error.c_str());
            SDL_Quit();
            return 1;
        }
        MeshData map_mesh;
        if (!build_ac5_map_texture_mesh(map_texture_set, {0.0f, 0.0f, 0.0f},
                                        10.0f, 10.0f, map_mesh, load_error)) {
            std::fprintf(stderr, "Map texture mesh failed: %s\n", load_error.c_str());
            SDL_Quit();
            return 1;
        }
        const TextureId map_atlas = scene.add_texture(std::move(map_texture_set.atlas));
        const MeshId map_mesh_id = scene.add_mesh(std::move(map_mesh));
        scene.add_draw(map_mesh_id, map_atlas, Mat4::identity(), model_transform);
        model_loaded = true;
    } else if (!heightmap_path.empty()) {
        TextureData heightmap;
        std::vector<float> elevations;
        std::uint32_t heightmap_width = 0;
        std::uint32_t heightmap_height = 0;
        std::string heightmap_extension =
            std::filesystem::path(heightmap_path).extension().string();
        std::transform(heightmap_extension.begin(), heightmap_extension.end(),
                       heightmap_extension.begin(), [](unsigned char c) {
                           return static_cast<char>(std::tolower(c));
                       });
        if (heightmap_extension == ".mha" || heightmap_extension == ".mhd") {
            std::filesystem::path selector_path =
                std::filesystem::path(heightmap_path);
            std::filesystem::path data_path = selector_path;
            if (heightmap_extension == ".mha") data_path.replace_extension(".mhd");
            else selector_path.replace_extension(".mha");
            if (!load_ac5_mha_mhd_heightfield(
                    path_to_utf8(selector_path), path_to_utf8(data_path),
                    elevations, load_error)) {
                std::fprintf(stderr, "Heightmap load failed: %s\n", load_error.c_str());
                SDL_Quit();
                return 1;
            }
            heightmap_width = Ac5MissionTerrain::height_width;
            heightmap_height = Ac5MissionTerrain::height_width;
            heightmap_spacing = Ac5MissionTerrain::height_spacing_world_units;
            heightmap_origin = {Ac5MissionTerrain::height_origin_xz_world_units,
                                0.0f,
                                Ac5MissionTerrain::height_origin_xz_world_units};
            const auto maximum = std::max_element(elevations.begin(), elevations.end());
            const float maximum_sample = maximum == elevations.end() ? 0.0f : *maximum / 80.0f;
            heightmap.width = heightmap_width;
            heightmap.height = heightmap_height;
            try {
                heightmap.rgba.resize(elevations.size() * 4u);
            } catch (const std::bad_alloc&) {
                std::fprintf(stderr, "Heightmap texture allocation failed\n");
                SDL_Quit();
                return 1;
            } catch (const std::length_error&) {
                std::fprintf(stderr, "Heightmap texture exceeds the platform limit\n");
                SDL_Quit();
                return 1;
            }
            for (std::size_t i = 0; i < elevations.size(); ++i) {
                const float sample = elevations[i] / 80.0f;
                const auto intensity = maximum_sample > 0.0f
                    ? static_cast<std::uint8_t>(std::clamp(
                          std::lround(sample * 255.0f / maximum_sample), 0l, 255l))
                    : std::uint8_t{0};
                heightmap.rgba[i * 4u] = intensity;
                heightmap.rgba[i * 4u + 1u] = intensity;
                heightmap.rgba[i * 4u + 2u] = intensity;
                heightmap.rgba[i * 4u + 3u] = 255;
            }
        } else {
            if (!load_texture_image(heightmap_path, heightmap, load_error)) {
                std::fprintf(stderr, "Heightmap load failed: %s\n", load_error.c_str());
                SDL_Quit();
                return 1;
            }
            heightmap_width = heightmap.width;
            heightmap_height = heightmap.height;
            const std::size_t sample_count = heightmap.rgba.size() / 4u;
            try {
                elevations.resize(sample_count);
            } catch (const std::bad_alloc&) {
                std::fprintf(stderr, "Heightmap sample allocation failed\n");
                SDL_Quit();
                return 1;
            } catch (const std::length_error&) {
                std::fprintf(stderr, "Heightmap exceeds the platform limit\n");
                SDL_Quit();
                return 1;
            }
            for (std::size_t i = 0; i < sample_count; ++i) {
                const float red = static_cast<float>(heightmap.rgba[i * 4u]) / 255.0f;
                elevations[i] = (red - 0.5f) * heightmap_scale;
            }
        }
        constexpr std::array<std::uint32_t, 4> terrain_strides{1, 2, 4, 8};
        demo_lod.meshes.reserve(terrain_strides.size());
        demo_lod.textures.resize(terrain_strides.size(), TextureId{0});
        demo_lod.thresholds.reserve(terrain_strides.size() - 1);
        for (const std::uint32_t stride : terrain_strides) {
            MeshData terrain;
            const bool built = stride == 1
                ? build_heightfield_mesh(elevations, heightmap_width, heightmap_height,
                                         heightmap_origin, heightmap_spacing,
                                         heightmap_spacing, terrain, load_error)
                : build_heightfield_mesh_lod(
                      elevations, heightmap_width, heightmap_height,
                      heightmap_origin, heightmap_spacing, heightmap_spacing,
                      stride, terrain, load_error);
            if (!built) {
                std::fprintf(stderr, "Heightmap mesh failed: %s\n", load_error.c_str());
                SDL_Quit();
                return 1;
            }
            demo_lod.meshes.push_back(scene.add_mesh(std::move(terrain)));
        }
        const float terrain_extent = std::sqrt(
            std::pow(static_cast<float>(heightmap_width - 1) * heightmap_spacing, 2.0f) +
            std::pow(static_cast<float>(heightmap_height - 1) * heightmap_spacing, 2.0f));
        demo_lod.thresholds = {terrain_extent * 0.5f,
                               terrain_extent * 1.5f,
                               terrain_extent * 4.0f};
        demo_lod.draw_index = scene.draws().size();
        scene.add_draw(demo_lod.meshes.front(), demo_lod.textures.front(),
                       Mat4::identity(), model_transform);
        demo_lod.active = true;
        demo_lod.world_distance_thresholds = true;
        if (texture_path.empty()) scene.set_texture(std::move(heightmap));
        model_loaded = true;
    }
    if (heightmap_path.empty() && !map_texture_mode &&
        !material_textures_path.empty()) {
        std::vector<AcmBundleTexture> materials;
        if (!load_pm_texture_bundle(material_textures_path, materials, load_error)) {
            std::fprintf(stderr, "PM material texture load failed: %s\n", load_error.c_str());
            SDL_Quit();
            return 1;
        }
        std::uint32_t maximum_slot = 0;
        for (const AcmBundleTexture& material : materials)
            maximum_slot = std::max(maximum_slot, material.bundle_slots.back());
        material_texture_ids.assign(static_cast<std::size_t>(maximum_slot) + 1u,
                                     std::numeric_limits<TextureId>::max());
        for (AcmBundleTexture& material : materials)
            material_texture_ids[material.bundle_slots.back()] =
                scene.add_texture(std::move(material.texture));
    }
    if (heightmap_path.empty() && !map_texture_mode && acm_model) {
        if (flight_rules || !material_textures_path.empty()) {
            animated_model.emplace();
            if (load_acm_part_model(model_path, *animated_model, load_error)) {
                animation_controller_flags =
                    infer_animation_controller_flags(*animated_model);
                part_transforms.reserve(animated_model->parts.size());
                shape_draws.resize(animated_model->parts.size());
                part_draw_indices.assign(animated_model->parts.size(),
                                          std::numeric_limits<std::size_t>::max());
                for (AcmModelPart& part : animated_model->parts) {
                    if (part.kind == static_cast<std::uint8_t>('W'))
                        gear_controller.capabilities |= 0x100u;
                    else if (part.kind == static_cast<std::uint8_t>('X'))
                        gear_controller.capabilities |= 0x200u;
                    else if (part.kind == static_cast<std::uint8_t>('['))
                        gear_controller.capabilities |= 0x400u;
                    else if (part.kind == static_cast<std::uint8_t>(']'))
                        gear_controller.capabilities |= 0x800u;
                    const Mat4 rotate_x = Mat4::rotation_x(part.rotation.x);
                    const Mat4 rotate_y = Mat4::rotation_y(part.rotation.y);
                    const Mat4 rotate_z = Mat4::rotation_z(part.rotation.z);
                    const Mat4 rotation = Mat4::multiply(
                        rotate_z, Mat4::multiply(rotate_y, rotate_x));
                    const Mat4 local = Mat4::multiply(
                        Mat4::translation(part.position), rotation);
                    const TransformId parent = part.parent == AcmModelPart::no_parent
                        ? model_transform : part_transforms.at(part.parent);
                    const TransformId transform = scene.add_transform(local, parent);
                    part_transforms.push_back(transform);
                    if (!part.mesh.indices.empty()) {
                        DemoFrame::ShapeDraw& shape = shape_draws[part_transforms.size() - 1];
                        shape.draw_index = scene.draws().size();
                        bool compatible_shapes = part.shape_track != AcmModelPart::no_parent &&
                                                 !part.shape_variants.empty();
                        for (const AcmModelPart::ShapeVariant& variant : part.shape_variants) {
                            compatible_shapes = compatible_shapes &&
                                variant.mesh.vertices.size() == part.mesh.vertices.size() &&
                                variant.mesh.indices == part.mesh.indices;
                        }
                        if (compatible_shapes) {
                            const AcmAnimationTrack& track =
                                animated_model->shape_tracks[part.shape_track];
                            for (const AcmAnimationKey& key : track.keys) {
                                compatible_shapes = compatible_shapes &&
                                    std::any_of(part.shape_variants.begin(),
                                        part.shape_variants.end(),
                                        [&key](const AcmModelPart::ShapeVariant& variant) {
                                            return variant.chain_index == key.shape_index;
                                        });
                            }
                        }
                        if (compatible_shapes) {
                            shape.dynamic = true;
                            shape.mesh = scene.add_dynamic_mesh(std::move(part.mesh));
                            for (AcmModelPart::ShapeVariant& variant : part.shape_variants)
                                shape.shapes.emplace_back(variant.chain_index,
                                                          std::move(variant.mesh));
                        } else {
                            shape.mesh = scene.add_mesh(std::move(part.mesh));
                            for (AcmModelPart::ShapeVariant& variant : part.shape_variants) {
                                const MeshId variant_mesh = scene.add_mesh(std::move(variant.mesh));
                                shape.static_variants.emplace_back(variant.chain_index,
                                                                   variant_mesh);
                            }
                        }
                        part_draw_indices[part_transforms.size() - 1] =
                            scene.draws().size();
                        TextureId texture_id{0};
                        if (!material_texture_ids.empty()) {
                            const std::uint32_t material_slot = part.source_words[3];
                            if (material_slot >= material_texture_ids.size() ||
                                material_texture_ids[material_slot] ==
                                    std::numeric_limits<TextureId>::max()) {
                                std::fprintf(stderr,
                                    "ACM part references unavailable PM material slot %u\n",
                                    material_slot);
                                SDL_Quit();
                                return 1;
                            }
                            texture_id = material_texture_ids[material_slot];
                        }
                        scene.add_draw(shape.mesh, texture_id, Mat4::identity(), transform);
                    }
                }
                model_loaded = !scene.draws().empty();
            }
        } else {
            AcmLodBundle lod_bundle;
            bool has_lod_bundle = false;
            if (!load_acm_lod_bundle(model_path, lod_bundle, has_lod_bundle, load_error)) {
                model_loaded = false;
            } else if (has_lod_bundle) {
                demo_lod.meshes.reserve(lod_bundle.levels.size());
                demo_lod.textures.reserve(lod_bundle.levels.size());
                demo_lod.thresholds.reserve(lod_bundle.levels.size());
                for (AcmLodLevel& level : lod_bundle.levels) {
                    demo_lod.meshes.push_back(scene.add_mesh(std::move(level.mesh)));
                    demo_lod.textures.push_back(
                        texture_path.empty() && level.has_texture
                            ? scene.add_texture(std::move(level.texture))
                            : TextureId{0});
                    demo_lod.thresholds.push_back(level.maximum_distance);
                }
                demo_lod.draw_index = scene.draws().size();
                scene.add_draw(demo_lod.meshes.front(), demo_lod.textures.front(),
                               Mat4::identity(), model_transform);
                demo_lod.active = true;
                model_loaded = true;
            } else {
                MeshData mesh;
                if (load_acm_model(model_path, mesh, load_error)) {
                    const MeshId mesh_id = scene.add_mesh(std::move(mesh));
                    scene.add_draw(mesh_id, TextureId{0}, Mat4::identity(), model_transform);
                    model_loaded = true;
                }
            }
        }
        if (model_loaded && texture_path.empty() && material_textures_path.empty() &&
            !demo_lod.active) {
            TextureData embedded_texture;
            bool found_texture = false;
            std::string texture_error;
            if (load_acm_bundle_texture(model_path, embedded_texture,
                                        found_texture, texture_error) && found_texture) {
                const TextureId texture_id = scene.add_texture(std::move(embedded_texture));
                scene.set_all_draw_textures(texture_id);
            } else if (!texture_error.empty()) {
                std::fprintf(stderr, "Embedded model texture unavailable: %s\n",
                             texture_error.c_str());
            }
        }
    } else if (heightmap_path.empty() && !map_texture_mode) {
        model_loaded = load_obj_scene(model_path, scene, load_error,
                                      texture_path.empty(), model_transform);
    }
    if (!model_loaded) {
        std::fprintf(stderr, "Model load failed: %s\n", load_error.c_str());
        SDL_Quit();
        return 1;
    }
    if (!texture_path.empty()) {
        TextureData texture;
        if (!load_texture_image(texture_path, texture, load_error)) {
            std::fprintf(stderr, "Texture load failed: %s\n", load_error.c_str());
            SDL_Quit();
            return 1;
        }
        scene.set_texture(std::move(texture));
        scene.set_all_draw_textures(0);
    }

    SDL_Window* window = SDL_CreateWindow(
        "AC5 Native", 1280, 720,
        SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    int result = 0;
    {
        VulkanRenderer renderer;
        if (!renderer.initialize(window)) {
            std::fprintf(stderr, "Vulkan initialization failed: %s\n",
                         renderer.error().c_str());
            result = 1;
        } else {
            DemoStateMachine scene_state{DemoSceneState::Viewing};
            SceneDispatcher<DemoSceneState, DemoSceneSubstate, DemoFrame> dispatcher;
            if (!dispatcher.register_handler(DemoSceneState::Viewing,
                                             DemoSceneSubstate::Frame,
                                             update_demo_scene)) {
                std::fprintf(stderr, "Could not register the demo scene handler\n");
                result = 1;
                scene_state.request_transition(DemoSceneState::Exiting);
            }
            DemoFrame frame{scene, model_transform};
            frame.mission_terrain = mission_terrain_data;
            frame.aircraft_hit_extents = mission_aircraft_hit_extents;
            frame.gear_controller = gear_controller;
            if (aircraft_gear_down) {
                frame.gear_down = true;
                frame.gear_controller.state = 2;
                for (std::size_t channel = 33; channel <= 38; ++channel)
                    frame.animation_channels[channel] = 50.0f;
                frame.animation_channels[39] = 100.0f;
            }
            frame.animation_controller_flags = animation_controller_flags;
            if (animation_rules) frame.animation_rules = *animation_rules;
            frame.animated_model = std::move(animated_model);
            frame.part_transforms = std::move(part_transforms);
            frame.shape_draws = std::move(shape_draws);
            frame.part_draw_indices = std::move(part_draw_indices);
            if (flight_rules) {
                AircraftFlightState flight_state{};
                if (aircraft_start_position)
                    flight_state.position = *aircraft_start_position;
                flight_state.speed_units_per_tick =
                    flight_rules->speed.cruise_kmh *
                    aircraft_world_units_per_kmh_per_second / 60.0;
                frame.flight_session.emplace(*flight_rules, flight_state);
            }
            if (weapon_rules && frame.flight_session) {
                AircraftWeaponState weapon_state{};
                if (!initialize_aircraft_weapons(*weapon_rules, weapon_state)) {
                    std::fprintf(stderr, "Aircraft weapon data is invalid\n");
                    result = 1;
                    scene_state.request_transition(DemoSceneState::Exiting);
                } else {
                    frame.weapon_rules = *weapon_rules;
                    frame.weapon_state = weapon_state;
                    frame.projectile_mesh = add_projectile_mesh(scene);
                    frame.has_projectile_mesh = true;
                }
            }
            InputSystem input;
            if (scene_state.current_state() == DemoSceneState::Viewing &&
                !renderer.upload_scene(scene)) {
                std::fprintf(stderr, "Mesh upload failed: %s\n", renderer.error().c_str());
                result = 1;
                scene_state.request_transition(DemoSceneState::Exiting);
            }
            scene_state.commit_transition();
            Camera camera{};
            CameraFit camera_fit{};
            const bool camera_fitted = fit_camera_to_scene(scene, camera, camera_fit);
            if (acm_model && !camera_fitted) camera.far_plane = 20000.0f;
            if (flight_rules) {
                if (frame.mission_terrain) {
                    camera.near_plane = 1.0f;
                    camera.far_plane = std::max(camera.far_plane, 1'000'000.0f);
                }
                camera.target = {0.0f, static_cast<float>(
                    frame.flight_session->state().position[1]), 0.0f};
                const float follow_height = camera_fitted && !frame.mission_terrain
                    ? camera_fit.radius * 0.12f : 8.0f;
                const float follow_distance = camera_fitted && !frame.mission_terrain
                    ? camera_fit.distance : 115.0f;
                frame.camera_follow_offset = {0.0f, follow_height, -follow_distance};
                camera.position = {camera.target.x + frame.camera_follow_offset.x,
                                   camera.target.y + frame.camera_follow_offset.y,
                                   camera.target.z + frame.camera_follow_offset.z};
                frame.camera = &camera;
            }
            select_demo_lod(scene, demo_lod, camera);
            const Uint64 counter_frequency = SDL_GetPerformanceFrequency();
            Uint64 previous_counter = SDL_GetPerformanceCounter();
            const auto process_event = [&](const SDL_Event& event) {
                input.handle_event(event);
                if (event.type == SDL_EVENT_QUIT ||
                    (event.type == SDL_EVENT_KEY_DOWN &&
                     event.key.key == SDLK_ESCAPE)) {
                    scene_state.request_transition(DemoSceneState::Exiting);
                } else if (event.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED) {
                    renderer.request_resize();
                } else if (event.type == SDL_EVENT_MOUSE_WHEEL) {
                    const Vec3 offset{camera.position.x - camera.target.x,
                                      camera.position.y - camera.target.y,
                                      camera.position.z - camera.target.z};
                    const float distance = std::sqrt(offset.x * offset.x +
                                                     offset.y * offset.y +
                                                     offset.z * offset.z);
                    if (distance > 1.0e-5f) {
                        const float zoomed = std::clamp(
                            distance * std::pow(0.85f, event.wheel.y), 2.0f, 18000.0f);
                        const float ratio = zoomed / distance;
                        camera.position = {camera.target.x + offset.x * ratio,
                                           camera.target.y + offset.y * ratio,
                                           camera.target.z + offset.z * ratio};
                        if (frame.flight_session) {
                            frame.camera_follow_offset.x *= ratio;
                            frame.camera_follow_offset.y *= ratio;
                            frame.camera_follow_offset.z *= ratio;
                        }
                    }
                }
            };
            while (scene_state.current_state() == DemoSceneState::Viewing) {
                const Uint64 current_counter = SDL_GetPerformanceCounter();
                const float delta_seconds = counter_frequency != 0
                    ? std::clamp(static_cast<float>(current_counter - previous_counter) /
                                     static_cast<float>(counter_frequency),
                                 0.0f, 0.1f)
                    : 0.0f;
                previous_counter = current_counter;
                SDL_Event event;
                while (SDL_PollEvent(&event)) process_event(event);
                if (scene_state.commit_transition()) break;
                int pixel_width = 0, pixel_height = 0;
                SDL_GetWindowSizeInPixels(window, &pixel_width, &pixel_height);
                if (pixel_width <= 0 || pixel_height <= 0) {
                    if (!SDL_WaitEvent(&event)) {
                        std::fprintf(stderr, "SDL_WaitEvent: %s\n", SDL_GetError());
                        result = 1;
                        break;
                    }
                    process_event(event);
                    previous_counter = SDL_GetPerformanceCounter();
                    continue;
                }
                const float aspect = pixel_width > 0 && pixel_height > 0
                    ? static_cast<float>(pixel_width) / static_cast<float>(pixel_height) : 1.0f;
                frame.delta_seconds = delta_seconds;
                frame.controls = input.sample();
                if (scene_state.commit_transition()) break;
                if (!dispatcher.dispatch(frame, scene_state)) {
                    std::fprintf(stderr, "No handler for the active demo scene state\n");
                    result = 1;
                    break;
                }
                if (frame.aircraft_crashed) {
                    SDL_SetWindowTitle(window, "AC5 Native | AIRFRAME CONTACT");
                } else if (frame.weapon_rules) {
                    std::array<char, 160> title{};
                    std::snprintf(title.data(), title.size(),
                        "AC5 Native | SELECTED %s | MSL %u | SP %u | GUN %u%s | shots %zu%s%s%s",
                        frame.weapon_state.selected == AircraftWeaponSlot::Standard
                            ? "MSL" : "SP",
                        static_cast<unsigned>(frame.weapon_state.missile_ammunition),
                        static_cast<unsigned>(frame.weapon_state.special_ammunition),
                        frame.weapon_state.unlimited_gun_ammunition
                            ? 0u : static_cast<unsigned>(frame.weapon_state.gun_ammunition),
                        frame.weapon_state.unlimited_gun_ammunition
                            ? "(unlimited)" : "",
                        frame.weapon_shot_count,
                        frame.unsupported_special_sequence
                            ? " | unsupported special sequence" : "",
                        frame.aircraft_on_ground ? " | GROUND ROLL" : "",
                        frame.aircraft_ground_trace_active
                            ? " | MARKER TRACE" : "");
                    SDL_SetWindowTitle(window, title.data());
                } else if (frame.flight_session) {
                    std::array<char, 96> title{};
                    std::snprintf(title.data(), title.size(), "%s%s%s",
                        frame.aircraft_on_ground ? "AC5 Native | GROUND ROLL" :
                                                   "AC5 Native | FLIGHT",
                        frame.aircraft_ground_trace_active ? " | MARKER TRACE" : "",
                        frame.gear_down ? " | GEAR DOWN" : "");
                    SDL_SetWindowTitle(window, title.data());
                }
                if (frame.has_projectile_mesh)
                    update_projectile_mesh(frame.scene, frame.projectile_mesh,
                                            frame.projectile_system);
                select_demo_lod(scene, demo_lod, camera);
                const Mat4 view_projection = camera.view_projection(aspect);
                if (!renderer.draw_frame(view_projection, scene)) {
                    std::fprintf(stderr, "Vulkan frame failed: %s\n",
                                 renderer.error().c_str());
                    result = 1;
                    scene_state.request_transition(DemoSceneState::Exiting);
                    scene_state.commit_transition();
                }
            }
            renderer.wait_idle();
        }
    }

    SDL_DestroyWindow(window);
    SDL_Quit();
    return result;
}
