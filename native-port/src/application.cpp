#include "application.hpp"

#include "acm_loader.hpp"
#include "flight_model.hpp"
#include "flight_rules_loader.hpp"
#include "flight_session.hpp"
#include "input.hpp"
#include "obj_loader.hpp"
#include "texture_loader.hpp"
#include "vulkan_renderer.hpp"

#include <SDL3/SDL.h>

#include <cstdio>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

using namespace ac5;

namespace {

enum class DemoSceneState : std::uint8_t { Viewing, Exiting };
enum class DemoSceneSubstate : std::uint8_t { Frame };

using DemoStateMachine = SceneStateMachine<DemoSceneState, DemoSceneSubstate>;

struct DemoFrame {
    Scene& scene;
    TransformId model_transform;
    DemoFrame(Scene& scene_ref, TransformId transform)
        : scene(scene_ref), model_transform(transform) {}
    Camera* camera = nullptr;
    std::optional<AircraftFlightSession> flight_session;
    std::optional<AcmPartModel> animated_model;
    std::vector<TransformId> part_transforms;
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
};

std::array<double, 64> aircraft_animation_channels(const FlightInput& controls) {
    std::array<double, 64> channels{};
    channels.fill(50.0);
    const auto set = [&channels](std::size_t channel, double value) {
        channels[channel] = std::clamp(value, 1.0, 100.0);
    };
    // These are the recovered direct stick mappings. Aircraft-specific
    // authority and speed curves need the ACM's companion animation config.
    set(1, 50.0 + 50.0 * controls.pitch);
    set(3, 50.0 + 50.0 * controls.yaw);
    set(5, 50.0 - 50.0 * controls.roll);
    set(6, 50.0 + 50.0 * controls.roll);
    set(8, 50.0 + 25.0 * controls.pitch);
    set(12, 50.0 - 50.0 * controls.roll);
    set(13, 50.0 + 50.0 * controls.roll);

    const double lever = std::clamp(static_cast<double>(controls.throttle), -1.0, 1.0);
    double dry = 0.0;
    double afterburner = 0.0;
    double airbrake = 0.0;
    if (lever >= 0.9) {
        dry = 1.0;
        afterburner = (lever - 0.9) / 0.1;
    } else if (lever >= -0.6) {
        dry = (lever + 0.6) / 1.5;
    } else {
        airbrake = (-0.6 - lever) / 0.4;
    }
    set(14, 50.0 * dry + 50.0 * afterburner);
    set(15, 50.0 + 50.0 * airbrake);
    return channels;
}

void update_aircraft_part_animation(DemoFrame& frame) {
    if (!frame.animated_model ||
        frame.animated_model->parts.size() != frame.part_transforms.size()) return;
    const std::array<double, 64> channels = aircraft_animation_channels(frame.controls);
    for (std::size_t i = 0; i < frame.animated_model->parts.size(); ++i) {
        const AcmModelPart& part = frame.animated_model->parts[i];
        Vec3 position = part.position;
        Vec3 rotation = part.rotation;
        Vec3 animated{};
        if (part.translation_track != AcmModelPart::no_parent) {
            const AcmAnimationTrack& track =
                frame.animated_model->translation_tracks[part.translation_track];
            if (track.channel < channels.size() &&
                sample_acm_animation_track(track, channels[track.channel], animated)) {
                position.x += animated.x;
                position.y += animated.y;
                position.z += animated.z;
            }
        }
        if (part.rotation_track != AcmModelPart::no_parent) {
            const AcmAnimationTrack& track =
                frame.animated_model->rotation_tracks[part.rotation_track];
            if (track.channel < channels.size() &&
                sample_acm_animation_track(track, channels[track.channel], animated)) {
                rotation.x += animated.x;
                rotation.y += animated.y;
                rotation.z += animated.z;
            }
        }
        const Mat4 rotation_x = Mat4::rotation_x(rotation.x);
        const Mat4 rotation_y = Mat4::rotation_y(rotation.y);
        const Mat4 rotation_z = Mat4::rotation_z(rotation.z);
        const Mat4 local_rotation = Mat4::multiply(
            rotation_z, Mat4::multiply(rotation_y, rotation_x));
        frame.scene.set_transform(frame.part_transforms[i], Mat4::multiply(
            Mat4::translation(position), local_rotation));
    }
}

void select_demo_lod(Scene& scene, const DemoLod& lod, const Camera& camera) {
    if (!lod.active || lod.meshes.empty()) return;
    const Vec3 offset{camera.position.x - camera.target.x,
                      camera.position.y - camera.target.y,
                      camera.position.z - camera.target.z};
    const float distance = std::sqrt(offset.x * offset.x + offset.y * offset.y +
                                     offset.z * offset.z);
    constexpr float retail_scale = 627.87f;
    const float scaled_distance = distance * 512.0f / retail_scale;
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

void update_demo_scene(DemoFrame& frame, DemoStateMachine& scene_state) {
    if (frame.flight_session) {
        if (frame.controls.approach_control.pressed) {
            const bool enabled =
                (frame.flight_session->state().flags.contact & 0x20u) == 0;
            set_aircraft_approach_control(frame.flight_session->state().flags,
                                           enabled);
            std::fprintf(stderr, "Approach control %s\n",
                         enabled ? "enabled" : "disabled");
        }
        const std::array<double, 3> previous_position =
            frame.flight_session->state().position;
        AircraftFlightInput flight_input{};
        flight_input.pitch = frame.controls.pitch;
        flight_input.roll = frame.controls.roll;
        flight_input.yaw = frame.controls.yaw;
        flight_input.throttle_lever = frame.controls.throttle;
        flight_input.ticks_per_second = 60;
        if (!frame.flight_session->advance_frame(frame.delta_seconds, flight_input)) {
            std::fprintf(stderr, "Native flight session rejected its rules or state\n");
            scene_state.request_transition(DemoSceneState::Exiting);
            return;
        }
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
            const Vec3 movement{
                static_cast<float>(flight_state.position[0] - previous_position[0]),
                static_cast<float>(flight_state.position[1] - previous_position[1]),
                static_cast<float>(flight_state.position[2] - previous_position[2])};
            frame.camera->position.x += movement.x;
            frame.camera->position.y += movement.y;
            frame.camera->position.z += movement.z;
            frame.camera->target.x += movement.x;
            frame.camera->target.y += movement.y;
            frame.camera->target.z += movement.z;
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
    std::string texture_path;
    std::string flight_rules_path;
    const auto usage = [](FILE* stream) {
        std::fprintf(stream,
                     "usage: ac5_native [--model path.obj|model.mpl|model.bin] "
                     "[--texture image.ppm|image.tga|image.gim] "
                     "[--flight-rules aircraft.pmb]\n"
                     "       ac5_native --help\n");
    };
    for (int i = 1; i < argc; ++i) {
        const std::string_view argument(argv[i]);
        if (argument == "--help" || argument == "-h") {
            usage(stdout);
            return 0;
        }
        if (argument == "--model" || argument == "--texture" ||
            argument == "--flight-rules") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing path after %s\n", argv[i]);
                usage(stderr);
                return 2;
            }
            if (argument == "--model") model_path = argv[++i];
            else if (argument == "--texture") texture_path = argv[++i];
            else flight_rules_path = argv[++i];
            continue;
        }
        std::fprintf(stderr, "unknown argument: %s\n", argv[i]);
        usage(stderr);
        return 2;
    }

    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS | SDL_INIT_GAMEPAD)) {
        std::fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    Scene scene;
    if (model_path.empty()) {
        const char* base = SDL_GetBasePath();
        model_path = base ? std::string(base) + "assets/demo.obj" : "assets/demo.obj";
    }
    const TransformId model_transform = scene.add_transform();
    std::string load_error;
    std::string extension = std::filesystem::path(model_path).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    const bool acm_model = extension == ".mpl" || extension == ".acm" ||
                           extension == ".bin";
    std::optional<AircraftFlightRules> flight_rules;
    std::optional<AcmPartModel> animated_model;
    std::vector<TransformId> part_transforms;
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
    bool model_loaded = false;
    DemoLod demo_lod;
    if (acm_model) {
        if (flight_rules) {
            animated_model.emplace();
            if (load_acm_part_model(model_path, *animated_model, load_error)) {
                part_transforms.reserve(animated_model->parts.size());
                for (AcmModelPart& part : animated_model->parts) {
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
                        const MeshId mesh_id = scene.add_mesh(std::move(part.mesh));
                        scene.add_draw(mesh_id, TextureId{0}, Mat4::identity(), transform);
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
        if (model_loaded && texture_path.empty() && !demo_lod.active) {
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
    } else {
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
            frame.animated_model = std::move(animated_model);
            frame.part_transforms = std::move(part_transforms);
            if (flight_rules) {
                AircraftFlightState flight_state{};
                flight_state.speed_units_per_tick =
                    flight_rules->speed.cruise_kmh * 1.4222219 / 60.0;
                frame.flight_session.emplace(*flight_rules, flight_state);
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
            if (acm_model) {
                camera.position = {0.0f, 12.0f, -115.0f};
                camera.target = {0.0f, 4.0f, 0.0f};
                camera.far_plane = 20000.0f;
            }
            if (flight_rules) {
                camera.target = {0.0f, static_cast<float>(
                    frame.flight_session->state().position[1]), 0.0f};
                camera.position = {0.0f, camera.target.y + 8.0f, -115.0f};
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
