#pragma once

#include "model_shading.hpp"
#include "scene.hpp"

#include <algorithm>
#include <cmath>

namespace ac5 {

// Use already-lit model vertex colors without applying the renderer's demo
// diffuse light a second time. Vertex opacity stays a separate material
// property because model-program GS alpha is not necessarily opacity.
inline void apply_model_base_shading(Vertex& vertex,
                                     const ModelVertexOut& shaded) noexcept {
    const auto channel = [](std::int32_t value) {
        return static_cast<float>(std::clamp(value, 0, 255)) / 255.0f;
    };
    vertex.color = {channel(shaded.rgba[0]), channel(shaded.rgba[1]),
                    channel(shaded.rgba[2])};
    vertex.light_response = 0.0f;
}

inline bool apply_environment_shading(Vertex& vertex,
                                      const ModelVertexOut& shaded) noexcept;

// Convert one decoded model vertex through the portable shading routine into
// the renderer's native vertex format. Opacity remains at the native default;
// decoded material state owns that choice separately from GS vertex alpha.
inline bool convert_model_vertex(Vertex& vertex, ModelShadeMode mode,
                                 const ModelShadeParams& params,
                                 const ModelVertexIn& input) noexcept {
    if (!std::isfinite(input.position[0]) || !std::isfinite(input.position[1]) ||
        !std::isfinite(input.position[2]) || !std::isfinite(input.uv[0]) ||
        !std::isfinite(input.uv[1]))
        return false;

    const ModelVertexOut shaded = shade_model_vertex(mode, params, input);
    Vertex converted;
    converted.position = {input.position[0], input.position[1], input.position[2]};
    converted.uv = {input.uv[0], input.uv[1]};
    converted.normal = {static_cast<float>(input.normal[0]) / 4096.0f,
                        static_cast<float>(input.normal[1]) / 4096.0f,
                        static_cast<float>(input.normal[2]) / 4096.0f};
    apply_model_base_shading(converted, shaded);
    if (mode == ModelShadeMode::Environment &&
        !apply_environment_shading(converted, shaded))
        return false;
    vertex = converted;
    return true;
}

// Copy the environment pass outputs from the portable model vertex program
// into the native Vulkan vertex representation. The model program emits
// projective STQ, while the native shader expects normalized UVs and applies
// perspective interpolation itself.
inline bool apply_environment_shading(Vertex& vertex,
                                      const ModelVertexOut& shaded) noexcept {
    const float q = shaded.environment_st[2];
    if (!std::isfinite(q) || std::abs(q) <= 1.0e-8f ||
        !std::isfinite(shaded.environment_st[0]) ||
        !std::isfinite(shaded.environment_st[1]))
        return false;

    const Vec2 uv{shaded.environment_st[0] / q,
                  shaded.environment_st[1] / q};
    if (!std::isfinite(uv.x) || !std::isfinite(uv.y)) return false;

    const auto channel = [](std::int32_t value) {
        return static_cast<float>(std::clamp(value, 0, 255)) / 255.0f;
    };
    const Vec3 color{channel(shaded.environment_rgba[0]),
                     channel(shaded.environment_rgba[1]),
                     channel(shaded.environment_rgba[2])};
    const float alpha = channel(shaded.environment_rgba[3]);

    vertex.environment_uv = uv;
    vertex.environment_color = color;
    vertex.environment_alpha = alpha;
    return true;
}

} // namespace ac5
