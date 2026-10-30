#pragma once

#include "model_shading.hpp"
#include "scene.hpp"

#include <algorithm>
#include <cmath>

namespace ac5 {

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
