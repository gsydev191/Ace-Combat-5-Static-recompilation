#pragma once

#include <array>
#include <cstdint>

namespace ac5 {

// Per-vertex transform and lighting for static models, ported from the
// behaviour of the original model vertex program as plain math with no
// vector-unit state. Matrices are column-major: `clip[c]` is column c.
//
// Environment mode returns the ordinary lit pass plus a second reflective
// pass in environment_st and environment_rgba.

using Float4 = std::array<float, 4>;
using Float4x4 = std::array<Float4, 4>;

enum class ModelShadeMode : std::uint8_t {
    Plain, // unlit: vertex colour passes through
    Lit,   // three directional lights plus ambient, cubed falloff
    Environment, // lit base pass plus reflective environment pass
};

struct ModelShadeParams {
    Float4x4 clip{};        // model-space position -> clip space
    // light_dir[k][j]: contribution of normal component k to light j.
    Float4x4 light_dir{};   // only the first three rows are used
    // light_col[j]: RGBA colour of light j (j == 3 is the ambient term).
    Float4x4 light_col{};
    // Environment direction transform applied to the signed 4.12 normal.
    Float4x4 environment_dir{}; // first three rows and columns are used
    float environment_alpha = 128.0f;
    float fog_bias = 0.0f;  // fog = fog_bias + clip.w * fog_scale
    float fog_scale = 0.0f;
    float depth_scale = 1.0f;
    bool aircraft = false;  // aircraft models carry a x16 depth scale
};

struct ModelVertexIn {
    Float4 position{};      // xyz used
    std::array<std::int16_t, 3> normal{}; // signed 4.12 fixed point
    std::array<std::int32_t, 4> color{};  // 0..255 per channel
    std::array<float, 2> uv{};
};

struct ModelVertexOut {
    Float4 screen{};        // x/w, y/w, z scaled+clamped, fog (0..255)
    Float4 st{};            // u/w, v/w, 1/w
    std::array<std::int32_t, 4> rgba{}; // lit colour, alpha fixed at 128
    Float4 environment_st{}; // reflected u/w, v/w, 1/w, 1
    std::array<std::int32_t, 4> environment_rgba{};
};

ModelVertexOut shade_model_vertex(ModelShadeMode mode, const ModelShadeParams& p,
                                  const ModelVertexIn& in) noexcept;

} // namespace ac5
