#include "model_shading.hpp"

#include <algorithm>

namespace ac5 {
namespace {

Float4 transform(const Float4x4& m, float x, float y, float z, float w) noexcept {
    Float4 o{};
    for (int i = 0; i < 4; ++i) {
        float acc = m[0][i] * x;
        acc = acc + m[1][i] * y;
        acc = acc + m[2][i] * z;
        o[i] = acc + m[3][i] * w;
    }
    return o;
}

} // namespace

ModelVertexOut shade_model_vertex(ModelShadeMode mode, const ModelShadeParams& p,
                                  const ModelVertexIn& in) noexcept {
    ModelVertexOut out;
    Float4 c = transform(p.clip, in.position[0], in.position[1], in.position[2], 1.0f);
    const float q = 1.0f / c[3];
    const float zscale = p.aircraft ? 16.0f : p.depth_scale;

    float fog = 1.0f * p.fog_bias + c[3] * p.fog_scale;
    float z = c[2] * zscale * q;
    out.screen = {c[0] * q, c[1] * q, 0.0f, 0.0f};
    out.st = {in.uv[0] * q, in.uv[1] * q, 1.0f * q, 1.0f};

    z = std::max(std::min(z, 16777215.0f), 0.0f);
    fog = std::max(std::min(fog, 255.0f), 0.0f);
    out.screen[2] = z;
    out.screen[3] = fog;

    if (mode == ModelShadeMode::Plain) {
        out.rgba = in.color;
        return out;
    }

    float n[3];
    for (int i = 0; i < 3; ++i) n[i] = static_cast<float>(in.normal[i]) / 4096.0f;
    float l[4];
    for (int j = 0; j < 3; ++j) {
        float acc = p.light_dir[0][j] * n[0];
        acc = acc + p.light_dir[1][j] * n[1];
        l[j] = acc + p.light_dir[2][j] * n[2];
    }
    l[3] = 1.0f;
    for (int j = 0; j < 4; ++j) l[j] = l[j] * l[j] * l[j];
    for (int j = 0; j < 3; ++j) l[j] *= 2.0f;
    for (int j = 0; j < 4; ++j) l[j] = std::max(l[j], 0.0f);

    const Float4 lc = transform(p.light_col, l[0], l[1], l[2], l[3]);
    for (int j = 0; j < 3; ++j) {
        float ch = static_cast<float>(in.color[j]) * std::max(lc[j], 0.0f);
        ch = std::min(ch, 255.0f);
        out.rgba[j] = static_cast<std::int32_t>(ch);
    }
    out.rgba[3] = 128;
    if (mode == ModelShadeMode::Environment) {
        Float4 reflected{};
        for (int j = 0; j < 3; ++j) {
            float acc = p.environment_dir[0][j] * n[0];
            acc = acc + p.environment_dir[1][j] * n[1];
            reflected[j] = acc + p.environment_dir[2][j] * n[2];
        }
        reflected[3] = 1.0f;

        Float4 environment_color{};
        for (int j = 0; j < 3; ++j) {
            const float intensity = std::clamp(reflected[j], 0.0f, 128.0f) * 1280.0f;
            environment_color[j] = std::min(intensity, 128.0f);
        }
        environment_color[3] = p.environment_alpha;
        out.environment_rgba = {
            static_cast<std::int32_t>(environment_color[0]),
            static_cast<std::int32_t>(environment_color[1]),
            static_cast<std::int32_t>(environment_color[2]),
            static_cast<std::int32_t>(environment_color[3])};

        reflected[0] = (reflected[0] + 1.0f) * 0.5f;
        reflected[1] = (reflected[1] + 1.0f) * 0.5f;
        reflected[2] = 1.0f;
        out.environment_st = {reflected[0] * q, reflected[1] * q, q, 1.0f};
    }
    return out;
}

} // namespace ac5
