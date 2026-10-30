#version 450

layout(location = 4) in vec2 environment_uv;
layout(location = 5) in vec3 environment_color;
layout(location = 6) in float environment_alpha;
layout(location = 0) out vec4 out_color;

layout(set = 0, binding = 0) uniform sampler2D environment_texture;

void main() {
    vec4 texel = texture(environment_texture, environment_uv);
    out_color = vec4(environment_color * texel.rgb, environment_alpha * texel.a);
}
