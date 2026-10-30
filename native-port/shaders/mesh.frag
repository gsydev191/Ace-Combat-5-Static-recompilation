#version 450

layout(location = 0) in vec3 color;
layout(location = 1) in vec2 uv;
layout(location = 2) in vec3 normal;
layout(location = 3) in float vertex_alpha;
layout(location = 7) in float light_response;
layout(location = 0) out vec4 out_color;

layout(set = 0, binding = 0) uniform sampler2D diffuse_texture;

void main() {
    vec4 texel = texture(diffuse_texture, uv);
    vec3 light_direction = normalize(vec3(0.35, 0.55, 1.0));
    float diffuse = max(dot(normalize(normal), light_direction), 0.0);
    vec3 demo_lighting = vec3(0.22) + vec3(0.78) * diffuse;
    vec3 lighting = mix(vec3(1.0), demo_lighting, light_response);
    out_color = vec4(color * texel.rgb * lighting, texel.a * vertex_alpha);
}
