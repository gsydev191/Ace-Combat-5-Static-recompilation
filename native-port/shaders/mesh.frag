#version 450

layout(location = 0) in vec3 color;
layout(location = 1) in vec2 uv;
layout(location = 2) in vec3 normal;
layout(location = 3) in float vertex_alpha;
layout(location = 7) in float light_response;
layout(location = 0) out vec4 out_color;

layout(set = 0, binding = 0) uniform sampler2D diffuse_texture;

layout(push_constant) uniform DrawConstants {
    mat4 view_projection_model;
    vec4 normal_columns[3];
    vec4 lighting;
} draw_constants;

void main() {
    vec4 texel = texture(diffuse_texture, uv);
    vec3 light_direction = normalize(draw_constants.lighting.xyz);
    float normal_length_squared = dot(normal, normal);
    float diffuse = 0.0;
    // ACM records can contain a zero normal. Keep such vertices at ambient
    // light instead of normalizing zero and propagating NaNs into the color.
    if (normal_length_squared > 1.0e-20 && !isinf(normal_length_squared) &&
        !isnan(normal_length_squared)) {
        vec3 unit_normal = normal * inversesqrt(normal_length_squared);
        diffuse = max(dot(unit_normal, light_direction), 0.0);
    }
    vec3 diffuse_lighting = vec3(draw_constants.lighting.w) +
                            vec3(1.0 - draw_constants.lighting.w) * diffuse;
    vec3 applied_lighting = mix(vec3(1.0), diffuse_lighting, light_response);
    out_color = vec4(color * texel.rgb * applied_lighting, texel.a * vertex_alpha);
}
