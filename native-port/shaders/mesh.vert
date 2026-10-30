#version 450

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_color;
layout(location = 2) in vec2 in_uv;
layout(location = 3) in vec3 in_normal;
layout(location = 4) in float in_alpha;
layout(location = 5) in vec2 in_environment_uv;
layout(location = 6) in vec3 in_environment_color;
layout(location = 7) in float in_environment_alpha;
layout(location = 8) in float in_light_response;
layout(location = 0) out vec3 color;
layout(location = 1) out vec2 uv;
layout(location = 2) out vec3 normal;
layout(location = 3) out float vertex_alpha;
layout(location = 4) out vec2 environment_uv;
layout(location = 5) out vec3 environment_color;
layout(location = 6) out float environment_alpha;
layout(location = 7) out float light_response;

layout(push_constant) uniform DrawConstants {
    mat4 view_projection_model;
    vec4 normal_columns[3];
} draw_constants;

void main() {
    gl_Position = draw_constants.view_projection_model * vec4(in_position, 1.0);
    color = in_color;
    uv = vec2(in_uv.x, 1.0 - in_uv.y);
    normal = mat3(draw_constants.normal_columns[0].xyz,
                  draw_constants.normal_columns[1].xyz,
                  draw_constants.normal_columns[2].xyz) * in_normal;
    vertex_alpha = in_alpha;
    environment_uv = in_environment_uv;
    environment_color = in_environment_color;
    environment_alpha = in_environment_alpha;
    light_response = in_light_response;
}
