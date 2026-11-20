#version 450

layout(location = 0) in vec3 a_position;
layout(location = 5) in mat4 i_model;

layout(set = 0, binding = 0) uniform Globals {
    mat4 view_proj;
    mat4 inv_view_proj;
    mat4 light_view_proj;
    vec4 cam_pos_time;
    vec4 sun_dir_intensity;
    vec4 sun_color_exposure;
    vec4 ambient_sky;
    vec4 ambient_ground;
    vec4 fog_color_density;
    vec4 screen;
    vec4 shadow_params;
    vec4 misc;
} g;

void main() {
    gl_Position = g.light_view_proj * i_model * vec4(a_position, 1.0);
}
