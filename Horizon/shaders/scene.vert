#version 450

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec4 a_tangent;
layout(location = 3) in vec2 a_uv;

layout(location = 5) in mat4 i_model;
layout(location = 9) in vec4 i_color;
layout(location = 10) in vec4 i_params;

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

layout(location = 0) out vec3 v_world;
layout(location = 1) out vec3 v_normal;
layout(location = 2) out vec4 v_tangent;
layout(location = 3) out vec2 v_uv;
layout(location = 4) out vec4 v_tint;
layout(location = 5) out vec4 v_params;

void main() {
    mat4 model = i_model;
    vec4 world = model * vec4(a_position, 1.0);
    mat3 nm = mat3(model);
    v_world = world.xyz;
    v_normal = nm * a_normal;
    v_tangent = vec4(nm * a_tangent.xyz, a_tangent.w);
    v_uv = a_uv;
    v_tint = i_color;
    v_params = i_params;
    gl_Position = g.view_proj * world;
}
