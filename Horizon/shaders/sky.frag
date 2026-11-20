#version 450

layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 out_color;

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

float hash(vec2 p) {
    p = fract(p * vec2(233.34, 851.73));
    p += dot(p, p + 23.45);
    return fract(p.x * p.y);
}

float vnoise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    float a = hash(i), b = hash(i + vec2(1, 0));
    float c = hash(i + vec2(0, 1)), d = hash(i + vec2(1, 1));
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y) * 2.0 - 1.0;
}

float fbm(vec2 p) {
    float s = 0.0, a = 0.5;
    for (int i = 0; i < 5; ++i) {
        s += vnoise(p) * a;
        p = p * 2.03 + vec2(11.3, 7.7);
        a *= 0.5;
    }
    return s;
}

void main() {
    vec2 ndc = v_uv * 2.0 - 1.0;
    vec4 far = g.inv_view_proj * vec4(ndc, 1.0, 1.0);
    vec3 dir = normalize(far.xyz / far.w - g.cam_pos_time.xyz);

    vec3 sun = -normalize(g.sun_dir_intensity.xyz);
    float cos_sun = max(dot(dir, sun), 0.0);
    float h = max(dir.y, 0.0);

    vec3 zenith = vec3(0.025, 0.075, 0.20);
    vec3 mid = vec3(0.12, 0.26, 0.52);
    vec3 horizon = g.fog_color_density.rgb;

    vec3 sky = mix(horizon, mid, smoothstep(0.0, 0.22, h));
    sky = mix(sky, zenith, smoothstep(0.18, 0.85, h));

    float mie = pow(cos_sun, 8.0);
    float mie2 = pow(cos_sun, 90.0);
    vec3 sun_tint = g.sun_color_exposure.rgb;
    sky += sun_tint * mie * 0.30;
    sky += sun_tint * mie2 * 0.9;

    float disk = smoothstep(0.99965, 0.99992, cos_sun);
    sky += sun_tint * disk * 18.0;

    /* procedural cloud deck */
    if (dir.y > 0.008) {
        vec2 cp = dir.xz / (dir.y + 0.06);
        cp = cp * 0.55 + vec2(g.cam_pos_time.w * 0.0035, g.cam_pos_time.w * 0.0012);
        float n = fbm(cp * 1.15);
        float n2 = fbm(cp * 2.7 + 13.0);
        float clouds = smoothstep(0.05, 0.66, n * 0.72 + n2 * 0.38 + 0.12);
        clouds *= smoothstep(0.0, 0.16, dir.y);
        vec3 cloud_lit = mix(vec3(0.30, 0.30, 0.36), vec3(1.05, 0.92, 0.82), pow(cos_sun, 2.0));
        cloud_lit += sun_tint * mie * 0.6;
        float density = clouds * 0.92;
        sky = mix(sky, cloud_lit, density * 0.85);
    }

    /* thin haze band hugging the horizon */
    float haze = exp(-h * 22.0);
    sky = mix(sky, horizon * 1.15 + sun_tint * mie * 0.5, haze * 0.7);

    out_color = vec4(max(sky, vec3(0.0)), 1.0);
}
