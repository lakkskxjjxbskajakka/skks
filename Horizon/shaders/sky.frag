#version 450

layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 out_color;

layout(set = 0, binding = 0) uniform Globals {
    mat4 view_proj;
    mat4 inv_view_proj;
    mat4 light_view_proj[4];
    vec4 cam_pos_time;
    vec4 cam_forward;
    vec4 sun_dir_intensity;
    vec4 sun_color_exposure;
    vec4 ambient_sky;
    vec4 ambient_ground;
    vec4 fog_color_density;
    vec4 screen;
    vec4 shadow_params;
    vec4 cascade_splits;
    vec4 misc;
} g;

float hash12(vec2 p) {
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

float vnoise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    float a = hash12(i);
    float b = hash12(i + vec2(1.0, 0.0));
    float c = hash12(i + vec2(0.0, 1.0));
    float d = hash12(i + vec2(1.0, 1.0));
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

float fbm(vec2 p) {
    float s = 0.0, a = 0.5, norm = 0.0;
    mat2 rot = mat2(0.86, 0.51, -0.51, 0.86);
    for (int i = 0; i < 6; ++i) {
        s += vnoise(p) * a;
        norm += a;
        p = rot * p * 2.03 + vec2(11.3, 7.7);
        a *= 0.5;
    }
    return s / norm;
}

void main() {
    vec2 ndc = v_uv * 2.0 - 1.0;
    vec4 far = g.inv_view_proj * vec4(ndc, 1.0, 1.0);
    vec3 dir = normalize(far.xyz / far.w - g.cam_pos_time.xyz);

    vec3 sun = -normalize(g.sun_dir_intensity.xyz);
    float cos_sun = dot(dir, sun);
    float h = max(dir.y, 0.0);
    float sun_up = clamp(sun.y, 0.0, 1.0);
    float time = g.cam_pos_time.w;

    vec3 sun_tint = g.sun_color_exposure.rgb;
    vec3 horizon = g.fog_color_density.rgb;

    /* ---- atmospheric gradient: cool zenith, warm scattering near the sun ---- */
    vec3 zenith = vec3(0.025, 0.070, 0.185);
    vec3 mid = vec3(0.135, 0.255, 0.470);
    vec3 sky = mix(horizon, mid, smoothstep(0.0, 0.18, h));
    sky = mix(sky, zenith, smoothstep(0.14, 0.92, h));

    float sun_side = pow(max(cos_sun, 0.0), 3.0);
    float low_band = 1.0 - smoothstep(0.0, 0.55, h);
    sky += sun_tint * sun_side * low_band * 0.55;

    /* subtle magenta band on the anti-solar side, typical of clear sunsets */
    float anti = pow(max(-cos_sun, 0.0), 2.0);
    sky += vec3(0.20, 0.10, 0.20) * anti * low_band * 0.35;

    /* ---- cloud deck ---- */
    if (dir.y > 0.004) {
        vec2 wind = vec2(time * 0.0022, time * 0.0009);

        /* cumulus layer */
        vec2 cp = dir.xz / (dir.y + 0.075) * 0.55 + wind;
        float warp = fbm(cp * 0.7 + 3.1);
        float n = fbm(cp * 1.15 + vec2(warp * 0.6));
        float n2 = fbm(cp * 2.9 + 13.0);
        float shape = n * 0.72 + n2 * 0.36 + 0.02;
        float clouds = smoothstep(0.46, 0.78, shape);
        clouds *= smoothstep(0.0, 0.14, dir.y);

        float lit = clamp(cos_sun * 0.5 + 0.5, 0.0, 1.0);
        vec3 cloud_dark = vec3(0.115, 0.115, 0.165);
        vec3 cloud_lit = mix(vec3(0.50, 0.38, 0.40), vec3(1.18, 0.86, 0.68), lit);
        cloud_lit += sun_tint * pow(max(cos_sun, 0.0), 7.0) * 0.85;
        float density = smoothstep(0.30, 0.92, shape);
        vec3 cloud = mix(cloud_dark, cloud_lit, density);
        sky = mix(sky, cloud, clouds * 0.93);

        /* thin high cirrus streaks */
        vec2 cp2 = vec2(dir.x * 2.4 + dir.z * 0.5, dir.z * 0.8 - dir.x * 0.3) /
                   (dir.y + 0.16);
        float c2 = fbm(cp2 * 1.4 + wind * 1.8 + 5.0);
        float cirrus = smoothstep(0.58, 0.86, c2) * smoothstep(0.015, 0.28, dir.y);
        vec3 cirrus_col = mix(vec3(0.55, 0.52, 0.62), vec3(1.25, 0.94, 0.82), lit);
        sky = mix(sky, cirrus_col, cirrus * 0.55);
    }

    /* ---- sun disk + halo ---- */
    float disk = smoothstep(0.99962, 0.99993, cos_sun);
    float halo = pow(max(cos_sun, 0.0), 260.0);
    float bloom = pow(max(cos_sun, 0.0), 18.0);
    sky += sun_tint * disk * 55.0;
    sky += sun_tint * halo * 3.4;
    sky += sun_tint * bloom * 0.35;

    /* faint crepuscular streaks radiating from the sun near the horizon */
    float ang = atan(dir.y, length(dir.xz));
    float streak = vnoise(vec2(ang * 46.0 + time * 0.05, time * 0.02));
    sky += sun_tint * streak * pow(max(cos_sun, 0.0), 6.0) * (1.0 - smoothstep(0.0, 0.4, h)) * 0.10;

    /* ---- horizon haze ---- */
    float haze = exp(-h * 16.0);
    vec3 haze_col = horizon * 1.12 + sun_tint * sun_side * 0.55;
    sky = mix(sky, haze_col, haze * 0.72);

    /* ground half-space matches the fog so the world edge blends away */
    if (dir.y < 0.0) {
        sky = mix(horizon * 0.86, horizon, smoothstep(-0.25, 0.0, dir.y));
    }

    /* dither to hide 16-bit banding in the long gradients */
    float dither = (hash12(gl_FragCoord.xy + fract(time)) - 0.5) / 255.0;
    sky += dither;

    out_color = vec4(max(sky, vec3(0.0)), 1.0);
}
