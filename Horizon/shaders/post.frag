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

layout(set = 1, binding = 0) uniform sampler2D u_hdr;
layout(set = 1, binding = 1) uniform sampler2D u_bloom;
layout(set = 1, binding = 2) uniform sampler2D u_depth;

layout(push_constant) uniform PC {
    vec4 p0; /* exposure, bloom intensity, saturation, contrast */
    vec4 p1; /* vignette, grain, time, fxaa */
    vec4 p2; /* encode-gamma flag, unused */
    vec4 p3; /* near, far, ssao strength, unused */
} pc;

vec3 aces_filmic(vec3 x) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

float luma(vec3 c) { return dot(c, vec3(0.2126, 0.7152, 0.0722)); }

float hash12(vec2 p) {
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

vec3 world_from_depth(vec2 uv, float d) {
    vec4 p = g.inv_view_proj * vec4(uv * 2.0 - 1.0, d, 1.0);
    return p.xyz / p.w;
}

float screen_ao(vec2 uv, float center_d, vec3 center_w, vec3 cam) {
    if (center_d > 0.99995) return 1.0;
    float center_lin = length(center_w - cam);
    if (center_lin > 220.0) return 1.0;

    float radius_px = mix(12.0, 5.0, clamp(center_lin / 180.0, 0.0, 1.0));
    vec2 texel = 1.0 / vec2(textureSize(u_depth, 0));
    float rot = hash12(uv * 917.0 + 13.0) * 6.2831853;
    float radius_world = 0.55 + center_lin * 0.010;
    float bias = 0.020 + center_lin * 0.0016;

    float occ = 0.0;
    const int N = 12;
    for (int i = 0; i < N; ++i) {
        float a = rot + float(i) * 2.39996323;
        float r = sqrt((float(i) + 0.5) / float(N));
        vec2 off = vec2(cos(a), sin(a)) * r * radius_px * texel;
        vec2 suv = uv + off;
        if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) continue;
        float sd = texture(u_depth, suv).r;
        if (sd > 0.99995) continue;
        vec3 sw = world_from_depth(suv, sd);
        float sl = length(sw - cam);
        float diff = center_lin - sl;
        if (diff > bias && diff < radius_world * 4.0) {
            occ += 1.0 - smoothstep(0.0, radius_world * 4.0, diff);
        }
    }
    return clamp(1.0 - (occ / float(N)) * 2.4, 0.0, 1.0);
}

vec3 fxaa(vec3 center, vec2 uv) {
    vec2 texel = 1.0 / vec2(textureSize(u_hdr, 0));
    vec3 n = texture(u_hdr, uv + vec2(0.0, -texel.y)).rgb;
    vec3 s = texture(u_hdr, uv + vec2(0.0, texel.y)).rgb;
    vec3 w = texture(u_hdr, uv + vec2(-texel.x, 0.0)).rgb;
    vec3 e = texture(u_hdr, uv + vec2(texel.x, 0.0)).rgb;
    float lc = luma(center);
    float ln = luma(n), ls = luma(s), lw = luma(w), le = luma(e);
    float lmin = min(lc, min(min(ln, ls), min(lw, le)));
    float lmax = max(lc, max(max(ln, ls), max(lw, le)));
    float contrast = lmax - lmin;
    if (contrast < 0.06) return center;
    vec3 average = (n + s + w + e + center) * 0.2;
    return mix(center, average, clamp(contrast * 2.5, 0.0, 0.65));
}

void main() {
    vec3 hdr = texture(u_hdr, v_uv).rgb;
    vec3 bloom = texture(u_bloom, v_uv).rgb;
    vec3 color = hdr + bloom * pc.p0.y;

    /* screen-space ambient occlusion from the scene depth */
    float d = texture(u_depth, v_uv).r;
    vec3 cam = g.cam_pos_time.xyz;
    float ao = screen_ao(v_uv, d, world_from_depth(v_uv, d), cam);
    color *= mix(1.0, ao, pc.p3.z);

    color *= pc.p0.x;

    if (pc.p1.w > 0.5) color = fxaa(color, v_uv);

    /* sun-side warm glow and cool shadow tint before the tone curve */
    float sun_vis = clamp(dot(normalize(g.cam_forward.xyz),
                              -normalize(g.sun_dir_intensity.xyz)), 0.0, 1.0);
    color *= mix(vec3(0.97, 0.99, 1.04), vec3(1.04, 1.0, 0.96), sun_vis * 0.5 + 0.25);

    color = aces_filmic(color);

    /* grade: saturation, contrast, then a gentle warm/cool split tone */
    float l = luma(color);
    color = mix(vec3(l), color, pc.p0.z);
    color = clamp((color - 0.5) * pc.p0.w + 0.5, 0.0, 1.0);
    color = mix(color, color * vec3(1.04, 1.0, 0.95), 0.35);

    if (pc.p2.x > 0.5) color = pow(color, vec3(1.0 / 2.2));

    /* vignette */
    vec2 q = v_uv - 0.5;
    float vig = 1.0 - dot(q, q) * pc.p1.x;
    color *= clamp(vig, 0.0, 1.0);

    /* subtle film grain */
    float n = hash12(v_uv * 1024.0 + pc.p1.z * 17.0);
    color += (n - 0.5) * pc.p1.y;

    out_color = vec4(clamp(color, 0.0, 1.0), 1.0);
}
