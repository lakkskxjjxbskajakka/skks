#version 450

layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 out_color;

layout(set = 0, binding = 0) uniform sampler2D u_hdr;
layout(set = 0, binding = 1) uniform sampler2D u_bloom;

layout(push_constant) uniform PC {
    vec4 p0; /* exposure, bloom intensity, saturation, contrast */
    vec4 p1; /* vignette, grain, time, fxaa */
    vec4 p2; /* bloom threshold (unused), unused */
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
    color *= pc.p0.x;

    if (pc.p1.w > 0.5) {
        /* edge-aware AA after exposure, on HDR values */
        color = fxaa(color, v_uv);
    }

    color = aces_filmic(color);

    /* grade: saturation + contrast pivot */
    float l = luma(color);
    color = mix(vec3(l), color, pc.p0.z);
    color = clamp((color - 0.5) * pc.p0.w + 0.5, 0.0, 1.0);
    if (pc.p2.x > 0.5) color = pow(color, vec3(1.0 / 2.2));

    /* vignette */
    vec2 q = v_uv - 0.5;
    float vig = 1.0 - dot(q, q) * pc.p1.x;
    color *= clamp(vig, 0.0, 1.0);

    /* subtle film grain */
    float n = fract(sin(dot(v_uv * 1024.0 + pc.p1.z, vec2(12.9898, 78.233))) * 43758.5453);
    color += (n - 0.5) * pc.p1.y;

    out_color = vec4(clamp(color, 0.0, 1.0), 1.0);
}
