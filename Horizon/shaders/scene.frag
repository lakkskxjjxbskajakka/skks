#version 450

layout(location = 0) in vec3 v_world;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec4 v_tangent;
layout(location = 3) in vec2 v_uv;
layout(location = 4) in vec4 v_tint;
layout(location = 5) in vec4 v_params;

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

layout(set = 0, binding = 1) uniform sampler2DArray u_shadow;

layout(set = 1, binding = 0) uniform Material {
    vec4 base_color;
    vec4 params;     /* metallic, roughness, emissive, flags */
    vec4 uv_scale;   /* uvx, uvy, normal strength, ao strength */
} mat;

layout(set = 1, binding = 1) uniform sampler2D u_albedo;
layout(set = 1, binding = 2) uniform sampler2D u_normal;
layout(set = 1, binding = 3) uniform sampler2D u_orm;
layout(set = 1, binding = 4) uniform sampler2D u_detail;

layout(location = 0) out vec4 out_color;

const float PI = 3.14159265359;

/* ------------------------------------------------------------------ noise */
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
    for (int i = 0; i < 5; ++i) {
        s += vnoise(p) * a;
        norm += a;
        p = rot * p * 2.03 + vec2(7.3, 11.7);
        a *= 0.5;
    }
    return s / norm;
}

/* ------------------------------------------------------------------- BRDF */
float distribution_ggx(vec3 n, vec3 h, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float ndh = max(dot(n, h), 0.0);
    float d = ndh * ndh * (a2 - 1.0) + 1.0;
    return a2 / max(PI * d * d, 1e-6);
}

float geometry_schlick_ggx(float ndv, float roughness) {
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    return ndv / (ndv * (1.0 - k) + k);
}

float geometry_smith(vec3 n, vec3 v, vec3 l, float roughness) {
    float ndv = max(dot(n, v), 0.0);
    float ndl = max(dot(n, l), 0.0);
    return geometry_schlick_ggx(ndv, roughness) * geometry_schlick_ggx(ndl, roughness);
}

vec3 fresnel_schlick(float cos_theta, vec3 f0) {
    return f0 + (1.0 - f0) * pow(clamp(1.0 - cos_theta, 0.0, 1.0), 5.0);
}

float hash21(vec2 p) {
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return fract(p.x * p.y);
}

/* ----------------------------------------------------------------- shadow */
float shadow_factor(vec3 world, vec3 n, vec3 l) {
    if (g.shadow_params.z < 0.5) return 1.0;

    float view_depth = dot(world - g.cam_pos_time.xyz, g.cam_forward.xyz);
    int cascade = 3;
    for (int i = 0; i < 4; ++i) {
        if (view_depth <= g.cascade_splits[i]) { cascade = i; break; }
    }

    float ndl = max(dot(n, l), 0.0);
    float texel = g.shadow_params.x;
    float normal_offset = texel * (1.4 + (1.0 - ndl) * 3.0);
    vec4 lp = g.light_view_proj[cascade] * vec4(world + n * normal_offset, 1.0);
    vec3 proj = lp.xyz / lp.w;
    vec2 uv = proj.xy * 0.5 + 0.5;
    if (uv.x <= 0.0 || uv.x >= 1.0 || uv.y <= 0.0 || uv.y >= 1.0 || proj.z >= 1.0)
        return 1.0;

    float bias = max(0.0022 * (1.0 - ndl), 0.00045);
    float sum = 0.0;
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            float d = texture(u_shadow, vec3(uv + vec2(x, y) * texel, float(cascade))).r;
            sum += (proj.z - bias) <= d ? 1.0 : 0.0;
        }
    }
    float sh = sum / 9.0;

    float far_split = g.cascade_splits[3];
    float fade = 1.0 - smoothstep(far_split * 0.90, far_split, view_depth);
    return mix(1.0, sh, fade);
}

/* -------------------------------------------------------------- atmosphere */
vec3 sky_ambient(vec3 dir) {
    float t = clamp(dir.y * 0.5 + 0.5, 0.0, 1.0);
    vec3 c = mix(g.ambient_ground.rgb, g.ambient_sky.rgb, t);
    float sun_glow = pow(max(dot(dir, -normalize(g.sun_dir_intensity.xyz)), 0.0), 6.0);
    return c + g.sun_color_exposure.rgb * sun_glow * 0.10 * g.sun_dir_intensity.w;
}

/* ------------------------------------------------------------------- water */
vec3 water_normal(vec2 p, float t) {
    vec2 g = vec2(0.0);
    g += vec2(cos(p.x * 0.31 + t * 1.15), cos(p.y * 0.27 - t * 0.92)) * 0.5;
    g += vec2(cos(p.x * 0.73 - p.y * 0.21 + t * 1.7),
              cos(p.y * 0.81 + p.x * 0.17 - t * 1.4)) * 0.28;
    g += vec2(cos(p.x * 2.1 + t * 3.1), cos(p.y * 2.4 - t * 2.7)) * 0.10;
    return normalize(vec3(-g.x * 0.11, 1.0, -g.y * 0.11));
}

void main() {
    int flags = int(mat.params.w + 0.5);

    vec3 n_world = normalize(v_normal);
    vec3 t_world = normalize(v_tangent.xyz - n_world * dot(n_world, v_tangent.xyz));
    vec3 b_world = cross(n_world, t_world) * v_tangent.w;

    vec2 uv = v_uv * mat.uv_scale.xy;
    vec3 albedo = texture(u_albedo, uv).rgb;
    vec3 orm = texture(u_orm, uv).rgb;
    float ao = mix(orm.r, 1.0, 1.0 - mat.uv_scale.w);
    float roughness = clamp(orm.g * mat.params.y, 0.035, 1.0);
    float metallic = orm.b * mat.params.x;
    float normal_strength = mat.uv_scale.z;

    if ((flags & 1) != 0) {
        /* ---- terrain: macro palette, two-scale detail, rock and snow ---- */
        vec2 wp = v_world.xz;
        vec3 tex_a = texture(u_albedo, wp * 0.047).rgb;
        vec3 tex_b = texture(u_albedo, wp * 0.011 + 0.37).rgb;
        float detail = dot(mix(tex_a, tex_b, 0.5), vec3(0.33));

        float m1 = fbm(wp * 0.0042);
        float m2 = fbm(wp * 0.019 + 9.1);
        float m3 = vnoise(wp * 0.42);

        vec3 lush = vec3(0.115, 0.235, 0.065);
        vec3 dry  = vec3(0.315, 0.285, 0.115);
        vec3 dark = vec3(0.052, 0.115, 0.040);
        vec3 dirt = vec3(0.205, 0.150, 0.092);
        vec3 grass = mix(lush, dry, smoothstep(0.34, 0.78, m1));
        grass = mix(grass, dark, smoothstep(0.42, 0.85, m2) * 0.55);
        grass = mix(grass, dirt, smoothstep(0.62, 0.86, m1 * 0.6 + m2 * 0.6) * 0.35);
        albedo = grass * (0.80 + detail * 1.25 + m3 * 0.12);

        float slope = 1.0 - clamp(n_world.y, 0.0, 1.0);
        float rock_mask = smoothstep(0.30, 0.62, slope + (m2 - 0.5) * 0.22);
        vec3 rock = texture(u_detail, wp * 0.085).rgb * vec3(0.62, 0.61, 0.58);
        albedo = mix(albedo, rock, rock_mask);
        roughness = mix(0.94, 0.82, rock_mask);
        ao = mix(0.70 + 0.30 * m2, 0.88, rock_mask);

        float snow_line = 40.0 + (m1 - 0.5) * 9.0;
        float snow = smoothstep(snow_line, snow_line + 14.0, v_world.y);
        snow *= 1.0 - smoothstep(0.45, 0.80, slope);
        albedo = mix(albedo, vec3(0.78, 0.82, 0.90), snow);
        roughness = mix(roughness, 0.58, snow);

        vec3 tn1 = texture(u_normal, wp * 0.047).xyz * 2.0 - 1.0;
        vec3 tn2 = texture(u_normal, wp * 0.011 + 0.37).xyz * 2.0 - 1.0;
        vec3 tn = normalize(vec3(tn1.xy + tn2.xy, 2.0));
        normal_strength *= 0.55;
        n_world = normalize(t_world * tn.x * normal_strength +
                            b_world * tn.y * normal_strength + n_world);
    } else if ((flags & 2) != 0) {
        /* ---- asphalt with lane markings, wear and a damp sheen ---- */
        float x = v_uv.x;
        vec3 a1 = texture(u_albedo, uv).rgb;
        vec3 a2 = texture(u_albedo, uv * 0.37 + 0.21).rgb;
        float grain = dot(mix(a1, a2, 0.5), vec3(0.33));
        albedo = vec3(0.090, 0.092, 0.100) * (0.62 + grain * 1.7);

        float wear = smoothstep(0.10, 0.22, x) * (1.0 - smoothstep(0.30, 0.42, x));
        wear += smoothstep(0.58, 0.70, x) * (1.0 - smoothstep(0.78, 0.90, x));
        albedo *= 1.0 - wear * 0.20;
        roughness = mix(roughness, roughness * 0.78, wear);

        float dash = step(fract(v_uv.y * 0.060), 0.52);
        float edge = step(x, 0.020) + step(0.980, x);
        float center = (1.0 - smoothstep(0.006, 0.012, abs(x - 0.5))) * dash;
        float line = clamp(edge + center, 0.0, 1.0);
        vec3 paint = vec3(0.74, 0.72, 0.62);
        albedo = mix(albedo, paint, line);
        roughness = mix(roughness, 0.46, line);
        metallic = mix(metallic, 0.0, line);

        vec3 tn = texture(u_normal, uv * 0.6).xyz * 2.0 - 1.0;
        normal_strength *= 0.30;
        n_world = normalize(t_world * tn.x * normal_strength +
                            b_world * tn.y * normal_strength + n_world);
    } else if ((flags & 32) != 0) {
        /* ---- lake water: animated ripples, fresnel sky reflection ---- */
        float t = g.cam_pos_time.w;
        vec3 wn = water_normal(v_world.xz, t);
        n_world = normalize(wn);

        vec3 v = normalize(g.cam_pos_time.xyz - v_world);
        float ndv = max(dot(n_world, v), 0.0);
        float fres = 0.02 + 0.98 * pow(1.0 - ndv, 5.0);

        vec3 r = reflect(-v, n_world);
        vec3 refl = sky_ambient(r);
        float sun_ref = pow(max(dot(r, -normalize(g.sun_dir_intensity.xyz)), 0.0), 90.0);
        refl += g.sun_color_exposure.rgb * sun_ref * g.sun_dir_intensity.w * 2.5;

        vec3 deep = vec3(0.012, 0.045, 0.060);
        vec3 shallow = vec3(0.030, 0.095, 0.105);
        float shore = 1.0 - smoothstep(0.0, 1.0, fres);
        vec3 body = mix(deep, shallow, shore * 0.6);
        vec3 color = mix(body, refl, fres);
        color += g.sun_color_exposure.rgb * pow(max(dot(n_world,
                     normalize(-g.sun_dir_intensity.xyz + v)), 0.0), 300.0) * 2.0;

        float dist = length(g.cam_pos_time.xyz - v_world);
        float fog_amount = 1.0 - exp(-g.fog_color_density.w * dist * 0.9);
        color = mix(color, g.fog_color_density.rgb, clamp(fog_amount, 0.0, 0.9));
        out_color = vec4(max(color, vec3(0.0)), 1.0);
        return;
    }

    vec3 tint = v_tint.rgb;
    albedo *= tint;

    if ((flags & 1) == 0 && (flags & 2) == 0) {
        vec3 ntex = texture(u_normal, uv).xyz * 2.0 - 1.0;
        ntex.xy *= normal_strength;
        n_world = normalize(t_world * ntex.x + b_world * ntex.y + n_world);
    }

    vec3 v = normalize(g.cam_pos_time.xyz - v_world);
    vec3 l = normalize(-g.sun_dir_intensity.xyz);
    vec3 h = normalize(v + l);
    vec3 f0 = mix(vec3(0.04), albedo, metallic);

    float ndl = max(dot(n_world, l), 0.0);
    float ndv = max(dot(n_world, v), 0.0);
    float vdh = max(dot(v, h), 0.0);

    vec3 radiance = g.sun_color_exposure.rgb * g.sun_dir_intensity.w;
    vec3 F = fresnel_schlick(vdh, f0);
    float D = distribution_ggx(n_world, h, roughness);
    float G = geometry_smith(n_world, v, l, roughness);
    vec3 spec = (D * G * F) / max(4.0 * ndv * ndl, 1e-5);
    vec3 kd = (vec3(1.0) - F) * (1.0 - metallic);
    vec3 direct = (kd * albedo / PI + spec) * radiance * ndl;

    float sh = shadow_factor(v_world, n_world, l);
    direct *= mix(0.12, 1.0, sh);
    ao *= mix(0.70, 1.0, sh);

    vec3 r = reflect(-v, n_world);
    vec3 ambient_diffuse = sky_ambient(n_world) * albedo * ao * 1.15;
    vec3 ambient_spec = sky_ambient(r) * mix(f0, vec3(1.0), 0.0) * ao;
    float spec_occlusion = mix(0.30, 1.0, pow(1.0 - roughness, 3.0));
    vec3 ambient = ambient_diffuse * (1.0 - metallic) + ambient_spec * spec_occlusion;
    ambient += g.ambient_ground.rgb * albedo * 0.10 * ao;

    if ((flags & 4) != 0) {
        /* clear coat + metallic flake lobe for car paint */
        float cc_rough = 0.045;
        vec3 cc_f0 = vec3(0.04);
        vec3 ccF = fresnel_schlick(vdh, cc_f0);
        float ccD = distribution_ggx(n_world, h, cc_rough);
        float ccG = geometry_smith(n_world, v, l, cc_rough);
        vec3 cc_spec = (ccD * ccG * ccF) / max(4.0 * ndv * ndl, 1e-5);
        direct += cc_spec * radiance * ndl * sh * 1.4;

        float flake = vnoise(v_world.xz * 140.0 + v_world.y * 90.0);
        direct += spec * radiance * ndl * flake * 0.6;

        float fres = pow(1.0 - ndv, 5.0);
        vec3 env = sky_ambient(r);
        float sun_env = pow(max(dot(r, -normalize(g.sun_dir_intensity.xyz)), 0.0), 48.0);
        env += g.sun_color_exposure.rgb * sun_env * g.sun_dir_intensity.w * 1.2;
        ambient += env * (0.05 + fres * 0.55) * ao;
    }

    if ((flags & 64) != 0) {
        /* automotive glass: dark tint with a strong grazing reflection */
        float fres = pow(1.0 - ndv, 4.0);
        vec3 env = sky_ambient(r);
        env += g.sun_color_exposure.rgb * pow(max(dot(r, -normalize(g.sun_dir_intensity.xyz)), 0.0), 32.0)
               * g.sun_dir_intensity.w * 1.5;
        vec3 glass = vec3(0.020, 0.028, 0.040) + env * (0.10 + fres * 0.85);
        glass += g.sun_color_exposure.rgb * pow(max(dot(r, l), 0.0), 200.0) * 2.0;
        direct = glass * (0.25 + 0.75 * ndl) * mix(0.35, 1.0, sh);
        ambient *= 0.15;
    }

    vec3 color = direct + ambient;

    if ((flags & 8) != 0) {
        float strength = mat.params.z;
        color += albedo * strength * 6.0;
    }

    if ((flags & 16) != 0) {
        float leaf = fbm(v_world.xz * 0.32 + v_world.y * 0.18);
        albedo *= 0.68 + leaf * 0.55;
        float trans = pow(max(dot(v, -l), 0.0), 2.0) * 0.18;
        color += albedo * radiance * trans * ndl * (0.5 + leaf);
        color *= 0.82 + leaf * 0.36;
    }

    /* height + distance fog, tinted toward the sun */
    float dist = length(g.cam_pos_time.xyz - v_world);
    float height_falloff = exp(-max(v_world.y, 0.0) * 0.020);
    float fog_amount = 1.0 - exp(-g.fog_color_density.w * dist * (0.45 + 0.55 * height_falloff));
    vec3 view_dir = normalize(v_world - g.cam_pos_time.xyz);
    vec3 sun_vis = g.sun_color_exposure.rgb *
                   pow(max(dot(view_dir, -normalize(g.sun_dir_intensity.xyz)), 0.0), 5.0) * 0.35;
    vec3 fog_col = g.fog_color_density.rgb + sun_vis;
    color = mix(color, fog_col, clamp(fog_amount, 0.0, 0.96));

    float dither = (hash21(gl_FragCoord.xy + g.cam_pos_time.w) - 0.5) / 255.0;
    color += dither;

    out_color = vec4(max(color, vec3(0.0)), 1.0);
}
