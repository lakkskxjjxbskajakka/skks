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

float shadow_factor(vec3 world, vec3 n, vec3 l) {
    if (g.shadow_params.z < 0.5) return 1.0;

    float view_depth = dot(world - g.cam_pos_time.xyz, g.cam_forward.xyz);
    int cascade = 3;
    for (int i = 0; i < 4; ++i) {
        if (view_depth <= g.cascade_splits[i]) { cascade = i; break; }
    }

    /* Slope-scaled normal offset keeps contact shadows while avoiding acne. */
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

    /* Fade the last cascade out instead of hard-clipping to full sun. */
    float far_split = g.cascade_splits[3];
    float fade = 1.0 - smoothstep(far_split * 0.90, far_split, view_depth);
    return mix(1.0, sh, fade);
}

float hash21(vec2 p) {
    p = fract(p * vec2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return fract(p.x * p.y);
}

vec3 sky_ambient(vec3 dir) {
    float t = clamp(dir.y * 0.5 + 0.5, 0.0, 1.0);
    vec3 c = mix(g.ambient_ground.rgb, g.ambient_sky.rgb, t);
    float sun_glow = pow(max(dot(dir, -normalize(g.sun_dir_intensity.xyz)), 0.0), 6.0);
    return c + g.sun_color_exposure.rgb * sun_glow * 0.10 * g.sun_dir_intensity.w;
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

    if ((flags & 1) != 0) {
        /* terrain: blend rock into grass by slope + macro variation */
        float slope = 1.0 - clamp(n_world.y, 0.0, 1.0);
        float macro = texture(u_detail, v_world.xz * 0.011).r;
        float rock_mask = smoothstep(0.18, 0.55, slope + (macro - 0.5) * 0.25);
        vec3 rock_albedo = texture(u_detail, v_world.xz * 0.09).rgb * 1.1;
        albedo = mix(albedo, rock_albedo, rock_mask);
        roughness = mix(roughness, 0.82, rock_mask);
        ao = mix(ao, 0.75, rock_mask);
        albedo *= 0.85 + macro * 0.4;
    } else if ((flags & 2) != 0) {
        /* asphalt with painted lane markings and tire wear */
        float x = v_uv.x;
        float wear = smoothstep(0.10, 0.22, x) * (1.0 - smoothstep(0.30, 0.42, x));
        wear += smoothstep(0.58, 0.70, x) * (1.0 - smoothstep(0.78, 0.90, x));
        albedo *= 1.0 - wear * 0.18;
        roughness = mix(roughness, roughness * 0.72, wear);
        float edge = step(x, 0.030) + step(0.970, x);
        float dash = step(fract(v_uv.y * 0.045), 0.55);
        float center = (1.0 - smoothstep(0.006, 0.014, abs(x - 0.5))) * dash;
        float line = clamp(edge + center, 0.0, 1.0);
        vec3 paint = vec3(0.78, 0.76, 0.66);
        albedo = mix(albedo, paint, line);
        roughness = mix(roughness, 0.45, line);
        metallic = mix(metallic, 0.0, line);
    }

    vec3 tint = v_tint.rgb;
    albedo *= tint;

    float nstrength = mat.uv_scale.z;
    vec3 ntex = texture(u_normal, uv).xyz * 2.0 - 1.0;
    ntex.xy *= nstrength;
    vec3 n = normalize(t_world * ntex.x + b_world * ntex.y + n_world * ntex.z);

    vec3 v = normalize(g.cam_pos_time.xyz - v_world);
    vec3 l = normalize(-g.sun_dir_intensity.xyz);
    vec3 h = normalize(v + l);
    vec3 f0 = mix(vec3(0.04), albedo, metallic);

    float ndl = max(dot(n, l), 0.0);
    float ndv = max(dot(n, v), 0.0);
    float ndh = max(dot(n, h), 0.0);
    float vdh = max(dot(v, h), 0.0);

    vec3 radiance = g.sun_color_exposure.rgb * g.sun_dir_intensity.w;
    vec3 F = fresnel_schlick(vdh, f0);
    float D = distribution_ggx(n, h, roughness);
    float G = geometry_smith(n, v, l, roughness);
    vec3 spec = (D * G * F) / max(4.0 * ndv * ndl, 1e-5);
    vec3 kd = (vec3(1.0) - F) * (1.0 - metallic);
    vec3 direct = (kd * albedo / PI + spec) * radiance * ndl;

    float sh = shadow_factor(v_world, n, l);
    direct *= mix(0.18, 1.0, sh);
    ao *= mix(0.72, 1.0, sh);

    vec3 r = reflect(-v, n);
    vec3 ambient_diffuse = sky_ambient(n) * albedo * ao * 1.15;
    vec3 ambient_spec = sky_ambient(r) * mix(f0, vec3(1.0), 0.0) * ao;
    float spec_occlusion = mix(0.35, 1.0, pow(1.0 - roughness, 3.0));
    vec3 ambient = ambient_diffuse * (1.0 - metallic) + ambient_spec * spec_occlusion;

    if ((flags & 4) != 0) {
        /* clear coat lobe for car paint */
        float cc_rough = 0.045;
        vec3 cc_f0 = vec3(0.04);
        vec3 ccF = fresnel_schlick(vdh, cc_f0);
        float ccD = distribution_ggx(n, h, cc_rough);
        float ccG = geometry_smith(n, v, l, cc_rough);
        vec3 cc_spec = (ccD * ccG * ccF) / max(4.0 * ndv * ndl, 1e-5);
        direct += cc_spec * radiance * ndl * sh * 1.4;
        float fres = pow(1.0 - ndv, 5.0);
        ambient += sky_ambient(r) * (0.04 + fres * 0.35) * ao;
    }

    vec3 color = direct + ambient;

    if ((flags & 8) != 0) {
        float strength = mat.params.z;
        color += albedo * strength * 6.0;
    }

    if ((flags & 16) != 0) {
        float trans = pow(max(dot(v, -l), 0.0), 2.0) * 0.10;
        color += albedo * radiance * trans * ndl * 0.6;
    }

    /* height + distance fog, tinted toward the sun */
    float dist = length(g.cam_pos_time.xyz - v_world);
    float height_falloff = exp(-max(v_world.y, 0.0) * 0.012);
    float fog_amount = 1.0 - exp(-g.fog_color_density.w * dist * (0.45 + 0.55 * height_falloff));
    vec3 sun_vis = g.sun_color_exposure.rgb * pow(max(dot(-v, -normalize(g.sun_dir_intensity.xyz)), 0.0), 4.0) * 0.25;
    vec3 fog_col = g.fog_color_density.rgb + sun_vis;
    color = mix(color, fog_col, clamp(fog_amount, 0.0, 0.96));

    /* dithering to hide 16F banding in the sky gradient */
    float dither = (hash21(gl_FragCoord.xy + g.cam_pos_time.w) - 0.5) / 255.0;
    color += dither;

    out_color = vec4(max(color, vec3(0.0)), 1.0);
}
