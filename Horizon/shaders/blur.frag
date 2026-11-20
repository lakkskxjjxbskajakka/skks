#version 450

layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 out_color;

layout(set = 0, binding = 0) uniform sampler2D u_source;
layout(push_constant) uniform PC {
    vec4 dir_texel; /* xy = direction * texel size */
} pc;

void main() {
    vec2 d = pc.dir_texel.xy;
    vec3 c = texture(u_source, v_uv).rgb * 0.2270270270;
    c += texture(u_source, v_uv + d * 1.3846153846).rgb * 0.3162162162;
    c += texture(u_source, v_uv - d * 1.3846153846).rgb * 0.3162162162;
    c += texture(u_source, v_uv + d * 3.2307692308).rgb * 0.0702702703;
    c += texture(u_source, v_uv - d * 3.2307692308).rgb * 0.0702702703;
    out_color = vec4(c, 1.0);
}
