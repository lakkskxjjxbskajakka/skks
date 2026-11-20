#version 450

layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 out_color;

layout(set = 0, binding = 0) uniform sampler2D u_hdr;
layout(push_constant) uniform PC {
    vec4 p; /* threshold, knee, intensity, unused */
} pc;

void main() {
    vec3 c = texture(u_hdr, v_uv).rgb;
    float lum = max(max(c.r, c.g), c.b);
    float threshold = pc.p.x;
    float knee = max(pc.p.y, 1e-4);
    float soft = clamp(lum - threshold + knee, 0.0, 2.0 * knee);
    soft = soft * soft / (4.0 * knee + 1e-5);
    float contribution = max(soft, lum - threshold) / max(lum, 1e-5);
    out_color = vec4(c * contribution * pc.p.z, 1.0);
}
