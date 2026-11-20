/* Horizon Engine - vector / matrix / quaternion math (column-major, RH, Vulkan depth). */
#ifndef HORIZON_MATH_H
#define HORIZON_MATH_H

#include "horizon/core.h"
#include <math.h>

#define HR_PI 3.14159265358979323846f
#define HR_TAU (2.0f * HR_PI)
#define HR_DEG2RAD (HR_PI / 180.0f)
#define HR_RAD2DEG (180.0f / HR_PI)

typedef union Vec2 { struct { float x, y; }; float v[2]; } Vec2;
typedef union Vec3 { struct { float x, y, z; }; float v[3]; } Vec3;
typedef union Vec4 { struct { float x, y, z, w; }; float v[4]; } Vec4;

typedef struct Mat3 { float m[9]; } Mat3;
typedef union Mat4 { float m[16]; float c[4][4]; } Mat4;
typedef union Quat { struct { float x, y, z, w; }; float v[4]; } Quat;

HR_INLINE Vec2 v2(float x, float y) { Vec2 r = {{x, y}}; return r; }
HR_INLINE Vec3 v3(float x, float y, float z) { Vec3 r = {{x, y, z}}; return r; }
HR_INLINE Vec4 v4(float x, float y, float z, float w) { Vec4 r = {{x, y, z, w}}; return r; }
HR_INLINE Vec3 v3s(float s) { Vec3 r = {{s, s, s}}; return r; }
HR_INLINE Vec4 v4s(float s) { Vec4 r = {{s, s, s, s}}; return r; }
HR_INLINE Vec3 v3_from4(Vec4 a) { Vec3 r = {{a.x, a.y, a.z}}; return r; }
HR_INLINE Vec4 v4_from3(Vec3 a, float w) { Vec4 r = {{a.x, a.y, a.z, w}}; return r; }

HR_INLINE Vec2 v2_add(Vec2 a, Vec2 b) { return v2(a.x + b.x, a.y + b.y); }
HR_INLINE Vec2 v2_sub(Vec2 a, Vec2 b) { return v2(a.x - b.x, a.y - b.y); }
HR_INLINE Vec2 v2_scale(Vec2 a, float s) { return v2(a.x * s, a.y * s); }
HR_INLINE float v2_dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
HR_INLINE float v2_len(Vec2 a) { return sqrtf(v2_dot(a, a)); }

HR_INLINE Vec3 v3_add(Vec3 a, Vec3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
HR_INLINE Vec3 v3_sub(Vec3 a, Vec3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
HR_INLINE Vec3 v3_mul(Vec3 a, Vec3 b) { return v3(a.x * b.x, a.y * b.y, a.z * b.z); }
HR_INLINE Vec3 v3_scale(Vec3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
HR_INLINE Vec3 v3_neg(Vec3 a) { return v3(-a.x, -a.y, -a.z); }
HR_INLINE float v3_dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
HR_INLINE Vec3 v3_cross(Vec3 a, Vec3 b) {
    return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
HR_INLINE float v3_len_sq(Vec3 a) { return v3_dot(a, a); }
HR_INLINE float v3_len(Vec3 a) { return sqrtf(v3_dot(a, a)); }
HR_INLINE float v3_dist(Vec3 a, Vec3 b) { return v3_len(v3_sub(a, b)); }
HR_INLINE Vec3 v3_norm(Vec3 a) {
    float l = v3_len(a);
    return l > 1e-20f ? v3_scale(a, 1.0f / l) : v3(0, 0, 0);
}
HR_INLINE Vec3 v3_lerp(Vec3 a, Vec3 b, float t) { return v3_add(a, v3_scale(v3_sub(b, a), t)); }
HR_INLINE Vec3 v3_min(Vec3 a, Vec3 b) { return v3(HR_MIN(a.x, b.x), HR_MIN(a.y, b.y), HR_MIN(a.z, b.z)); }
HR_INLINE Vec3 v3_max(Vec3 a, Vec3 b) { return v3(HR_MAX(a.x, b.x), HR_MAX(a.y, b.y), HR_MAX(a.z, b.z)); }
HR_INLINE Vec3 v3_reflect(Vec3 i, Vec3 n) { return v3_sub(i, v3_scale(n, 2.0f * v3_dot(i, n))); }
HR_INLINE Vec3 v3_splat(Vec3 a, int axis) { return v3(a.v[axis], a.v[axis], a.v[axis]); }
HR_INLINE float v3_comp(Vec3 a, int axis) { return a.v[axis]; }
HR_INLINE Vec3 v3_abs(Vec3 a) { return v3(fabsf(a.x), fabsf(a.y), fabsf(a.z)); }

HR_INLINE Vec4 v4_add(Vec4 a, Vec4 b) { return v4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w); }
HR_INLINE Vec4 v4_sub(Vec4 a, Vec4 b) { return v4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w); }
HR_INLINE Vec4 v4_scale(Vec4 a, float s) { return v4(a.x * s, a.y * s, a.z * s, a.w * s); }
HR_INLINE float v4_dot(Vec4 a, Vec4 b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
HR_INLINE Vec4 v4_lerp(Vec4 a, Vec4 b, float t) { return v4_add(a, v4_scale(v4_sub(b, a), t)); }

/* ---------------------------------------------------------------- mat4 */
HR_INLINE Mat4 mat4_identity(void) {
    Mat4 r; memset(&r, 0, sizeof(r));
    r.c[0][0] = r.c[1][1] = r.c[2][2] = r.c[3][3] = 1.0f;
    return r;
}

HR_INLINE Mat4 mat4_cols(Vec4 a, Vec4 b, Vec4 c, Vec4 d) {
    Mat4 r;
    r.c[0] = a; r.c[1] = b; r.c[2] = c; r.c[3] = d;
    return r;
}

HR_INLINE Mat4 mat4_mul(Mat4 a, Mat4 b) {
    Mat4 r;
    for (int c = 0; c < 4; ++c) {
        for (int row = 0; row < 4; ++row) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k) s += a.c[k][row] * b.c[c][k];
            r.c[c][row] = s;
        }
    }
    return r;
}

HR_INLINE Vec4 mat4_mul_vec4(Mat4 a, Vec4 v) {
    Vec4 r;
    for (int row = 0; row < 4; ++row)
        r.v[row] = a.c[0][row] * v.x + a.c[1][row] * v.y + a.c[2][row] * v.z + a.c[3][row] * v.w;
    return r;
}

HR_INLINE Vec3 mat4_mul_point(Mat4 a, Vec3 p) {
    Vec4 r = mat4_mul_vec4(a, v4_from3(p, 1.0f));
    float inv = fabsf(r.w) > 1e-20f ? 1.0f / r.w : 1.0f;
    return v3(r.x * inv, r.y * inv, r.z * inv);
}

HR_INLINE Vec3 mat4_mul_dir(Mat4 a, Vec3 d) {
    Vec4 r = mat4_mul_vec4(a, v4_from3(d, 0.0f));
    return v3_from4(r);
}

HR_INLINE Mat4 mat4_translate(Vec3 t) {
    Mat4 r = mat4_identity();
    r.c[3][0] = t.x; r.c[3][1] = t.y; r.c[3][2] = t.z;
    return r;
}

HR_INLINE Mat4 mat4_scale(Vec3 s) {
    Mat4 r = mat4_identity();
    r.c[0][0] = s.x; r.c[1][1] = s.y; r.c[2][2] = s.z;
    return r;
}

HR_INLINE Mat4 mat4_scale_uniform(float s) { return mat4_scale(v3s(s)); }

Mat4 mat4_rotate_axis(Vec3 axis, float radians);
Mat4 mat4_rotate_x(float r);
Mat4 mat4_rotate_y(float r);
Mat4 mat4_rotate_z(float r);
Mat4 mat4_transpose(Mat4 a);
Mat4 mat4_invert(Mat4 a);
Mat4 mat4_perspective_vk(float fovy_rad, float aspect, float znear, float zfar);
Mat4 mat4_ortho_vk(float left, float right, float bottom, float top, float znear, float zfar);
Mat4 mat4_look_at(Vec3 eye, Vec3 center, Vec3 up);
Mat4 mat4_from_quat(Quat q);
Mat4 mat4_compose(Vec3 translation, Quat rotation, Vec3 scale);
Mat4 mat4_trs(Vec3 translation, Vec3 euler_xyz, Vec3 scale);

/* ---------------------------------------------------------------- quat */
HR_INLINE Quat quat_identity(void) { Quat q = {{0, 0, 0, 1}}; return q; }
Quat quat_axis_angle(Vec3 axis, float radians);
Quat quat_mul(Quat a, Quat b);
Quat quat_norm(Quat q);
Vec3 quat_rotate(Quat q, Vec3 v);
Quat quat_slerp(Quat a, Quat b, float t);
Quat quat_from_euler(Vec3 e);
Vec3 quat_to_euler(Quat q);

/* --------------------------------------------------------------- misc */
HR_INLINE float hr_smoothstep(float e0, float e1, float x) {
    float t = HR_CLAMP((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}
HR_INLINE float hr_lerp(float a, float b, float t) { return a + (b - a) * t; }
HR_INLINE float hr_clamp01(float x) { return HR_CLAMP(x, 0.0f, 1.0f); }
HR_INLINE float hr_sign(float x) { return x < 0.0f ? -1.0f : 1.0f; }
HR_INLINE float hr_radians(float d) { return d * HR_DEG2RAD; }
HR_INLINE float hr_damp(float current, float target, float lambda, float dt) {
    return hr_lerp(current, target, 1.0f - expf(-lambda * dt));
}
HR_INLINE Vec3 v3_damp(Vec3 current, Vec3 target, float lambda, float dt) {
    return v3_lerp(current, target, 1.0f - expf(-lambda * dt));
}
HR_INLINE float hr_angle_damp(float current, float target, float lambda, float dt) {
    float d = fmodf(target - current + HR_PI, HR_TAU);
    if (d < 0.0f) d += HR_TAU;
    d -= HR_PI;
    return current + d * (1.0f - expf(-lambda * dt));
}

/* Deterministic hash / noise helpers (CPU side). */
u32 hr_hash_u32(u32 x);
u32 hr_hash_combine(u32 a, u32 b);
float hr_hash_float(u32 x);
float hr_value_noise2(float x, float y, u32 seed);
float hr_value_noise3(float x, float y, float z, u32 seed);
float hr_fbm2(float x, float y, int octaves, u32 seed);
float hr_fbm3(float x, float y, float z, int octaves, u32 seed);

/* extract 6 frustum planes (a,b,c,d) from a view-projection matrix */
void hr_frustum_planes(Mat4 vp, Vec4 planes[6]);
bool hr_sphere_in_frustum(const Vec4 planes[6], Vec3 center, float radius);

#endif /* HORIZON_MATH_H */
