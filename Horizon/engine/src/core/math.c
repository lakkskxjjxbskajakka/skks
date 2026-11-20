#include "horizon/math.h"

Mat4 mat4_rotate_axis(Vec3 axis, float radians) {
    Vec3 a = v3_norm(axis);
    float c = cosf(radians), s = sinf(radians), t = 1.0f - c;
    Mat4 r = mat4_identity();
    r.c[0][0] = c + a.x * a.x * t;
    r.c[0][1] = a.y * a.x * t + a.z * s;
    r.c[0][2] = a.z * a.x * t - a.y * s;
    r.c[1][0] = a.x * a.y * t - a.z * s;
    r.c[1][1] = c + a.y * a.y * t;
    r.c[1][2] = a.z * a.y * t + a.x * s;
    r.c[2][0] = a.x * a.z * t + a.y * s;
    r.c[2][1] = a.y * a.z * t - a.x * s;
    r.c[2][2] = c + a.z * a.z * t;
    return r;
}

Mat4 mat4_rotate_x(float r) { return mat4_rotate_axis(v3(1, 0, 0), r); }
Mat4 mat4_rotate_y(float r) { return mat4_rotate_axis(v3(0, 1, 0), r); }
Mat4 mat4_rotate_z(float r) { return mat4_rotate_axis(v3(0, 0, 1), r); }

Mat4 mat4_transpose(Mat4 a) {
    Mat4 r;
    for (int c = 0; c < 4; ++c)
        for (int row = 0; row < 4; ++row)
            r.c[c][row] = a.c[row][c];
    return r;
}

Mat4 mat4_invert(Mat4 m) {
    const float* a = m.m;
    Mat4 out;
    float* o = out.m;
    float s0 = a[0] * a[5] - a[1] * a[4];
    float s1 = a[0] * a[6] - a[2] * a[4];
    float s2 = a[0] * a[7] - a[3] * a[4];
    float s3 = a[1] * a[6] - a[2] * a[5];
    float s4 = a[1] * a[7] - a[3] * a[5];
    float s5 = a[2] * a[7] - a[3] * a[6];
    float c5 = a[10] * a[15] - a[11] * a[14];
    float c4 = a[9] * a[15] - a[11] * a[13];
    float c3 = a[9] * a[14] - a[10] * a[13];
    float c2 = a[8] * a[15] - a[11] * a[12];
    float c1 = a[8] * a[14] - a[10] * a[12];
    float c0 = a[8] * a[13] - a[9] * a[12];

    float det = s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;
    if (fabsf(det) < 1e-20f) return mat4_identity();
    float inv = 1.0f / det;

    o[0] = (a[5] * c5 - a[6] * c4 + a[7] * c3) * inv;
    o[1] = (-a[1] * c5 + a[2] * c4 - a[3] * c3) * inv;
    o[2] = (a[13] * s5 - a[14] * s4 + a[15] * s3) * inv;
    o[3] = (-a[9] * s5 + a[10] * s4 - a[11] * s3) * inv;

    o[4] = (-a[4] * c5 + a[6] * c2 - a[7] * c1) * inv;
    o[5] = (a[0] * c5 - a[2] * c2 + a[3] * c1) * inv;
    o[6] = (-a[12] * s5 + a[14] * s2 - a[15] * s1) * inv;
    o[7] = (a[8] * s5 - a[10] * s2 + a[11] * s1) * inv;

    o[8] = (a[4] * c4 - a[5] * c2 + a[7] * c0) * inv;
    o[9] = (-a[0] * c4 + a[1] * c2 - a[3] * c0) * inv;
    o[10] = (a[12] * s4 - a[13] * s2 + a[15] * s0) * inv;
    o[11] = (-a[8] * s4 + a[9] * s2 - a[11] * s0) * inv;

    o[12] = (-a[4] * c3 + a[5] * c1 - a[6] * c0) * inv;
    o[13] = (a[0] * c3 - a[1] * c1 + a[2] * c0) * inv;
    o[14] = (-a[12] * s3 + a[13] * s1 - a[14] * s0) * inv;
    o[15] = (a[8] * s3 - a[9] * s1 + a[10] * s0) * inv;
    return out;
}

Mat4 mat4_perspective_vk(float fovy_rad, float aspect, float znear, float zfar) {
    float f = 1.0f / tanf(fovy_rad * 0.5f);
    Mat4 r; memset(&r, 0, sizeof(r));
    r.c[0][0] = f / aspect;
    r.c[1][1] = -f;
    r.c[2][2] = zfar / (znear - zfar);
    r.c[2][3] = -1.0f;
    r.c[3][2] = (zfar * znear) / (znear - zfar);
    return r;
}

Mat4 mat4_ortho_vk(float left, float right, float bottom, float top, float znear, float zfar) {
    Mat4 r = mat4_identity();
    r.c[0][0] = 2.0f / (right - left);
    r.c[1][1] = 2.0f / (top - bottom);
    r.c[2][2] = -1.0f / (zfar - znear);
    r.c[3][0] = -(right + left) / (right - left);
    r.c[3][1] = -(top + bottom) / (top - bottom);
    r.c[3][2] = -znear / (zfar - znear);
    return r;
}

Mat4 mat4_look_at(Vec3 eye, Vec3 center, Vec3 up) {
    Vec3 f = v3_norm(v3_sub(center, eye));
    Vec3 s = v3_norm(v3_cross(f, up));
    Vec3 u = v3_cross(s, f);
    Mat4 r = mat4_identity();
    r.c[0][0] = s.x; r.c[1][0] = s.y; r.c[2][0] = s.z;
    r.c[0][1] = u.x; r.c[1][1] = u.y; r.c[2][1] = u.z;
    r.c[0][2] = -f.x; r.c[1][2] = -f.y; r.c[2][2] = -f.z;
    r.c[3][0] = -v3_dot(s, eye);
    r.c[3][1] = -v3_dot(u, eye);
    r.c[3][2] = v3_dot(f, eye);
    return r;
}

Quat quat_axis_angle(Vec3 axis, float radians) {
    Vec3 a = v3_norm(axis);
    float h = radians * 0.5f;
    float s = sinf(h);
    Quat q = {{a.x * s, a.y * s, a.z * s, cosf(h)}};
    return q;
}

Quat quat_mul(Quat a, Quat b) {
    Quat r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return r;
}

Quat quat_norm(Quat q) {
    float l = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (l < 1e-20f) return quat_identity();
    float inv = 1.0f / l;
    Quat r = {{q.x * inv, q.y * inv, q.z * inv, q.w * inv}};
    return r;
}

Vec3 quat_rotate(Quat q, Vec3 v) {
    Vec3 u = v3(q.x, q.y, q.z);
    float s = q.w;
    Vec3 t = v3_scale(v3_cross(u, v), 2.0f);
    return v3_add(v3_add(v, v3_scale(t, s)), v3_cross(u, t));
}

Quat quat_slerp(Quat a, Quat b, float t) {
    float d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (d < 0.0f) { b.x = -b.x; b.y = -b.y; b.z = -b.z; b.w = -b.w; d = -d; }
    if (d > 0.9995f) {
        Quat r = {{hr_lerp(a.x, b.x, t), hr_lerp(a.y, b.y, t),
                   hr_lerp(a.z, b.z, t), hr_lerp(a.w, b.w, t)}};
        return quat_norm(r);
    }
    float theta = acosf(HR_CLAMP(d, -1.0f, 1.0f));
    float st = sinf(theta);
    float wa = sinf((1.0f - t) * theta) / st;
    float wb = sinf(t * theta) / st;
    Quat r = {{a.x * wa + b.x * wb, a.y * wa + b.y * wb,
               a.z * wa + b.z * wb, a.w * wa + b.w * wb}};
    return r;
}

Quat quat_from_euler(Vec3 e) {
    float cx = cosf(e.x * 0.5f), sx = sinf(e.x * 0.5f);
    float cy = cosf(e.y * 0.5f), sy = sinf(e.y * 0.5f);
    float cz = cosf(e.z * 0.5f), sz = sinf(e.z * 0.5f);
    Quat q;
    q.w = cx * cy * cz + sx * sy * sz;
    q.x = sx * cy * cz - cx * sy * sz;
    q.y = cx * sy * cz + sx * cy * sz;
    q.z = cx * cy * sz - sx * sy * cz;
    return q;
}

Vec3 quat_to_euler(Quat q) {
    Vec3 e;
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    e.x = atan2f(sinr_cosp, cosr_cosp);
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    e.y = fabsf(sinp) >= 1.0f ? copysignf(HR_PI * 0.5f, sinp) : asinf(sinp);
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    e.z = atan2f(siny_cosp, cosy_cosp);
    return e;
}

Mat4 mat4_from_quat(Quat q) {
    q = quat_norm(q);
    float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    Mat4 r = mat4_identity();
    r.c[0][0] = 1.0f - 2.0f * (yy + zz);
    r.c[0][1] = 2.0f * (xy + wz);
    r.c[0][2] = 2.0f * (xz - wy);
    r.c[1][0] = 2.0f * (xy - wz);
    r.c[1][1] = 1.0f - 2.0f * (xx + zz);
    r.c[1][2] = 2.0f * (yz + wx);
    r.c[2][0] = 2.0f * (xz + wy);
    r.c[2][1] = 2.0f * (yz - wx);
    r.c[2][2] = 1.0f - 2.0f * (xx + yy);
    return r;
}

Mat4 mat4_compose(Vec3 translation, Quat rotation, Vec3 scale) {
    Mat4 r = mat4_from_quat(rotation);
    r.c[0][0] *= scale.x; r.c[0][1] *= scale.x; r.c[0][2] *= scale.x;
    r.c[1][0] *= scale.y; r.c[1][1] *= scale.y; r.c[1][2] *= scale.y;
    r.c[2][0] *= scale.z; r.c[2][1] *= scale.z; r.c[2][2] *= scale.z;
    r.c[3][0] = translation.x; r.c[3][1] = translation.y; r.c[3][2] = translation.z;
    return r;
}

Mat4 mat4_trs(Vec3 translation, Vec3 euler_xyz, Vec3 scale) {
    return mat4_compose(translation, quat_from_euler(euler_xyz), scale);
}

/* ------------------------------------------------------------- noise/hash */
u32 hr_hash_u32(u32 x) {
    x ^= x >> 16; x *= 0x7feb352du;
    x ^= x >> 15; x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

u32 hr_hash_combine(u32 a, u32 b) {
    return hr_hash_u32(a ^ (hr_hash_u32(b) + 0x9e3779b9u + (a << 6) + (a >> 2)));
}

float hr_hash_float(u32 x) {
    return (float)(hr_hash_u32(x) & 0x00ffffffu) * (1.0f / 16777215.0f);
}

static float hash2f(int x, int y, u32 seed) {
    u32 h = hr_hash_combine((u32)x, (u32)y);
    h = hr_hash_combine(h, seed);
    return hr_hash_float(h);
}

float hr_value_noise2(float x, float y, u32 seed) {
    int ix = (int)floorf(x), iy = (int)floorf(y);
    float fx = x - (float)ix, fy = y - (float)iy;
    float ux = fx * fx * (3.0f - 2.0f * fx);
    float uy = fy * fy * (3.0f - 2.0f * fy);
    float a = hash2f(ix, iy, seed);
    float b = hash2f(ix + 1, iy, seed);
    float c = hash2f(ix, iy + 1, seed);
    float d = hash2f(ix + 1, iy + 1, seed);
    return hr_lerp(hr_lerp(a, b, ux), hr_lerp(c, d, ux), uy) * 2.0f - 1.0f;
}

static float hash3f(int x, int y, int z, u32 seed) {
    u32 h = hr_hash_combine((u32)x, (u32)y);
    h = hr_hash_combine(h, (u32)z);
    h = hr_hash_combine(h, seed);
    return hr_hash_float(h);
}

float hr_value_noise3(float x, float y, float z, u32 seed) {
    int ix = (int)floorf(x), iy = (int)floorf(y), iz = (int)floorf(z);
    float fx = x - (float)ix, fy = y - (float)iy, fz = z - (float)iz;
    float ux = fx * fx * (3.0f - 2.0f * fx);
    float uy = fy * fy * (3.0f - 2.0f * fy);
    float uz = fz * fz * (3.0f - 2.0f * fz);
    float a = hash3f(ix, iy, iz, seed);
    float b = hash3f(ix + 1, iy, iz, seed);
    float c = hash3f(ix, iy + 1, iz, seed);
    float d = hash3f(ix + 1, iy + 1, iz, seed);
    float e = hash3f(ix, iy, iz + 1, seed);
    float f = hash3f(ix + 1, iy, iz + 1, seed);
    float g = hash3f(ix, iy + 1, iz + 1, seed);
    float h = hash3f(ix + 1, iy + 1, iz + 1, seed);
    float x00 = hr_lerp(hr_lerp(a, b, ux), hr_lerp(c, d, ux), uy);
    float x10 = hr_lerp(hr_lerp(e, f, ux), hr_lerp(g, h, ux), uy);
    return hr_lerp(x00, x10, uz) * 2.0f - 1.0f;
}

float hr_fbm2(float x, float y, int octaves, u32 seed) {
    float sum = 0.0f, amp = 0.5f, freq = 1.0f;
    for (int i = 0; i < octaves; ++i) {
        sum += hr_value_noise2(x * freq, y * freq, seed + (u32)i * 101u) * amp;
        freq *= 2.0f;
        amp *= 0.5f;
    }
    return sum;
}

float hr_fbm3(float x, float y, float z, int octaves, u32 seed) {
    float sum = 0.0f, amp = 0.5f, freq = 1.0f;
    for (int i = 0; i < octaves; ++i) {
        sum += hr_value_noise3(x * freq, y * freq, z * freq, seed + (u32)i * 101u) * amp;
        freq *= 2.0f;
        amp *= 0.5f;
    }
    return sum;
}

/* --------------------------------------------------------------- frustum */
void hr_frustum_planes(Mat4 vp, Vec4 planes[6]) {
    for (int row = 0; row < 3; ++row) {
        Vec4 r = v4(vp.c[0][row], vp.c[1][row], vp.c[2][row], vp.c[3][row]);
        Vec4 w = v4(vp.c[0][3], vp.c[1][3], vp.c[2][3], vp.c[3][3]);
        planes[row * 2 + 0] = v4_add(w, r);
        planes[row * 2 + 1] = v4_sub(w, r);
    }
    /* Vulkan depth range [0,1]: near = z >= 0, far = z <= w */
    {
        Vec4 r2 = v4(vp.c[0][2], vp.c[1][2], vp.c[2][2], vp.c[3][2]);
        Vec4 w = v4(vp.c[0][3], vp.c[1][3], vp.c[2][3], vp.c[3][3]);
        planes[4] = r2;
        planes[5] = v4_sub(w, r2);
    }
    for (int i = 0; i < 6; ++i) {
        float l = sqrtf(planes[i].x * planes[i].x + planes[i].y * planes[i].y + planes[i].z * planes[i].z);
        if (l > 1e-20f) planes[i] = v4_scale(planes[i], 1.0f / l);
    }
}

bool hr_sphere_in_frustum(const Vec4 planes[6], Vec3 center, float radius) {
    for (int i = 0; i < 6; ++i) {
        float d = planes[i].x * center.x + planes[i].y * center.y +
                  planes[i].z * center.z + planes[i].w;
        if (d < -radius) return false;
    }
    return true;
}
