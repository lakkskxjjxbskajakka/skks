/* Horizon Engine - procedural supercar mesh + draw. */
#include "world_internal.h"
#include "horizon/memory.h"

typedef struct LoftSection {
    float z;
    float half_width;
    float bottom;
    float top;
} LoftSection;

static void loft(HrMeshData* m, const LoftSection* sections, int count, int segments,
                 float exp_x, float exp_y, bool caps) {
    u32 base = m->vertex_count;
    float z0 = sections[0].z;
    float len = sections[count - 1].z - z0;
    for (int s = 0; s < count; ++s) {
        const LoftSection* sec = &sections[s];
        float cy = (sec->bottom + sec->top) * 0.5f;
        float hh = (sec->top - sec->bottom) * 0.5f;
        for (int k = 0; k <= segments; ++k) {
            float a = (float)k / (float)segments * HR_TAU;
            float cx = cosf(a), sy = sinf(a);
            float px = sec->half_width * copysignf(powf(fabsf(cx), exp_x), cx);
            float py = cy + hh * copysignf(powf(fabsf(sy), exp_y), sy);
            float u = (float)k / (float)segments;
            float v = (sec->z - z0) / HR_MAX(len, 1e-4f);
            hr_mesh_vertex(m, v3(px, py, sec->z), v3(0, 1, 0), v2(u, v));
        }
    }
    for (int s = 0; s + 1 < count; ++s) {
        u32 a = base + (u32)s * (u32)(segments + 1);
        u32 b = a + (u32)(segments + 1);
        for (int k = 0; k < segments; ++k) {
            hr_mesh_quad(m, a + (u32)k, a + (u32)k + 1, b + (u32)k + 1, b + (u32)k);
        }
    }
    if (caps) {
        for (int end = 0; end < 2; ++end) {
            const LoftSection* sec = &sections[end ? count - 1 : 0];
            float cy = (sec->bottom + sec->top) * 0.5f;
            u32 center = hr_mesh_vertex(m, v3(0, cy, sec->z), v3(0, 0, end ? 1.0f : -1.0f),
                                        v2(0.5f, 0.5f));
            u32 ring = base + (u32)(end ? count - 1 : 0) * (u32)(segments + 1);
            for (int k = 0; k < segments; ++k) {
                if (end) hr_mesh_triangle(m, center, ring + (u32)k, ring + (u32)k + 1);
                else hr_mesh_triangle(m, center, ring + (u32)k + 1, ring + (u32)k);
            }
        }
    }
}

static void wheel_tire(HrMeshData* m, float radius, float inner, float width, int segments) {
    float hw = width * 0.5f;
    u32 base = m->vertex_count;
    int stride = (segments + 1) * 2;
    for (int side = 0; side < 2; ++side) {
        float x = side ? hw : -hw;
        for (int i = 0; i <= segments; ++i) {
            float a = (float)i / (float)segments * HR_TAU;
            float cy = cosf(a), sz = sinf(a);
            hr_mesh_vertex(m, v3(x, cy * radius, sz * radius), v3(0, cy, sz),
                           v2((float)i / (float)segments, (float)side));
            hr_mesh_vertex(m, v3(x, cy * inner, sz * inner), v3(side ? 1.0f : -1.0f, 0, 0),
                           v2((float)i / (float)segments, (float)side * 0.25f));
        }
    }
    for (int i = 0; i < segments; ++i) {
        u32 ol = base + (u32)i * 2;
        u32 orr = base + (u32)stride + (u32)i * 2;
        hr_mesh_quad(m, ol, ol + 2, orr + 2, orr);
        u32 il = ol + 1;
        u32 ir = orr + 1;
        hr_mesh_quad(m, ol, il, il + 2, ol + 2);
        hr_mesh_quad(m, orr, orr + 2, ir + 2, ir);
    }
}

static void cylinder_z(HrMeshData* m, float radius, float length, int segments) {
    float hz = length * 0.5f;
    u32 base = m->vertex_count;
    for (int i = 0; i <= segments; ++i) {
        float a = (float)i / (float)segments * HR_TAU;
        float cx = cosf(a), sy = sinf(a);
        Vec2 uv = v2((float)i / (float)segments, 0.0f);
        hr_mesh_vertex(m, v3(cx * radius, sy * radius, -hz), v3(cx, sy, 0), uv);
        hr_mesh_vertex(m, v3(cx * radius, sy * radius, hz), v3(cx, sy, 0), uv);
    }
    for (int i = 0; i < segments; ++i) {
        u32 a = base + (u32)i * 2;
        hr_mesh_quad(m, a, a + 2, a + 3, a + 1);
    }
}

static void add_rotated_box(HrMeshData* m, Vec3 center, Vec3 half, float angle_x) {
    float ca = cosf(angle_x), sa = sinf(angle_x);
    Vec3 p[8];
    for (int i = 0; i < 8; ++i) {
        float x = ((i & 1) ? half.x : -half.x) + center.x;
        float y = ((i & 2) ? half.y : -half.y) + center.y;
        float z = ((i & 4) ? half.z : -half.z) + center.z;
        p[i] = v3(x, y * ca - z * sa, y * sa + z * ca);
    }
    struct Face { int a, b, c, d; };
    const struct Face faces[6] = {{2,3,7,6},{0,4,5,1},{1,5,7,3},{0,2,6,4},{4,6,7,5},{0,1,3,2}};
    for (int f = 0; f < 6; ++f) {
        u32 base = m->vertex_count;
        int idx[4] = {faces[f].a, faces[f].b, faces[f].c, faces[f].d};
        Vec3 n = v3_norm(v3_cross(v3_sub(p[idx[1]], p[idx[0]]), v3_sub(p[idx[2]], p[idx[0]])));
        for (int i = 0; i < 4; ++i)
            hr_mesh_vertex(m, p[idx[i]], n, v2((float)(i & 1), (float)((i >> 1) & 1)));
        hr_mesh_quad(m, base, base + 1, base + 2, base + 3);
    }
}

static void wheel_rim(HrMeshData* m, float inner, float width) {
    int seg = 20;
    for (int side = 0; side < 2; ++side) {
        float sign = side ? 1.0f : -1.0f;
        float x = sign * (width * 0.5f + 0.012f);
        Vec3 normal = v3(sign, 0, 0);
        u32 base = m->vertex_count;
        for (int i = 0; i <= seg; ++i) {
            float a = (float)i / (float)seg * HR_TAU;
            float cy = cosf(a), sz = sinf(a);
            hr_mesh_vertex(m, v3(x, cy * inner, sz * inner), normal, v2((float)i / (float)seg, 0));
            hr_mesh_vertex(m, v3(x, cy * inner * 0.78f, sz * inner * 0.78f), normal,
                           v2((float)i / (float)seg, 1));
        }
        for (int i = 0; i < seg; ++i) {
            u32 a = base + (u32)i * 2;
            if (side) hr_mesh_quad(m, a, a + 1, a + 3, a + 2);
            else hr_mesh_quad(m, a, a + 2, a + 3, a + 1);
        }
        for (int s = 0; s < 5; ++s) {
            float ang = (float)s / 5.0f * HR_TAU;
            add_rotated_box(m, v3(x + sign * 0.006f, inner * 0.44f, 0),
                            v3(0.016f, inner * 0.36f, 0.036f), ang);
        }
        u32 hb = m->vertex_count;
        float hx = x + sign * 0.015f;
        for (int i = 0; i <= seg; ++i) {
            float a = (float)i / (float)seg * HR_TAU;
            hr_mesh_vertex(m, v3(hx, cosf(a) * 0.085f, sinf(a) * 0.085f), normal,
                           v2((float)i / (float)seg, 1));
        }
        u32 hc = hr_mesh_vertex(m, v3(hx, 0, 0), normal, v2(0.5f, 0.5f));
        for (int i = 0; i < seg; ++i) {
            if (side) hr_mesh_triangle(m, hc, hb + (u32)i, hb + (u32)i + 1);
            else hr_mesh_triangle(m, hc, hb + (u32)i + 1, hb + (u32)i);
        }
    }
}

static int upload(HrWorld* w, HrMeshData* m, const char* name) {
    hr_mesh_finalize(m);
    int id = hr_renderer_upload_mesh(w->renderer, m, name);
    hr_mesh_free(m);
    return id;
}

void hr_world_build_car(HrWorld* w) {
    static const LoftSection body[] = {
        {-2.16f, 0.76f, 0.34f, 0.68f},
        {-1.96f, 0.93f, 0.30f, 0.79f},
        {-1.25f, 0.99f, 0.27f, 0.90f},
        {-0.45f, 1.00f, 0.25f, 0.92f},
        { 0.35f, 0.97f, 0.25f, 0.88f},
        { 1.05f, 0.92f, 0.28f, 0.77f},
        { 1.65f, 0.84f, 0.32f, 0.61f},
        { 2.10f, 0.64f, 0.38f, 0.52f},
    };
    static const LoftSection cabin[] = {
        {-0.95f, 0.58f, 0.86f, 1.01f},
        {-0.45f, 0.68f, 0.86f, 1.29f},
        { 0.30f, 0.65f, 0.86f, 1.32f},
        { 0.96f, 0.51f, 0.86f, 1.03f},
    };

    HrMeshData m;
    hr_mesh_init(&m, "car_body");
    loft(&m, body, HR_ARRAY_COUNT(body), 20, 0.62f, 0.62f, true);
    hr_mesh_recompute_normals(&m);
    w->mesh_car_body = upload(w, &m, "car_body");

    hr_mesh_init(&m, "car_glass");
    loft(&m, cabin, HR_ARRAY_COUNT(cabin), 16, 0.66f, 0.66f, true);
    hr_mesh_recompute_normals(&m);
    w->mesh_car_glass = upload(w, &m, "car_glass");

    hr_mesh_init(&m, "car_roof");
    hr_mesh_box(&m, v3(0, 1.325f, 0.05f), v3(0.44f, 0.012f, 0.34f));
    w->mesh_car_roof = upload(w, &m, "car_roof");

    hr_mesh_init(&m, "car_trim");
    hr_mesh_box(&m, v3(0, 0.17f, 2.04f), v3(0.78f, 0.045f, 0.12f));
    hr_mesh_box(&m, v3(0, 0.22f, -2.06f), v3(0.68f, 0.07f, 0.13f));
    hr_mesh_box(&m, v3(0.95f, 0.255f, 0.0f), v3(0.055f, 0.06f, 1.15f));
    hr_mesh_box(&m, v3(-0.95f, 0.255f, 0.0f), v3(0.055f, 0.06f, 1.15f));
    hr_mesh_box(&m, v3(0, 0.795f, 1.20f), v3(0.40f, 0.018f, 0.22f));
    hr_mesh_box(&m, v3(0, 1.02f, -1.92f), v3(0.78f, 0.028f, 0.18f));
    hr_mesh_box(&m, v3(0.52f, 0.87f, -1.92f), v3(0.035f, 0.16f, 0.10f));
    hr_mesh_box(&m, v3(-0.52f, 0.87f, -1.92f), v3(0.035f, 0.16f, 0.10f));
    hr_mesh_box(&m, v3(0.98f, 0.865f, 0.56f), v3(0.10f, 0.052f, 0.075f));
    hr_mesh_box(&m, v3(-0.98f, 0.865f, 0.56f), v3(0.10f, 0.052f, 0.075f));
    hr_mesh_box(&m, v3(0.92f, 0.82f, 0.56f), v3(0.05f, 0.028f, 0.04f));
    hr_mesh_box(&m, v3(-0.92f, 0.82f, 0.56f), v3(0.05f, 0.028f, 0.04f));
    hr_mesh_box(&m, v3(0.90f, 0.56f, -0.42f), v3(0.085f, 0.10f, 0.34f));
    hr_mesh_box(&m, v3(-0.90f, 0.56f, -0.42f), v3(0.085f, 0.10f, 0.34f));
    w->mesh_car_trim = upload(w, &m, "car_trim");

    hr_mesh_init(&m, "car_metal");
    cylinder_z(&m, 0.066f, 0.17f, 10);
    for (u32 i = 0; i < m.vertex_count; ++i)
        m.vertices[i].position = v3_add(m.vertices[i].position, v3(0.38f, 0.29f, -2.11f));
    u32 count = m.vertex_count;
    cylinder_z(&m, 0.066f, 0.17f, 10);
    for (u32 i = count; i < m.vertex_count; ++i)
        m.vertices[i].position = v3_add(m.vertices[i].position, v3(-0.38f, 0.29f, -2.11f));
    w->mesh_car_metal = upload(w, &m, "car_metal");

    hr_mesh_init(&m, "car_light_w");
    hr_mesh_box(&m, v3(0.56f, 0.445f, 2.085f), v3(0.20f, 0.042f, 0.035f));
    hr_mesh_box(&m, v3(-0.56f, 0.445f, 2.085f), v3(0.20f, 0.042f, 0.035f));
    w->mesh_car_light_w = upload(w, &m, "car_light_w");

    hr_mesh_init(&m, "car_light_r");
    hr_mesh_box(&m, v3(0.43f, 0.655f, -2.155f), v3(0.31f, 0.045f, 0.032f));
    hr_mesh_box(&m, v3(-0.43f, 0.655f, -2.155f), v3(0.31f, 0.045f, 0.032f));
    hr_mesh_box(&m, v3(0, 0.655f, -2.145f), v3(0.10f, 0.012f, 0.020f));
    w->mesh_car_light_r = upload(w, &m, "car_light_r");

    hr_mesh_init(&m, "car_wheel");
    wheel_tire(&m, 0.365f, 0.235f, 0.27f, 18);
    w->mesh_car_wheel = upload(w, &m, "car_wheel");

    hr_mesh_init(&m, "car_rim");
    wheel_rim(&m, 0.235f, 0.27f);
    w->mesh_car_rim = upload(w, &m, "car_rim");
}

Mat4 hr_car_transform(const HrWorld* w) {
    Vec3 euler = v3(w->car_pitch, w->car_yaw, w->car_roll);
    return mat4_compose(v3(w->car_pos.x, w->car_y + w->car_bob, w->car_pos.z),
                        quat_from_euler(euler), v3s(1.0f));
}

void hr_world_draw_car(HrWorld* w, HrRenderer* r) {
    Mat4 car = hr_car_transform(w);
    Vec4 paint = v4(0.88f, 0.08f, 0.05f, 1.0f);
    Vec4 glass = v4(0.035f, 0.045f, 0.06f, 1.0f);
    Vec4 trim = v4(0.035f, 0.035f, 0.04f, 1.0f);
    Vec4 metal = v4(0.62f, 0.62f, 0.66f, 1.0f);

    hr_renderer_draw(r, w->mesh_car_body, HR_MAT_CAR_PAINT, car, paint);
    hr_renderer_draw(r, w->mesh_car_roof, HR_MAT_CAR_PAINT, car, paint);
    hr_renderer_draw(r, w->mesh_car_glass, HR_MAT_GLASS, car, glass);
    hr_renderer_draw(r, w->mesh_car_trim, HR_MAT_METAL, car, trim);
    hr_renderer_draw(r, w->mesh_car_metal, HR_MAT_METAL, car, metal);
    hr_renderer_draw(r, w->mesh_car_light_w, HR_MAT_LIGHT_WHITE, car, v4s(1));
    hr_renderer_draw(r, w->mesh_car_light_r, HR_MAT_LIGHT_RED, car,
                     w->brake_lights ? v4(2.2f, 0.4f, 0.2f, 1) : v4(0.8f, 0.1f, 0.05f, 1));

    static const Vec3 wheel_pos[4] = {
        { 0.99f, 0.365f,  1.34f},
        {-0.99f, 0.365f,  1.34f},
        { 0.99f, 0.365f, -1.38f},
        {-0.99f, 0.365f, -1.38f},
    };
    for (int i = 0; i < 4; ++i) {
        Mat4 local = mat4_translate(wheel_pos[i]);
        if (i < 2) local = mat4_mul(local, mat4_rotate_y(w->car_steer * 0.55f));
        local = mat4_mul(local, mat4_rotate_x(w->car_wheel_spin));
        hr_renderer_draw(r, w->mesh_car_wheel, HR_MAT_RUBBER, mat4_mul(car, local),
                         v4(0.9f, 0.9f, 0.9f, 1));
        hr_renderer_draw(r, w->mesh_car_rim, HR_MAT_METAL, mat4_mul(car, local), metal);
    }
}
