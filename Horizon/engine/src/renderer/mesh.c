#include "horizon/mesh.h"
#include "horizon/memory.h"

void hr_mesh_init(HrMeshData* m, const char* name) {
    memset(m, 0, sizeof(*m));
    m->name = name;
}

void hr_mesh_free(HrMeshData* m) {
    hr_free(m->vertices);
    hr_free(m->indices);
    memset(m, 0, sizeof(*m));
}

u32 hr_mesh_vertex(HrMeshData* m, Vec3 p, Vec3 n, Vec2 uv) {
    if (m->vertex_count == m->vertex_capacity) {
        u32 nc = m->vertex_capacity ? m->vertex_capacity * 2 : 1024;
        HrVertex* nv = (HrVertex*)hr_realloc(m->vertices, sizeof(HrVertex) * nc);
        HR_ASSERT(nv);
        m->vertices = nv;
        m->vertex_capacity = nc;
    }
    HrVertex* v = &m->vertices[m->vertex_count];
    v->position = p;
    v->normal = n;
    v->tangent = v4(1, 0, 0, 1);
    v->uv = uv;
    return m->vertex_count++;
}

static void push_index(HrMeshData* m, u32 i) {
    if (m->index_count == m->index_capacity) {
        u32 nc = m->index_capacity ? m->index_capacity * 2 : 2048;
        u32* ni = (u32*)hr_realloc(m->indices, sizeof(u32) * nc);
        HR_ASSERT(ni);
        m->indices = ni;
        m->index_capacity = nc;
    }
    m->indices[m->index_count++] = i;
}

void hr_mesh_triangle(HrMeshData* m, u32 a, u32 b, u32 c) {
    push_index(m, a); push_index(m, b); push_index(m, c);
}

void hr_mesh_quad(HrMeshData* m, u32 a, u32 b, u32 c, u32 d) {
    push_index(m, a); push_index(m, b); push_index(m, c);
    push_index(m, a); push_index(m, c); push_index(m, d);
}

void hr_mesh_recompute_normals(HrMeshData* m) {
    for (u32 i = 0; i < m->vertex_count; ++i) m->vertices[i].normal = v3(0, 0, 0);
    for (u32 i = 0; i + 2 < m->index_count; i += 3) {
        u32 ia = m->indices[i], ib = m->indices[i + 1], ic = m->indices[i + 2];
        Vec3 a = m->vertices[ia].position;
        Vec3 b = m->vertices[ib].position;
        Vec3 c = m->vertices[ic].position;
        Vec3 n = v3_cross(v3_sub(b, a), v3_sub(c, a));
        m->vertices[ia].normal = v3_add(m->vertices[ia].normal, n);
        m->vertices[ib].normal = v3_add(m->vertices[ib].normal, n);
        m->vertices[ic].normal = v3_add(m->vertices[ic].normal, n);
    }
    for (u32 i = 0; i < m->vertex_count; ++i)
        m->vertices[i].normal = v3_norm(m->vertices[i].normal);
}

void hr_mesh_compute_tangents(HrMeshData* m) {
    Vec3* tan = (Vec3*)hr_calloc(m->vertex_count, sizeof(Vec3));
    Vec3* bit = (Vec3*)hr_calloc(m->vertex_count, sizeof(Vec3));
    if (!tan || !bit) { hr_free(tan); hr_free(bit); return; }
    for (u32 i = 0; i + 2 < m->index_count; i += 3) {
        u32 ia = m->indices[i], ib = m->indices[i + 1], ic = m->indices[i + 2];
        HrVertex* va = &m->vertices[ia];
        HrVertex* vb = &m->vertices[ib];
        HrVertex* vc = &m->vertices[ic];
        Vec3 e1 = v3_sub(vb->position, va->position);
        Vec3 e2 = v3_sub(vc->position, va->position);
        float du1 = vb->uv.x - va->uv.x, dv1 = vb->uv.y - va->uv.y;
        float du2 = vc->uv.x - va->uv.x, dv2 = vc->uv.y - va->uv.y;
        float det = du1 * dv2 - du2 * dv1;
        if (fabsf(det) < 1e-12f) continue;
        float r = 1.0f / det;
        Vec3 t = v3_scale(v3_sub(v3_scale(e1, dv2), v3_scale(e2, dv1)), r);
        Vec3 b = v3_scale(v3_sub(v3_scale(e2, du1), v3_scale(e1, du2)), r);
        tan[ia] = v3_add(tan[ia], t); tan[ib] = v3_add(tan[ib], t); tan[ic] = v3_add(tan[ic], t);
        bit[ia] = v3_add(bit[ia], b); bit[ib] = v3_add(bit[ib], b); bit[ic] = v3_add(bit[ic], b);
    }
    for (u32 i = 0; i < m->vertex_count; ++i) {
        HrVertex* v = &m->vertices[i];
        Vec3 n = v->normal;
        Vec3 t = v3_sub(tan[i], v3_scale(n, v3_dot(n, tan[i])));
        if (v3_len_sq(t) < 1e-12f) t = v3(1, 0, 0);
        t = v3_norm(t);
        float w = v3_dot(v3_cross(n, t), bit[i]) < 0.0f ? -1.0f : 1.0f;
        v->tangent = v4_from3(t, w);
    }
    hr_free(tan);
    hr_free(bit);
}

void hr_mesh_finalize(HrMeshData* m) {
    hr_mesh_compute_tangents(m);
    Vec3 mn = v3(1e30f, 1e30f, 1e30f), mx = v3(-1e30f, -1e30f, -1e30f);
    for (u32 i = 0; i < m->vertex_count; ++i) {
        mn = v3_min(mn, m->vertices[i].position);
        mx = v3_max(mx, m->vertices[i].position);
    }
    if (m->vertex_count == 0) { mn = mx = v3(0, 0, 0); }
    m->bounds_min = mn;
    m->bounds_max = mx;
    m->bounds_center = v3_scale(v3_add(mn, mx), 0.5f);
    m->bounds_radius = v3_len(v3_sub(mx, mn)) * 0.5f;
}

void hr_mesh_box(HrMeshData* m, Vec3 center, Vec3 h) {
    Vec3 p[8];
    for (int i = 0; i < 8; ++i) {
        p[i] = v3(center.x + ((i & 1) ? h.x : -h.x),
                  center.y + ((i & 2) ? h.y : -h.y),
                  center.z + ((i & 4) ? h.z : -h.z));
    }
    struct Face { int a, b, c, d; Vec3 n; };
    const struct Face faces[6] = {
        {2, 3, 7, 6, {1, 0, 0}},  {0, 4, 5, 1, {-1, 0, 0}},
        {1, 5, 7, 3, {0, 1, 0}},  {0, 2, 6, 4, {0, -1, 0}},
        {4, 6, 7, 5, {0, 0, 1}},  {0, 1, 3, 2, {0, 0, -1}},
    };
    for (int f = 0; f < 6; ++f) {
        Vec2 uv[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
        u32 base = m->vertex_count;
        int idx[4] = {faces[f].a, faces[f].b, faces[f].c, faces[f].d};
        for (int i = 0; i < 4; ++i)
            hr_mesh_vertex(m, p[idx[i]], faces[f].n, uv[i]);
        hr_mesh_quad(m, base, base + 1, base + 2, base + 3);
    }
}

void hr_mesh_cylinder(HrMeshData* m, Vec3 center, float radius, float height,
                      int segments, bool capped) {
    if (segments < 3) segments = 3;
    u32 base = m->vertex_count;
    for (int i = 0; i <= segments; ++i) {
        float a = (float)i / (float)segments * HR_TAU;
        float cx = cosf(a), sy = sinf(a);
        Vec2 uv0 = {(float)i / (float)segments, 1.0f};
        Vec2 uv1 = {(float)i / (float)segments, 0.0f};
        hr_mesh_vertex(m, v3(center.x + cx * radius, center.y, center.z + sy * radius),
                       v3(cx, 0, sy), uv0);
        hr_mesh_vertex(m, v3(center.x + cx * radius, center.y + height, center.z + sy * radius),
                       v3(cx, 0, sy), uv1);
    }
    for (int i = 0; i < segments; ++i) {
        u32 a = base + (u32)i * 2;
        hr_mesh_quad(m, a, a + 1, a + 3, a + 2);
    }
    if (capped) {
        for (int side = 0; side < 2; ++side) {
            float y = center.y + (side ? height : 0.0f);
            Vec3 n = side ? v3(0, 1, 0) : v3(0, -1, 0);
            u32 cbase = m->vertex_count;
            hr_mesh_vertex(m, v3(center.x, y, center.z), n, v2(0.5f, 0.5f));
            for (int i = 0; i <= segments; ++i) {
                float a = (float)i / (float)segments * HR_TAU;
                hr_mesh_vertex(m, v3(center.x + cosf(a) * radius, y, center.z + sinf(a) * radius),
                               n, v2(0.5f + cosf(a) * 0.5f, 0.5f + sinf(a) * 0.5f));
            }
            for (int i = 0; i < segments; ++i) {
                if (side) hr_mesh_triangle(m, cbase, cbase + (u32)i + 2, cbase + (u32)i + 1);
                else hr_mesh_triangle(m, cbase, cbase + (u32)i + 1, cbase + (u32)i + 2);
            }
        }
    }
}

void hr_mesh_uv_sphere(HrMeshData* m, Vec3 center, float radius, int rings, int segments) {
    if (rings < 2) rings = 2;
    if (segments < 3) segments = 3;
    u32 base = m->vertex_count;
    for (int r = 0; r <= rings; ++r) {
        float v = (float)r / (float)rings;
        float phi = v * HR_PI;
        for (int s = 0; s <= segments; ++s) {
            float u = (float)s / (float)segments;
            float theta = u * HR_TAU;
            Vec3 n = v3(sinf(phi) * cosf(theta), cosf(phi), sinf(phi) * sinf(theta));
            hr_mesh_vertex(m, v3_add(center, v3_scale(n, radius)), n, v2(u, v));
        }
    }
    for (int r = 0; r < rings; ++r) {
        for (int s = 0; s < segments; ++s) {
            u32 a = base + (u32)(r * (segments + 1) + s);
            u32 b = a + (u32)(segments + 1);
            hr_mesh_quad(m, a, a + 1, b + 1, b);
        }
    }
}

void hr_mesh_plane(HrMeshData* m, Vec3 center, Vec2 size, Vec2 uv_scale) {
    Vec3 n = v3(0, 1, 0);
    u32 base = m->vertex_count;
    hr_mesh_vertex(m, v3(center.x - size.x * 0.5f, center.y, center.z - size.y * 0.5f), n, v2(0, 0));
    hr_mesh_vertex(m, v3(center.x + size.x * 0.5f, center.y, center.z - size.y * 0.5f), n, v2(uv_scale.x, 0));
    hr_mesh_vertex(m, v3(center.x + size.x * 0.5f, center.y, center.z + size.y * 0.5f), n, v2(uv_scale.x, uv_scale.y));
    hr_mesh_vertex(m, v3(center.x - size.x * 0.5f, center.y, center.z + size.y * 0.5f), n, v2(0, uv_scale.y));
    hr_mesh_quad(m, base, base + 1, base + 2, base + 3);
}
