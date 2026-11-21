/* Horizon Engine - rigid-body physics: uniform-grid broadphase, contact
 * manifolds, sequential-impulse solver, ray casting and heightfield ground. */
#include "horizon/physics.h"
#include "horizon/memory.h"
#include "horizon/containers.h"

#include <math.h>

#define HR_PHYS_GROUND_SENTINEL 0xffffffffu

typedef struct HrBody {
    HrShape shape;
    Vec3 pos;
    Quat rot;
    Vec3 vel;
    Vec3 angvel;
    Vec3 inv_inertia_local;
    float inv_mass;
    float restitution;
    float friction;
    u32 layer;
    u32 mask;
    u32 flags;
    u32 generation;
    bool alive;
    bool sleeping;
    float sleep_timer;
    Vec3 aabb_min;
    Vec3 aabb_max;
} HrBody;

typedef struct HrContact {
    u32 a;
    u32 b;
    Vec3 normal;      /* from a to b */
    Vec3 point;
    Vec3 tangent1;
    Vec3 tangent2;
    float depth;
    float normal_impulse;
    float tangent_impulse1;
    float tangent_impulse2;
    float friction;
    float restitution;
} HrContact;

typedef struct HrU32List {
    u32* items;
    usize len;
    usize cap;
} HrU32List;

struct HrPhysicsWorld {
    HrPhysicsConfig cfg;
    HrBody* bodies;
    u32 capacity;
    u32* free_list;
    u32 free_count;
    u32 live_count;

    u32 grid_mask;
    u32 grid_buckets;
    HrU32List* cells;

    u64* pair_keys;
    u32 pair_capacity;
    u32 pair_count;
    u32 broadphase_pairs;

    HR_DA(HrContact) contacts;

    HrGroundHeightFn ground_fn;
    void* ground_user;

    u32 last_iterations;
};

static u32 next_pow2(u32 v) {
    u32 p = 1;
    while (p < v && p < 0x40000000u) p <<= 1;
    return p;
}

void hr_physics_default_config(HrPhysicsConfig* out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->gravity = v3(0.0f, -9.81f, 0.0f);
    out->solver_iterations = 8;
    out->max_bodies = 2048;
    out->cell_size = 4.0f;
    out->sleep_linear = 0.08f;
    out->sleep_angular = 0.08f;
    out->sleep_time = 0.8f;
    out->contact_slop = 0.005f;
    out->correction = 0.22f;
    out->restitution_cutoff = 1.0f;
}

static HrBody* body_at(HrPhysicsWorld* w, u32 index) {
    if (index == HR_PHYS_GROUND_SENTINEL || index >= w->capacity) return NULL;
    HrBody* b = &w->bodies[index];
    return b->alive ? b : NULL;
}

static const HrBody* body_at_const(const HrPhysicsWorld* w, u32 index) {
    if (index == HR_PHYS_GROUND_SENTINEL || index >= w->capacity) return NULL;
    const HrBody* b = &w->bodies[index];
    return b->alive ? b : NULL;
}

static void compute_inertia(HrBody* b) {
    b->inv_inertia_local = v3(0.0f, 0.0f, 0.0f);
    if (b->inv_mass <= 0.0f) return;
    float m = 1.0f / b->inv_mass;
    switch (b->shape.type) {
    case HR_SHAPE_SPHERE: {
        float i = 0.4f * m * b->shape.radius * b->shape.radius;
        if (i > 1e-9f) b->inv_inertia_local = v3s(1.0f / i);
        break;
    }
    case HR_SHAPE_BOX: {
        Vec3 h = b->shape.half_extents;
        float ix = (m / 3.0f) * (h.y * h.y + h.z * h.z);
        float iy = (m / 3.0f) * (h.x * h.x + h.z * h.z);
        float iz = (m / 3.0f) * (h.x * h.x + h.y * h.y);
        b->inv_inertia_local = v3(ix > 1e-9f ? 1.0f / ix : 0.0f,
                                  iy > 1e-9f ? 1.0f / iy : 0.0f,
                                  iz > 1e-9f ? 1.0f / iz : 0.0f);
        break;
    }
    case HR_SHAPE_CAPSULE: {
        float r = b->shape.radius;
        float hh = b->shape.half_height;
        float iy = 0.5f * m * r * r;
        float ix = (m / 12.0f) * (3.0f * r * r + 4.0f * hh * hh);
        b->inv_inertia_local = v3(ix > 1e-9f ? 1.0f / ix : 0.0f,
                                  iy > 1e-9f ? 1.0f / iy : 0.0f,
                                  ix > 1e-9f ? 1.0f / ix : 0.0f);
        break;
    }
    default:
        break;
    }
}

static Vec3 inv_inertia_mul(const HrBody* b, Vec3 v) {
    if (!b) return v3(0.0f, 0.0f, 0.0f);
    Vec3 local = quat_rotate(quat_conj(b->rot), v);
    local = v3_mul(local, b->inv_inertia_local);
    return quat_rotate(b->rot, local);
}

static void update_aabb(HrBody* b) {
    if (b->shape.type == HR_SHAPE_PLANE) {
        b->aabb_min = v3(-1e30f, -1e30f, -1e30f);
        b->aabb_max = v3( 1e30f,  1e30f,  1e30f);
        return;
    }
    Vec3 extent;
    if (b->shape.type == HR_SHAPE_SPHERE) {
        extent = v3s(b->shape.radius);
    } else if (b->shape.type == HR_SHAPE_BOX) {
        Mat4 m = mat4_from_quat(b->rot);
        Vec3 e = b->shape.half_extents;
        extent = v3(fabsf(m.c[0][0]) * e.x + fabsf(m.c[1][0]) * e.y + fabsf(m.c[2][0]) * e.z,
                    fabsf(m.c[0][1]) * e.x + fabsf(m.c[1][1]) * e.y + fabsf(m.c[2][1]) * e.z,
                    fabsf(m.c[0][2]) * e.x + fabsf(m.c[1][2]) * e.y + fabsf(m.c[2][2]) * e.z);
    } else {
        Vec3 axis = quat_rotate(b->rot, v3(0.0f, 1.0f, 0.0f));
        Vec3 a = v3_scale(v3_abs(axis), b->shape.half_height);
        extent = v3_add(a, v3s(b->shape.radius));
    }
    b->aabb_min = v3_sub(b->pos, extent);
    b->aabb_max = v3_add(b->pos, extent);
}

static bool aabb_overlap(const HrBody* a, const HrBody* b) {
    return a->aabb_min.x <= b->aabb_max.x && a->aabb_max.x >= b->aabb_min.x &&
           a->aabb_min.y <= b->aabb_max.y && a->aabb_max.y >= b->aabb_min.y &&
           a->aabb_min.z <= b->aabb_max.z && a->aabb_max.z >= b->aabb_min.z;
}

static u64 pair_hash_u64(u64 key) {
    key ^= key >> 33;
    key *= 0xff51afd7ed558ccdull;
    key ^= key >> 33;
    key *= 0xc4ceb9fe1a85ec53ull;
    key ^= key >> 33;
    return key;
}

static void pair_set_clear(HrPhysicsWorld* w) {
    memset(w->pair_keys, 0, (usize)w->pair_capacity * sizeof(u64));
    w->pair_count = 0;
}

static bool pair_set_insert(HrPhysicsWorld* w, u32 a, u32 b) {
    if (a > b) { u32 t = a; a = b; b = t; }
    u64 key = ((u64)a << 32) | (u64)b;
    if (key == 0) key = 1;
    u32 mask = w->pair_capacity - 1u;
    u32 i = (u32)pair_hash_u64(key) & mask;
    for (u32 n = 0; n < w->pair_capacity; ++n) {
        if (w->pair_keys[i] == key) return false;
        if (w->pair_keys[i] == 0) {
            w->pair_keys[i] = key;
            w->pair_count++;
            return true;
        }
        i = (i + 1u) & mask;
    }
    return false;
}

static void u32list_push(HrU32List* l, u32 v) {
    if (l->len >= l->cap) {
        usize cap = l->cap ? l->cap * 2u : 8u;
        u32* items = (u32*)hr_realloc(l->items, cap * sizeof(u32));
        HR_ASSERT(items);
        l->items = items;
        l->cap = cap;
    }
    l->items[l->len++] = v;
}

static void grid_insert(HrPhysicsWorld* w, u32 index, const HrBody* b) {
    float cs = w->cfg.cell_size > 0.1f ? w->cfg.cell_size : 4.0f;
    i32 x0 = (i32)floorf(b->aabb_min.x / cs);
    i32 y0 = (i32)floorf(b->aabb_min.y / cs);
    i32 z0 = (i32)floorf(b->aabb_min.z / cs);
    i32 x1 = (i32)floorf(b->aabb_max.x / cs);
    i32 y1 = (i32)floorf(b->aabb_max.y / cs);
    i32 z1 = (i32)floorf(b->aabb_max.z / cs);
    const i32 span = 32;
    if (x1 - x0 > span) x1 = x0 + span;
    if (y1 - y0 > span) y1 = y0 + span;
    if (z1 - z0 > span) z1 = z0 + span;
    for (i32 x = x0; x <= x1; ++x) {
        for (i32 y = y0; y <= y1; ++y) {
            for (i32 z = z0; z <= z1; ++z) {
                u32 h = ((u32)x * 73856093u) ^ ((u32)y * 19349663u) ^ ((u32)z * 83492791u);
                u32 bucket = h & w->grid_mask;
                u32list_push(&w->cells[bucket], index);
            }
        }
    }
}

static void add_contact(HrPhysicsWorld* w, u32 a, u32 b, Vec3 normal, Vec3 point,
                        float depth, float restitution, float friction) {
    if (w->contacts.len >= (usize)w->capacity * 16u) return;
    HrContact c;
    memset(&c, 0, sizeof(c));
    c.a = a;
    c.b = b;
    c.normal = normal;
    c.point = point;
    c.depth = depth;
    c.restitution = restitution;
    c.friction = friction;
    Vec3 up = fabsf(normal.y) < 0.9f ? v3(0.0f, 1.0f, 0.0f) : v3(1.0f, 0.0f, 0.0f);
    c.tangent1 = v3_norm(v3_cross(normal, up));
    c.tangent2 = v3_cross(normal, c.tangent1);
    hr_da_push(&w->contacts, c);
}

static bool is_dynamic(const HrBody* b) {
    return b->inv_mass > 0.0f && (b->flags & HR_BODY_KINEMATIC) == 0u;
}

static bool is_kinematic(const HrBody* b) {
    return (b->flags & HR_BODY_KINEMATIC) != 0u;
}

static void wake_body(HrPhysicsWorld* w, u32 index) {
    HrBody* b = body_at(w, index);
    if (!b || !b->sleeping) return;
    b->sleeping = false;
    b->sleep_timer = 0.0f;
}

/* ------------------------------------------------------------ narrowphase */

static void contact_sphere_sphere(HrPhysicsWorld* w, u32 ia, u32 ib,
                                  Vec3 pa, Vec3 pb, float ra, float rb,
                                  float restitution, float friction) {
    Vec3 d = v3_sub(pb, pa);
    float dist2 = v3_len_sq(d);
    float sum = ra + rb;
    if (dist2 >= sum * sum) return;
    float dist = sqrtf(dist2);
    Vec3 n = dist > 1e-6f ? v3_scale(d, 1.0f / dist) : v3(1.0f, 0.0f, 0.0f);
    float depth = sum - dist;
    Vec3 point = v3_add(pa, v3_scale(n, ra - depth * 0.5f));
    add_contact(w, ia, ib, n, point, depth, restitution, friction);
}

static void contact_sphere_box(HrPhysicsWorld* w, u32 isphere, u32 ibox,
                               const HrBody* sphere, const HrBody* box,
                               float restitution, float friction) {
    Quat inv = quat_conj(box->rot);
    Vec3 local = quat_rotate(inv, v3_sub(sphere->pos, box->pos));
    Vec3 h = box->shape.half_extents;
    Vec3 closest = v3(HR_CLAMP(local.x, -h.x, h.x),
                      HR_CLAMP(local.y, -h.y, h.y),
                      HR_CLAMP(local.z, -h.z, h.z));
    Vec3 delta = v3_sub(local, closest);
    float dist2 = v3_len_sq(delta);
    Vec3 n_local;
    float depth;
    Vec3 contact_local = closest;
    float r = sphere->shape.radius;
    if (dist2 > 1e-12f) {
        if (dist2 >= r * r) return;
        float dist = sqrtf(dist2);
        n_local = v3_scale(delta, 1.0f / dist);
        depth = r - dist;
    } else {
        float dx = h.x - fabsf(local.x);
        float dy = h.y - fabsf(local.y);
        float dz = h.z - fabsf(local.z);
        if (dx <= dy && dx <= dz) {
            n_local = v3(local.x < 0.0f ? -1.0f : 1.0f, 0.0f, 0.0f);
            depth = r + dx;
        } else if (dy <= dz) {
            n_local = v3(0.0f, local.y < 0.0f ? -1.0f : 1.0f, 0.0f);
            depth = r + dy;
        } else {
            n_local = v3(0.0f, 0.0f, local.z < 0.0f ? -1.0f : 1.0f);
            depth = r + dz;
        }
        contact_local = local;
    }
    Vec3 n_world = quat_rotate(box->rot, n_local); /* box -> sphere */
    Vec3 point = v3_add(box->pos, quat_rotate(box->rot, contact_local));
    add_contact(w, ibox, isphere, n_world, point, depth, restitution, friction);
}

static void get_box_axes(const HrBody* b, Vec3 axes[3]) {
    Mat4 m = mat4_from_quat(b->rot);
    axes[0] = v3(m.c[0][0], m.c[0][1], m.c[0][2]);
    axes[1] = v3(m.c[1][0], m.c[1][1], m.c[1][2]);
    axes[2] = v3(m.c[2][0], m.c[2][1], m.c[2][2]);
}

static void box_support(const HrBody* b, const Vec3 axes[3], Vec3 normal, Vec3* out) {
    Vec3 h = b->shape.half_extents;
    Vec3 p = b->pos;
    for (int i = 0; i < 3; ++i) {
        float sign = v3_dot(axes[i], normal) >= 0.0f ? 1.0f : -1.0f;
        p = v3_add(p, v3_scale(axes[i], sign * h.v[i]));
    }
    *out = p;
}

static bool box_box_sat(const HrBody* a, const HrBody* b, Vec3* out_normal, float* out_depth) {
    Vec3 axes_a[3], axes_b[3];
    get_box_axes(a, axes_a);
    get_box_axes(b, axes_b);
    Vec3 d = v3_sub(b->pos, a->pos);
    float best = 1e30f;
    Vec3 best_axis = v3(1.0f, 0.0f, 0.0f);

    for (int i = 0; i < 15; ++i) {
        Vec3 axis;
        if (i < 3) axis = axes_a[i];
        else if (i < 6) axis = axes_b[i - 3];
        else {
            axis = v3_cross(axes_a[(i - 6) / 3], axes_b[(i - 6) % 3]);
            float l2 = v3_len_sq(axis);
            if (l2 < 1e-8f) continue;
            axis = v3_scale(axis, 1.0f / sqrtf(l2));
        }
        float ra = fabsf(v3_dot(axis, axes_a[0])) * a->shape.half_extents.x +
                   fabsf(v3_dot(axis, axes_a[1])) * a->shape.half_extents.y +
                   fabsf(v3_dot(axis, axes_a[2])) * a->shape.half_extents.z;
        float rb = fabsf(v3_dot(axis, axes_b[0])) * b->shape.half_extents.x +
                   fabsf(v3_dot(axis, axes_b[1])) * b->shape.half_extents.y +
                   fabsf(v3_dot(axis, axes_b[2])) * b->shape.half_extents.z;
        float dist = fabsf(v3_dot(d, axis));
        float overlap = ra + rb - dist;
        if (overlap < 0.0f) return false;
        if (overlap < best) {
            best = overlap;
            best_axis = axis;
        }
    }

    if (v3_dot(best_axis, d) < 0.0f) best_axis = v3_neg(best_axis);
    *out_normal = best_axis;
    *out_depth = best;
    return true;
}

static void contact_box_box(HrPhysicsWorld* w, u32 ia, u32 ib,
                            const HrBody* a, const HrBody* b,
                            float restitution, float friction) {
    Vec3 normal;
    float depth;
    if (!box_box_sat(a, b, &normal, &depth)) return;
    Vec3 axes_a[3], axes_b[3];
    get_box_axes(a, axes_a);
    get_box_axes(b, axes_b);
    Vec3 sa, sb;
    box_support(a, axes_a, normal, &sa);
    box_support(b, axes_b, v3_neg(normal), &sb);
    Vec3 point = v3_scale(v3_add(sa, sb), 0.5f);
    add_contact(w, ia, ib, normal, point, depth, restitution, friction);
}

static void contact_plane_sphere(HrPhysicsWorld* w, u32 iplane, u32 isphere,
                                 Vec3 plane_pos, Vec3 plane_normal,
                                 const HrBody* sphere, float restitution, float friction) {
    float dist = v3_dot(v3_sub(sphere->pos, plane_pos), plane_normal) - sphere->shape.radius;
    if (dist >= 0.0f) return;
    float depth = -dist;
    Vec3 point = v3_sub(sphere->pos, v3_scale(plane_normal, sphere->shape.radius - depth * 0.5f));
    add_contact(w, iplane, isphere, plane_normal, point, depth, restitution, friction);
}

static void contact_plane_box(HrPhysicsWorld* w, u32 iplane, u32 ibox,
                              Vec3 plane_pos, Vec3 plane_normal,
                              const HrBody* box, float restitution, float friction) {
    Mat4 m = mat4_from_quat(box->rot);
    Vec3 axes[3] = { v3(m.c[0][0], m.c[0][1], m.c[0][2]),
                     v3(m.c[1][0], m.c[1][1], m.c[1][2]),
                     v3(m.c[2][0], m.c[2][1], m.c[2][2]) };
    Vec3 h = box->shape.half_extents;
    /* collect up to 4 deepest corners */
    Vec3 corners[8];
    float depths[8];
    int count = 0;
    for (int ix = -1; ix <= 1; ix += 2) {
        for (int iy = -1; iy <= 1; iy += 2) {
            for (int iz = -1; iz <= 1; iz += 2) {
                Vec3 local = v3(h.x * (float)ix, h.y * (float)iy, h.z * (float)iz);
                Vec3 world = v3_add(box->pos,
                    v3_add(v3_add(v3_scale(axes[0], local.x), v3_scale(axes[1], local.y)),
                           v3_scale(axes[2], local.z)));
                float d = v3_dot(v3_sub(world, plane_pos), plane_normal);
                if (d < 0.0f) {
                    corners[count] = world;
                    depths[count] = -d;
                    count++;
                }
            }
        }
    }
    /* keep 4 deepest */
    for (int i = 0; i < count; ++i) {
        for (int j = i + 1; j < count; ++j) {
            if (depths[j] > depths[i]) {
                float td = depths[i]; depths[i] = depths[j]; depths[j] = td;
                Vec3 tc = corners[i]; corners[i] = corners[j]; corners[j] = tc;
            }
        }
    }
    int emit = count < 4 ? count : 4;
    for (int i = 0; i < emit; ++i) {
        Vec3 point = v3_sub(corners[i], v3_scale(plane_normal, depths[i] * 0.5f));
        add_contact(w, iplane, ibox, plane_normal, point, depths[i], restitution, friction);
    }
}

static void contact_plane_capsule(HrPhysicsWorld* w, u32 iplane, u32 icap,
                                  Vec3 plane_pos, Vec3 plane_normal,
                                  const HrBody* cap, float restitution, float friction) {
    Vec3 axis = quat_rotate(cap->rot, v3(0.0f, 1.0f, 0.0f));
    Vec3 p0 = v3_add(cap->pos, v3_scale(axis, cap->shape.half_height));
    Vec3 p1 = v3_sub(cap->pos, v3_scale(axis, cap->shape.half_height));
    float d0 = v3_dot(v3_sub(p0, plane_pos), plane_normal) - cap->shape.radius;
    float d1 = v3_dot(v3_sub(p1, plane_pos), plane_normal) - cap->shape.radius;
    if (d0 < 0.0f) {
        float depth = -d0;
        Vec3 point = v3_sub(p0, v3_scale(plane_normal, cap->shape.radius - depth * 0.5f));
        add_contact(w, iplane, icap, plane_normal, point, depth, restitution, friction);
    }
    if (d1 < 0.0f) {
        float depth = -d1;
        Vec3 point = v3_sub(p1, v3_scale(plane_normal, cap->shape.radius - depth * 0.5f));
        add_contact(w, iplane, icap, plane_normal, point, depth, restitution, friction);
    }
}

static Vec3 closest_point_on_segment(Vec3 p, Vec3 a, Vec3 b) {
    Vec3 ab = v3_sub(b, a);
    float t = v3_dot(v3_sub(p, a), ab);
    float den = v3_dot(ab, ab);
    if (den < 1e-12f) return a;
    t = HR_CLAMP(t / den, 0.0f, 1.0f);
    return v3_add(a, v3_scale(ab, t));
}

static void closest_points_segments(Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2, Vec3* c1, Vec3* c2) {
    Vec3 d1 = v3_sub(q1, p1), d2 = v3_sub(q2, p2), r = v3_sub(p1, p2);
    float a = v3_dot(d1, d1), e = v3_dot(d2, d2), f = v3_dot(d2, r);
    float s, t;
    if (a <= 1e-8f && e <= 1e-8f) { *c1 = p1; *c2 = p2; return; }
    if (a <= 1e-8f) {
        s = 0.0f; t = HR_CLAMP(f / e, 0.0f, 1.0f);
    } else {
        float c = v3_dot(d1, r);
        if (e <= 1e-8f) {
            t = 0.0f; s = HR_CLAMP(-c / a, 0.0f, 1.0f);
        } else {
            float b = v3_dot(d1, d2);
            float denom = a * e - b * b;
            s = denom != 0.0f ? HR_CLAMP((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
            t = (b * s + f) / e;
            if (t < 0.0f) { t = 0.0f; s = HR_CLAMP(-c / a, 0.0f, 1.0f); }
            else if (t > 1.0f) { t = 1.0f; s = HR_CLAMP((b - c) / a, 0.0f, 1.0f); }
        }
    }
    *c1 = v3_add(p1, v3_scale(d1, s));
    *c2 = v3_add(p2, v3_scale(d2, t));
}

static void contact_capsule_sphere(HrPhysicsWorld* w, u32 icap, u32 isphere,
                                   const HrBody* cap, const HrBody* sphere,
                                   float restitution, float friction) {
    Vec3 axis = quat_rotate(cap->rot, v3(0.0f, 1.0f, 0.0f));
    Vec3 p0 = v3_add(cap->pos, v3_scale(axis, cap->shape.half_height));
    Vec3 p1 = v3_sub(cap->pos, v3_scale(axis, cap->shape.half_height));
    Vec3 closest = closest_point_on_segment(sphere->pos, p0, p1);
    contact_sphere_sphere(w, icap, isphere, closest, sphere->pos,
                          cap->shape.radius, sphere->shape.radius, restitution, friction);
}

static void contact_capsule_box(HrPhysicsWorld* w, u32 icap, u32 ibox,
                                const HrBody* cap, const HrBody* box,
                                float restitution, float friction) {
    Vec3 axis = quat_rotate(cap->rot, v3(0.0f, 1.0f, 0.0f));
    Vec3 p0 = v3_add(cap->pos, v3_scale(axis, cap->shape.half_height));
    Vec3 p1 = v3_sub(cap->pos, v3_scale(axis, cap->shape.half_height));
    /* approximate by finding closest segment point to box center, then sphere-box */
    Vec3 closest = closest_point_on_segment(box->pos, p0, p1);
    Quat inv = quat_conj(box->rot);
    Vec3 local = quat_rotate(inv, v3_sub(closest, box->pos));
    Vec3 h = box->shape.half_extents;
    Vec3 clamped = v3(HR_CLAMP(local.x, -h.x, h.x),
                      HR_CLAMP(local.y, -h.y, h.y),
                      HR_CLAMP(local.z, -h.z, h.z));
    Vec3 delta = v3_sub(local, clamped);
    float dist2 = v3_len_sq(delta);
    Vec3 n_local;
    float depth;
    Vec3 contact_local = clamped;
    float r = cap->shape.radius;
    if (dist2 > 1e-12f) {
        if (dist2 >= r * r) return;
        float dist = sqrtf(dist2);
        n_local = v3_scale(delta, 1.0f / dist);
        depth = r - dist;
    } else {
        float dx = h.x - fabsf(local.x);
        float dy = h.y - fabsf(local.y);
        float dz = h.z - fabsf(local.z);
        if (dx <= dy && dx <= dz) { n_local = v3(local.x < 0.0f ? -1.0f : 1.0f, 0, 0); depth = r + dx; }
        else if (dy <= dz) { n_local = v3(0, local.y < 0.0f ? -1.0f : 1.0f, 0); depth = r + dy; }
        else { n_local = v3(0, 0, local.z < 0.0f ? -1.0f : 1.0f); depth = r + dz; }
        contact_local = local;
    }
    Vec3 n_world = quat_rotate(box->rot, n_local);
    Vec3 point = v3_add(box->pos, quat_rotate(box->rot, contact_local));
    add_contact(w, ibox, icap, n_world, point, depth, restitution, friction);
}

static void contact_capsule_capsule(HrPhysicsWorld* w, u32 ia, u32 ib,
                                    const HrBody* a, const HrBody* b,
                                    float restitution, float friction) {
    Vec3 axA = quat_rotate(a->rot, v3(0, 1, 0));
    Vec3 axB = quat_rotate(b->rot, v3(0, 1, 0));
    Vec3 a0 = v3_add(a->pos, v3_scale(axA, a->shape.half_height));
    Vec3 a1 = v3_sub(a->pos, v3_scale(axA, a->shape.half_height));
    Vec3 b0 = v3_add(b->pos, v3_scale(axB, b->shape.half_height));
    Vec3 b1 = v3_sub(b->pos, v3_scale(axB, b->shape.half_height));
    Vec3 ca, cb;
    closest_points_segments(a0, a1, b0, b1, &ca, &cb);
    contact_sphere_sphere(w, ia, ib, ca, cb, a->shape.radius, b->shape.radius, restitution, friction);
}

static void contact_ground_sphere(HrPhysicsWorld* w, u32 ib, const HrBody* b,
                                  float ground_h, Vec3 ground_n,
                                  float restitution, float friction) {
    float dist = v3_dot(v3_sub(b->pos, v3(0, ground_h, 0)), ground_n) - b->shape.radius;
    if (dist >= 0.0f) return;
    float depth = -dist;
    Vec3 point = v3_sub(b->pos, v3_scale(ground_n, b->shape.radius - depth * 0.5f));
    add_contact(w, HR_PHYS_GROUND_SENTINEL, ib, ground_n, point, depth, restitution, friction);
}

static void contact_ground_box(HrPhysicsWorld* w, u32 ib, const HrBody* b,
                               Vec3 ground_point, Vec3 ground_n,
                               float restitution, float friction) {
    Mat4 m = mat4_from_quat(b->rot);
    Vec3 axes[3] = { v3(m.c[0][0], m.c[0][1], m.c[0][2]),
                     v3(m.c[1][0], m.c[1][1], m.c[1][2]),
                     v3(m.c[2][0], m.c[2][1], m.c[2][2]) };
    Vec3 h = b->shape.half_extents;
    typedef struct { Vec3 p; float depth; } Cand;
    Cand cands[8];
    int count = 0;
    for (int ix = -1; ix <= 1; ix += 2) {
        for (int iy = -1; iy <= 1; iy += 2) {
            for (int iz = -1; iz <= 1; iz += 2) {
                Vec3 local = v3(h.x * (float)ix, h.y * (float)iy, h.z * (float)iz);
                Vec3 world = v3_add(b->pos,
                    v3_add(v3_add(v3_scale(axes[0], local.x), v3_scale(axes[1], local.y)),
                           v3_scale(axes[2], local.z)));
                float d = v3_dot(v3_sub(world, ground_point), ground_n);
                if (d < 0.0f) { cands[count].p = world; cands[count].depth = -d; count++; }
            }
        }
    }
    for (int i = 0; i < count; ++i) {
        for (int j = i + 1; j < count; ++j) {
            if (cands[j].depth > cands[i].depth) { Cand t = cands[i]; cands[i] = cands[j]; cands[j] = t; }
        }
    }
    int emit = count < 4 ? count : 4;
    for (int i = 0; i < emit; ++i) {
        Vec3 point = v3_sub(cands[i].p, v3_scale(ground_n, cands[i].depth * 0.5f));
        add_contact(w, HR_PHYS_GROUND_SENTINEL, ib, ground_n, point, cands[i].depth, restitution, friction);
    }
}

static void contact_ground_capsule(HrPhysicsWorld* w, u32 ib, const HrBody* b,
                                   Vec3 ground_point, Vec3 ground_n,
                                   float restitution, float friction) {
    Vec3 axis = quat_rotate(b->rot, v3(0, 1, 0));
    Vec3 pts[2] = { v3_add(b->pos, v3_scale(axis, b->shape.half_height)),
                    v3_sub(b->pos, v3_scale(axis, b->shape.half_height)) };
    for (int i = 0; i < 2; ++i) {
        float d = v3_dot(v3_sub(pts[i], ground_point), ground_n) - b->shape.radius;
        if (d < 0.0f) {
            float depth = -d;
            Vec3 point = v3_sub(pts[i], v3_scale(ground_n, b->shape.radius - depth * 0.5f));
            add_contact(w, HR_PHYS_GROUND_SENTINEL, ib, ground_n, point, depth, restitution, friction);
        }
    }
}

static void generate_pair_contacts(HrPhysicsWorld* w, u32 ia, u32 ib) {
    HrBody* a = body_at(w, ia);
    HrBody* b = body_at(w, ib);
    if (!a || !b) return;
    if ((a->layer & b->mask) == 0u || (b->layer & a->mask) == 0u) return;
    if (a->inv_mass <= 0.0f && b->inv_mass <= 0.0f) return;
    if (is_kinematic(a) && is_kinematic(b)) return;
    if (!aabb_overlap(a, b)) return;

    w->broadphase_pairs++;
    float restitution = HR_MIN(a->restitution, b->restitution);
    float friction = sqrtf(HR_MAX(0.0f, a->friction * b->friction));

    /* wake sleeping body if other is moving */
    float va = v3_len_sq(a->vel) + v3_len_sq(a->angvel);
    float vb = v3_len_sq(b->vel) + v3_len_sq(b->angvel);
    if (a->sleeping && vb > 0.01f) wake_body(w, ia);
    if (b->sleeping && va > 0.01f) wake_body(w, ib);

    HrShapeType ta = a->shape.type, tb = b->shape.type;

    /* plane cases */
    if (ta == HR_SHAPE_PLANE && tb != HR_SHAPE_PLANE) {
        Vec3 pn = v3_norm(quat_rotate(a->rot, v3(0, 1, 0)));
        if (tb == HR_SHAPE_SPHERE) contact_plane_sphere(w, ia, ib, a->pos, pn, b, restitution, friction);
        else if (tb == HR_SHAPE_BOX) contact_plane_box(w, ia, ib, a->pos, pn, b, restitution, friction);
        else if (tb == HR_SHAPE_CAPSULE) contact_plane_capsule(w, ia, ib, a->pos, pn, b, restitution, friction);
        return;
    }
    if (tb == HR_SHAPE_PLANE && ta != HR_SHAPE_PLANE) {
        Vec3 pn = v3_norm(quat_rotate(b->rot, v3(0, 1, 0)));
        /* plane normal is b->a, but we want contact normal a->b = -pn */
        if (ta == HR_SHAPE_SPHERE) {
            /* create contacts with normal from plane(b) to sphere(a)? swap: treat b as plane, a as sphere, then flip */
            u32 before = (u32)w->contacts.len;
            contact_plane_sphere(w, ib, ia, b->pos, pn, a, restitution, friction);
            for (u32 i = before; i < (u32)w->contacts.len; ++i) {
                HrContact* c = &w->contacts.items[i];
                u32 tmp = c->a; c->a = c->b; c->b = tmp;
                c->normal = v3_neg(c->normal);
                c->tangent1 = v3_neg(c->tangent1);
            }
        } else if (ta == HR_SHAPE_BOX) {
            u32 before = (u32)w->contacts.len;
            contact_plane_box(w, ib, ia, b->pos, pn, a, restitution, friction);
            for (u32 i = before; i < (u32)w->contacts.len; ++i) {
                HrContact* c = &w->contacts.items[i];
                u32 tmp = c->a; c->a = c->b; c->b = tmp;
                c->normal = v3_neg(c->normal);
                c->tangent1 = v3_neg(c->tangent1);
            }
        } else if (ta == HR_SHAPE_CAPSULE) {
            u32 before = (u32)w->contacts.len;
            contact_plane_capsule(w, ib, ia, b->pos, pn, a, restitution, friction);
            for (u32 i = before; i < (u32)w->contacts.len; ++i) {
                HrContact* c = &w->contacts.items[i];
                u32 tmp = c->a; c->a = c->b; c->b = tmp;
                c->normal = v3_neg(c->normal);
                c->tangent1 = v3_neg(c->tangent1);
            }
        }
        return;
    }
    if (ta == HR_SHAPE_PLANE && tb == HR_SHAPE_PLANE) return;

    if (ta == HR_SHAPE_SPHERE && tb == HR_SHAPE_SPHERE) {
        contact_sphere_sphere(w, ia, ib, a->pos, b->pos, a->shape.radius, b->shape.radius, restitution, friction);
    } else if (ta == HR_SHAPE_SPHERE && tb == HR_SHAPE_BOX) {
        contact_sphere_box(w, ia, ib, a, b, restitution, friction);
    } else if (ta == HR_SHAPE_BOX && tb == HR_SHAPE_SPHERE) {
        contact_sphere_box(w, ib, ia, b, a, restitution, friction);
    } else if (ta == HR_SHAPE_BOX && tb == HR_SHAPE_BOX) {
        contact_box_box(w, ia, ib, a, b, restitution, friction);
    } else if (ta == HR_SHAPE_CAPSULE && tb == HR_SHAPE_SPHERE) {
        contact_capsule_sphere(w, ia, ib, a, b, restitution, friction);
    } else if (ta == HR_SHAPE_SPHERE && tb == HR_SHAPE_CAPSULE) {
        contact_capsule_sphere(w, ib, ia, b, a, restitution, friction);
    } else if (ta == HR_SHAPE_CAPSULE && tb == HR_SHAPE_BOX) {
        contact_capsule_box(w, ia, ib, a, b, restitution, friction);
    } else if (ta == HR_SHAPE_BOX && tb == HR_SHAPE_CAPSULE) {
        contact_capsule_box(w, ib, ia, b, a, restitution, friction);
    } else if (ta == HR_SHAPE_CAPSULE && tb == HR_SHAPE_CAPSULE) {
        contact_capsule_capsule(w, ia, ib, a, b, restitution, friction);
    }
}


/* ---------------------------------------------------------------- broadphase */

static void broadphase(HrPhysicsWorld* w) {
    for (u32 i = 0; i < w->grid_buckets; ++i) w->cells[i].len = 0;
    pair_set_clear(w);
    w->broadphase_pairs = 0;
    hr_da_clear(&w->contacts);

    for (u32 i = 0; i < w->capacity; ++i) {
        HrBody* b = &w->bodies[i];
        if (!b->alive) continue;
        update_aabb(b);
        if (b->shape.type == HR_SHAPE_PLANE) continue;
        grid_insert(w, i, b);
    }

    for (u32 cell = 0; cell < w->grid_buckets; ++cell) {
        HrU32List* items = &w->cells[cell];
        for (u32 i = 0; i < (u32)items->len; ++i) {
            for (u32 j = i + 1; j < (u32)items->len; ++j) {
                u32 ia = items->items[i];
                u32 ib = items->items[j];
                if (!pair_set_insert(w, ia, ib)) continue;
                generate_pair_contacts(w, ia, ib);
            }
        }
    }

    /* planes are infinite: test against all non-plane bodies */
    for (u32 i = 0; i < w->capacity; ++i) {
        HrBody* a = &w->bodies[i];
        if (!a->alive || a->shape.type != HR_SHAPE_PLANE) continue;
        for (u32 j = 0; j < w->capacity; ++j) {
            if (i == j) continue;
            HrBody* b = &w->bodies[j];
            if (!b->alive || b->shape.type == HR_SHAPE_PLANE) continue;
            if (!pair_set_insert(w, i, j)) continue;
            generate_pair_contacts(w, i, j);
        }
    }
}

static void ground_contacts(HrPhysicsWorld* w) {
    if (!w->ground_fn) return;
    for (u32 i = 0; i < w->capacity; ++i) {
        HrBody* b = &w->bodies[i];
        if (!b->alive) continue;
        if (b->inv_mass <= 0.0f) continue; /* static/kinematic */
        if (b->shape.type == HR_SHAPE_PLANE) continue;
        float h = w->ground_fn(w->ground_user, b->pos.x, b->pos.z);
        float eps = 0.5f;
        float hx = (w->ground_fn(w->ground_user, b->pos.x + eps, b->pos.z) -
                    w->ground_fn(w->ground_user, b->pos.x - eps, b->pos.z)) / (2.0f * eps);
        float hz = (w->ground_fn(w->ground_user, b->pos.x, b->pos.z + eps) -
                    w->ground_fn(w->ground_user, b->pos.x, b->pos.z - eps)) / (2.0f * eps);
        Vec3 n = v3_norm(v3(-hx, 1.0f, -hz));
        Vec3 gp = v3(b->pos.x, h, b->pos.z);
        float restitution = HR_MIN(b->restitution, 0.2f);
        float friction = HR_MAX(b->friction, 0.6f);
        if (b->shape.type == HR_SHAPE_SPHERE) {
            contact_ground_sphere(w, i, b, h, n, restitution, friction);
        } else if (b->shape.type == HR_SHAPE_BOX) {
            contact_ground_box(w, i, b, gp, n, restitution, friction);
        } else if (b->shape.type == HR_SHAPE_CAPSULE) {
            contact_ground_capsule(w, i, b, gp, n, restitution, friction);
        }
    }
}

/* ------------------------------------------------------------------ solver */

static float inv_mass_of(const HrPhysicsWorld* w, u32 index) {
    const HrBody* b = body_at_const(w, index);
    return b ? b->inv_mass : 0.0f;
}

static void apply_impulse_body(HrBody* b, Vec3 impulse, Vec3 r) {
    if (!b || b->inv_mass <= 0.0f) return;
    b->vel = v3_add(b->vel, v3_scale(impulse, b->inv_mass));
    Vec3 torque = v3_cross(r, impulse);
    b->angvel = v3_add(b->angvel, inv_inertia_mul(b, torque));
}

static float effective_mass(const HrPhysicsWorld* w, u32 ia, u32 ib,
                            Vec3 ra, Vec3 rb, Vec3 dir) {
    const HrBody* a = body_at_const(w, ia);
    const HrBody* b = body_at_const(w, ib);
    float k = inv_mass_of(w, ia) + inv_mass_of(w, ib);
    if (a) {
        Vec3 ra_x = v3_cross(ra, dir);
        Vec3 ia_ra_x = inv_inertia_mul(a, ra_x);
        k += v3_dot(dir, v3_cross(ia_ra_x, ra));
    }
    if (b) {
        Vec3 rb_x = v3_cross(rb, dir);
        Vec3 ib_rb_x = inv_inertia_mul(b, rb_x);
        k += v3_dot(dir, v3_cross(ib_rb_x, rb));
    }
    return k > 1e-9f ? k : 1e-9f;
}

static void solve_contacts(HrPhysicsWorld* w, float dt) {
    float inv_dt = dt > 1e-6f ? 1.0f / dt : 0.0f;
    for (u32 iter = 0; iter < w->cfg.solver_iterations; ++iter) {
        for (u32 ci = 0; ci < (u32)w->contacts.len; ++ci) {
            HrContact* c = &w->contacts.items[ci];
            HrBody* a = body_at(w, c->a);
            HrBody* b = body_at(w, c->b);
            if (!a && !b) continue;

            Vec3 pa = a ? a->pos : c->point;
            Vec3 pb = b ? b->pos : c->point;
            Vec3 ra = v3_sub(c->point, pa);
            Vec3 rb = v3_sub(c->point, pb);

            Vec3 va = a ? v3_add(a->vel, v3_cross(a->angvel, ra)) : v3(0, 0, 0);
            Vec3 vb = b ? v3_add(b->vel, v3_cross(b->angvel, rb)) : v3(0, 0, 0);
            Vec3 rv = v3_sub(vb, va);

            float vn = v3_dot(rv, c->normal);
            float kn = effective_mass(w, c->a, c->b, ra, rb, c->normal);
            float e = fabsf(vn) < w->cfg.restitution_cutoff ? 0.0f : c->restitution;
            float bias = w->cfg.correction * inv_dt * HR_MAX(c->depth - w->cfg.contact_slop, 0.0f);
            float jn = (-(1.0f + e) * vn + bias) / kn;
            float old_n = c->normal_impulse;
            c->normal_impulse = HR_MAX(old_n + jn, 0.0f);
            jn = c->normal_impulse - old_n;
            Vec3 Pn = v3_scale(c->normal, jn);
            if (a) apply_impulse_body(a, v3_neg(Pn), ra);
            if (b) apply_impulse_body(b, Pn, rb);

            /* friction */
            va = a ? v3_add(a->vel, v3_cross(a->angvel, ra)) : v3(0, 0, 0);
            vb = b ? v3_add(b->vel, v3_cross(b->angvel, rb)) : v3(0, 0, 0);
            rv = v3_sub(vb, va);
            float vt1 = v3_dot(rv, c->tangent1);
            float vt2 = v3_dot(rv, c->tangent2);
            float kt1 = effective_mass(w, c->a, c->b, ra, rb, c->tangent1);
            float kt2 = effective_mass(w, c->a, c->b, ra, rb, c->tangent2);
            float jt1 = -vt1 / kt1;
            float jt2 = -vt2 / kt2;
            float max_f = c->friction * c->normal_impulse;
            float old_t1 = c->tangent_impulse1;
            float old_t2 = c->tangent_impulse2;
            c->tangent_impulse1 = HR_CLAMP(old_t1 + jt1, -max_f, max_f);
            c->tangent_impulse2 = HR_CLAMP(old_t2 + jt2, -max_f, max_f);
            jt1 = c->tangent_impulse1 - old_t1;
            jt2 = c->tangent_impulse2 - old_t2;
            Vec3 Pf = v3_add(v3_scale(c->tangent1, jt1), v3_scale(c->tangent2, jt2));
            if (a) apply_impulse_body(a, v3_neg(Pf), ra);
            if (b) apply_impulse_body(b, Pf, rb);
        }
    }
}

static void positional_correction(HrPhysicsWorld* w) {
    for (u32 ci = 0; ci < (u32)w->contacts.len; ++ci) {
        const HrContact* c = &w->contacts.items[ci];
        HrBody* a = body_at(w, c->a);
        HrBody* b = body_at(w, c->b);
        float imA = a ? a->inv_mass : 0.0f;
        float imB = b ? b->inv_mass : 0.0f;
        float sum = imA + imB;
        if (sum <= 1e-9f) continue;
        float corr = HR_MAX(c->depth - w->cfg.contact_slop, 0.0f) * w->cfg.correction / sum;
        Vec3 move = v3_scale(c->normal, corr);
        if (a && imA > 0.0f) a->pos = v3_sub(a->pos, v3_scale(move, imA));
        if (b && imB > 0.0f) b->pos = v3_add(b->pos, v3_scale(move, imB));
    }
}

static void integrate(HrPhysicsWorld* w, float dt) {
    for (u32 i = 0; i < w->capacity; ++i) {
        HrBody* b = &w->bodies[i];
        if (!b->alive || b->sleeping) continue;
        bool dynamic = is_dynamic(b);
        bool kinematic = is_kinematic(b);
        if (!dynamic && !kinematic) continue;

        if (dynamic && (b->flags & HR_BODY_GRAVITY)) {
            b->vel = v3_add(b->vel, v3_scale(w->cfg.gravity, dt));
        }

        if (dynamic) {
            b->pos = v3_add(b->pos, v3_scale(b->vel, dt));
            float wl = v3_len(b->angvel);
            if (wl > 1e-8f) {
                float half = 0.5f * dt;
                Quat wq = {{b->angvel.x * half, b->angvel.y * half, b->angvel.z * half, 0.0f}};
                Quat dq = quat_mul(wq, b->rot);
                b->rot.x += dq.x; b->rot.y += dq.y; b->rot.z += dq.z; b->rot.w += dq.w;
                b->rot = quat_norm(b->rot);
            }
            b->vel = v3_scale(b->vel, 0.999f);
            b->angvel = v3_scale(b->angvel, 0.98f);
        }
        /* Kinematic bodies are moved by the caller; velocity is kept for contacts. */
    }
}

static void update_sleep(HrPhysicsWorld* w, float dt) {
    float sl2 = w->cfg.sleep_linear * w->cfg.sleep_linear;
    float sa2 = w->cfg.sleep_angular * w->cfg.sleep_angular;
    for (u32 i = 0; i < w->capacity; ++i) {
        HrBody* b = &w->bodies[i];
        if (!b->alive || !is_dynamic(b)) continue;
        if ((b->flags & HR_BODY_SLEEPABLE) == 0u) { b->sleeping = false; b->sleep_timer = 0; continue; }
        if (v3_len_sq(b->vel) < sl2 && v3_len_sq(b->angvel) < sa2) {
            b->sleep_timer += dt;
            if (b->sleep_timer >= w->cfg.sleep_time) {
                b->sleeping = true;
                b->vel = v3(0, 0, 0);
                b->angvel = v3(0, 0, 0);
            }
        } else {
            b->sleep_timer = 0.0f;
            b->sleeping = false;
        }
    }
}

/* -------------------------------------------------------------------- API */

bool hr_physics_create(HrPhysicsWorld** out, const HrPhysicsConfig* config) {
    if (!out) return false;
    HrPhysicsConfig cfg;
    if (config) cfg = *config;
    else hr_physics_default_config(&cfg);
    if (cfg.max_bodies == 0) cfg.max_bodies = 1024;
    if (cfg.max_bodies > 65535u) cfg.max_bodies = 65535u;
    if (cfg.solver_iterations == 0) cfg.solver_iterations = 8;
    if (cfg.cell_size < 0.1f) cfg.cell_size = 4.0f;

    HrPhysicsWorld* w = (HrPhysicsWorld*)hr_calloc(1, sizeof(HrPhysicsWorld));
    if (!w) return false;
    w->cfg = cfg;
    w->capacity = cfg.max_bodies;
    w->bodies = (HrBody*)hr_calloc(w->capacity, sizeof(HrBody));
    w->free_list = (u32*)hr_alloc((usize)w->capacity * sizeof(u32));
    if (!w->bodies || !w->free_list) { hr_physics_destroy(w); return false; }
    for (u32 i = 0; i < w->capacity; ++i) {
        w->bodies[i].generation = 1;
        w->free_list[i] = w->capacity - 1u - i;
    }
    w->free_count = w->capacity;

    w->grid_buckets = next_pow2(HR_MAX(64u, w->capacity));
    w->grid_mask = w->grid_buckets - 1u;
    w->cells = (HrU32List*)hr_calloc(w->grid_buckets, sizeof(HrU32List));
    if (!w->cells) { hr_physics_destroy(w); return false; }

    w->pair_capacity = next_pow2(HR_MAX(256u, w->capacity * 8u));
    w->pair_keys = (u64*)hr_calloc(w->pair_capacity, sizeof(u64));
    if (!w->pair_keys) { hr_physics_destroy(w); return false; }

    hr_da_init(&w->contacts);
    hr_da_reserve(&w->contacts, w->capacity * 4u);
    *out = w;
    return true;
}

void hr_physics_destroy(HrPhysicsWorld* w) {
    if (!w) return;
    if (w->cells) {
        for (u32 i = 0; i < w->grid_buckets; ++i) hr_free(w->cells[i].items);
        hr_free(w->cells);
    }
    hr_da_free(&w->contacts);
    hr_free(w->pair_keys);
    hr_free(w->bodies);
    hr_free(w->free_list);
    hr_free(w);
}

HrBodyHandle hr_physics_add_body(HrPhysicsWorld* w, const HrBodyDesc* desc) {
    if (!w || !desc || w->free_count == 0) return HR_BODY_INVALID;
    u32 index = w->free_list[--w->free_count];
    HrBody* b = &w->bodies[index];
    u32 generation = b->generation;
    memset(b, 0, sizeof(*b));
    b->shape = desc->shape;
    b->pos = desc->position;
    b->rot = quat_norm(desc->rotation);
    if (v3_len_sq(v3(b->rot.x, b->rot.y, b->rot.z)) < 1e-10f && fabsf(b->rot.w) < 1e-10f)
        b->rot = quat_identity();
    b->vel = desc->linear_velocity;
    b->angvel = desc->angular_velocity;
    b->restitution = HR_CLAMP(desc->restitution, 0.0f, 1.0f);
    b->friction = HR_MAX(0.0f, desc->friction);
    b->layer = desc->layer ? desc->layer : 1u;
    b->mask = desc->mask ? desc->mask : 0xffffffffu;
    b->flags = desc->flags;
    b->alive = true;
    b->generation = generation;
    b->sleeping = false;
    b->sleep_timer = 0.0f;

    bool is_static = (b->flags & HR_BODY_STATIC) != 0u || (desc->mass <= 0.0f && (b->flags & HR_BODY_KINEMATIC) == 0u);
    if (is_static) {
        b->inv_mass = 0.0f;
        b->flags |= HR_BODY_STATIC;
        b->flags &= ~HR_BODY_KINEMATIC;
    } else if ((b->flags & HR_BODY_KINEMATIC) != 0u) {
        b->inv_mass = 0.0f;
    } else {
        b->inv_mass = 1.0f / HR_MAX(desc->mass, 1e-6f);
    }
    if (b->shape.type == HR_SHAPE_PLANE) {
        b->inv_mass = 0.0f;
        b->flags |= HR_BODY_STATIC;
    }
    compute_inertia(b);
    update_aabb(b);
    w->live_count++;

    HrBodyHandle h = { index, generation };
    return h;
}

void hr_physics_remove_body(HrPhysicsWorld* w, HrBodyHandle handle) {
    if (!w || !hr_physics_body_alive(w, handle)) return;
    HrBody* b = &w->bodies[handle.index];
    b->alive = false;
    b->generation++;
    if (b->generation == 0) b->generation = 1;
    w->free_list[w->free_count++] = handle.index;
    w->live_count--;
}

bool hr_physics_body_alive(const HrPhysicsWorld* w, HrBodyHandle handle) {
    if (!w || handle.index >= w->capacity) return false;
    const HrBody* b = &w->bodies[handle.index];
    return b->alive && b->generation == handle.generation;
}

bool hr_physics_body_sleeping(const HrPhysicsWorld* w, HrBodyHandle handle) {
    if (!hr_physics_body_alive(w, handle)) return false;
    return w->bodies[handle.index].sleeping;
}

bool hr_physics_get_transform(const HrPhysicsWorld* w, HrBodyHandle handle,
                              Vec3* out_position, Quat* out_rotation) {
    if (!hr_physics_body_alive(w, handle)) return false;
    const HrBody* b = &w->bodies[handle.index];
    if (out_position) *out_position = b->pos;
    if (out_rotation) *out_rotation = b->rot;
    return true;
}

bool hr_physics_get_velocity(const HrPhysicsWorld* w, HrBodyHandle handle,
                             Vec3* out_linear, Vec3* out_angular) {
    if (!hr_physics_body_alive(w, handle)) return false;
    const HrBody* b = &w->bodies[handle.index];
    if (out_linear) *out_linear = b->vel;
    if (out_angular) *out_angular = b->angvel;
    return true;
}

void hr_physics_set_transform(HrPhysicsWorld* w, HrBodyHandle handle, Vec3 position, Quat rotation) {
    if (!hr_physics_body_alive(w, handle)) return;
    HrBody* b = &w->bodies[handle.index];
    b->pos = position;
    b->rot = quat_norm(rotation);
    update_aabb(b);
}

void hr_physics_set_velocity(HrPhysicsWorld* w, HrBodyHandle handle, Vec3 linear, Vec3 angular) {
    if (!hr_physics_body_alive(w, handle)) return;
    HrBody* b = &w->bodies[handle.index];
    b->vel = linear;
    b->angvel = angular;
    if (v3_len_sq(linear) > 1e-8f || v3_len_sq(angular) > 1e-8f) { b->sleeping = false; b->sleep_timer = 0; }
}

void hr_physics_apply_impulse(HrPhysicsWorld* w, HrBodyHandle handle, Vec3 impulse, Vec3 world_point) {
    if (!hr_physics_body_alive(w, handle)) return;
    HrBody* b = &w->bodies[handle.index];
    if (b->inv_mass <= 0.0f) return;
    b->sleeping = false;
    b->sleep_timer = 0.0f;
    Vec3 r = v3_sub(world_point, b->pos);
    apply_impulse_body(b, impulse, r);
}

void hr_physics_wake(HrPhysicsWorld* w, HrBodyHandle handle) {
    if (!hr_physics_body_alive(w, handle)) return;
    w->bodies[handle.index].sleeping = false;
    w->bodies[handle.index].sleep_timer = 0.0f;
}

void hr_physics_set_ground(HrPhysicsWorld* w, HrGroundHeightFn fn, void* user) {
    if (!w) return;
    w->ground_fn = fn;
    w->ground_user = user;
}

void hr_physics_step(HrPhysicsWorld* w, float dt) {
    if (!w || dt <= 0.0f) return;
    dt = HR_MIN(dt, 0.1f);
    const float max_sub = 1.0f / 120.0f;
    int steps = (int)ceilf(dt / max_sub);
    if (steps < 1) steps = 1;
    if (steps > 8) steps = 8;
    float sub_dt = dt / (float)steps;
    for (int s = 0; s < steps; ++s) {
        integrate(w, sub_dt);
        broadphase(w);
        ground_contacts(w);
        solve_contacts(w, sub_dt);
        positional_correction(w);
        update_sleep(w, sub_dt);
    }
    w->last_iterations = w->cfg.solver_iterations;
}

void hr_physics_get_stats(const HrPhysicsWorld* w, HrPhysicsStats* out) {
    if (!w || !out) return;
    memset(out, 0, sizeof(*out));
    out->bodies = w->live_count;
    for (u32 i = 0; i < w->capacity; ++i) {
        const HrBody* b = &w->bodies[i];
        if (b->alive && !b->sleeping) out->awake_bodies++;
    }
    out->broadphase_pairs = w->broadphase_pairs;
    out->contacts = (u32)w->contacts.len;
    out->solver_iterations = w->last_iterations;
}

/* ----------------------------------------------------------------- raycast */

static bool ray_sphere(Vec3 o, Vec3 d, float max_t, Vec3 center, float radius,
                       float* out_t, Vec3* out_n) {
    Vec3 oc = v3_sub(o, center);
    float b = v3_dot(oc, d);
    float c = v3_dot(oc, oc) - radius * radius;
    float disc = b * b - c;
    if (disc < 0.0f) return false;
    float s = sqrtf(disc);
    float t = -b - s;
    if (t < 0.0f) t = -b + s;
    if (t < 0.0f || t > max_t) return false;
    *out_t = t;
    *out_n = v3_norm(v3_sub(v3_add(o, v3_scale(d, t)), center));
    return true;
}

static bool ray_box(Vec3 o, Vec3 d, float max_t, const HrBody* b,
                    float* out_t, Vec3* out_n) {
    Quat inv = quat_conj(b->rot);
    Vec3 lo = quat_rotate(inv, v3_sub(o, b->pos));
    Vec3 ld = quat_rotate(inv, d);
    Vec3 h = b->shape.half_extents;
    float tmin = 0.0f, tmax = max_t;
    int axis = 0;
    float sign = 1.0f;
    for (int i = 0; i < 3; ++i) {
        float oi = lo.v[i], di = ld.v[i], hi = h.v[i];
        if (fabsf(di) < 1e-8f) {
            if (oi < -hi || oi > hi) return false;
        } else {
            float invd = 1.0f / di;
            float t1 = (-hi - oi) * invd;
            float t2 = ( hi - oi) * invd;
            float s = -1.0f;
            if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; s = 1.0f; }
            if (t1 > tmin) { tmin = t1; axis = i; sign = s; }
            if (t2 < tmax) tmax = t2;
            if (tmin > tmax) return false;
        }
    }
    if (tmin < 0.0f || tmin > max_t) return false;
    *out_t = tmin;
    Vec3 n_local = v3(0, 0, 0);
    n_local.v[axis] = sign;
    *out_n = quat_rotate(b->rot, n_local);
    return true;
}

static bool ray_plane(Vec3 o, Vec3 d, float max_t, Vec3 pp, Vec3 pn,
                      float* out_t, Vec3* out_n) {
    float denom = v3_dot(d, pn);
    if (fabsf(denom) < 1e-8f) return false;
    float t = v3_dot(v3_sub(pp, o), pn) / denom;
    if (t < 0.0f || t > max_t) return false;
    *out_t = t;
    *out_n = denom < 0.0f ? pn : v3_neg(pn);
    return true;
}

static bool ray_capsule(Vec3 o, Vec3 d, float max_t, const HrBody* b,
                        float* out_t, Vec3* out_n) {
    Quat inv = quat_conj(b->rot);
    Vec3 lo = quat_rotate(inv, v3_sub(o, b->pos));
    Vec3 ld = quat_rotate(inv, d);
    float r = b->shape.radius;
    float hh = b->shape.half_height;
    bool hit = false;
    float best_t = max_t;
    Vec3 best_n = v3(0, 1, 0);

    /* cylinder (infinite in Y, clipped to [-hh,hh]) */
    float a = ld.x * ld.x + ld.z * ld.z;
    float bb = 2.0f * (lo.x * ld.x + lo.z * ld.z);
    float c = lo.x * lo.x + lo.z * lo.z - r * r;
    if (fabsf(a) > 1e-8f) {
        float disc = bb * bb - 4.0f * a * c;
        if (disc >= 0.0f) {
            float s = sqrtf(disc);
            float t1 = (-bb - s) / (2.0f * a);
            float t2 = (-bb + s) / (2.0f * a);
            float ts[2] = { t1, t2 };
            for (int i = 0; i < 2; ++i) {
                float t = ts[i];
                if (t < 0.0f || t > best_t) continue;
                float y = lo.y + ld.y * t;
                if (y < -hh || y > hh) continue;
                Vec3 p = v3_add(lo, v3_scale(ld, t));
                Vec3 nl = v3_norm(v3(p.x, 0.0f, p.z));
                best_t = t; best_n = nl; hit = true;
            }
        }
    }

    /* caps */
    Vec3 caps[2] = { v3(0, hh, 0), v3(0, -hh, 0) };
    for (int i = 0; i < 2; ++i) {
        float t, dummy_t; Vec3 n;
        (void)dummy_t;
        if (ray_sphere(lo, ld, best_t, caps[i], r, &t, &n)) {
            best_t = t; best_n = n; hit = true;
        }
    }

    if (hit) {
        *out_t = best_t;
        *out_n = quat_rotate(b->rot, best_n);
        return true;
    }
    return false;
}

bool hr_physics_raycast(HrPhysicsWorld* w, Vec3 origin, Vec3 direction,
                        float max_distance, u32 layer_mask, HrRayHit* out_hit) {
    if (!w || !out_hit) return false;
    float dir_len = v3_len(direction);
    if (dir_len < 1e-8f) return false;
    Vec3 d = v3_scale(direction, 1.0f / dir_len);
    float max_t = max_distance / dir_len;
    bool hit = false;
    HrRayHit best;
    memset(&best, 0, sizeof(best));
    best.t = max_t;

    for (u32 i = 0; i < w->capacity; ++i) {
        HrBody* b = &w->bodies[i];
        if (!b->alive) continue;
        if ((layer_mask & b->layer) == 0u) continue;
        float t = 0.0f;
        Vec3 n = v3(0, 1, 0);
        bool bhit = false;
        if (b->shape.type == HR_SHAPE_SPHERE) {
            bhit = ray_sphere(origin, d, best.t, b->pos, b->shape.radius, &t, &n);
        } else if (b->shape.type == HR_SHAPE_BOX) {
            bhit = ray_box(origin, d, best.t, b, &t, &n);
        } else if (b->shape.type == HR_SHAPE_PLANE) {
            Vec3 pn = v3_norm(quat_rotate(b->rot, v3(0, 1, 0)));
            bhit = ray_plane(origin, d, best.t, b->pos, pn, &t, &n);
        } else if (b->shape.type == HR_SHAPE_CAPSULE) {
            bhit = ray_capsule(origin, d, best.t, b, &t, &n);
        }
        if (bhit && t < best.t) {
            hit = true;
            best.t = t;
            best.normal = n;
            best.point = v3_add(origin, v3_scale(d, t));
            best.body.index = i;
            best.body.generation = b->generation;
        }
    }
    if (hit) *out_hit = best;
    return hit;
}

bool hr_physics_overlap_sphere(const HrPhysicsWorld* w, Vec3 center, float radius,
                               u32 layer_mask, HrBodyHandle* out_body) {
    if (!w) return false;
    for (u32 i = 0; i < w->capacity; ++i) {
        const HrBody* b = &w->bodies[i];
        if (!b->alive) continue;
        if ((layer_mask & b->layer) == 0u) continue;
        bool overlap = false;
        if (b->shape.type == HR_SHAPE_SPHERE) {
            float rr = radius + b->shape.radius;
            overlap = v3_len_sq(v3_sub(center, b->pos)) <= rr * rr;
        } else if (b->shape.type == HR_SHAPE_PLANE) {
            Vec3 pn = v3_norm(quat_rotate(b->rot, v3(0, 1, 0)));
            overlap = fabsf(v3_dot(v3_sub(center, b->pos), pn)) <= radius;
        } else if (b->shape.type == HR_SHAPE_BOX) {
            Quat inv = quat_conj(b->rot);
            Vec3 local = quat_rotate(inv, v3_sub(center, b->pos));
            Vec3 h = b->shape.half_extents;
            Vec3 closest = v3(HR_CLAMP(local.x, -h.x, h.x),
                              HR_CLAMP(local.y, -h.y, h.y),
                              HR_CLAMP(local.z, -h.z, h.z));
            overlap = v3_len_sq(v3_sub(local, closest)) <= radius * radius;
        } else if (b->shape.type == HR_SHAPE_CAPSULE) {
            Vec3 axis = quat_rotate(b->rot, v3(0, 1, 0));
            Vec3 p0 = v3_add(b->pos, v3_scale(axis, b->shape.half_height));
            Vec3 p1 = v3_sub(b->pos, v3_scale(axis, b->shape.half_height));
            Vec3 closest = closest_point_on_segment(center, p0, p1);
            float rr = radius + b->shape.radius;
            overlap = v3_len_sq(v3_sub(center, closest)) <= rr * rr;
        }
        if (overlap) {
            if (out_body) { out_body->index = i; out_body->generation = b->generation; }
            return true;
        }
    }
    return false;
}
