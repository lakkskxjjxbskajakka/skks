/* Horizon Engine - rigid-body physics: uniform-grid broadphase, contact
 * manifolds, sequential-impulse solver, ray casting and heightfield ground. */
#include "horizon/physics.h"
#include "horizon/memory.h"
#include "horizon/containers.h"

#include <math.h>

#define HR_PHYS_MAX_CONTACTS_PER_PAIR 8
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

struct HrPhysicsWorld {
    HrPhysicsConfig cfg;
    HrBody* bodies;
    u32 capacity;
    u32* free_list;
    u32 free_count;
    u32 live_count;

    u32 grid_mask;
    u32 grid_buckets;
    HR_DA(u32)* cells;

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
    b->inv_inertia_local = v3(0, 0, 0);
    if (b->inv_mass <= 0.0f) return;
    float m = 1.0f / b->inv_mass;
    switch (b->shape.type) {
    case HR_SHAPE_SPHERE: {
        float i = 0.4f * m * b->shape.radius * b->shape.radius;
        if (i > 1e-9f) b->inv_inertia_local = v3(1.0f / i, 1.0f / i, 1.0f / i);
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
        float h = b->shape.half_height;
        float iy = 0.5f * m * r * r;
        float ix = (m / 12.0f) * (3.0f * r * r + 4.0f * h * h);
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
    if (!b) return v3(0, 0, 0);
    Vec3 local = quat_rotate(quat_conj(b->rot), v);
    local = v3_mul(local, b->inv_inertia_local);
    return quat_rotate(b->rot, local);
}

static void update_aabb(HrBody* b) {
    if (b->shape.type == HR_SHAPE_PLANE) {
        b->aabb_min = v3(-1e30f, -1e30f, -1e30f);
        b->aabb_max = v3(1e30f, 1e30f, 1e30f);
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
        Vec3 axis = quat_rotate(b->rot, v3(0, 1, 0));
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

/* ------------------------------------------------------------------ pairs */
static u64 pair_hash(u64 key) {
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
    u32 i = (u32)pair_hash(key) & mask;
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

static void cell_push(HrPhysicsWorld* w, u32 bucket, u32 body) {
    HR_DA(u32)* cell = &w->cells[bucket];
    hr_da_push(cell, body);
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
                cell_push(w, h & w->grid_mask, index);
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
    Vec3 up = fabsf(normal.y) < 0.9f ? v3(0, 1, 0) : v3(1, 0, 0);
    c.tangent1 = v3_norm(v3_cross(normal, up));
    c.tangent2 = v3_cross(normal, c.tangent1);
    hr_da_push(&w->contacts, c);
}

/* ------------------------------------------------------------ narrowphase */
static void sphere_sphere(HrPhysicsWorld* w, u32 ia, const HrBody* a,
                          u32 ib, const HrBody* b, Vec3 a_center, Vec3 b_center,
                          float ra, float rb, float restitution, float friction) {
    Vec3 d = v3_sub(b_center, a_center);
    float dist2 = v3_len_sq(d);
    float sum = ra + rb;
    if (dist2 >= sum * sum) return;
    float dist = sqrtf(dist2);
    Vec3 n = dist > 1e-6f ? v3_scale(d, 1.0f / dist) : v3(1, 0, 0);
    float depth = sum - dist;
    Vec3 point = v3_add(a_center, v3_scale(n, ra - depth * 0.5f));
    add_contact(w, ia, ib, n, point, depth, restitution, friction);
}

static void sphere_box(HrPhysicsWorld* w, u32 isphere, const HrBody* sphere,
                       u32 ibox, const HrBody* box, Vec3 sphere_center,
                       float radius, float restitution, float friction) {
    Quat inv = quat_conj(box->rot);
    Vec3 local = quat_rotate(inv, v3_sub(sphere_center, box->pos));
    Vec3 h = box->shape.half_extents;
    Vec3 closest = v3(HR_CLAMP(local.x, -h.x, h.x),
                      HR_CLAMP(local.y, -h.y, h.y),
                      HR_CLAMP(local.z, -h.z, h.z));
    Vec3 delta = v3_sub(local, closest);
    float dist2 = v3_len_sq(delta);
    Vec3 n_local;
    float depth;
    if (dist2 > 1e-12f) {
        if (dist2 >= radius * radius) return;
        float dist = sqrtf(dist2);
        n_local = v3_scale(delta, 1.0f / dist);
        depth = radius - dist;
    } else {
        /* centre inside the box: push out through the nearest face */
        float dx = h.x - fabsf(local.x);
        float dy = h.y - fabsf(local.y);
        float dz = h.z - fabsf(local.z);
        if (dx <= dy && dx <= dz) n_local = v3(local.x < 0.0f ? -1.0f : 1.0f, 0, 0);
        else if (dy <= dz) n_local = v3(0, local.y < 0.0f ? -1.0f : 1.0f, 0);
        else n_local = v3(0, 0, local.z < 0.0f ? -1.0f : 1.0f);
        depth = radius + HR_MIN(dx, HR_MIN(dy, dz));
        closest = v3_sub(local, v3_scale(n_local, HR_MIN(dx, HR_MIN(dy, dz))));
    }
    /* n_local points from box surface toward the sphere.  Contacts are stored
     * from a to b, so if a is the sphere the normal is -n_local. */
    Vec3 n_world = quat_rotate(box->rot, n_local);
    Vec3 point = v3_add(box->pos, quat_rotate(box->rot, closest));
    if (isphere == ibox) return;
    /* caller passes either (sphere, box) or (box, sphere) */
    HrBodyHandle dummy = { 0, 0 };
    (void)dummy;
    if (a_is_sphere_hack(isphere, ibox)) {
        add_contact(w, isphere, ibox, v3_neg(n_world), point, depth, restitution, friction);
    } else {
        add_contact(w, ibox, isphere, n_world, point, depth, restitution, friction);
    }
}
