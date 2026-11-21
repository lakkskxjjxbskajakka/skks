#include "horizon/physics.h"
#include "horizon/memory.h"

#include <float.h>

#define HR_PHYS_MAX_BODIES 65535u
#define HR_PHYS_SLEEP_EPS 0.0f

typedef struct PhysBody {
    HrBodyState s;
    u32 generation;
    Vec3 force;
    float inv_inertia;
} PhysBody;

typedef struct PhysContact {
    u32 a, b;
    Vec3 point;
    Vec3 normal;      /* from a to b */
    float penetration;
    float normal_impulse;
    Vec3 t1, t2;
    float t1_impulse, t2_impulse;
    float normal_mass;
    float t1_mass, t2_mass;
    float friction;
    float bias;
} PhysContact;

typedef struct PhysProxy {
    u32 index;
    Vec3 min, max;
} PhysProxy;

struct HrPhysicsWorld {
    PhysBody* bodies;
    u32 capacity;
    u32 live;
    u32* free_list;
    u32 free_count;
    Vec3 gravity;

    PhysContact* contacts;
    u32 contact_count;
    u32 contact_cap;

    PhysProxy* proxies;
    u32 proxy_cap;
};

/* ------------------------------------------------------------------ helpers */
static Quat quat_conj(Quat q) { return (Quat){{-q.x, -q.y, -q.z, q.w}}; }

static Vec3 quat_rotate_inv(Quat q, Vec3 v) { return quat_rotate(quat_conj(q), v); }

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

static bool decode(const HrPhysicsWorld* w, u32 handle, u32* index) {
    if (handle == 0) return false;
    u32 i = (handle & 0xFFFFu) - 1u;
    u32 gen = handle >> 16;
    if (i >= w->capacity) return false;
    const PhysBody* b = &w->bodies[i];
    if (!b->s.active || (b->generation & 0xFFFFu) != gen) return false;
    *index = i;
    return true;
}

static u32 encode(const PhysBody* b, u32 index) {
    return ((b->generation & 0xFFFFu) << 16) | (index + 1u);
}

static bool layer_match(const PhysBody* a, const PhysBody* b) {
    return (a->s.layer & b->s.mask) != 0 && (b->s.layer & a->s.mask) != 0;
}

static bool is_dynamic(const PhysBody* b) { return b->s.type == HR_BODY_DYNAMIC; }

static Vec3 relative_velocity(const PhysBody* a, const PhysBody* b, Vec3 ra, Vec3 rb) {
    Vec3 va = v3_add(a->s.linear_velocity, v3_cross(a->s.angular_velocity, ra));
    Vec3 vb = v3_add(b->s.linear_velocity, v3_cross(b->s.angular_velocity, rb));
    return v3_sub(vb, va);
}

static void apply_impulse_at(PhysBody* b, Vec3 impulse, Vec3 r) {
    if (b->s.inv_mass == 0.0f && b->inv_inertia == 0.0f) return;
    b->s.linear_velocity = v3_add(b->s.linear_velocity, v3_scale(impulse, b->s.inv_mass));
    Vec3 torque = v3_cross(r, impulse);
    b->s.angular_velocity = v3_add(b->s.angular_velocity,
                                   v3_scale(torque, b->inv_inertia));
}

/* ------------------------------------------------------------------- setup */
HrPhysicsWorld* hr_physics_create(Vec3 gravity, u32 capacity) {
    if (capacity == 0) capacity = 64;
    if (capacity > HR_PHYS_MAX_BODIES) capacity = HR_PHYS_MAX_BODIES;

    HrPhysicsWorld* w = (HrPhysicsWorld*)hr_calloc(1, sizeof(HrPhysicsWorld));
    if (!w) return NULL;
    w->bodies = (PhysBody*)hr_calloc(capacity, sizeof(PhysBody));
    w->free_list = (u32*)hr_calloc(capacity, sizeof(u32));
    w->proxy_cap = capacity;
    w->proxies = (PhysProxy*)hr_calloc(w->proxy_cap, sizeof(PhysProxy));
    w->contact_cap = capacity * 4u + 32u;
    w->contacts = (PhysContact*)hr_calloc(w->contact_cap, sizeof(PhysContact));
    if (!w->bodies || !w->free_list || !w->proxies || !w->contacts) {
        hr_physics_destroy(w);
        return NULL;
    }
    w->capacity = capacity;
    w->gravity = gravity;
    for (u32 i = 0; i < capacity; ++i) {
        w->free_list[i] = capacity - 1u - i;
        w->bodies[i].s.active = false;
    }
    w->free_count = capacity;
    HR_INFO("physics", "world created: capacity=%u gravity=(%.1f %.1f %.1f)",
            capacity, gravity.x, gravity.y, gravity.z);
    return w;
}

void hr_physics_destroy(HrPhysicsWorld* w) {
    if (!w) return;
    hr_free(w->bodies);
    hr_free(w->free_list);
    hr_free(w->proxies);
    hr_free(w->contacts);
    hr_free(w);
}

void hr_physics_clear(HrPhysicsWorld* w) {
    if (!w) return;
    for (u32 i = 0; i < w->capacity; ++i) {
        if (w->bodies[i].s.active) {
            w->bodies[i].s.active = false;
            w->bodies[i].generation++;
            w->free_list[w->free_count++] = i;
        }
    }
    w->live = 0;
    w->contact_count = 0;
}

static float compute_inv_inertia(const HrBodyState* s, float mass) {
    if (mass <= 0.0f) return 0.0f;
    float I = 0.0f;
    if (s->shape == HR_SHAPE_SPHERE) {
        I = 0.4f * mass * s->radius * s->radius;
    } else if (s->shape == HR_SHAPE_BOX) {
        const Vec3 h = s->half_extents;
        I = (2.0f * mass / 9.0f) * (h.x * h.x + h.y * h.y + h.z * h.z);
    }
    return I > 1e-9f ? 1.0f / I : 0.0f;
}

u32 hr_physics_add_body(HrPhysicsWorld* w, const HrBodyDesc* desc) {
    if (!w || !desc || w->free_count == 0) return 0;
    u32 index = w->free_list[--w->free_count];
    PhysBody* b = &w->bodies[index];
    memset(b, 0, sizeof(*b));

    HrBodyState* s = &b->s;
    s->type = desc->type;
    s->shape = desc->shape;
    if (s->shape == HR_SHAPE_PLANE) s->type = HR_BODY_STATIC;
    s->position = desc->position;
    s->rotation = quat_norm(desc->rotation);
    s->linear_velocity = desc->linear_velocity;
    s->angular_velocity = desc->angular_velocity;
    s->half_extents = desc->half_extents;
    if (s->half_extents.x <= 0.0f) s->half_extents.x = 0.5f;
    if (s->half_extents.y <= 0.0f) s->half_extents.y = 0.5f;
    if (s->half_extents.z <= 0.0f) s->half_extents.z = 0.5f;
    s->radius = desc->radius > 0.0f ? desc->radius : 0.5f;
    s->plane_normal = v3_len_sq(desc->plane_normal) > 1e-12f
                          ? v3_norm(desc->plane_normal)
                          : v3(0, 1, 0);
    s->mass = desc->mass > 0.0f ? desc->mass : 1.0f;
    s->restitution = clampf(desc->restitution, 0.0f, 1.0f);
    s->friction = desc->friction > 0.0f ? desc->friction : 0.5f;
    s->layer = desc->layer ? desc->layer : 1u;
    s->mask = desc->mask ? desc->mask : 0xFFFFFFFFu;
    s->user = desc->user;
    s->active = true;
    s->inv_mass = is_dynamic(b) ? 1.0f / s->mass : 0.0f;
    b->inv_inertia = is_dynamic(b) ? compute_inv_inertia(s, s->mass) : 0.0f;
    b->generation = (b->generation + 1u) & 0xFFFFu;
    if (b->generation == 0) b->generation = 1;
    w->live++;
    return encode(b, index);
}

void hr_physics_remove_body(HrPhysicsWorld* w, u32 body) {
    u32 i;
    if (!w || !decode(w, body, &i)) return;
    PhysBody* b = &w->bodies[i];
    b->s.active = false;
    b->generation = (b->generation + 1u) & 0xFFFFu;
    if (b->generation == 0) b->generation = 1;
    w->free_list[w->free_count++] = i;
    if (w->live > 0) w->live--;
}

bool hr_physics_get_body(const HrPhysicsWorld* w, u32 body, HrBodyState* out) {
    u32 i;
    if (!w || !decode(w, body, &i)) return false;
    if (out) *out = w->bodies[i].s;
    return true;
}

bool hr_physics_set_transform(HrPhysicsWorld* w, u32 body, Vec3 position, Quat rotation) {
    u32 i;
    if (!w || !decode(w, body, &i)) return false;
    w->bodies[i].s.position = position;
    w->bodies[i].s.rotation = quat_norm(rotation);
    return true;
}

bool hr_physics_set_linear_velocity(HrPhysicsWorld* w, u32 body, Vec3 v) {
    u32 i;
    if (!w || !decode(w, body, &i)) return false;
    w->bodies[i].s.linear_velocity = v;
    return true;
}

bool hr_physics_set_angular_velocity(HrPhysicsWorld* w, u32 body, Vec3 v) {
    u32 i;
    if (!w || !decode(w, body, &i)) return false;
    w->bodies[i].s.angular_velocity = v;
    return true;
}

void hr_physics_apply_impulse(HrPhysicsWorld* w, u32 body, Vec3 impulse, Vec3 at_world) {
    u32 i;
    if (!w || !decode(w, body, &i)) return;
    PhysBody* b = &w->bodies[i];
    apply_impulse_at(b, impulse, v3_sub(at_world, b->s.position));
}

void hr_physics_set_gravity(HrPhysicsWorld* w, Vec3 g) { if (w) w->gravity = g; }
Vec3 hr_physics_gravity(const HrPhysicsWorld* w) {
    return w ? w->gravity : v3(0, -9.81f, 0);
}
u32 hr_physics_body_count(const HrPhysicsWorld* w) { return w ? w->live : 0; }
u32 hr_physics_contact_count(const HrPhysicsWorld* w) { return w ? w->contact_count : 0; }

u32 hr_physics_active_count(const HrPhysicsWorld* w) {
    if (!w) return 0;
    u32 n = 0;
    for (u32 i = 0; i < w->capacity; ++i) {
        const PhysBody* b = &w->bodies[i];
        if (!b->s.active || b->s.type != HR_BODY_DYNAMIC) continue;
        if (v3_len_sq(b->s.linear_velocity) > 1e-4f ||
            v3_len_sq(b->s.angular_velocity) > 1e-4f) n++;
    }
    return n;
}

/* --------------------------------------------------------------- contacts */
static bool ensure_contact_capacity(HrPhysicsWorld* w, u32 need) {
    if (need <= w->contact_cap) return true;
    u32 cap = w->contact_cap ? w->contact_cap : 32u;
    while (cap < need) cap *= 2u;
    PhysContact* nc = (PhysContact*)hr_realloc(w->contacts, (usize)cap * sizeof(PhysContact));
    if (!nc) return false;
    w->contacts = nc;
    w->contact_cap = cap;
    return true;
}

static void add_contact(HrPhysicsWorld* w, u32 a, u32 b, Vec3 point, Vec3 normal,
                        float penetration) {
    if (penetration <= 0.0f) return;
    if (!ensure_contact_capacity(w, w->contact_count + 1u)) return;
    PhysContact* c = &w->contacts[w->contact_count++];
    memset(c, 0, sizeof(*c));
    c->a = a;
    c->b = b;
    c->point = point;
    c->normal = v3_norm(normal);
    c->penetration = penetration;
}

static void collide_sphere_sphere(HrPhysicsWorld* w, u32 ia, u32 ib) {
    const PhysBody* a = &w->bodies[ia];
    const PhysBody* b = &w->bodies[ib];
    Vec3 d = v3_sub(b->s.position, a->s.position);
    float dist2 = v3_len_sq(d);
    float r = a->s.radius + b->s.radius;
    if (dist2 > r * r) return;
    float dist = sqrtf(dist2);
    Vec3 n = dist > 1e-6f ? v3_scale(d, 1.0f / dist) : v3(1, 0, 0);
    Vec3 point = v3_add(a->s.position, v3_scale(n, a->s.radius - (r - dist) * 0.5f));
    add_contact(w, ia, ib, point, n, r - dist);
}

static void collide_sphere_box(HrPhysicsWorld* w, u32 si, u32 bi) {
    const PhysBody* s = &w->bodies[si];
    const PhysBody* b = &w->bodies[bi];
    Vec3 local = quat_rotate_inv(b->s.rotation, v3_sub(s->s.position, b->s.position));
    const Vec3 h = b->s.half_extents;
    Vec3 closest = v3(clampf(local.x, -h.x, h.x),
                      clampf(local.y, -h.y, h.y),
                      clampf(local.z, -h.z, h.z));
    Vec3 delta = v3_sub(local, closest);
    float d2 = v3_len_sq(delta);
    float r = s->s.radius;
    if (d2 > r * r) return;

    Vec3 n_local;
    float pen;
    if (d2 > 1e-12f) {
        float d = sqrtf(d2);
        n_local = v3_scale(delta, 1.0f / d);
        pen = r - d;
    } else {
        float dx = h.x - fabsf(local.x);
        float dy = h.y - fabsf(local.y);
        float dz = h.z - fabsf(local.z);
        float m = dx;
        Vec3 axis = v3(1, 0, 0);
        if (dy < m) { m = dy; axis = v3(0, 1, 0); }
        if (dz < m) { m = dz; axis = v3(0, 0, 1); }
        n_local = v3_scale(axis, local.x * axis.x + local.y * axis.y + local.z * axis.z < 0.0f ? -1.0f : 1.0f);
        pen = r + m;
    }
    Vec3 n_box_to_sphere = quat_rotate(b->s.rotation, n_local);
    Vec3 point = v3_add(b->s.position, quat_rotate(b->s.rotation, closest));
    add_contact(w, si, bi, point, v3_neg(n_box_to_sphere), pen);
}

static void collide_box_box(HrPhysicsWorld* w, u32 ia, u32 ib) {
    const PhysBody* a = &w->bodies[ia];
    const PhysBody* b = &w->bodies[ib];
    Vec3 ax[3] = {
        quat_rotate(a->s.rotation, v3(1, 0, 0)),
        quat_rotate(a->s.rotation, v3(0, 1, 0)),
        quat_rotate(a->s.rotation, v3(0, 0, 1))
    };
    Vec3 bx[3] = {
        quat_rotate(b->s.rotation, v3(1, 0, 0)),
        quat_rotate(b->s.rotation, v3(0, 1, 0)),
        quat_rotate(b->s.rotation, v3(0, 0, 1))
    };
    float ha[3] = { a->s.half_extents.x, a->s.half_extents.y, a->s.half_extents.z };
    float hb[3] = { b->s.half_extents.x, b->s.half_extents.y, b->s.half_extents.z };
    Vec3 d = v3_sub(b->s.position, a->s.position);

    Vec3 candidates[15];
    int n = 0;
    for (int i = 0; i < 3; ++i) candidates[n++] = ax[i];
    for (int i = 0; i < 3; ++i) candidates[n++] = bx[i];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) candidates[n++] = v3_cross(ax[i], bx[j]);

    float min_overlap = FLT_MAX;
    Vec3 best = v3(0, 1, 0);
    for (int k = 0; k < n; ++k) {
        Vec3 axis = candidates[k];
        float len = v3_len(axis);
        if (len < 1e-6f) continue;
        axis = v3_scale(axis, 1.0f / len);
        float ra = fabsf(v3_dot(axis, ax[0])) * ha[0] +
                   fabsf(v3_dot(axis, ax[1])) * ha[1] +
                   fabsf(v3_dot(axis, ax[2])) * ha[2];
        float rb = fabsf(v3_dot(axis, bx[0])) * hb[0] +
                   fabsf(v3_dot(axis, bx[1])) * hb[1] +
                   fabsf(v3_dot(axis, bx[2])) * hb[2];
        float dist = fabsf(v3_dot(d, axis));
        float overlap = ra + rb - dist;
        if (overlap < 0.0f) return;
        if (overlap < min_overlap) {
            min_overlap = overlap;
            best = axis;
        }
    }
    if (v3_dot(d, best) < 0.0f) best = v3_neg(best);

    Vec3 support_a = a->s.position;
    for (int i = 0; i < 3; ++i)
        support_a = v3_add(support_a, v3_scale(ax[i],
            (v3_dot(best, ax[i]) >= 0.0f ? 1.0f : -1.0f) * ha[i]));
    Vec3 support_b = b->s.position;
    for (int i = 0; i < 3; ++i)
        support_b = v3_add(support_b, v3_scale(bx[i],
            (v3_dot(best, bx[i]) >= 0.0f ? -1.0f : 1.0f) * hb[i]));
    Vec3 point = v3_scale(v3_add(support_a, support_b), 0.5f);
    add_contact(w, ia, ib, point, best, min_overlap);
}

static void collide_sphere_plane(HrPhysicsWorld* w, u32 si, u32 pi) {
    const PhysBody* s = &w->bodies[si];
    const PhysBody* p = &w->bodies[pi];
    Vec3 n = p->s.plane_normal;
    float dist = v3_dot(v3_sub(s->s.position, p->s.position), n);
    if (dist >= s->s.radius) return;
    Vec3 point = v3_sub(s->s.position, v3_scale(n, dist));
    add_contact(w, si, pi, point, v3_neg(n), s->s.radius - dist);
}

static void collide_box_plane(HrPhysicsWorld* w, u32 bi, u32 pi) {
    const PhysBody* b = &w->bodies[bi];
    const PhysBody* p = &w->bodies[pi];
    Vec3 n = p->s.plane_normal;
    const Vec3 h = b->s.half_extents;
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                Vec3 local = v3((float)sx * h.x, (float)sy * h.y, (float)sz * h.z);
                Vec3 corner = v3_add(b->s.position, quat_rotate(b->s.rotation, local));
                float dist = v3_dot(v3_sub(corner, p->s.position), n);
                if (dist < 0.0f) add_contact(w, bi, pi, corner, v3_neg(n), -dist);
            }
        }
    }
}

static void collide_pair(HrPhysicsWorld* w, u32 ia, u32 ib) {
    PhysBody* a = &w->bodies[ia];
    PhysBody* b = &w->bodies[ib];
    if (!a->s.active || !b->s.active) return;
    if (a->s.type != HR_BODY_DYNAMIC && b->s.type != HR_BODY_DYNAMIC) return;
    if (!layer_match(a, b)) return;

    HrShapeType sa = a->s.shape, sb = b->s.shape;
    if (sa == HR_SHAPE_SPHERE && sb == HR_SHAPE_SPHERE) collide_sphere_sphere(w, ia, ib);
    else if (sa == HR_SHAPE_SPHERE && sb == HR_SHAPE_BOX) collide_sphere_box(w, ia, ib);
    else if (sa == HR_SHAPE_BOX && sb == HR_SHAPE_SPHERE) collide_sphere_box(w, ib, ia);
    else if (sa == HR_SHAPE_BOX && sb == HR_SHAPE_BOX) collide_box_box(w, ia, ib);
    else if (sa == HR_SHAPE_SPHERE && sb == HR_SHAPE_PLANE) collide_sphere_plane(w, ia, ib);
    else if (sa == HR_SHAPE_BOX && sb == HR_SHAPE_PLANE) collide_box_plane(w, ia, ib);
    else if (sa == HR_SHAPE_PLANE && sb == HR_SHAPE_SPHERE) collide_sphere_plane(w, ib, ia);
    else if (sa == HR_SHAPE_PLANE && sb == HR_SHAPE_BOX) collide_box_plane(w, ib, ia);
}

/* --------------------------------------------------------------- broadphase */
static Vec3 body_extents(const PhysBody* b) {
    if (b->s.shape == HR_SHAPE_SPHERE) return v3s(b->s.radius);
    const Vec3 h = b->s.half_extents;
    Vec3 e = v3_add(v3_add(
        v3_abs(quat_rotate(b->s.rotation, v3(h.x, 0, 0))),
        v3_abs(quat_rotate(b->s.rotation, v3(0, h.y, 0)))),
        v3_abs(quat_rotate(b->s.rotation, v3(0, 0, h.z))));
    return e;
}

static int proxy_cmp(const void* pa, const void* pb) {
    const PhysProxy* a = (const PhysProxy*)pa;
    const PhysProxy* b = (const PhysProxy*)pb;
    if (a->min.x < b->min.x) return -1;
    if (a->min.x > b->min.x) return 1;
    return 0;
}

static void build_contacts(HrPhysicsWorld* w) {
    w->contact_count = 0;

    /* Plane bodies are infinite; collide them directly against every body. */
    for (u32 i = 0; i < w->capacity; ++i) {
        PhysBody* p = &w->bodies[i];
        if (!p->s.active || p->s.shape != HR_SHAPE_PLANE) continue;
        for (u32 j = 0; j < w->capacity; ++j) {
            PhysBody* b = &w->bodies[j];
            if (!b->s.active || b == p || b->s.shape == HR_SHAPE_PLANE) continue;
            collide_pair(w, j, i);
        }
    }

    /* Sweep-and-prune over finite shapes. */
    u32 n = 0;
    for (u32 i = 0; i < w->capacity; ++i) {
        PhysBody* b = &w->bodies[i];
        if (!b->s.active || b->s.shape == HR_SHAPE_PLANE) continue;
        Vec3 e = body_extents(b);
        PhysProxy* pr = &w->proxies[n++];
        pr->index = i;
        pr->min = v3_sub(b->s.position, e);
        pr->max = v3_add(b->s.position, e);
    }
    if (n < 2) return;
    qsort(w->proxies, n, sizeof(PhysProxy), proxy_cmp);

    for (u32 i = 0; i < n; ++i) {
        const PhysProxy* a = &w->proxies[i];
        for (u32 j = i + 1; j < n; ++j) {
            const PhysProxy* b = &w->proxies[j];
            if (b->min.x > a->max.x) break;
            if (a->min.y > b->max.y || b->min.y > a->max.y) continue;
            if (a->min.z > b->max.z || b->min.z > a->max.z) continue;
            collide_pair(w, a->index, b->index);
        }
    }
}

/* ------------------------------------------------------------------ solver */
static void prepare_contacts(HrPhysicsWorld* w) {
    for (u32 k = 0; k < w->contact_count; ++k) {
        PhysContact* c = &w->contacts[k];
        PhysBody* a = &w->bodies[c->a];
        PhysBody* b = &w->bodies[c->b];
        Vec3 ra = v3_sub(c->point, a->s.position);
        Vec3 rb = v3_sub(c->point, b->s.position);
        Vec3 n = c->normal;

        float rna = v3_len_sq(v3_cross(ra, n));
        float rnb = v3_len_sq(v3_cross(rb, n));
        float denom = a->s.inv_mass + b->s.inv_mass +
                      a->inv_inertia * rna + b->inv_inertia * rnb;
        c->normal_mass = denom > 1e-12f ? 1.0f / denom : 0.0f;

        Vec3 ref = fabsf(n.x) < 0.9f ? v3(1, 0, 0) : v3(0, 1, 0);
        c->t1 = v3_norm(v3_cross(n, ref));
        c->t2 = v3_cross(n, c->t1);
        float rta1 = v3_len_sq(v3_cross(ra, c->t1));
        float rtb1 = v3_len_sq(v3_cross(rb, c->t1));
        float dt1 = a->s.inv_mass + b->s.inv_mass +
                    a->inv_inertia * rta1 + b->inv_inertia * rtb1;
        c->t1_mass = dt1 > 1e-12f ? 1.0f / dt1 : 0.0f;
        float rta2 = v3_len_sq(v3_cross(ra, c->t2));
        float rtb2 = v3_len_sq(v3_cross(rb, c->t2));
        float dt2 = a->s.inv_mass + b->s.inv_mass +
                    a->inv_inertia * rta2 + b->inv_inertia * rtb2;
        c->t2_mass = dt2 > 1e-12f ? 1.0f / dt2 : 0.0f;

        c->friction = sqrtf(a->s.friction * b->s.friction);
        float e = 0.5f * (a->s.restitution + b->s.restitution);
        Vec3 rel = relative_velocity(a, b, ra, rb);
        float vn = v3_dot(rel, n);
        c->bias = vn < -0.5f ? -e * vn : 0.0f;
    }
}

static void solve_contacts(HrPhysicsWorld* w, int iterations) {
    for (int it = 0; it < iterations; ++it) {
        for (u32 k = 0; k < w->contact_count; ++k) {
            PhysContact* c = &w->contacts[k];
            PhysBody* a = &w->bodies[c->a];
            PhysBody* b = &w->bodies[c->b];
            Vec3 ra = v3_sub(c->point, a->s.position);
            Vec3 rb = v3_sub(c->point, b->s.position);
            Vec3 n = c->normal;

            Vec3 rel = relative_velocity(a, b, ra, rb);
            float vn = v3_dot(rel, n);
            float dj = (c->bias - vn) * c->normal_mass;
            float old = c->normal_impulse;
            c->normal_impulse = HR_MAX(old + dj, 0.0f);
            dj = c->normal_impulse - old;
            Vec3 imp = v3_scale(n, dj);
            apply_impulse_at(a, v3_neg(imp), ra);
            apply_impulse_at(b, imp, rb);

            float max_f = c->friction * c->normal_impulse;
            rel = relative_velocity(a, b, ra, rb);
            float vt1 = v3_dot(rel, c->t1);
            float jt1 = -vt1 * c->t1_mass;
            float old1 = c->t1_impulse;
            c->t1_impulse = clampf(old1 + jt1, -max_f, max_f);
            jt1 = c->t1_impulse - old1;
            imp = v3_scale(c->t1, jt1);
            apply_impulse_at(a, v3_neg(imp), ra);
            apply_impulse_at(b, imp, rb);

            rel = relative_velocity(a, b, ra, rb);
            float vt2 = v3_dot(rel, c->t2);
            float jt2 = -vt2 * c->t2_mass;
            float old2 = c->t2_impulse;
            c->t2_impulse = clampf(old2 + jt2, -max_f, max_f);
            jt2 = c->t2_impulse - old2;
            imp = v3_scale(c->t2, jt2);
            apply_impulse_at(a, v3_neg(imp), ra);
            apply_impulse_at(b, imp, rb);
        }
    }
}

static void correct_positions(HrPhysicsWorld* w) {
    for (int it = 0; it < 2; ++it) {
        for (u32 k = 0; k < w->contact_count; ++k) {
            PhysContact* c = &w->contacts[k];
            PhysBody* a = &w->bodies[c->a];
            PhysBody* b = &w->bodies[c->b];
            float total = a->s.inv_mass + b->s.inv_mass;
            if (total <= 0.0f) continue;
            float corr = HR_MAX(c->penetration - 0.005f, 0.0f) * 0.6f;
            if (corr <= 0.0f) continue;
            Vec3 move = v3_scale(c->normal, corr / total);
            a->s.position = v3_sub(a->s.position, v3_scale(move, a->s.inv_mass));
            b->s.position = v3_add(b->s.position, v3_scale(move, b->s.inv_mass));
            c->penetration = HR_MAX(c->penetration - corr, 0.0f);
        }
    }
}

static void integrate(HrPhysicsWorld* w, float dt) {
    for (u32 i = 0; i < w->capacity; ++i) {
        PhysBody* b = &w->bodies[i];
        if (!b->s.active || b->s.type == HR_BODY_STATIC) continue;

        if (b->s.type == HR_BODY_DYNAMIC) {
            Vec3 accel = v3_add(w->gravity, v3_scale(b->force, b->s.inv_mass));
            b->s.linear_velocity = v3_add(b->s.linear_velocity, v3_scale(accel, dt));
            b->s.linear_velocity = v3_scale(b->s.linear_velocity, 1.0f / (1.0f + 0.05f * dt));
            b->s.angular_velocity = v3_scale(b->s.angular_velocity, 1.0f / (1.0f + 0.10f * dt));
        }
        b->s.position = v3_add(b->s.position, v3_scale(b->s.linear_velocity, dt));

        Vec3 omega = b->s.angular_velocity;
        float wl = v3_len(omega);
        if (wl > 1e-6f) {
            Quat dq = quat_axis_angle(v3_scale(omega, 1.0f / wl), wl * dt);
            b->s.rotation = quat_norm(quat_mul(dq, b->s.rotation));
        }
        b->force = v3(0, 0, 0);
    }
}

void hr_physics_step(HrPhysicsWorld* w, float dt) {
    if (!w || dt <= 0.0f) return;
    if (dt > 0.05f) dt = 0.05f;

    integrate(w, dt);
    build_contacts(w);
    prepare_contacts(w);
    solve_contacts(w, 10);
    correct_positions(w);
}

/* ---------------------------------------------------------------- queries */
static bool ray_sphere(Vec3 o, Vec3 d, Vec3 c, float r, float* t) {
    Vec3 oc = v3_sub(o, c);
    float b = v3_dot(oc, d);
    float cc = v3_dot(oc, oc) - r * r;
    float disc = b * b - cc;
    if (disc < 0.0f) return false;
    float s = sqrtf(disc);
    float t0 = -b - s;
    if (t0 < 0.0f) t0 = -b + s;
    if (t0 < 0.0f) return false;
    *t = t0;
    return true;
}

static bool ray_box(Vec3 o, Vec3 d, Vec3 c, Quat q, Vec3 h, float* t, Vec3* normal) {
    Vec3 lo = quat_rotate_inv(q, v3_sub(o, c));
    Vec3 ld = quat_rotate_inv(q, d);
    float tmin = -FLT_MAX, tmax = FLT_MAX;
    int axis = -1;
    float sign = 1.0f;
    for (int i = 0; i < 3; ++i) {
        float hh = h.v[i];
        if (fabsf(ld.v[i]) < 1e-8f) {
            if (lo.v[i] < -hh || lo.v[i] > hh) return false;
            continue;
        }
        float inv = 1.0f / ld.v[i];
        float t1 = (-hh - lo.v[i]) * inv;
        float t2 = (hh - lo.v[i]) * inv;
        float s = -1.0f;
        if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; s = 1.0f; }
        if (t1 > tmin) { tmin = t1; axis = i; sign = s; }
        if (t2 < tmax) tmax = t2;
        if (tmin > tmax) return false;
    }
    float tt = tmin >= 0.0f ? tmin : tmax;
    if (tt < 0.0f) return false;
    *t = tt;
    Vec3 ln = v3(0, 0, 0);
    if (axis >= 0) ln.v[axis] = sign;
    *normal = quat_rotate(q, ln);
    return true;
}

bool hr_physics_raycast(const HrPhysicsWorld* w, Vec3 origin, Vec3 dir,
                        float max_dist, u32 layer_mask, HrRayHit* out) {
    if (!w || !out) return false;
    float dl = v3_len(dir);
    if (dl < 1e-8f) return false;
    Vec3 d = v3_scale(dir, 1.0f / dl);
    float best = max_dist > 0.0f ? max_dist : FLT_MAX;
    bool hit = false;
    HrRayHit result;
    memset(&result, 0, sizeof(result));

    for (u32 i = 0; i < w->capacity; ++i) {
        const PhysBody* b = &w->bodies[i];
        if (!b->s.active) continue;
        if (layer_mask && !(b->s.layer & layer_mask)) continue;

        float t = 0.0f;
        Vec3 normal = v3(0, 0, 0);
        bool ok = false;
        if (b->s.shape == HR_SHAPE_SPHERE) {
            ok = ray_sphere(origin, d, b->s.position, b->s.radius, &t);
            if (ok) normal = v3_norm(v3_sub(v3_add(origin, v3_scale(d, t)), b->s.position));
        } else if (b->s.shape == HR_SHAPE_BOX) {
            ok = ray_box(origin, d, b->s.position, b->s.rotation,
                         b->s.half_extents, &t, &normal);
        } else if (b->s.shape == HR_SHAPE_PLANE) {
            float denom = v3_dot(d, b->s.plane_normal);
            if (fabsf(denom) > 1e-8f) {
                t = v3_dot(v3_sub(b->s.position, origin), b->s.plane_normal) / denom;
                if (t >= 0.0f) { ok = true; normal = b->s.plane_normal; }
            }
        }
        if (!ok || t > best) continue;
        best = t;
        hit = true;
        result.point = v3_add(origin, v3_scale(d, t));
        result.normal = normal;
        result.distance = t;
        result.body = encode(b, i);
        result.user = b->s.user;
    }
    if (hit) *out = result;
    return hit;
}

u32 hr_physics_query_sphere(const HrPhysicsWorld* w, Vec3 center, float radius,
                            u32 layer_mask, u32* out, u32 max) {
    if (!w || !out || max == 0) return 0;
    u32 n = 0;
    for (u32 i = 0; i < w->capacity && n < max; ++i) {
        const PhysBody* b = &w->bodies[i];
        if (!b->s.active) continue;
        if (layer_mask && !(b->s.layer & layer_mask)) continue;
        bool overlap = false;
        if (b->s.shape == HR_SHAPE_SPHERE) {
            float rr = radius + b->s.radius;
            overlap = v3_len_sq(v3_sub(center, b->s.position)) <= rr * rr;
        } else if (b->s.shape == HR_SHAPE_BOX) {
            Vec3 local = quat_rotate_inv(b->s.rotation, v3_sub(center, b->s.position));
            Vec3 closest = v3(clampf(local.x, -b->s.half_extents.x, b->s.half_extents.x),
                              clampf(local.y, -b->s.half_extents.y, b->s.half_extents.y),
                              clampf(local.z, -b->s.half_extents.z, b->s.half_extents.z));
            overlap = v3_len_sq(v3_sub(local, closest)) <= radius * radius;
        } else if (b->s.shape == HR_SHAPE_PLANE) {
            float dist = v3_dot(v3_sub(center, b->s.position), b->s.plane_normal);
            overlap = fabsf(dist) <= radius;
        }
        if (overlap) out[n++] = encode(b, i);
    }
    return n;
}
