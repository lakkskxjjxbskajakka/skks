/* Horizon Engine - modular rigid-body physics.
 *
 * The world keeps bodies in a contiguous slot array (handles carry a
 * generation counter so stale handles are rejected).  Broadphase is a uniform
 * spatial hash rebuilt every step, narrowphase produces contact manifolds for
 * sphere/box/capsule/plane pairs, and a sequential-impulse solver resolves
 * them with friction and Baumgarte position correction.  No allocation happens
 * inside the step once the world is created.
 */
#ifndef HORIZON_PHYSICS_H
#define HORIZON_PHYSICS_H

#include "horizon/core.h"
#include "horizon/math.h"

typedef struct HrPhysicsWorld HrPhysicsWorld;

typedef struct HrBodyHandle {
    u32 index;
    u32 generation;
} HrBodyHandle;

#define HR_BODY_INVALID ((HrBodyHandle){ 0xffffffffu, 0u })
#define HR_BODY_IS_VALID(h) ((h).index != 0xffffffffu)

typedef enum HrShapeType {
    HR_SHAPE_SPHERE = 0,
    HR_SHAPE_BOX,
    HR_SHAPE_CAPSULE,
    HR_SHAPE_PLANE
} HrShapeType;

typedef struct HrShape {
    HrShapeType type;
    Vec3 half_extents;   /* box half size (x, y, z)                    */
    float radius;        /* sphere / capsule radius                    */
    float half_height;   /* capsule cylindrical half height along local Y */
} HrShape;

typedef enum HrBodyFlags {
    HR_BODY_STATIC    = 1u << 0,
    HR_BODY_KINEMATIC = 1u << 1,  /* infinite mass, moved by the caller      */
    HR_BODY_GRAVITY   = 1u << 2,
    HR_BODY_SLEEPABLE = 1u << 3
} HrBodyFlags;

typedef struct HrBodyDesc {
    HrShape shape;
    Vec3 position;
    Quat rotation;
    Vec3 linear_velocity;
    Vec3 angular_velocity;
    float mass;          /* <= 0 with no static/kinematic flag means static */
    float restitution;   /* 0..1 */
    float friction;      /* >= 0 */
    u32 layer;           /* one bit identifying the body's layer */
    u32 mask;            /* layers this body collides with      */
    u32 flags;
} HrBodyDesc;

typedef struct HrRayHit {
    HrBodyHandle body;
    float t;
    Vec3 point;
    Vec3 normal;
} HrRayHit;

typedef struct HrPhysicsStats {
    u32 bodies;
    u32 awake_bodies;
    u32 broadphase_pairs;
    u32 contacts;
    u32 solver_iterations;
} HrPhysicsStats;

typedef struct HrPhysicsConfig {
    Vec3 gravity;
    u32 solver_iterations;
    u32 max_bodies;
    float cell_size;
    float sleep_linear;
    float sleep_angular;
    float sleep_time;
    float contact_slop;
    float correction;      /* Baumgarte position correction ratio (0..1) */
    float restitution_cutoff;
} HrPhysicsConfig;

/* Optional heightfield hook.  When set, every dynamic sphere/box is tested
 * against the ground surface at its centre, so props rest on terrain without a
 * triangle mesh collider.  Return the world-space height at (x, z). */
typedef float (*HrGroundHeightFn)(void* user, float x, float z);

void hr_physics_default_config(HrPhysicsConfig* out);

bool hr_physics_create(HrPhysicsWorld** out, const HrPhysicsConfig* config);
void hr_physics_destroy(HrPhysicsWorld* world);

HrBodyHandle hr_physics_add_body(HrPhysicsWorld* world, const HrBodyDesc* desc);
void hr_physics_remove_body(HrPhysicsWorld* world, HrBodyHandle handle);
bool hr_physics_body_alive(const HrPhysicsWorld* world, HrBodyHandle handle);
bool hr_physics_body_sleeping(const HrPhysicsWorld* world, HrBodyHandle handle);

bool hr_physics_get_transform(const HrPhysicsWorld* world, HrBodyHandle handle,
                              Vec3* out_position, Quat* out_rotation);
bool hr_physics_get_velocity(const HrPhysicsWorld* world, HrBodyHandle handle,
                             Vec3* out_linear, Vec3* out_angular);
void hr_physics_set_transform(HrPhysicsWorld* world, HrBodyHandle handle,
                              Vec3 position, Quat rotation);
void hr_physics_set_velocity(HrPhysicsWorld* world, HrBodyHandle handle,
                             Vec3 linear, Vec3 angular);
void hr_physics_apply_impulse(HrPhysicsWorld* world, HrBodyHandle handle,
                              Vec3 impulse, Vec3 world_point);
void hr_physics_wake(HrPhysicsWorld* world, HrBodyHandle handle);

void hr_physics_set_ground(HrPhysicsWorld* world, HrGroundHeightFn fn, void* user);

void hr_physics_step(HrPhysicsWorld* world, float dt);
void hr_physics_get_stats(const HrPhysicsWorld* world, HrPhysicsStats* out);

bool hr_physics_raycast(HrPhysicsWorld* world, Vec3 origin, Vec3 direction,
                        float max_distance, u32 layer_mask, HrRayHit* out_hit);
bool hr_physics_overlap_sphere(const HrPhysicsWorld* world, Vec3 center, float radius,
                               u32 layer_mask, HrBodyHandle* out_body);

#endif /* HORIZON_PHYSICS_H */
