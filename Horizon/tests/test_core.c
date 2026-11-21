/* Horizon Engine - unit tests.  Build: cmake --build build && ctest --test-dir build */
#include "horizon/core.h"
#include "horizon/memory.h"
#include "horizon/containers.h"
#include "horizon/math.h"
#include "horizon/jobs.h"
#include "horizon/ecs.h"
#include "horizon/scene.h"
#include "horizon/mesh.h"
#include "horizon/textures.h"
#include "horizon/physics.h"
#include "horizon/animation.h"
#include "horizon/assets.h"
#include "horizon/adaptive.h"

#include <stdatomic.h>

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond) do { \
    ++g_checks; \
    if (!(cond)) { \
        ++g_failures; \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define CHECK_NEAR(a, b, eps) do { \
    ++g_checks; \
    float _a = (float)(a), _b = (float)(b); \
    if (!(fabsf(_a - _b) <= (float)(eps))) { \
        ++g_failures; \
        fprintf(stderr, "FAIL %s:%d: %s=%f != %s=%f\n", __FILE__, __LINE__, #a, _a, #b, _b); \
    } \
} while (0)

static void test_memory(void) {
    usize before = hr_mem_used();
    u8* p = (u8*)hr_alloc(100);
    CHECK(p != NULL);
    CHECK(hr_mem_used() >= before + 100);
    memset(p, 0xab, 100);
    p = (u8*)hr_realloc(p, 400);
    CHECK(p != NULL);
    CHECK(p[0] == 0xab && p[99] == 0xab);
    hr_free(p);
    CHECK(hr_mem_used() == before);

    u32* zeros = (u32*)hr_calloc(32, sizeof(u32));
    CHECK(zeros != NULL);
    for (int i = 0; i < 32; ++i) CHECK(zeros[i] == 0);
    hr_free(zeros);

    HrArena arena;
    CHECK(hr_arena_init(&arena, 1024, "test"));
    void* a0 = hr_arena_alloc(&arena, 13, 1);
    void* a1 = hr_arena_alloc(&arena, 64, 64);
    CHECK(a0 != NULL && a1 != NULL);
    CHECK(((uintptr_t)a1 % 64u) == 0);
    usize mark = hr_arena_mark(&arena);
    CHECK(hr_arena_alloc(&arena, 512, 16) != NULL);
    hr_arena_rewind(&arena, mark);
    CHECK(hr_arena_mark(&arena) == mark);
    hr_arena_reset(&arena);
    CHECK(hr_arena_mark(&arena) == 0);
    hr_arena_destroy(&arena);

    HrPool pool;
    CHECK(hr_pool_init(&pool, sizeof(u64), 8, "test"));
    void* objs[8];
    for (int i = 0; i < 8; ++i) { objs[i] = hr_pool_alloc(&pool); CHECK(objs[i] != NULL); }
    CHECK(hr_pool_alloc(&pool) == NULL);
    for (int i = 0; i < 8; i += 2) hr_pool_free(&pool, objs[i]);
    CHECK(hr_pool_alloc(&pool) != NULL);
    hr_pool_destroy(&pool);

    HrFreeList fl;
    CHECK(hr_freelist_init(&fl, 4096, "test"));
    void* x = hr_freelist_alloc(&fl, 100, 16);
    void* y = hr_freelist_alloc(&fl, 200, 16);
    void* z = hr_freelist_alloc(&fl, 50, 16);
    CHECK(x && y && z);
    CHECK(hr_freelist_blocks(&fl) >= 3);
    hr_freelist_free(&fl, y);
    hr_freelist_free(&fl, x);
    hr_freelist_free(&fl, z);
    CHECK(hr_freelist_used(&fl) == 0);
    CHECK(hr_freelist_blocks(&fl) == 1);
    hr_freelist_destroy(&fl);
}

static void test_containers(void) {
    hr_dynarray(int) values;
    hr_da_init(&values);
    for (int i = 0; i < 1000; ++i) hr_da_push(&values, i * 3);
    CHECK(values.len == 1000);
    CHECK(values.items[0] == 0 && values.items[999] == 2997);
    (void)hr_da_pop(&values);
    CHECK(values.len == 999);
    hr_da_clear(&values);
    CHECK(values.len == 0);
    hr_da_free(&values);
}

static void test_math(void) {
    Mat4 id = mat4_identity();
    Mat4 t = mat4_translate(v3(1, 2, 3));
    Mat4 s = mat4_scale_uniform(2.0f);
    Mat4 m = mat4_mul(t, s);
    Vec3 p = mat4_mul_point(m, v3(1, 1, 1));
    CHECK_NEAR(p.x, 3.0f, 1e-5f);
    CHECK_NEAR(p.y, 4.0f, 1e-5f);
    CHECK_NEAR(p.z, 5.0f, 1e-5f);

    Mat4 inv = mat4_invert(m);
    Mat4 r = mat4_mul(m, inv);
    for (int c = 0; c < 4; ++c)
        for (int row = 0; row < 4; ++row)
            CHECK_NEAR(r.c[c][row], id.c[c][row], 1e-4f);

    Quat q = quat_axis_angle(v3(0, 1, 0), HR_PI * 0.5f);
    Vec3 v = quat_rotate(q, v3(1, 0, 0));
    CHECK_NEAR(v.x, 0.0f, 1e-5f);
    CHECK_NEAR(v.z, -1.0f, 1e-5f);

    Mat4 vp = mat4_mul(mat4_perspective_vk(HR_PI * 0.5f, 1.0f, 0.1f, 100.0f),
                       mat4_look_at(v3(0, 0, 5), v3(0, 0, 0), v3(0, 1, 0)));
    Vec4 planes[6];
    hr_frustum_planes(vp, planes);
    CHECK(hr_sphere_in_frustum(planes, v3(0, 0, 0), 0.5f));
    CHECK(!hr_sphere_in_frustum(planes, v3(0, 0, 50), 0.5f));

    CHECK(hr_value_noise2(1.25f, -3.5f, 7u) == hr_value_noise2(1.25f, -3.5f, 7u));
    float n = hr_fbm2(12.3f, -8.1f, 5, 1u);
    CHECK(n > -2.0f && n < 2.0f);
}

static void sum_range(void* arg, u32 begin, u32 end) {
    _Atomic u64* sum = (_Atomic u64*)arg;
    u64 local = 0;
    for (u32 i = begin; i < end; ++i) local += i;
    atomic_fetch_add(sum, local);
}

static void test_jobs(void) {
    HrJobSystem* js = NULL;
    CHECK(hr_jobs_init(&js, 3));
    _Atomic u64 sum = 0;
    hr_jobs_parallel_for(js, 10000, 128, sum_range, &sum);
    CHECK(atomic_load(&sum) == (u64)10000 * 9999 / 2);
    hr_jobs_parallel_for(js, 1, 8, sum_range, &sum);
    CHECK(atomic_load(&sum) == (u64)10000 * 9999 / 2);
    hr_jobs_wait_idle(js);
    hr_jobs_shutdown(js);
}

typedef struct TestPosition { float x, y, z; } TestPosition;
typedef struct TestVelocity { float x, y, z; } TestVelocity;

static void test_ecs(void) {
    HrWorld* w = hr_world_create();
    CHECK(w != NULL);
    HrEntity e0 = hr_world_create_entity(w);
    HrEntity e1 = hr_world_create_entity(w);
    HrEntity e2 = hr_world_create_entity(w);
    CHECK(e0 && e1 && e2 && e0 != e1);
    CHECK(hr_world_entity_count(w) == 3);

    TestPosition* p = (TestPosition*)hr_world_add_component(w, e0, 0, sizeof(TestPosition));
    CHECK(p != NULL);
    p->x = 1.5f; p->y = 2.5f; p->z = 3.5f;
    TestPosition* p2 = (TestPosition*)hr_world_get_component(w, e0, 0);
    CHECK(p2 == p);
    CHECK_NEAR(p2->x, 1.5f, 1e-6f);
    CHECK(hr_world_has_component(w, e0, 0));
    CHECK(!hr_world_has_component(w, e1, 0));

    TestVelocity* v = (TestVelocity*)hr_world_add_component(w, e1, 1, sizeof(TestVelocity));
    CHECK(v != NULL);
    v->x = -1.0f;
    CHECK(hr_world_component_count(w, 1) == 1);

    /* Removing from the middle must keep the sparse set consistent. */
    hr_world_add_component(w, e2, 0, sizeof(TestPosition));
    hr_world_remove_component(w, e0, 0);
    CHECK(!hr_world_has_component(w, e0, 0));
    CHECK(hr_world_has_component(w, e2, 0));
    CHECK(hr_world_component_count(w, 0) == 1);

    hr_world_destroy_entity(w, e1);
    CHECK(!hr_world_is_alive(w, e1));
    CHECK(hr_world_component_count(w, 1) == 0);
    CHECK(hr_world_component_count(w, 0) == 1);

    HrEntity reused = hr_world_create_entity(w);
    CHECK(reused == e1);
    CHECK(hr_world_is_alive(w, reused));
    CHECK(!hr_world_has_component(w, reused, 1));

    hr_world_destroy_entity(w, reused);
    hr_world_destroy_entity(w, e2);
    hr_world_destroy_entity(w, e0);
    CHECK(hr_world_entity_count(w) == 0);
    hr_world_destroy(w);
}

static void count_node(HrEntity e, void* user) {
    (void)e;
    (*(u32*)user)++;
}

static void test_scene(void) {
    HrWorld* w = hr_world_create();
    HrScene* scene = hr_scene_create(w);
    CHECK(scene != NULL);

    HrEntity root = hr_scene_create_entity(scene, "root", HR_ENTITY_INVALID);
    HrEntity child = hr_scene_create_entity(scene, "child", root);
    HrEntity grandchild = hr_scene_create_entity(scene, "grandchild", child);
    HrEntity other_root = hr_scene_create_entity(scene, "other", HR_ENTITY_INVALID);
    CHECK(root && child && grandchild && other_root);
    CHECK(hr_scene_node_count(scene) == 4);
    CHECK(hr_scene_find(scene, "grandchild") == grandchild);

    hr_scene_set_local_transform(scene, root, mat4_translate(v3(1, 0, 0)));
    hr_scene_set_local_transform(scene, child, mat4_translate(v3(0, 2, 0)));
    hr_scene_set_local_transform(scene, grandchild, mat4_translate(v3(0, 0, 3)));
    Vec3 p = mat4_mul_point(hr_scene_get_transform(scene, grandchild), v3(0, 0, 0));
    CHECK_NEAR(p.x, 1.0f, 1e-5f);
    CHECK_NEAR(p.y, 2.0f, 1e-5f);
    CHECK_NEAR(p.z, 3.0f, 1e-5f);

    /* Reparent under another root and verify the cached transform updates. */
    hr_scene_add_child(scene, child, other_root);
    hr_scene_set_local_transform(scene, other_root, mat4_translate(v3(0, 10, 0)));
    p = mat4_mul_point(hr_scene_get_transform(scene, grandchild), v3(0, 0, 0));
    CHECK_NEAR(p.x, 0.0f, 1e-5f);
    CHECK_NEAR(p.y, 12.0f, 1e-5f);
    CHECK_NEAR(p.z, 3.0f, 1e-5f);

    u32 count = 0;
    hr_scene_for_each_node(scene, count_node, &count);
    CHECK(count == 4);
    count = 0;
    hr_scene_for_each_entity_recursive(scene, child, count_node, &count);
    CHECK(count == 2);

    hr_scene_remove_child(scene, child);
    CHECK(hr_scene_node_count(scene) == 4);
    p = mat4_mul_point(hr_scene_get_transform(scene, grandchild), v3(0, 0, 0));
    CHECK_NEAR(p.y, 2.0f, 1e-5f);

    hr_scene_destroy_entity(scene, child);
    CHECK(hr_scene_node_count(scene) == 2);
    CHECK(!hr_world_is_alive(w, child));
    CHECK(!hr_world_is_alive(w, grandchild));
    CHECK(hr_world_is_alive(w, root));
    CHECK(hr_world_is_alive(w, other_root));

    hr_scene_destroy(scene);
    hr_world_destroy(w);
}

static void test_mesh(void) {
    HrMeshData m;
    hr_mesh_init(&m, "box");
    hr_mesh_box(&m, v3(0, 0, 0), v3(1, 2, 3));
    hr_mesh_finalize(&m);
    CHECK(m.vertex_count == 24);
    CHECK(m.index_count == 36);
    CHECK_NEAR(m.bounds_center.x, 0.0f, 1e-5f);
    CHECK_NEAR(m.bounds_center.y, 0.0f, 1e-5f);
    CHECK_NEAR(m.bounds_center.z, 0.0f, 1e-5f);
    CHECK(m.bounds_radius > 3.0f);
    int finite = 1;
    for (u32 i = 0; i < m.vertex_count; ++i) {
        HrVertex* v = &m.vertices[i];
        if (!(v->normal.x == v->normal.x)) finite = 0;
        if (fabsf(v3_len(v3_from4(v->tangent)) - 1.0f) > 1e-3f) finite = 0;
    }
    CHECK(finite);
    hr_mesh_free(&m);

    HrMeshData plane;
    hr_mesh_init(&plane, "plane");
    hr_mesh_plane(&plane, v3(0, 0, 0), v2(10, 10), v2(1, 1));
    hr_mesh_finalize(&plane);
    CHECK(plane.vertex_count == 4 && plane.index_count == 6);
    hr_mesh_free(&plane);
}

static void test_textures(void) {
    HrTextureSet set;
    hr_texture_set_generate(&set, HR_TEX_ASPHALT, 64, 123u);
    CHECK(set.albedo.rgba != NULL && set.normal.rgba != NULL && set.orm.rgba != NULL);
    CHECK(set.albedo.width == 64 && set.albedo.height == 64);
    CHECK(set.albedo.rgba[3] == 255);
    CHECK(set.orm.rgba[1] > 40); /* roughness in a plausible range */
    int varied = 0;
    for (int i = 4; i < 64 * 64 * 4; i += 4)
        if (set.albedo.rgba[i] != set.albedo.rgba[0]) { varied = 1; break; }
    CHECK(varied);
    hr_image_free(&set.albedo);
    hr_image_free(&set.normal);
    hr_image_free(&set.orm);
}

static float test_ground_flat(void* user, float x, float z) {
    (void)x; (void)z;
    return *(float*)user;
}

static void test_physics(void) {
    HrPhysicsConfig cfg;
    hr_physics_default_config(&cfg);
    cfg.solver_iterations = 12;
    HrPhysicsWorld* w = NULL;
    CHECK(hr_physics_create(&w, &cfg));

    HrBodyDesc plane = {0};
    plane.shape.type = HR_SHAPE_PLANE;
    plane.position = v3(0, 0, 0);
    plane.rotation = quat_identity();
    plane.friction = 0.8f;
    plane.restitution = 0.0f;
    plane.flags = HR_BODY_STATIC;
    HrBodyHandle ground = hr_physics_add_body(w, &plane);
    CHECK(hr_physics_body_alive(w, ground));

    HrBodyDesc sphere = {0};
    sphere.shape.type = HR_SHAPE_SPHERE;
    sphere.shape.radius = 0.5f;
    sphere.position = v3(0, 5.0f, 0);
    sphere.rotation = quat_identity();
    sphere.mass = 1.0f;
    sphere.friction = 0.5f;
    sphere.restitution = 0.0f;
    sphere.flags = HR_BODY_GRAVITY | HR_BODY_SLEEPABLE;
    HrBodyHandle ball = hr_physics_add_body(w, &sphere);
    CHECK(HR_BODY_IS_VALID(ball));

    for (int i = 0; i < 240; ++i) hr_physics_step(w, 1.0f / 120.0f);
    Vec3 pos; Quat rot;
    CHECK(hr_physics_get_transform(w, ball, &pos, &rot));
    CHECK_NEAR(pos.y, 0.5f, 0.08f);
    CHECK(hr_physics_body_sleeping(w, ball) || fabsf(pos.y - 0.5f) < 0.1f);

    /* Raycast from above should hit the resting sphere. */
    HrRayHit hit;
    memset(&hit, 0, sizeof(hit));
    CHECK(hr_physics_raycast(w, v3(0, 5.0f, 0), v3(0, -1, 0), 10.0f, 0xffffffffu, &hit));
    CHECK(hit.body.index == ball.index);
    CHECK(hit.t > 3.5f && hit.t < 5.5f);

    /* Sphere-sphere impulse exchange. */
    HrPhysicsWorld* w2 = NULL;
    CHECK(hr_physics_create(&w2, &cfg));
    HrBodyDesc s1 = sphere; s1.position = v3(-2, 0.5f, 0); s1.linear_velocity = v3(3, 0, 0); s1.flags = HR_BODY_GRAVITY;
    HrBodyDesc s2 = sphere; s2.position = v3( 2, 0.5f, 0); s2.linear_velocity = v3(-3, 0, 0); s2.flags = HR_BODY_GRAVITY;
    HrBodyHandle h1 = hr_physics_add_body(w2, &s1);
    HrBodyHandle h2 = hr_physics_add_body(w2, &s2);
    hr_physics_add_body(w2, &plane);
    for (int i = 0; i < 120; ++i) hr_physics_step(w2, 1.0f / 120.0f);
    Vec3 v1, v2, a1, a2;
    hr_physics_get_velocity(w2, h1, &v1, &a1);
    hr_physics_get_velocity(w2, h2, &v2, &a2);
    CHECK(v1.x < 0.5f);
    CHECK(v2.x > -0.5f);

    /* Box rests on ground callback. */
    HrPhysicsWorld* w3 = NULL;
    CHECK(hr_physics_create(&w3, &cfg));
    float ground_h = 2.0f;
    hr_physics_set_ground(w3, test_ground_flat, &ground_h);
    HrBodyDesc box = {0};
    box.shape.type = HR_SHAPE_BOX;
    box.shape.half_extents = v3(0.5f, 0.5f, 0.5f);
    box.position = v3(0, 6.0f, 0);
    box.rotation = quat_identity();
    box.mass = 2.0f;
    box.friction = 0.8f;
    box.flags = HR_BODY_GRAVITY | HR_BODY_SLEEPABLE;
    HrBodyHandle bh = hr_physics_add_body(w3, &box);
    for (int i = 0; i < 300; ++i) hr_physics_step(w3, 1.0f / 120.0f);
    hr_physics_get_transform(w3, bh, &pos, &rot);
    CHECK_NEAR(pos.y, 2.5f, 0.12f);

    /* Kinematic body pushes dynamic body. */
    HrPhysicsWorld* w4 = NULL;
    CHECK(hr_physics_create(&w4, &cfg));
    hr_physics_add_body(w4, &plane);
    HrBodyDesc kin = {0};
    kin.shape.type = HR_SHAPE_BOX;
    kin.shape.half_extents = v3(0.5f, 0.5f, 0.5f);
    kin.position = v3(-2, 0.5f, 0);
    kin.rotation = quat_identity();
    kin.linear_velocity = v3(2.0f, 0, 0);
    kin.flags = HR_BODY_KINEMATIC;
    HrBodyHandle kh = hr_physics_add_body(w4, &kin);
    HrBodyDesc dyn = box;
    dyn.position = v3(0, 0.5f, 0);
    dyn.flags = HR_BODY_GRAVITY;
    HrBodyHandle dh = hr_physics_add_body(w4, &dyn);
    for (int i = 0; i < 120; ++i) {
        Vec3 kp; Quat kr;
        hr_physics_get_transform(w4, kh, &kp, &kr);
        kp.x += 2.0f * (1.0f / 120.0f);
        hr_physics_set_transform(w4, kh, kp, kr);
        hr_physics_set_velocity(w4, kh, v3(2.0f, 0, 0), v3(0, 0, 0));
        hr_physics_step(w4, 1.0f / 120.0f);
    }
    Vec3 dp; hr_physics_get_transform(w4, dh, &dp, &rot);
    CHECK(dp.x > 0.2f);

    hr_physics_destroy(w4);
    hr_physics_destroy(w3);
    hr_physics_destroy(w2);
    hr_physics_destroy(w);
}

static void test_animation(void) {
    HrSkeleton sk;
    hr_skeleton_init(&sk);
    i32 root = hr_skeleton_add_joint(&sk, "root", -1, mat4_identity());
    i32 child = hr_skeleton_add_joint(&sk, "child", root, mat4_translate(v3(0, 1, 0)));
    CHECK(root == 0 && child == 1);
    CHECK(hr_skeleton_find_joint(&sk, "child") == child);
    CHECK(hr_skeleton_add_joint(&sk, "bad", 99, mat4_identity()) == -1);

    HrAnimationClip* walk = hr_animation_clip_create("walk", 1.0f, true);
    CHECK(walk != NULL);
    HrAnimTrack* tr = hr_animation_clip_add_track(walk, 0);
    CHECK(tr != NULL);
    CHECK(hr_anim_track_add_position(tr, 0.0f, v3(0, 0, 0)));
    CHECK(hr_anim_track_add_position(tr, 1.0f, v3(10, 0, 0)));
    CHECK(hr_anim_track_add_rotation(tr, 0.0f, quat_identity()));
    CHECK(hr_anim_track_add_rotation(tr, 1.0f, quat_identity()));

    HrPose pose;
    CHECK(hr_animation_sample(walk, 0.5f, &pose, 2));
    CHECK_NEAR(pose.translations[0].x, 5.0f, 1e-4f);
    CHECK(hr_animation_sample(walk, 1.25f, &pose, 2));
    CHECK_NEAR(pose.translations[0].x, 2.5f, 1e-4f);

    HrPose a, b, mixed;
    hr_pose_identity(&a, 2);
    hr_pose_identity(&b, 2);
    b.translations[0] = v3(10, 0, 0);
    hr_pose_lerp(&a, &b, 0.25f, &mixed);
    CHECK_NEAR(mixed.translations[0].x, 2.5f, 1e-4f);

    HrPose bind_pose;
    hr_pose_bind(&sk, &bind_pose);
    bind_pose.translations[1] = v3(0, 2, 0);
    Mat4 palette[HR_ANIM_MAX_JOINTS];
    hr_pose_palette(&sk, &bind_pose, palette);
    Vec3 wp = mat4_mul_point(palette[1], v3(0, 0, 0));
    CHECK_NEAR(wp.y, 3.0f, 1e-4f);

    HrAnimationClip* run = hr_animation_clip_create("run", 1.0f, true);
    HrAnimTrack* rt = hr_animation_clip_add_track(run, 0);
    hr_anim_track_add_position(rt, 0.0f, v3(0, 0, 0));
    hr_anim_track_add_position(rt, 1.0f, v3(20, 0, 0));

    HrAnimController ctrl;
    hr_anim_controller_init(&ctrl, &sk);
    hr_anim_controller_play(&ctrl, walk, 0.0f);
    hr_anim_controller_update(&ctrl, 0.5f);
    hr_anim_controller_pose(&ctrl, &pose);
    CHECK_NEAR(pose.translations[0].x, 5.0f, 1e-4f);

    hr_anim_controller_play(&ctrl, run, 1.0f);
    hr_anim_controller_update(&ctrl, 0.5f);
    hr_anim_controller_pose(&ctrl, &pose);
    CHECK(pose.translations[0].x >= 4.0f && pose.translations[0].x <= 11.0f);
    hr_anim_controller_update(&ctrl, 1.0f);
    hr_anim_controller_pose(&ctrl, &pose);
    CHECK_NEAR(pose.translations[0].x, 10.0f, 1e-3f);

    hr_animation_clip_destroy(walk);
    hr_animation_clip_destroy(run);
}

static void test_assets(void) {
    HrMeshData mesh;
    hr_mesh_init(&mesh, "cube");
    hr_mesh_box(&mesh, v3(0, 0, 0), v3s(1));
    hr_mesh_finalize(&mesh);
    void* blob = NULL; usize blob_size = 0;
    CHECK(hr_mesh_serialize(&mesh, &blob, &blob_size));
    CHECK(blob && blob_size > 0);
    HrMeshData restored;
    CHECK(hr_mesh_deserialize(blob, blob_size, &restored));
    CHECK(restored.vertex_count == mesh.vertex_count);
    CHECK(restored.index_count == mesh.index_count);
    CHECK_NEAR(restored.bounds_radius, mesh.bounds_radius, 1e-4f);
    hr_mesh_free(&restored);
    hr_free(blob);
    hr_mesh_free(&mesh);

    HrAssetPack pack;
    CHECK(hr_asset_pack_begin(&pack));
    const char* payload = "hello asset";
    CHECK(hr_asset_pack_add(&pack, HR_ASSET_BLOB, "greeting", payload, strlen(payload) + 1));
    CHECK(hr_asset_pack_add(&pack, HR_ASSET_BLOB, "numbers", "1234", 5));
    const char* path = "/tmp/horizon_test_assets.hras";
    CHECK(hr_asset_pack_write(&pack, path));
    hr_asset_pack_free(&pack);

    HrAssetPack loaded;
    CHECK(hr_asset_pack_open(&loaded, path));
    CHECK(loaded.entry_count == 2);
    const HrAssetEntry* e = hr_asset_pack_find(&loaded, "greeting");
    CHECK(e != NULL);
    usize sz = 0;
    const void* data = hr_asset_pack_data(&loaded, e, &sz);
    CHECK(data && sz == strlen(payload) + 1);
    CHECK(memcmp(data, payload, sz) == 0);
    CHECK(hr_asset_pack_find(&loaded, "missing") == NULL);
    hr_asset_pack_free(&loaded);

    HrJobSystem* js = NULL;
    CHECK(hr_jobs_init(&js, 1));
    HrAsyncAsset async;
    hr_asset_async_init(&async, path);
    CHECK(hr_asset_async_submit(&async, js));
    hr_jobs_wait_idle(js);
    CHECK(hr_asset_async_state(&async) == HR_ASSET_LOAD_READY);
    void* adata = NULL; usize asz = 0;
    CHECK(hr_asset_async_take(&async, &adata, &asz));
    CHECK(adata && asz > 0);
    hr_free(adata);
    hr_asset_async_destroy(&async);
    hr_jobs_shutdown(js);
}

static void test_adaptive(void) {
    HrAdaptiveConfig cfg;
    hr_adaptive_default_config(&cfg);
    cfg.cooldown_frames = 0;
    HrAdaptiveState s;
    hr_adaptive_init(&s, &cfg, 1.0f, 2048);
    for (int i = 0; i < 10; ++i) hr_adaptive_update(&s, 30.0f);
    CHECK(s.render_scale < 1.0f || s.shadow_resolution < 2048);
    hr_adaptive_reset(&s, 0.5f, 1024);
    for (int i = 0; i < 40; ++i) hr_adaptive_update(&s, 8.0f);
    CHECK(s.render_scale > 0.5f || s.shadow_resolution > 1024);
}

static void test_profiler(void) {
    hr_prof_enabled = 1;
    hr_prof_frame_begin();
    hr_prof_zone_begin("unit");
    hr_prof_zone_end();
    hr_prof_frame_end();
    CHECK(hr_prof_zone_ms("unit") >= 0.0);
    CHECK(hr_time_ns() > 0);
}

int main(void) {
    hr_log_set_level(HR_LOG_WARN);
    test_memory();
    test_containers();
    test_math();
    test_jobs();
    test_ecs();
    test_scene();
    test_mesh();
    test_textures();
    test_physics();
    test_animation();
    test_assets();
    test_adaptive();
    test_profiler();

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
