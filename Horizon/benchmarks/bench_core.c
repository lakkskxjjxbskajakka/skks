/* Horizon Engine - micro benchmarks.  Run ./horizon_bench [iterations]. */
#include "horizon/core.h"
#include "horizon/memory.h"
#include "horizon/containers.h"
#include "horizon/math.h"
#include "horizon/jobs.h"
#include "horizon/ecs.h"
#include "horizon/scene.h"

#include <stdatomic.h>

static u32 g_iters = 2000000;

static double now_ms(void) { return (double)hr_time_ns() * 1e-6; }

#define BENCHN(label, count, body) do { \
    double _t0 = now_ms(); \
    body; \
    double _dt = now_ms() - _t0; \
    printf("%-28s %10.3f ms  %10.2f Mops/s\n", label, _dt, \
           _dt > 0.0 ? (double)(count) / (_dt * 1000.0) : 0.0); \
} while (0)
#define BENCH(label, body) BENCHN(label, g_iters, body)

static void bench_memory(void) {
    printf("\n-- allocators --\n");
    BENCH("hr_alloc/hr_free", {
        for (u32 i = 0; i < g_iters; ++i) { void* p = hr_alloc(64); hr_free(p); }
    });

    HrArena arena;
    hr_arena_init(&arena, (usize)g_iters * 64u + 4096u, "bench");
    BENCH("arena_alloc", {
        for (u32 i = 0; i < g_iters; ++i) (void)hr_arena_alloc(&arena, 64, 16);
    });
    hr_arena_destroy(&arena);

    HrPool pool;
    hr_pool_init(&pool, 64, g_iters, "bench");
    void** objs = (void**)hr_alloc(sizeof(void*) * g_iters);
    BENCH("pool_alloc", {
        for (u32 i = 0; i < g_iters; ++i) objs[i] = hr_pool_alloc(&pool);
    });
    BENCH("pool_free", {
        for (u32 i = 0; i < g_iters; ++i) hr_pool_free(&pool, objs[i]);
    });
    hr_free(objs);
    hr_pool_destroy(&pool);
}

static void bench_containers(void) {
    printf("\n-- containers --\n");
    BENCH("dynarray_push", {
        hr_dynarray(u32) a;
        hr_da_init(&a);
        for (u32 i = 0; i < g_iters; ++i) hr_da_push(&a, i);
        hr_da_free(&a);
    });
}

static void bench_math(void) {
    printf("\n-- math --\n");
    Mat4 a = mat4_trs(v3(1, 2, 3), v3(0.3f, 0.5f, 0.7f), v3s(1.2f));
    Mat4 b = mat4_perspective_vk(HR_PI * 0.4f, 1.7f, 0.1f, 1000.0f);
    volatile float sink = 0.0f;
    BENCH("mat4_mul", {
        for (u32 i = 0; i < g_iters; ++i) { b = mat4_mul(a, b); sink += b.m[0]; }
    });
    b = mat4_perspective_vk(HR_PI * 0.4f, 1.7f, 0.1f, 1000.0f);
    BENCH("mat4_invert", {
        for (u32 i = 0; i < g_iters; ++i) { Mat4 inv = mat4_invert(b); sink += inv.m[0]; }
    });
    Quat q = quat_axis_angle(v3(0.3f, 1.0f, -0.2f), 1.1f);
    BENCH("quat_rotate", {
        for (u32 i = 0; i < g_iters; ++i) { Vec3 r = quat_rotate(q, v3(1, 0, 0)); sink += r.x; }
    });
    (void)sink;
}

static void sum_range(void* arg, u32 begin, u32 end) {
    _Atomic u64* sum = (_Atomic u64*)arg;
    u64 local = 0;
    for (u32 i = begin; i < end; ++i) local += i;
    atomic_fetch_add(sum, local);
}

static void bench_jobs(void) {
    printf("\n-- jobs --\n");
    for (u32 workers = 1; workers <= 4; workers *= 2) {
        HrJobSystem* js = NULL;
        if (!hr_jobs_init(&js, workers)) return;
        _Atomic u64 sum = 0;
        u32 count = g_iters;
        double t0 = now_ms();
        hr_jobs_parallel_for(js, count, 256, sum_range, &sum);
        double dt = now_ms() - t0;
        printf("parallel_for (%u workers)    %10.3f ms  %10.2f Mops/s\n",
               workers, dt, dt > 0.0 ? (double)count / (dt * 1000.0) : 0.0);
        atomic_store(&sum, 0);
        hr_jobs_shutdown(js);
    }
}

typedef struct BenchPosition { float x, y, z; } BenchPosition;

static void bench_ecs(void) {
    printf("\n-- ecs --\n");
    u32 entities = HR_MIN(g_iters, HR_ECS_MAX_ENTITIES);
    HrWorld* w = hr_world_create();
    HrEntity* ids = (HrEntity*)hr_alloc(sizeof(HrEntity) * entities);
    BENCHN("create_entity", entities, {
        for (u32 i = 0; i < entities; ++i) ids[i] = hr_world_create_entity(w);
    });
    BENCHN("add_component", entities, {
        for (u32 i = 0; i < entities; ++i)
            (void)hr_world_add_component(w, ids[i], 0, sizeof(BenchPosition));
    });
    volatile float sink = 0.0f;
    BENCHN("get_component", entities, {
        for (u32 i = 0; i < entities; ++i) {
            BenchPosition* p = (BenchPosition*)hr_world_get_component(w, ids[i], 0);
            if (p) sink += p->x;
        }
    });
    BENCHN("remove_component", entities, {
        for (u32 i = 0; i < entities; ++i) hr_world_remove_component(w, ids[i], 0);
    });
    BENCHN("destroy_entity", entities, {
        for (u32 i = 0; i < entities; ++i) hr_world_destroy_entity(w, ids[i]);
    });
    (void)sink;
    hr_free(ids);
    hr_world_destroy(w);
}

static void bench_scene(void) {
    printf("\n-- scene --\n");
    u32 nodes = HR_MIN(g_iters, HR_SCENE_MAX_NODES - 1);
    HrWorld* w = hr_world_create();
    HrScene* s = hr_scene_create(w);
    HrEntity* ids = (HrEntity*)hr_alloc(sizeof(HrEntity) * nodes);
    double t0 = now_ms();
    for (u32 i = 0; i < nodes; ++i) {
        HrEntity parent = i > 0 ? ids[i - 1] : HR_ENTITY_INVALID;
        ids[i] = hr_scene_create_entity(s, "node", parent);
    }
    double dt = now_ms() - t0;
    printf("%-28s %10.3f ms  %10.2f Mops/s\n", "scene_create_entity", dt,
           dt > 0.0 ? (double)nodes / (dt * 1000.0) : 0.0);

    Mat4 local = mat4_translate(v3(0, 1, 0));
    t0 = now_ms();
    for (u32 i = 0; i < nodes; ++i) hr_scene_set_local_transform(s, ids[i], local);
    dt = now_ms() - t0;
    printf("%-28s %10.3f ms  %10.2f Mops/s\n", "set_local_transform", dt,
           dt > 0.0 ? (double)nodes / (dt * 1000.0) : 0.0);

    t0 = now_ms();
    hr_scene_update_all_transforms(s);
    dt = now_ms() - t0;
    printf("%-28s %10.3f ms  %10.2f Mops/s\n", "update_all_transforms", dt,
           dt > 0.0 ? (double)nodes / (dt * 1000.0) : 0.0);

    hr_free(ids);
    hr_scene_destroy(s);
    hr_world_destroy(w);
}

int main(int argc, char** argv) {
    if (argc > 1) {
        long n = strtol(argv[1], NULL, 10);
        if (n > 0) g_iters = (u32)n;
    }
    printf("Horizon benchmarks (%u iterations)\n", g_iters);
    bench_memory();
    bench_containers();
    bench_math();
    bench_jobs();
    bench_ecs();
    bench_scene();
    printf("\npeak heap: %.2f MiB\n", (double)hr_mem_peak() / (1024.0 * 1024.0));
    return 0;
}
