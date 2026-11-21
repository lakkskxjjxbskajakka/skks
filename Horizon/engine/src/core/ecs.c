#include "horizon/ecs.h"
#include "horizon/memory.h"

#if defined(_MSC_VER)
#  include <intrin.h>
#endif

static u32 first_bit_u64(u64 bits) {
#if defined(_MSC_VER)
    unsigned long index;
    _BitScanForward64(&index, bits);
    return (u32)index;
#else
    return (u32)__builtin_ctzll(bits);
#endif
}

static bool pool_init(HrComponentPool* pool, usize stride) {
    if (!pool->sparse) {
        pool->sparse = (u32*)hr_alloc(sizeof(u32) * HR_ECS_MAX_ENTITIES);
        if (!pool->sparse) return false;
        for (u32 i = 0; i < HR_ECS_MAX_ENTITIES; ++i) pool->sparse[i] = HR_ECS_SPARSE_NONE;
    }
    if (pool->stride == 0) pool->stride = HR_ALIGN_UP(stride < 1 ? 1 : stride, 16);
    return stride <= pool->stride;
}

static bool pool_reserve(HrComponentPool* pool, u32 needed) {
    if (needed <= pool->capacity) return true;
    u32 cap = pool->capacity ? pool->capacity : 8;
    while (cap < needed) cap *= 2;
    u8* data = (u8*)hr_realloc(pool->data, (usize)cap * pool->stride);
    if (!data) return false;
    pool->data = data;
    HrEntity* ents = (HrEntity*)hr_realloc(pool->entities, sizeof(HrEntity) * cap);
    if (!ents) return false;
    pool->entities = ents;
    pool->capacity = cap;
    return true;
}

static u32 pool_index(const HrComponentPool* pool, HrEntity entity) {
    return pool->sparse[entity];
}

static void pool_remove_index(HrComponentPool* pool, u32 idx) {
    u32 last = pool->count - 1u;
    HrEntity removed = pool->entities[idx];
    HrEntity moved = pool->entities[last];
    if (idx != last) {
        memcpy(pool->data + (usize)idx * pool->stride,
               pool->data + (usize)last * pool->stride, pool->stride);
        pool->entities[idx] = moved;
        pool->sparse[moved] = idx;
    }
    pool->sparse[removed] = HR_ECS_SPARSE_NONE;
    pool->count = last;
}

HrWorld* hr_world_create(void) {
    HrWorld* world = (HrWorld*)hr_calloc(1, sizeof(HrWorld));
    if (!world) return NULL;
    world->next_entity = 1; /* 0 is the invalid entity */
    return world;
}

void hr_world_destroy(HrWorld* world) {
    if (!world) return;
    for (u32 i = 0; i < HR_ECS_MAX_COMPONENTS; ++i) {
        hr_free(world->pools[i].data);
        hr_free(world->pools[i].entities);
        hr_free(world->pools[i].sparse);
    }
    hr_free(world);
}

HrEntity hr_world_create_entity(HrWorld* world) {
    if (!world) return HR_ENTITY_INVALID;
    HrEntity entity;
    if (world->free_count > 0) {
        entity = world->free_list[--world->free_count];
    } else {
        if (world->next_entity >= HR_ECS_MAX_ENTITIES) return HR_ENTITY_INVALID;
        entity = world->next_entity++;
    }
    world->signatures[entity] = 0;
    world->alive[entity] = 1;
    world->entity_count++;
    return entity;
}

void hr_world_destroy_entity(HrWorld* world, HrEntity entity) {
    if (!world || entity == HR_ENTITY_INVALID || entity >= HR_ECS_MAX_ENTITIES) return;
    if (!world->alive[entity]) return;

    u64 sig = world->signatures[entity];
    while (sig) {
        u32 type = first_bit_u64(sig);
        sig &= sig - 1u;
        HrComponentPool* pool = &world->pools[type];
        if (pool->sparse && pool->sparse[entity] != HR_ECS_SPARSE_NONE)
            pool_remove_index(pool, pool->sparse[entity]);
    }
    world->signatures[entity] = 0;
    world->alive[entity] = 0;
    if (world->entity_count) world->entity_count--;
    world->free_list[world->free_count++] = entity;
}

bool hr_world_is_alive(HrWorld* world, HrEntity entity) {
    return world && entity != HR_ENTITY_INVALID && entity < HR_ECS_MAX_ENTITIES &&
           world->alive[entity] != 0;
}

u32 hr_world_entity_count(const HrWorld* world) { return world ? world->entity_count : 0; }

void* hr_world_add_component(HrWorld* world, HrEntity entity, u32 component_type, usize size) {
    if (!world || entity == HR_ENTITY_INVALID || entity >= HR_ECS_MAX_ENTITIES) return NULL;
    if (component_type >= HR_ECS_MAX_COMPONENTS || !world->alive[entity] || size == 0) return NULL;

    HrComponentPool* pool = &world->pools[component_type];
    if (!pool_init(pool, size)) return NULL;
    if (size > pool->stride) return NULL;
    pool->enabled = true;
    if (world->component_count <= component_type) world->component_count = component_type + 1;

    u32 existing = pool_index(pool, entity);
    if (existing != HR_ECS_SPARSE_NONE) {
        world->signatures[entity] |= (1ull << component_type);
        return pool->data + (usize)existing * pool->stride;
    }
    if (!pool_reserve(pool, pool->count + 1u)) return NULL;
    u32 idx = pool->count++;
    pool->entities[idx] = entity;
    pool->sparse[entity] = idx;
    world->signatures[entity] |= (1ull << component_type);
    void* p = pool->data + (usize)idx * pool->stride;
    memset(p, 0, pool->stride);
    return p;
}

void* hr_world_get_component(HrWorld* world, HrEntity entity, u32 component_type) {
    if (!world || entity == HR_ENTITY_INVALID || entity >= HR_ECS_MAX_ENTITIES) return NULL;
    if (component_type >= HR_ECS_MAX_COMPONENTS) return NULL;
    if (!world->alive[entity]) return NULL;
    HrComponentPool* pool = &world->pools[component_type];
    if (!pool->sparse) return NULL;
    u32 idx = pool_index(pool, entity);
    if (idx == HR_ECS_SPARSE_NONE) return NULL;
    return pool->data + (usize)idx * pool->stride;
}

bool hr_world_has_component(HrWorld* world, HrEntity entity, u32 component_type) {
    return hr_world_get_component(world, entity, component_type) != NULL;
}

void hr_world_remove_component(HrWorld* world, HrEntity entity, u32 component_type) {
    if (!world || entity == HR_ENTITY_INVALID || entity >= HR_ECS_MAX_ENTITIES) return;
    if (component_type >= HR_ECS_MAX_COMPONENTS || !world->alive[entity]) return;
    HrComponentPool* pool = &world->pools[component_type];
    if (!pool->sparse) return;
    u32 idx = pool_index(pool, entity);
    if (idx == HR_ECS_SPARSE_NONE) return;
    pool_remove_index(pool, idx);
    world->signatures[entity] &= ~(1ull << component_type);
}

u32 hr_world_component_count(const HrWorld* world, u32 component_type) {
    if (!world || component_type >= HR_ECS_MAX_COMPONENTS) return 0;
    return world->pools[component_type].count;
}

void hr_world_for_each_component(HrWorld* world, u32 component_type, HrEntityFn fn, void* userdata) {
    if (!world || !fn || component_type >= HR_ECS_MAX_COMPONENTS) return;
    HrComponentPool* pool = &world->pools[component_type];
    if (!pool->sparse) return;
    /* Snapshot count so callbacks may remove the current component safely. */
    u32 count = pool->count;
    for (u32 i = 0; i < count && i < pool->count; ++i) fn(pool->entities[i], userdata);
}

void hr_world_for_each_entity(HrWorld* world, HrEntityFn fn, void* userdata) {
    if (!world || !fn) return;
    for (HrEntity e = 1; e < world->next_entity; ++e) {
        if (world->alive[e]) fn(e, userdata);
    }
}
