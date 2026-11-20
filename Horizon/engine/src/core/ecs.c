#include "horizon/ecs.h"
#include "horizon/memory.h"

HrWorld* hr_world_create(void) {
    HrWorld* world = (HrWorld*)hr_calloc(1, sizeof(HrWorld));
    if (!world) return NULL;
    world->free_count = 0;
    world->next_entity = 1; /* 0 is invalid */
    return world;
}

void hr_world_destroy(HrWorld* world) {
    if (!world) return;
    hr_free(world);
}

HrEntity hr_world_create_entity(HrWorld* world) {
    HrEntity entity;
    if (world->free_count > 0) {
        entity = world->free_list[--world->free_count];
    } else {
        if (world->next_entity >= HR_ECS_MAX_ENTITIES) return (HrEntity)0;
        entity = world->next_entity++;
    }
    world->component_signatures[entity] = 0;
    return entity;
}

void hr_world_destroy_entity(HrWorld* world, HrEntity entity) {
    if (entity == HR_ENTITY_INVALID || entity >= HR_ECS_MAX_ENTITIES) return;
    world->free_list[world->free_count++] = entity;
    world->component_signatures[entity] = 0;
}

bool hr_world_has_component(HrWorld* world, HrEntity entity, u32 component_type) {
    if (entity == HR_ENTITY_INVALID) return false;
    return (world->component_signatures[entity] & (1ull << component_type)) != 0;
}

void hr_world_add_component(HrWorld* world, HrEntity entity, u32 component_type, usize size) {
    if (entity == HR_ENTITY_INVALID) return;
    if (component_type >= HR_ECS_MAX_COMPONENTS) return;

    world->component_signatures[entity] |= (1ull << component_type);

    /* Ensure mapper exists for this component type */
    if (world->component_count <= component_type) {
        world->component_count = component_type + 1;
    }

    /* Store component mapper info */
    HrComponentMapper* mapper = &world->components[component_type];
    mapper->id.id = (u16)component_type;
    mapper->stride = HR_ALIGN_UP(size, 16);
    mapper->offset = 0; /* will be set when component data is allocated */
}

void hr_world_remove_component(HrWorld* world, HrEntity entity, u32 component_type) {
    if (entity == HR_ENTITY_INVALID) return;
    world->component_signatures[entity] &= ~(1ull << component_type);
}

void hr_world_for_each_component(HrWorld* world, u32 component_type, HrEntityFn fn, void* userdata) {
    for (HrEntity e = 0; e < HR_ECS_MAX_ENTITIES; ++e) {
        if (hr_world_has_component(world, e, component_type)) {
            fn(e, userdata);
        }
    }
}

void hr_world_for_each_entity(HrWorld* world, HrEntityFn fn, void* userdata) {
    for (HrEntity e = 0; e < HR_ECS_MAX_ENTITIES; ++e) {
        if (world->component_signatures[e] != 0) {
            fn(e, userdata);
        }
    }
}