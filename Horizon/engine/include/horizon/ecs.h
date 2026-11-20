#ifndef HORIZON_ECS_H
#define HORIZON_ECS_H

#include "horizon/core.h"
#include "horizon/memory.h"

#define HR_ECS_MAX_COMPONENTS 64
#define HR_ECS_MAX_ENTITIES 4096
#define HR_ECS_ARCHETYPE_CAP 1024

typedef u32 HrEntity;

typedef struct HrComponentId {
    u16 id;
    u16 size;
} HrComponentId;

typedef struct HrComponentMapper {
    HrComponentId id;
    usize stride;
    usize offset;
} HrComponentMapper;

typedef void (*HrEntityFn)(HrEntity entity, void* userdata);

typedef struct HrWorld {
    HrEntity free_list[HR_ECS_MAX_ENTITIES];
    u32 free_count;
    u32 next_entity;
    HrComponentMapper components[HR_ECS_MAX_COMPONENTS];
    u32 component_count;
    bool component_enabled[HR_ECS_MAX_COMPONENTS];
    u64 component_signatures[HR_ECS_MAX_ENTITIES];
} HrWorld;

HrWorld* hr_world_create(void);
void hr_world_destroy(HrWorld* world);

HrEntity hr_world_create_entity(HrWorld* world);
void hr_world_destroy_entity(HrWorld* world, HrEntity entity);

bool hr_world_has_component(HrWorld* world, HrEntity entity, u32 component_type);
void hr_world_add_component(HrWorld* world, HrEntity entity, u32 component_type, usize size);
void hr_world_remove_component(HrWorld* world, HrEntity entity, u32 component_type);

void hr_world_for_each_component(HrWorld* world, u32 component_type, HrEntityFn fn, void* userdata);
void hr_world_for_each_entity(HrWorld* world, HrEntityFn fn, void* userdata);

#endif /* HORIZON_ECS_H */