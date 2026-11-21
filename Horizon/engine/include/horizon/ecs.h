/* Horizon Engine - data-oriented entity component system.
 *
 * Components live in dense, component-type specific sparse sets.  Adding or
 * removing a component never touches unrelated component arrays, and querying
 * a component for an entity is O(1).  This keeps iteration cache-friendly for
 * the systems that actually need it while staying small and predictable.
 */
#ifndef HORIZON_ECS_H
#define HORIZON_ECS_H

#include "horizon/core.h"
#include "horizon/memory.h"

#define HR_ECS_MAX_COMPONENTS 64
#define HR_ECS_MAX_ENTITIES 4096
#define HR_ECS_SPARSE_NONE 0xffffffffu

typedef u32 HrEntity;
typedef u32 HrComponentType;

typedef struct HrComponentPool {
    u8* data;                 /* dense component payloads            */
    HrEntity* entities;       /* dense entity ids                    */
    u32* sparse;              /* entity -> dense index (or NONE)     */
    u32 count;                /* number of live components           */
    u32 capacity;             /* dense capacity                      */
    usize stride;             /* aligned component size in bytes     */
    bool enabled;             /* pool has been seen at least once    */
} HrComponentPool;

typedef struct HrWorld {
    HrComponentPool pools[HR_ECS_MAX_COMPONENTS];
    u64 signatures[HR_ECS_MAX_ENTITIES];
    u8 alive[HR_ECS_MAX_ENTITIES];
    HrEntity free_list[HR_ECS_MAX_ENTITIES];
    u32 free_count;
    HrEntity next_entity;
    u32 entity_count;
    u32 component_count;
} HrWorld;

typedef void (*HrEntityFn)(HrEntity entity, void* userdata);

HrWorld* hr_world_create(void);
void hr_world_destroy(HrWorld* world);

HrEntity hr_world_create_entity(HrWorld* world);
void hr_world_destroy_entity(HrWorld* world, HrEntity entity);
bool hr_world_is_alive(HrWorld* world, HrEntity entity);
u32 hr_world_entity_count(const HrWorld* world);

/* Adds the component if missing and returns a pointer to its storage, which
 * stays valid until the component is removed or another component of the same
 * type is added/removed.  Returns NULL on invalid input / allocation failure. */
void* hr_world_add_component(HrWorld* world, HrEntity entity, u32 component_type, usize size);
void* hr_world_get_component(HrWorld* world, HrEntity entity, u32 component_type);
bool  hr_world_has_component(HrWorld* world, HrEntity entity, u32 component_type);
void  hr_world_remove_component(HrWorld* world, HrEntity entity, u32 component_type);

/* Number of live components for a type (0 if never used). */
u32 hr_world_component_count(const HrWorld* world, u32 component_type);

void hr_world_for_each_component(HrWorld* world, u32 component_type, HrEntityFn fn, void* userdata);
void hr_world_for_each_entity(HrWorld* world, HrEntityFn fn, void* userdata);

#endif /* HORIZON_ECS_H */
