/* Horizon Engine - fixed-capacity scene graph with cached transforms.
 *
 * Nodes live in a contiguous array owned by the scene; parent/child links are
 * indices rather than pointers, so the structure is stable across loads and
 * cannot dangle.  A node may have any number of children (sibling list).
 */
#ifndef HORIZON_SCENE_H
#define HORIZON_SCENE_H

#include "horizon/core.h"
#include "horizon/math.h"
#include "horizon/memory.h"
#include "horizon/ecs.h"

#define HR_SCENE_MAX_NODES 4096

typedef struct HrSceneNode {
    HrEntity entity;
    char name[64];
    i32 parent;        /* node index or -1 */
    i32 first_child;   /* node index or -1 */
    i32 last_child;    /* node index or -1 */
    i32 next_sibling;  /* node index or -1 */
    i32 prev_sibling;  /* node index or -1 */
    u32 child_count;
    Mat4 transform;
    Mat4 local_transform;
    u64 cached_version;
    bool transform_valid;
    bool alive;
} HrSceneNode;

typedef struct HrScene {
    HrWorld* world;
    HrSceneNode* root;  /* first root node, or NULL */
    u32 node_count;
    i32 roots;          /* first root node index, or -1 */
    i32 root_tail;      /* last root node index, or -1 */
    i32 free_list;      /* first free node index, or -1 */
    u64 transform_version;
    i32 entity_to_node[HR_ECS_MAX_ENTITIES];
    HrSceneNode nodes[HR_SCENE_MAX_NODES];
} HrScene;

HrScene* hr_scene_create(HrWorld* world);
void hr_scene_destroy(HrScene* scene);

HrEntity hr_scene_create_entity(HrScene* scene, const char* name, HrEntity parent);
/* Destroys the entity together with its whole subtree. */
void hr_scene_destroy_entity(HrScene* scene, HrEntity entity);

/* Replaces the node's local transform and updates the cached world transform. */
void hr_scene_set_transform(HrScene* scene, HrEntity entity, Mat4 transform);
void hr_scene_set_local_transform(HrScene* scene, HrEntity entity, Mat4 local_transform);
Mat4 hr_scene_get_transform(HrScene* scene, HrEntity entity);
Mat4 hr_scene_get_local_transform(HrScene* scene, HrEntity entity);

/* Reparents a node, detaching it from its previous parent/root first. */
void hr_scene_add_child(HrScene* scene, HrEntity child, HrEntity parent);
/* Detaches a node (and its subtree) from its parent; it becomes a root. */
void hr_scene_remove_child(HrScene* scene, HrEntity child);

void hr_scene_update_transforms(HrScene* scene, HrEntity entity);
void hr_scene_update_all_transforms(HrScene* scene);

void hr_scene_for_each_node(HrScene* scene, HrEntityFn fn, void* userdata);
void hr_scene_for_each_entity_recursive(HrScene* scene, HrEntity entity, HrEntityFn fn, void* userdata);

u32 hr_scene_node_count(const HrScene* scene);
HrEntity hr_scene_find(const HrScene* scene, const char* name);

#endif /* HORIZON_SCENE_H */
