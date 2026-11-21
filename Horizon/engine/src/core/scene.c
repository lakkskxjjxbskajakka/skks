#include "horizon/scene.h"
#include "horizon/memory.h"
#include <string.h>

static i32 node_index(const HrScene* scene, HrEntity entity) {
    if (!scene || entity == HR_ENTITY_INVALID || entity >= HR_ECS_MAX_ENTITIES) return -1;
    i32 idx = scene->entity_to_node[entity];
    if (idx < 0 || idx >= HR_SCENE_MAX_NODES || !scene->nodes[idx].alive) return -1;
    return idx;
}

static void refresh_root(HrScene* scene) {
    scene->root = scene->roots >= 0 ? &scene->nodes[scene->roots] : NULL;
}

static void append_child(HrScene* scene, i32 parent, i32 child) {
    HrSceneNode* p = &scene->nodes[parent];
    HrSceneNode* c = &scene->nodes[child];
    c->parent = parent;
    c->next_sibling = -1;
    c->prev_sibling = p->last_child;
    if (p->last_child >= 0) scene->nodes[p->last_child].next_sibling = child;
    else p->first_child = child;
    p->last_child = child;
    p->child_count++;
}

static void append_root(HrScene* scene, i32 idx) {
    HrSceneNode* n = &scene->nodes[idx];
    n->parent = -1;
    n->next_sibling = -1;
    n->prev_sibling = scene->root_tail;
    if (scene->root_tail >= 0) scene->nodes[scene->root_tail].next_sibling = idx;
    else scene->roots = idx;
    scene->root_tail = idx;
}

static void unlink_node(HrScene* scene, i32 idx) {
    HrSceneNode* n = &scene->nodes[idx];
    if (n->parent >= 0) {
        HrSceneNode* p = &scene->nodes[n->parent];
        if (p->first_child == idx) p->first_child = n->next_sibling;
        if (p->last_child == idx) p->last_child = n->prev_sibling;
        if (p->child_count) p->child_count--;
    } else {
        if (scene->roots == idx) scene->roots = n->next_sibling;
        if (scene->root_tail == idx) scene->root_tail = n->prev_sibling;
    }
    if (n->prev_sibling >= 0) scene->nodes[n->prev_sibling].next_sibling = n->next_sibling;
    if (n->next_sibling >= 0) scene->nodes[n->next_sibling].prev_sibling = n->prev_sibling;
    n->parent = -1;
    n->prev_sibling = -1;
    n->next_sibling = -1;
}

static void ensure_node(HrScene* scene, i32 idx) {
    HrSceneNode* n = &scene->nodes[idx];
    if (n->transform_valid && n->cached_version == scene->transform_version) return;
    Mat4 parent = mat4_identity();
    if (n->parent >= 0) {
        ensure_node(scene, n->parent);
        parent = scene->nodes[n->parent].transform;
    }
    n->transform = mat4_mul(parent, n->local_transform);
    n->cached_version = scene->transform_version;
    n->transform_valid = true;
}

static void recompute_subtree(HrScene* scene, i32 idx) {
    ensure_node(scene, idx);
    for (i32 c = scene->nodes[idx].first_child; c >= 0; c = scene->nodes[c].next_sibling)
        recompute_subtree(scene, c);
}

HrScene* hr_scene_create(HrWorld* world) {
    HrScene* scene = (HrScene*)hr_calloc(1, sizeof(HrScene));
    if (!scene) return NULL;
    scene->world = world;
    scene->roots = -1;
    scene->root_tail = -1;
    scene->free_list = 0;
    for (i32 i = 0; i < HR_ECS_MAX_ENTITIES; ++i) scene->entity_to_node[i] = -1;
    for (i32 i = 0; i < HR_SCENE_MAX_NODES; ++i) {
        scene->nodes[i].parent = -1;
        scene->nodes[i].first_child = -1;
        scene->nodes[i].last_child = -1;
        scene->nodes[i].next_sibling = (i + 1 < HR_SCENE_MAX_NODES) ? i + 1 : -1;
        scene->nodes[i].prev_sibling = -1;
    }
    return scene;
}

void hr_scene_destroy(HrScene* scene) {
    if (!scene) return;
    hr_free(scene);
}

HrEntity hr_scene_create_entity(HrScene* scene, const char* name, HrEntity parent) {
    if (!scene || !scene->world || scene->free_list < 0) return HR_ENTITY_INVALID;
    HrEntity entity = hr_world_create_entity(scene->world);
    if (entity == HR_ENTITY_INVALID) return HR_ENTITY_INVALID;

    i32 idx = scene->free_list;
    HrSceneNode* node = &scene->nodes[idx];
    scene->free_list = node->next_sibling;
    memset(node, 0, sizeof(*node));
    node->entity = entity;
    snprintf(node->name, sizeof(node->name), "%s", name ? name : "unnamed");
    node->parent = -1;
    node->first_child = -1;
    node->last_child = -1;
    node->next_sibling = -1;
    node->prev_sibling = -1;
    node->transform = mat4_identity();
    node->local_transform = mat4_identity();
    node->transform_valid = false;
    node->alive = true;
    scene->entity_to_node[entity] = idx;
    scene->node_count++;

    i32 parent_idx = node_index(scene, parent);
    if (parent_idx >= 0) append_child(scene, parent_idx, idx);
    else append_root(scene, idx);
    refresh_root(scene);
    return entity;
}

static void destroy_subtree(HrScene* scene, i32 idx) {
    HrSceneNode* node = &scene->nodes[idx];
    i32 child = node->first_child;
    while (child >= 0) {
        i32 next = scene->nodes[child].next_sibling;
        destroy_subtree(scene, child);
        child = next;
    }
    unlink_node(scene, idx);
    if (scene->entity_to_node[node->entity] == idx)
        scene->entity_to_node[node->entity] = -1;
    hr_world_destroy_entity(scene->world, node->entity);
    node->alive = false;
    node->transform_valid = false;
    node->next_sibling = scene->free_list;
    node->prev_sibling = -1;
    node->parent = -1;
    node->first_child = -1;
    node->last_child = -1;
    scene->free_list = idx;
    if (scene->node_count) scene->node_count--;
    refresh_root(scene);
}

void hr_scene_destroy_entity(HrScene* scene, HrEntity entity) {
    i32 idx = node_index(scene, entity);
    if (idx >= 0) destroy_subtree(scene, idx);
}

void hr_scene_set_local_transform(HrScene* scene, HrEntity entity, Mat4 local_transform) {
    i32 idx = node_index(scene, entity);
    if (idx < 0) return;
    scene->nodes[idx].local_transform = local_transform;
    scene->nodes[idx].transform_valid = false;
    scene->transform_version++;
}

void hr_scene_set_transform(HrScene* scene, HrEntity entity, Mat4 transform) {
    hr_scene_set_local_transform(scene, entity, transform);
}

Mat4 hr_scene_get_local_transform(HrScene* scene, HrEntity entity) {
    i32 idx = node_index(scene, entity);
    return idx >= 0 ? scene->nodes[idx].local_transform : mat4_identity();
}

Mat4 hr_scene_get_transform(HrScene* scene, HrEntity entity) {
    i32 idx = node_index(scene, entity);
    if (idx < 0) return mat4_identity();
    ensure_node(scene, idx);
    return scene->nodes[idx].transform;
}

void hr_scene_add_child(HrScene* scene, HrEntity child, HrEntity parent) {
    i32 cidx = node_index(scene, child);
    i32 pidx = node_index(scene, parent);
    if (cidx < 0 || pidx < 0 || cidx == pidx) return;
    for (i32 it = pidx; it >= 0; it = scene->nodes[it].parent)
        if (it == cidx) return;
    unlink_node(scene, cidx);
    append_child(scene, pidx, cidx);
    refresh_root(scene);
    scene->nodes[cidx].transform_valid = false;
    scene->transform_version++;
}

void hr_scene_remove_child(HrScene* scene, HrEntity child) {
    i32 idx = node_index(scene, child);
    if (idx < 0 || scene->nodes[idx].parent < 0) return;
    unlink_node(scene, idx);
    append_root(scene, idx);
    refresh_root(scene);
    scene->nodes[idx].transform_valid = false;
    scene->transform_version++;
}

void hr_scene_update_transforms(HrScene* scene, HrEntity entity) {
    i32 idx = node_index(scene, entity);
    if (idx >= 0) recompute_subtree(scene, idx);
}

void hr_scene_update_all_transforms(HrScene* scene) {
    if (!scene) return;
    for (i32 r = scene->roots; r >= 0; r = scene->nodes[r].next_sibling)
        recompute_subtree(scene, r);
}

static void for_each_recursive(HrScene* scene, i32 idx, HrEntityFn fn, void* userdata) {
    fn(scene->nodes[idx].entity, userdata);
    for (i32 c = scene->nodes[idx].first_child; c >= 0; c = scene->nodes[c].next_sibling)
        for_each_recursive(scene, c, fn, userdata);
}

void hr_scene_for_each_node(HrScene* scene, HrEntityFn fn, void* userdata) {
    if (!scene || !fn) return;
    for (i32 r = scene->roots; r >= 0; r = scene->nodes[r].next_sibling)
        for_each_recursive(scene, r, fn, userdata);
}

void hr_scene_for_each_entity_recursive(HrScene* scene, HrEntity entity, HrEntityFn fn, void* userdata) {
    i32 idx = node_index(scene, entity);
    if (idx >= 0 && fn) for_each_recursive(scene, idx, fn, userdata);
}

u32 hr_scene_node_count(const HrScene* scene) { return scene ? scene->node_count : 0; }

HrEntity hr_scene_find(const HrScene* scene, const char* name) {
    if (!scene || !name) return HR_ENTITY_INVALID;
    for (i32 i = 0; i < HR_SCENE_MAX_NODES; ++i) {
        if (scene->nodes[i].alive && strcmp(scene->nodes[i].name, name) == 0)
            return scene->nodes[i].entity;
    }
    return HR_ENTITY_INVALID;
}
