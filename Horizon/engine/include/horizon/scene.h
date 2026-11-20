#ifndef HORIZON_SCENE_H
#define HORIZON_SCENE_H

#include "horizon/ecs.h"
#include "horizon/math.h"
#include "horizon/mesh.h"

typedef struct HrSceneNode HrSceneNode;

struct HrSceneNode {
    HrEntity entity;
    char name[64];
    HrSceneNode* parent;
    hr_dynarray(HrSceneNode*) children;
    hr_dynarray(HrSceneNode*) siblings;
    
    Mat4 transform;
    Mat4 local_transform;
    bool dirty;
};

typedef struct HrScene {
    HrWorld* world;
    HrSceneNode* root;
    hr_dynarray(HrSceneNode*) nodes;
    bool loaded;
} HrScene;

HrScene* hr_scene_create(HrWorld* world);
void hr_scene_destroy(HrScene* scene);

HrEntity hr_scene_create_entity(HrScene* scene, const char* name, HrEntity parent);
void hr_scene_destroy_entity(HrScene* scene, HrEntity entity);

void hr_scene_set_transform(HrScene* scene, HrEntity entity, Mat4 transform);
Mat4 hr_scene_get_transform(HrScene* scene, HrEntity entity);

void hr_scene_add_child(HrScene* scene, HrEntity child, HrEntity parent);
void hr_scene_remove_child(HrScene* scene, HrEntity child);

void hr_scene_update_transforms(HrScene* scene, HrEntity entity);
void hr_scene_update_all_transforms(HrScene* scene);

void hr_scene_for_each_node(HrScene* scene, HrEntityFn fn, void* userdata);
void hr_scene_for_each_entity_recursive(HrScene* scene, HrEntity entity, HrEntityFn fn, void* userdata);

#endif /* HORIZON_SCENE_H */