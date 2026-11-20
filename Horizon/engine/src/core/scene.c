#include "horizon/scene.h"
#include "horizon/memory.h"
#include <string.h>

#define HR_SCENE_MAX_NODES 4096

void hr_for_each_entity_node(HrScene* scene, HrSceneNode* node, HrEntityFn fn, void* userdata);
void hr_for_each_node_from(HrSceneNode* node, HrEntityFn fn, void* userdata);
void hr_update_all_transforms_recursive(HrScene* scene, HrSceneNode* node);
void hr_update_children_transforms(HrScene* scene, HrSceneNode* node);
void hr_scene_node_free(HrSceneNode* node);

HrScene* hr_scene_create(HrWorld* world) {
    HrScene* scene = (HrScene*)hr_calloc(1, sizeof(HrScene));
    if (!scene) return NULL;
    scene->world = world;
    scene->root = NULL;
    return scene;
}

void hr_scene_destroy(HrScene* scene) {
    if (!scene) return;
    if (scene->root) hr_scene_node_free(scene->root);
    hr_free(scene);
}

void hr_scene_node_free(HrSceneNode* node) {
    if (!node) return;
    for (u32 i = 0; i < node->child_count; ++i) {
        hr_scene_node_free(node->children[i]);
    }
    hr_free(node);
}

HrEntity hr_scene_create_entity(HrScene* scene, const char* name, HrEntity parent) {
    if (!scene) return HR_ENTITY_INVALID;
    HrEntity entity = hr_world_create_entity(scene->world);
    if (entity == HR_ENTITY_INVALID) return HR_ENTITY_INVALID;
    
    HrSceneNode* node = (HrSceneNode*)hr_calloc(1, sizeof(HrSceneNode));
    if (!node) return HR_ENTITY_INVALID;
    
    node->entity = entity;
    snprintf(node->name, sizeof(node->name), "%s", name ? name : "unnamed");
    node->parent = NULL;
    node->dirty = true;
    node->transform = mat4_identity();
    node->local_transform = mat4_identity();
    
    if (scene->node_count < HR_SCENE_MAX_NODES) {
        scene->nodes[scene->node_count++] = *node;
    }
    
    if (parent != HR_ENTITY_INVALID) {
        HrSceneNode* parent_node = NULL;
        for (u32 i = 0; i < scene->node_count; ++i) {
            if (scene->nodes[i].entity == parent) {
                parent_node = &scene->nodes[i];
                break;
            }
        }
        if (parent_node) {
            if (parent_node->child_count < 8) {
                parent_node->children[parent_node->child_count++] = node;
                node->parent = parent_node;
            }
        }
    } else {
        if (!scene->root) scene->root = node;
    }
    
    return entity;
}

void hr_scene_destroy_entity(HrScene* scene, HrEntity entity) {
    if (!scene) return;
    for (i32 i = (i32)scene->node_count - 1; i >= 0; --i) {
        if (scene->nodes[i].entity == entity) {
            hr_scene_node_free(&scene->nodes[i]);
            if (i < scene->node_count - 1) {
                scene->nodes[i] = scene->nodes[scene->node_count - 1];
            }
            scene->node_count--;
            break;
        }
    }
}

void hr_scene_add_child(HrScene* scene, HrEntity child, HrEntity parent) {
    if (!scene) return;
    
    HrSceneNode* child_node = NULL;
    for (u32 i = 0; i < scene->node_count; ++i) {
        if (scene->nodes[i].entity == child) {
            child_node = &scene->nodes[i];
            break;
        }
    }
    
    HrSceneNode* parent_node = NULL;
    for (u32 i = 0; i < scene->node_count; ++i) {
        if (scene->nodes[i].entity == parent) {
            parent_node = &scene->nodes[i];
            break;
        }
    }
    
    if (child_node && parent_node && parent_node->child_count < 8) {
        parent_node->children[parent_node->child_count++] = child_node;
        child_node->parent = parent_node;
    }
}

void hr_scene_remove_child(HrScene* scene, HrEntity child) {
    if (!scene) return;
    for (i32 i = (i32)scene->node_count - 1; i >= 0; --i) {
        if (scene->nodes[i].entity == child) {
            hr_scene_node_free(&scene->nodes[i]);
            if (i < scene->node_count - 1) {
                scene->nodes[i] = scene->nodes[scene->node_count - 1];
            }
            scene->node_count--;
            break;
        }
    }
}

void hr_scene_set_transform(HrScene* scene, HrEntity entity, Mat4 transform) {
    if (!scene) return;
    for (u32 i = 0; i < scene->node_count; ++i) {
        if (scene->nodes[i].entity == entity) {
            memcpy(&scene->nodes[i].transform, &transform, sizeof(Mat4));
            scene->nodes[i].dirty = true;
            break;
        }
    }
}

Mat4 hr_scene_get_transform(HrScene* scene, HrEntity entity) {
    if (!scene) return mat4_identity();
    for (u32 i = 0; i < scene->node_count; ++i) {
        if (scene->nodes[i].entity == entity) {
            return scene->nodes[i].transform;
        }
    }
    return mat4_identity();
}

void hr_scene_update_transforms(HrScene* scene, HrEntity entity) {
    if (!scene) return;
    for (u32 i = 0; i < scene->node_count; ++i) {
        if (scene->nodes[i].entity == entity) {
            if (scene->nodes[i].parent) {
                Mat4 parent_transform = hr_scene_get_transform(scene, scene->nodes[i].parent->entity);
                for (int r = 0; r < 4; ++r) {
                    for (int c = 0; c < 4; ++c) {
                        scene->nodes[i].transform.c[r][c] = 0.0f;
                        for (int k = 0; k < 4; ++k) {
                            scene->nodes[i].transform.c[r][c] += parent_transform.c[r][k] * scene->nodes[i].local_transform.c[k][c];
                        }
                    }
                }
            }
            hr_update_children_transforms(scene, &scene->nodes[i]);
            break;
        }
    }
}

void hr_update_children_transforms(HrScene* scene, HrSceneNode* node) {
    for (u32 i = 0; i < node->child_count; ++i) {
        HrSceneNode* child = node->children[i];
        Mat4 parent_transform = hr_scene_get_transform(scene, node->entity);
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                child->transform.c[r][c] = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    child->transform.c[r][c] += parent_transform.c[r][k] * child->local_transform.c[k][c];
                }
            }
        }
        hr_update_children_transforms(scene, child);
    }
}

void hr_scene_update_all_transforms(HrScene* scene) {
    if (!scene || !scene->root) return;
    hr_update_all_transforms_recursive(scene, scene->root);
}

void hr_update_all_transforms_recursive(HrScene* scene, HrSceneNode* node) {
    if (!node) return;
    
    if (node->parent) {
        Mat4 parent_transform = hr_scene_get_transform(scene, node->parent->entity);
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                node->transform.c[r][c] = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    node->transform.c[r][c] += parent_transform.c[r][k] * node->local_transform.c[k][c];
                }
            }
        }
    }
    
    for (u32 i = 0; i < node->child_count; ++i) {
        hr_update_all_transforms_recursive(scene, node->children[i]);
    }
}

void hr_scene_for_each_node(HrScene* scene, HrEntityFn fn, void* userdata) {
    if (!scene) return;
    if (scene->root) {
        hr_for_each_node_from(scene->root, fn, userdata);
    }
}

void hr_for_each_node_from(HrSceneNode* node, HrEntityFn fn, void* userdata) {
    (void)node;
    if (!node) return;
    if (node->entity != HR_ENTITY_INVALID) {
        fn(node->entity, userdata);
    }
    for (u32 i = 0; i < node->child_count; ++i) {
        hr_for_each_node_from(node->children[i], fn, userdata);
    }
}

void hr_scene_for_each_entity_recursive(HrScene* scene, HrEntity entity, HrEntityFn fn, void* userdata) {
    if (!scene) return;
    hr_for_each_entity_node(scene, scene->root, fn, userdata);
}

void hr_for_each_entity_node(HrScene* scene, HrSceneNode* node, HrEntityFn fn, void* userdata) {
    (void)scene;
    if (!node) return;
    if (node->entity != HR_ENTITY_INVALID) {
        fn(node->entity, userdata);
    }
    for (u32 i = 0; i < node->child_count; ++i) {
        hr_for_each_entity_node(scene, node->children[i], fn, userdata);
    }
}

void hr_scene_set_local_transform(HrScene* scene, HrEntity entity, Mat4 local_transform) {
    if (!scene) return;
    for (u32 i = 0; i < scene->node_count; ++i) {
        if (scene->nodes[i].entity == entity) {
            memcpy(&scene->nodes[i].local_transform, &local_transform, sizeof(Mat4));
            scene->nodes[i].dirty = true;
            break;
        }
    }
}