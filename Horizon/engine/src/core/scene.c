#include "horizon/scene.h"
#include "horizon/memory.h"

HrScene* hr_scene_create(HrWorld* world) {
    HrScene* scene = (HrScene*)hr_calloc(1, sizeof(HrScene));
    if (!scene) return NULL;
    scene->world = world;
    scene->root = NULL;
    scene->loaded = false;
    hr_da_init(&scene->nodes);
    return scene;
}

void hr_scene_destroy(HrScene* scene) {
    if (!scene) return;
    if (scene->root) hr_scene_node_free(scene->root);
    hr_da_free(&scene->nodes);
    hr_free(scene);
}

static void hr_scene_node_free(HrSceneNode* node) {
    if (!node) return;
    for (u32 i = 0; i < hr_da_len(node->children); ++i) {
        hr_scene_node_free(node->children[i]);
    }
    hr_da_free(node->children);
    hr_da_free(node->siblings);
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
    hr_da_init(node->children);
    hr_da_init(node->siblings);
    mat4_identity(node->transform);
    mat4_identity(node->local_transform);
    
    hr_da_push(scene->nodes, node);
    
    if (parent != HR_ENTITY_INVALID) {
        HrSceneNode* parent_node = NULL;
        /* Find parent node */
        for (u32 i = 0; i < hr_da_len(scene->nodes); ++i) {
            if (scene->nodes[i]->entity == parent) {
                parent_node = scene->nodes[i];
                break;
            }
        }
        if (parent_node) {
            hr_scene_add_child_node(scene, node, parent_node);
        }
    } else {
        /* Set as root */
        if (!scene->root) scene->root = node;
    }
    
    return entity;
}

void hr_scene_destroy_entity(HrScene* scene, HrEntity entity) {
    if (!scene) return;
    /* Find and remove the node */
    HrSceneNode* target = NULL;
    HrSceneNode* prev = NULL;
    for (u32 i = 0; i < hr_da_len(scene->nodes); ++i) {
        if (scene->nodes[i]->entity == entity) {
            target = scene->nodes[i];
            if (prev) prev->next_sibling = target->next_sibling; /* simplified */
            break;
        }
        prev = scene->nodes[i];
    }
    if (target) {
        hr_scene_node_free(target);
        /* Remove from nodes array - simplified, just mark as freed */
    }
}

void hr_scene_add_child(HrScene* scene, HrEntity child, HrEntity parent) {
    if (!scene) return;
    
    /* Find child node */
    HrSceneNode* child_node = NULL;
    for (u32 i = 0; i < hr_da_len(scene->nodes); ++i) {
        if (scene->nodes[i]->entity == child) {
            child_node = scene->nodes[i];
            break;
        }
    }
    
    /* Find parent node */
    HrSceneNode* parent_node = NULL;
    for (u32 i = 0; i < hr_da_len(scene->nodes); ++i) {
        if (scene->nodes[i]->entity == parent) {
            parent_node = scene->nodes[i];
            break;
        }
    }
    
    if (child_node && parent_node) {
        hr_scene_add_child_node(scene, child_node, parent_node);
    }
}

static void hr_scene_add_child_node(HrScene* scene, HrSceneNode* child, HrSceneNode* parent) {
    child->parent = parent;
    hr_da_push(parent->children, child);
    /* Add child to parent's sibling list at the end */
    if (parent->siblings.cap > 0) {
        u32 last = hr_da_len(parent->siblings) - 1;
        /* simple linking */
    }
}

void hr_scene_remove_child(HrScene* scene, HrEntity child) {
    if (!scene) return;
    /* Find and remove child node */
    for (u32 i = 0; i < hr_da_len(scene->nodes); ++i) {
        if (scene->nodes[i]->entity == child) {
            hr_scene_node_free(scene->nodes[i]);
            /* Remove from array - simplified */
            break;
        }
    }
}

void hr_scene_set_transform(HrScene* scene, HrEntity entity, Mat4 transform) {
    if (!scene) return;
    /* Find node and set transform, mark dirty */
    for (u32 i = 0; i < hr_da_len(scene->nodes); ++i) {
        if (scene->nodes[i]->entity == entity) {
            memcpy(&scene->nodes[i]->transform, transform, sizeof(Mat4));
            scene->nodes[i]->dirty = true;
            break;
        }
    }
}

Mat4 hr_scene_get_transform(HrScene* scene, HrEntity entity) {
    if (!scene) return mat4_identity();
    /* Find node and get transform */
    for (u32 i = 0; i < hr_da_len(scene->nodes); ++i) {
        if (scene->nodes[i]->entity == entity) {
            return scene->nodes[i]->transform;
        }
    }
    return mat4_identity();
}

void hr_scene_update_transforms(HrScene* scene, HrEntity entity) {
    if (!scene) return;
    /* Update transform for entity and children */
    for (u32 i = 0; i < hr_da_len(scene->nodes); ++i) {
        if (scene->nodes[i]->entity == entity) {
            /* Calculate world transform */
            if (scene->nodes[i]->parent) {
                Mat4 parent_transform = hr_scene_get_transform(scene, scene->nodes[i]->parent->entity);
                /* Multiply: world = parent * local */
                for (int r = 0; r < 4; ++r) {
                    for (int c = 0; c < 4; ++c) {
                        scene->nodes[i]->transform.c[r][c] = 0.0f;
                        for (int k = 0; k < 4; ++k) {
                            scene->nodes[i]->transform.c[r][c] += parent_transform.c[r][k] * scene->nodes[i]->local_transform.c[k][c];
                        }
                    }
                }
            }
            hr_scene_update_all_transforms_recursive(scene, scene->nodes[i]);
            break;
        }
    }
}

void hr_scene_update_all_transforms(HrScene* scene) {
    if (!scene || !scene->root) return;
    hr_scene_update_all_transforms_recursive(scene, scene->root);
}

static void hr_scene_update_all_transforms_recursive(HrScene* scene, HrSceneNode* node) {
    if (!node) return;
    
    /* Calculate world transform */
    if (node->parent) {
        Mat4 parent_transform = hr_scene_get_transform(scene, node->parent->entity);
        Mat4 local = node->local_transform;
        /* world = parent * local */
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                node->transform.c[r][c] = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    node->transform.c[r][c] += parent_transform.c[r][k] * local.c[k][c];
                }
            }
        }
    }
    
    /* Recurse children */
    for (u32 i = 0; i < hr_da_len(node->children); ++i) {
        hr_scene_update_all_transforms_recursive(scene, node->children[i]);
    }
}

void hr_scene_for_each_node(HrScene* scene, HrEntityFn fn, void* userdata) {
    if (!scene) return;
    hr_scene_for_each_node_recursive(scene->root, fn, userdata);
}

static void hr_scene_for_each_node_recursive(HrSceneNode* node, HrEntityFn fn, void* userdata) {
    if (!node) return;
    if (node->entity != HR_ENTITY_INVALID) {
        fn(node->entity, userdata);
    }
    for (u32 i = 0; i < hr_da_len(node->children); ++i) {
        hr_scene_for_each_node_recursive(node->children[i], fn, userdata);
    }
}

void hr_scene_for_each_entity_recursive(HrScene* scene, HrEntity entity, HrEntityFn fn, void* userdata) {
    if (!scene) return;
    hr_scene_for_each_entity_recursive_node(scene, entity, fn, userdata);
}

static void hr_scene_for_each_entity_recursive_node(HrScene* scene, HrEntity entity, HrEntityFn fn, void* userdata) {
    /* Find node with this entity */
    for (u32 i = 0; i < hr_da_len(scene->nodes); ++i) {
        if (scene->nodes[i]->entity == entity) {
            /* Process this node and all children */
            hr_scene_for_each_entity_recursive_node_helper(scene, scene->nodes[i], fn, userdata);
            return;
        }
    }
    /* Try all nodes */
    for (u32 i = 0; i < hr_da_len(scene->nodes); ++i) {
        hr_scene_for_each_entity_recursive_node_helper(scene, scene->nodes[i], fn, userdata);
    }
}

static void hr_scene_for_each_entity_recursive_node_helper(HrScene* scene, HrSceneNode* node, HrEntityFn fn, void* userdata) {
    if (!node) return;
    fn(node->entity, userdata);
    for (u32 i = 0; i < hr_da_len(node->children); ++i) {
        hr_scene_for_each_entity_recursive_node_helper(scene, node->children[i], fn, userdata);
    }
}