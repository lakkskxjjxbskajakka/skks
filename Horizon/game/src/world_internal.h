/* Horizon Engine - internal world structures shared by world/car/mesh modules. */
#ifndef HORIZON_WORLD_INTERNAL_H
#define HORIZON_WORLD_INTERNAL_H

#include "world.h"
#include "horizon/jobs.h"
#include "horizon/mesh.h"
#include "horizon/physics.h"

#define HR_ROAD_SAMPLES 1024
#define HR_TERRAIN_SEGMENTS 176
#define HR_TERRAIN_SIZE 900.0f
#define HR_ROAD_HALF_WIDTH 4.6f
#define HR_SHOULDER_WIDTH 2.2f

typedef struct HrRoadSample {
    Vec3 pos;
    Vec3 tangent;
    Vec3 right;
    float distance;
    float curvature;
} HrRoadSample;

typedef struct HrTreeInstance {
    Vec3 pos;
    float scale;
    float rotation;
    Vec4 tint;
    int kind; /* 0 pine, 1 broadleaf */
} HrTreeInstance;

typedef struct HrRockInstance {
    Vec3 pos;
    Vec3 scale;
    float rotation;
    Vec4 tint;
} HrRockInstance;

typedef struct HrPostInstance {
    Vec3 pos;
    float rotation;
} HrPostInstance;

struct HrGameWorld {
    HrRenderer* renderer;
    HrJobSystem* jobs;

    HrRoadSample* road;
    int road_count;
    float road_length;

    HrTreeInstance* trees;
    int tree_count;
    HrRockInstance* rocks;
    int rock_count;
    HrPostInstance* posts;
    int post_count;

    int mesh_terrain;
    int mesh_road;
    int mesh_shoulder;
    int mesh_trunk;
    int mesh_pine;
    int mesh_leaf;
    int mesh_rock;
    int mesh_post;

    int mesh_car_body;
    int mesh_car_glass;
    int mesh_car_roof;
    int mesh_car_trim;
    int mesh_car_wheel;
    int mesh_car_rim;
    int mesh_car_light_w;
    int mesh_car_light_r;
    int mesh_car_metal;
    int mesh_barrel;

    Vec3 car_pos;
    float car_yaw;
    float car_speed;
    float car_steer;
    float car_wheel_spin;
    float car_y;
    float car_pitch;
    float car_roll;
    float car_bob;
    Vec3 car_forward;
    Vec3 car_right;
    bool brake_lights;
    bool boost_active;

    Vec3 cam_pos;
    Vec3 cam_target;
    float cam_fov;
    float cam_shake;

    float distance_traveled;
    float progress;
    float offroad;
    int nearest;
    float time;

    Vec3 sun_direction;
    Vec3 sun_color;
    Vec3 ambient_sky;
    Vec3 ambient_ground;
    Vec3 fog_color;
    float fog_density;

    HrPhysicsWorld* physics;
    HrBodyHandle car_body;
    HrBodyHandle* props;
    Vec3* prop_spawn;
    int prop_count;
    int prop_capacity;
};

/* world_mesh.c */
void hr_game_world_build_environment(HrGameWorld* w);
float hr_ground_height(HrGameWorld* w, float x, float z);
int hr_road_nearest(HrGameWorld* w, float x, float z, float* out_distance);
Vec3 hr_road_point(const HrGameWorld* w, float t);

/* car.c */
void hr_game_world_build_car(HrGameWorld* w);
void hr_game_world_draw_car(HrGameWorld* w, HrRenderer* r);
Mat4 hr_car_transform(const HrGameWorld* w);

#endif /* HORIZON_WORLD_INTERNAL_H */
