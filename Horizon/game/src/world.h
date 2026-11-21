/* Horizon Engine - procedural driving world + arcade vehicle. */
#ifndef HORIZON_WORLD_H
#define HORIZON_WORLD_H

#include "horizon/renderer.h"
#include "horizon/jobs.h"

typedef struct HrGameWorld HrGameWorld;

typedef struct HrGameWorldInput {
    float throttle;   /* -1 reverse .. 1 forward */
    float steer;      /* -1 left .. 1 right */
    float brake;      /* 0 .. 1 */
    bool handbrake;
    bool boost;
    bool autopilot;
} HrGameWorldInput;

bool hr_game_world_create(HrGameWorld** out, HrRenderer* r, HrJobSystem* js);
void hr_game_world_destroy(HrGameWorld* w);
void hr_game_world_reset(HrGameWorld* w);
void hr_game_world_update(HrGameWorld* w, float dt, const HrGameWorldInput* input);
void hr_game_world_render(HrGameWorld* w);

/* Fills the per-frame renderer description with camera, sun and atmosphere. */
void hr_game_world_describe_frame(HrGameWorld* w, HrFrameDesc* frame, float render_scale, float time);

float hr_game_world_speed_kph(const HrGameWorld* w);
float hr_game_world_distance(const HrGameWorld* w);
Vec3  hr_game_world_car_position(const HrGameWorld* w);
float hr_game_world_progress(const HrGameWorld* w);
const char* hr_game_world_area(const HrGameWorld* w);

#endif /* HORIZON_WORLD_H */
