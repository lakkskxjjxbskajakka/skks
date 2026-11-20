/* Horizon Engine - procedural driving world + arcade vehicle. */
#ifndef HORIZON_WORLD_H
#define HORIZON_WORLD_H

#include "horizon/renderer.h"
#include "horizon/jobs.h"

typedef struct HrWorld HrWorld;

typedef struct HrWorldInput {
    float throttle;   /* -1 reverse .. 1 forward */
    float steer;      /* -1 left .. 1 right */
    float brake;      /* 0 .. 1 */
    bool handbrake;
    bool boost;
    bool autopilot;
} HrWorldInput;

bool hr_world_create(HrWorld** out, HrRenderer* r, HrJobSystem* js);
void hr_world_destroy(HrWorld* w);
void hr_world_reset(HrWorld* w);
void hr_world_update(HrWorld* w, float dt, const HrWorldInput* input);
void hr_world_render(HrWorld* w);

/* Fills the per-frame renderer description with camera, sun and atmosphere. */
void hr_world_describe_frame(HrWorld* w, HrFrameDesc* frame, float render_scale, float time);

float hr_world_speed_kph(const HrWorld* w);
float hr_world_distance(const HrWorld* w);
Vec3  hr_world_car_position(const HrWorld* w);
float hr_world_progress(const HrWorld* w);
const char* hr_world_area(const HrWorld* w);

#endif /* HORIZON_WORLD_H */
