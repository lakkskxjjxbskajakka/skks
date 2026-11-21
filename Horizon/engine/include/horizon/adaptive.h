/* Horizon Engine - adaptive quality controller. */
#ifndef HORIZON_ADAPTIVE_H
#define HORIZON_ADAPTIVE_H

#include "horizon/core.h"

typedef struct HrAdaptiveConfig {
    float target_frame_ms;
    float low_frame_ms;
    float high_frame_ms;
    float scale_min;
    float scale_max;
    float scale_step_down;
    float scale_step_up;
    int shadow_min;
    int shadow_max;
    u32 cooldown_frames;
    float ema_alpha;
} HrAdaptiveConfig;

typedef struct HrAdaptiveState {
    HrAdaptiveConfig cfg;
    float ema_ms;
    float render_scale;
    int shadow_resolution;
    u32 cooldown;
    u32 degrade_count;
    u32 upgrade_count;
} HrAdaptiveState;

void hr_adaptive_default_config(HrAdaptiveConfig* out);
void hr_adaptive_init(HrAdaptiveState* s, const HrAdaptiveConfig* cfg, float initial_scale, int initial_shadow);
void hr_adaptive_reset(HrAdaptiveState* s, float initial_scale, int initial_shadow);
void hr_adaptive_update(HrAdaptiveState* s, float frame_ms);

#endif /* HORIZON_ADAPTIVE_H */
