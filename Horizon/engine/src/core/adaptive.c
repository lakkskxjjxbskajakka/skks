/* Horizon Engine - adaptive quality controller. */
#include "horizon/adaptive.h"

#include <math.h>

void hr_adaptive_default_config(HrAdaptiveConfig* out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->target_frame_ms = 16.67f;
    out->low_frame_ms = 12.5f;
    out->high_frame_ms = 19.0f;
    out->scale_min = 0.50f;
    out->scale_max = 1.0f;
    out->scale_step_down = 0.06f;
    out->scale_step_up = 0.04f;
    out->shadow_min = 1024;
    out->shadow_max = 2048;
    out->cooldown_frames = 24;
    out->ema_alpha = 0.10f;
}

void hr_adaptive_init(HrAdaptiveState* s, const HrAdaptiveConfig* cfg, float initial_scale, int initial_shadow) {
    if (!s) return;
    memset(s, 0, sizeof(*s));
    if (cfg) s->cfg = *cfg;
    else hr_adaptive_default_config(&s->cfg);
    s->render_scale = HR_CLAMP(initial_scale, s->cfg.scale_min, s->cfg.scale_max);
    s->shadow_resolution = HR_CLAMP(initial_shadow, s->cfg.shadow_min, s->cfg.shadow_max);
    s->ema_ms = s->cfg.target_frame_ms;
}

void hr_adaptive_reset(HrAdaptiveState* s, float initial_scale, int initial_shadow) {
    if (!s) return;
    HrAdaptiveConfig cfg = s->cfg;
    hr_adaptive_init(s, &cfg, initial_scale, initial_shadow);
}

void hr_adaptive_update(HrAdaptiveState* s, float frame_ms) {
    if (!s || frame_ms <= 0.0f) return;
    float a = HR_CLAMP(s->cfg.ema_alpha, 0.01f, 1.0f);
    s->ema_ms = s->ema_ms * (1.0f - a) + frame_ms * a;
    if (s->cooldown > 0) { s->cooldown--; return; }

    if (s->ema_ms > s->cfg.high_frame_ms) {
        if (s->render_scale > s->cfg.scale_min + 0.001f) {
            s->render_scale = HR_MAX(s->cfg.scale_min, s->render_scale - s->cfg.scale_step_down);
            s->cooldown = s->cfg.cooldown_frames;
            s->degrade_count++;
        } else if (s->shadow_resolution > s->cfg.shadow_min) {
            s->shadow_resolution = HR_MAX(s->cfg.shadow_min, s->shadow_resolution / 2);
            s->cooldown = s->cfg.cooldown_frames * 2u;
            s->degrade_count++;
        }
    } else if (s->ema_ms < s->cfg.low_frame_ms) {
        if (s->shadow_resolution < s->cfg.shadow_max) {
            int next = s->shadow_resolution * 2;
            s->shadow_resolution = HR_MIN(s->cfg.shadow_max, next);
            s->cooldown = s->cfg.cooldown_frames * 3u;
            s->upgrade_count++;
        } else if (s->render_scale < s->cfg.scale_max - 0.001f) {
            s->render_scale = HR_MIN(s->cfg.scale_max, s->render_scale + s->cfg.scale_step_up);
            s->cooldown = s->cfg.cooldown_frames * 3u;
            s->upgrade_count++;
        }
    }
}
