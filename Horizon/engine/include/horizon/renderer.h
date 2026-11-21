/* Horizon Engine - Vulkan forward+ renderer interface (PBR, CSM, HDR, bloom). */
#ifndef HORIZON_RENDERER_H
#define HORIZON_RENDERER_H

#include "horizon/core.h"
#include "horizon/math.h"
#include "horizon/mesh.h"
#include "horizon/platform.h"

typedef struct HrRenderer HrRenderer;

typedef enum HrMaterialPreset {
    HR_MAT_TERRAIN = 0,
    HR_MAT_ROAD,
    HR_MAT_CAR_PAINT,
    HR_MAT_GLASS,
    HR_MAT_RUBBER,
    HR_MAT_METAL,
    HR_MAT_FOLIAGE,
    HR_MAT_TRUNK,
    HR_MAT_ROCK,
    HR_MAT_CONCRETE,
    HR_MAT_LIGHT_WHITE,
    HR_MAT_LIGHT_RED,
    HR_MAT_SIGN,
    HR_MAT_WATER,
    HR_MAT_WINDOW,
    HR_MAT_COUNT
} HrMaterialPreset;

typedef struct HrRendererDesc {
    HrWindow* window;
    bool vsync;
    bool validation;
    int shadow_resolution;   /* 512..4096 */
} HrRendererDesc;

typedef struct HrFrameDesc {
    Vec3 camera_position;
    Vec3 camera_target;
    Vec3 camera_up;
    float fov_deg;
    float near_plane;
    float far_plane;
    Vec3 focus;

    Vec3 sun_direction;      /* direction the light travels (from sun to world) */
    Vec3 sun_color;
    float sun_intensity;
    Vec3 ambient_sky;
    Vec3 ambient_ground;

    float exposure;
    float time;
    Vec3 fog_color;
    float fog_density;

    float render_scale;      /* 0.5 .. 1.0 */
    float shadow_distance;
    int shadows_enabled;
    int bloom_enabled;
    int fxaa_enabled;
    int shadow_resolution;

    float bloom_intensity;
    float saturation;
    float contrast;
    float vignette;
    float grain;

    const char* screenshot_path;
} HrFrameDesc;

typedef struct HrRendererStats {
    double cpu_frame_ms;
    double gpu_frame_ms;
    u32 draw_calls;
    u32 instances;
    u32 triangles;
    u32 hdr_width, hdr_height;
    u32 output_width, output_height;
    u64 frame_index;
    const char* backend;
    const char* device_name;
    bool vsync;
} HrRendererStats;

bool hr_renderer_init(HrRenderer** out, const HrRendererDesc* desc);
void hr_renderer_shutdown(HrRenderer* r);
void hr_renderer_wait_idle(HrRenderer* r);

int  hr_renderer_upload_mesh(HrRenderer* r, const HrMeshData* mesh, const char* name);
bool hr_renderer_begin_frame(HrRenderer* r, const HrFrameDesc* frame);
void hr_renderer_draw(HrRenderer* r, int mesh, HrMaterialPreset material, Mat4 model, Vec4 tint);
void hr_renderer_end_frame(HrRenderer* r);
void hr_renderer_get_stats(HrRenderer* r, HrRendererStats* out);

#endif /* HORIZON_RENDERER_H */
