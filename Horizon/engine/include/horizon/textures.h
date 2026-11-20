/* Horizon Engine - procedural PBR texture generation (CPU, load-time only). */
#ifndef HORIZON_TEXTURES_H
#define HORIZON_TEXTURES_H

#include "horizon/core.h"

typedef enum HrTexturePreset {
    HR_TEX_ASPHALT = 0,
    HR_TEX_GRASS,
    HR_TEX_ROCK,
    HR_TEX_CONCRETE,
    HR_TEX_METAL,
    HR_TEX_PAINT,
    HR_TEX_FOLIAGE,
    HR_TEX_TRUNK,
    HR_TEX_GLASS,
    HR_TEX_PRESET_COUNT
} HrTexturePreset;

typedef struct HrImage {
    u8* rgba;
    int width;
    int height;
} HrImage;

typedef struct HrTextureSet {
    HrImage albedo;  /* sRGB   */
    HrImage normal;  /* linear */
    HrImage orm;     /* linear: R=AO, G=roughness, B=metal */
    const char* name;
} HrTextureSet;

void hr_texture_set_generate(HrTextureSet* set, HrTexturePreset preset, int size, u32 seed);
void hr_image_free(HrImage* img);

#endif /* HORIZON_TEXTURES_H */
