#include "horizon/textures.h"
#include "horizon/math.h"
#include "horizon/memory.h"

static u32 wrapi(int v, int period) {
    if (period <= 0) return 0;
    int m = v % period;
    return (u32)(m < 0 ? m + period : m);
}

static float hashf(int x, int y, u32 seed) {
    u32 h = hr_hash_combine((u32)x, (u32)y);
    h = hr_hash_combine(h, seed);
    return hr_hash_float(h);
}

static float noise2p(float x, float y, int period, u32 seed) {
    int ix = (int)floorf(x), iy = (int)floorf(y);
    float fx = x - (float)ix, fy = y - (float)iy;
    float ux = fx * fx * (3.0f - 2.0f * fx);
    float uy = fy * fy * (3.0f - 2.0f * fy);
    int x0 = (int)wrapi(ix, period), x1 = (int)wrapi(ix + 1, period);
    int y0 = (int)wrapi(iy, period), y1 = (int)wrapi(iy + 1, period);
    float a = hashf(x0, y0, seed), b = hashf(x1, y0, seed);
    float c = hashf(x0, y1, seed), d = hashf(x1, y1, seed);
    return hr_lerp(hr_lerp(a, b, ux), hr_lerp(c, d, ux), uy) * 2.0f - 1.0f;
}

static float fbm2p(float x, float y, int octaves, int period, u32 seed) {
    float sum = 0.0f, amp = 0.5f;
    int p = period;
    for (int i = 0; i < octaves; ++i) {
        sum += noise2p(x, y, p, seed + (u32)i * 73u) * amp;
        x *= 2.0f; y *= 2.0f;
        p *= 2;
        amp *= 0.5f;
    }
    return sum;
}


static float height_of(HrTexturePreset preset, float u, float v, int period, u32 seed) {
    float x = u * (float)period, y = v * (float)period;
    switch (preset) {
        case HR_TEX_ASPHALT: {
            float f = fbm2p(x * 4.0f, y * 4.0f, 4, period * 4, seed + 11u);
            float g = noise2p(x * 26.0f, y * 26.0f, period * 26, seed + 12u);
            return f * 0.35f + g * 0.35f;
        }
        case HR_TEX_GRASS: {
            float r = fbm2p(x * 1.7f, y * 1.7f, 4, period, seed + 13u);
            float detail = noise2p(x * 9.0f, y * 9.0f, period * 9, seed + 14u);
            return r * 0.7f + detail * 0.3f;
        }
        case HR_TEX_ROCK: {
            float r = fbm2p(x * 2.2f, y * 2.2f, 5, period, seed + 15u);
            float detail = noise2p(x * 13.0f, y * 13.0f, period * 13, seed + 16u);
            return fabsf(r) * 0.7f + detail * 0.3f;
        }
        case HR_TEX_CONCRETE: {
            float base = fbm2p(x * 0.6f, y * 0.6f, 4, period, seed) * 0.6f;
            float crack = 1.0f - fabsf(noise2p(x * 2.0f, y * 2.0f, period * 2, seed + 9u));
            crack = powf(HR_MAX(crack, 0.0f), 12.0f);
            return base - crack * 0.6f;
        }
        case HR_TEX_METAL: {
            float brushed = noise2p(x * 40.0f, y * 0.6f, period * 40, seed);
            float base = fbm2p(x * 0.5f, y * 0.5f, 3, period / 2, seed + 2u);
            return brushed * 0.65f + base * 0.35f;
        }
        case HR_TEX_PAINT: {
            float flake = noise2p(x * 20.0f, y * 20.0f, period * 20, seed + 4u);
            float low = fbm2p(x * 0.4f, y * 0.4f, 3, period / 2, seed);
            return flake * 0.08f + low * 0.04f;
        }
        case HR_TEX_FOLIAGE: {
            float clumps = fbm2p(x * 1.1f, y * 1.1f, 5, period, seed);
            float leaves = noise2p(x * 14.0f, y * 14.0f, period * 14, seed + 6u);
            return clumps * 0.55f + leaves * 0.45f;
        }
        case HR_TEX_TRUNK: {
            float vertical = noise2p(x * 4.0f, y * 1.2f, period * 4, seed);
            float bark = fbm2p(x * 2.0f, y * 2.0f, 5, period, seed + 8u);
            return vertical * 0.5f + bark * 0.5f;
        }
        case HR_TEX_GLASS: {
            return noise2p(x * 0.8f, y * 0.8f, period, seed) * 0.05f;
        }
        default:
            return 0.0f;
    }
}

static void color_of(HrTexturePreset preset, float u, float v, float h, int period, u32 seed,
                     float* r, float* g, float* b, float* rough, float* metal, float* ao) {
    float x = u * (float)period, y = v * (float)period;
    float n = (h + 1.0f) * 0.5f;
    *r = *g = *b = 0.5f;
    *rough = 0.8f; *metal = 0.0f; *ao = 1.0f;
    switch (preset) {
        case HR_TEX_ASPHALT: {
            float tint = 0.085f + n * 0.055f;
            *r = tint * 1.03f; *g = tint * 1.02f; *b = tint * 1.05f;
            float speck = noise2p(x * 30.0f, y * 30.0f, period * 30, seed + 31u);
            if (speck > 0.82f) { *r += 0.06f; *g += 0.06f; *b += 0.06f; }
            *rough = HR_CLAMP(0.72f + n * 0.18f - (speck > 0.8f ? 0.1f : 0.0f), 0.5f, 0.98f);
            *metal = 0.0f;
            *ao = 0.85f + n * 0.15f;
            break;
        }
        case HR_TEX_GRASS: {
            float dry = fbm2p(x * 0.5f, y * 0.5f, 3, period / 2, seed + 17u);
            float green = 0.12f + n * 0.12f;
            *r = green * 0.55f * (0.75f + dry * 0.25f);
            *g = 0.165f + n * 0.17f;
            *b = 0.045f + n * 0.035f;
            if (dry > 0.25f) { *r += dry * 0.06f; *g += dry * 0.02f; *b -= 0.005f; }
            *rough = 0.88f; *metal = 0.0f; *ao = 0.85f + n * 0.15f;
            break;
        }
        case HR_TEX_ROCK: {
            float shade = 0.20f + n * 0.18f;
            *r = shade * 1.02f; *g = shade * 0.98f; *b = shade * 0.93f;
            *rough = 0.70f + n * 0.20f; *metal = 0.0f; *ao = 0.75f + n * 0.25f;
            break;
        }
        case HR_TEX_CONCRETE: {
            float shade = 0.38f + n * 0.18f;
            *r = shade; *g = shade * 0.99f; *b = shade * 0.96f;
            *rough = 0.82f; *metal = 0.0f; *ao = 0.88f + n * 0.12f;
            break;
        }
        case HR_TEX_METAL: {
            float shade = 0.55f + n * 0.16f;
            *r = shade; *g = shade; *b = shade * 1.02f;
            *rough = 0.22f + (n * 0.5f + 0.5f) * 0.22f;
            *metal = 1.0f; *ao = 1.0f;
            break;
        }
        case HR_TEX_PAINT: {
            *r = 0.92f; *g = 0.92f; *b = 0.93f;
            *rough = 0.20f + (n * 0.5f + 0.5f) * 0.08f;
            *metal = 0.65f; *ao = 1.0f;
            break;
        }
        case HR_TEX_FOLIAGE: {
            float green = 0.09f + n * 0.13f;
            *r = green * 0.42f; *g = 0.115f + n * 0.18f; *b = green * 0.36f;
            *rough = 0.78f; *metal = 0.0f; *ao = 0.66f + n * 0.34f;
            break;
        }
        case HR_TEX_TRUNK: {
            float shade = 0.11f + n * 0.10f;
            *r = shade * 1.35f; *g = shade * 0.9f; *b = shade * 0.6f;
            *rough = 0.9f; *metal = 0.0f; *ao = 0.75f + n * 0.25f;
            break;
        }
        case HR_TEX_GLASS: {
            *r = 0.03f; *g = 0.035f; *b = 0.045f;
            *rough = 0.04f; *metal = 0.0f; *ao = 1.0f;
            break;
        }
        default: break;
    }
}

static u8 lin_to_srgb8(float c) {
    if (c < 0.0f) c = 0.0f;
    if (c > 1.0f) c = 1.0f;
    float s = c <= 0.0031308f ? 12.92f * c : 1.055f * powf(c, 1.0f / 2.4f) - 0.055f;
    return (u8)HR_CLAMP(s * 255.0f + 0.5f, 0.0f, 255.0f);
}

static void generate_image(HrImage* img, int size) {
    img->width = size;
    img->height = size;
    img->rgba = (u8*)hr_alloc((usize)size * (usize)size * 4u);
    HR_ASSERT(img->rgba);
}

void hr_texture_set_generate(HrTextureSet* set, HrTexturePreset preset, int size, u32 seed) {
    if (size < 64) size = 64;
    memset(set, 0, sizeof(*set));
    static const char* names[] = {
        "asphalt", "grass", "rock", "concrete", "metal", "paint", "foliage", "trunk", "glass"
    };
    set->name = (preset >= 0 && preset < HR_TEX_PRESET_COUNT) ? names[preset] : "unknown";

    generate_image(&set->albedo, size);
    generate_image(&set->normal, size);
    generate_image(&set->orm, size);

    int period = 8;
    float strength = 2.2f;
    switch (preset) {
        case HR_TEX_ASPHALT: period = 12; strength = 2.0f; break;
        case HR_TEX_GRASS: period = 10; strength = 2.6f; break;
        case HR_TEX_ROCK: period = 8; strength = 5.5f; break;
        case HR_TEX_CONCRETE: period = 6; strength = 1.5f; break;
        case HR_TEX_METAL: period = 10; strength = 0.8f; break;
        case HR_TEX_PAINT: period = 8; strength = 0.35f; break;
        case HR_TEX_FOLIAGE: period = 10; strength = 3.0f; break;
        case HR_TEX_TRUNK: period = 8; strength = 3.5f; break;
        case HR_TEX_GLASS: period = 4; strength = 0.2f; break;
        default: break;
    }

    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            float u = ((float)x + 0.5f) / (float)size;
            float v = ((float)y + 0.5f) / (float)size;
            float h = height_of(preset, u, v, period, seed);
            float r, g, b, rough, metal, ao;
            color_of(preset, u, v, h, period, seed, &r, &g, &b, &rough, &metal, &ao);

            usize i = ((usize)y * (usize)size + (usize)x) * 4u;
            set->albedo.rgba[i + 0] = lin_to_srgb8(r);
            set->albedo.rgba[i + 1] = lin_to_srgb8(g);
            set->albedo.rgba[i + 2] = lin_to_srgb8(b);
            set->albedo.rgba[i + 3] = 255;

            set->orm.rgba[i + 0] = (u8)(HR_CLAMP(ao, 0, 1) * 255.0f);
            set->orm.rgba[i + 1] = (u8)(HR_CLAMP(rough, 0, 1) * 255.0f);
            set->orm.rgba[i + 2] = (u8)(HR_CLAMP(metal, 0, 1) * 255.0f);
            set->orm.rgba[i + 3] = 255;

            /* central differences on the height field for the tangent normal */
            float du = 1.0f / (float)size;
            float hl = height_of(preset, u - du, v, period, seed);
            float hr2 = height_of(preset, u + du, v, period, seed);
            float hd = height_of(preset, u, v - du, period, seed);
            float hu = height_of(preset, u, v + du, period, seed);
            float dx = (hr2 - hl) * strength * 8.0f;
            float dy = (hu - hd) * strength * 8.0f;
            Vec3 n = v3_norm(v3(-dx, -dy, 1.0f));
            set->normal.rgba[i + 0] = (u8)((n.x * 0.5f + 0.5f) * 255.0f);
            set->normal.rgba[i + 1] = (u8)((n.y * 0.5f + 0.5f) * 255.0f);
            set->normal.rgba[i + 2] = (u8)((n.z * 0.5f + 0.5f) * 255.0f);
            set->normal.rgba[i + 3] = 255;
        }
    }
    HR_DEBUG("textures", "generated %s %dx%d (seed %u)", set->name, size, size, seed);
}

void hr_image_free(HrImage* img) {
    if (!img) return;
    hr_free(img->rgba);
    img->rgba = NULL;
    img->width = img->height = 0;
}
