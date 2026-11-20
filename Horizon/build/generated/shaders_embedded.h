#ifndef HR_SHADERS_EMBEDDED_H
#define HR_SHADERS_EMBEDDED_H
#include <stdint.h>
typedef struct HrShaderBlob {
    const char* name;
    const uint32_t* words;
    uint32_t word_count;
} HrShaderBlob;
extern const HrShaderBlob hr_shader_blobs[];
extern const int hr_shader_blob_count;
#endif
