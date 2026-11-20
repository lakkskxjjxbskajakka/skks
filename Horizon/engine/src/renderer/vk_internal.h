/* Horizon Engine - shared Vulkan renderer internals. */
#ifndef HR_VK_INTERNAL_H
#define HR_VK_INTERNAL_H

#if defined(_WIN32)
#  define VK_USE_PLATFORM_WIN32_KHR
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#else
#  define VK_USE_PLATFORM_XLIB_KHR
#endif
#include <vulkan/vulkan.h>

#include "horizon/renderer.h"
#include "horizon/memory.h"
#include "horizon/textures.h"
#include "horizon/containers.h"
#include "shaders_embedded.h"

#include <string.h>

#if defined(_WIN32)
#  include <windows.h>
#else
#  include <X11/Xlib.h>
#endif

#define HR_VK_FRAMES 2
#define HR_VK_MAX_DRAWS 16384
#define HR_VK_MAX_INSTANCES 12288
#define HR_VK_MAX_MESHES 96

#define VK_CHECK(x) do { VkResult _r = (x); if (_r != VK_SUCCESS) { \
    HR_FATAL("vulkan", "%s failed with VkResult %d (%s:%d)", #x, (int)_r, __FILE__, __LINE__); } } while (0)

typedef struct HrGlobals {
    Mat4 view_proj;
    Mat4 inv_view_proj;
    Mat4 light_view_proj;
    Vec4 cam_pos_time;
    Vec4 sun_dir_intensity;
    Vec4 sun_color_exposure;
    Vec4 ambient_sky;
    Vec4 ambient_ground;
    Vec4 fog_color_density;
    Vec4 screen;
    Vec4 shadow_params;
    Vec4 misc;
} HrGlobals;

typedef struct HrInstanceGpu {
    Mat4 model;
    Vec4 color;
    Vec4 params;
} HrInstanceGpu;

typedef struct HrGpuBuffer {
    VkBuffer buffer;
    VkDeviceMemory memory;
    VkDeviceSize size;
    void* mapped;
} HrGpuBuffer;

typedef struct HrGpuTexture {
    VkImage image;
    VkDeviceMemory memory;
    VkImageView view;
    VkSampler sampler;
    VkFormat format;
    int width, height, mips;
} HrGpuTexture;

typedef struct HrGpuMesh {
    HrGpuBuffer vbo;
    HrGpuBuffer ibo;
    u32 index_count;
    Vec3 center;
    float radius;
    char name[64];
} HrGpuMesh;

typedef struct HrGpuMaterial {
    VkDescriptorSet set;
    VkBuffer ubo;
    VkDeviceMemory ubo_mem;
    void* ubo_map;
    HrGpuTexture* albedo;
    HrGpuTexture* normal;
    HrGpuTexture* orm;
    HrGpuTexture* detail;
} HrGpuMaterial;

typedef struct HrTargets {
    u32 hdr_width, hdr_height;
    u32 shadow_size;
    u32 ldr_width, ldr_height;
    VkImage hdr, depth, bloom_a, bloom_b, ldr, shadow;
    VkDeviceMemory hdr_mem, depth_mem, bloom_a_mem, bloom_b_mem, ldr_mem, shadow_mem;
    VkImageView hdr_view, depth_view, bloom_a_view, bloom_b_view, ldr_view, shadow_view;
    VkFramebuffer hdr_fb, bloom_a_fb, bloom_b_fb, ldr_fb, shadow_fb;
    HrGpuBuffer capture;
} HrTargets;

typedef struct HrFrameRes {
    VkCommandBuffer cmd;
    VkSemaphore image_available;
    VkSemaphore render_finished;
    VkFence fence;
    VkBuffer ubo;
    VkDeviceMemory ubo_mem;
    void* ubo_map;
    HrGpuBuffer instances;
    VkDescriptorSet global_set;
    VkDescriptorSet bright_set;
    VkDescriptorSet blurh_set;
    VkDescriptorSet blurv_set;
    VkDescriptorSet post_set;
    VkQueryPool query_pool;
    bool submitted;
    bool queries_ready;
} HrFrameRes;

typedef struct HrDrawCmd {
    u32 mesh;
    u32 material;
    Mat4 model;
    Vec4 tint;
} HrDrawCmd;

typedef struct HrBatch {
    u32 mesh;
    u32 material;
    u32 first_instance;
    u32 instance_count;
} HrBatch;

typedef struct HrMaterialUbo {
    Vec4 base_color;
    Vec4 params;
    Vec4 uv_scale;
} HrMaterialUbo;

struct HrRenderer {
    HrWindow* window;
    bool validation;
    bool vsync;

    VkInstance instance;
    VkDebugUtilsMessengerEXT debug_messenger;
    VkSurfaceKHR surface;
    VkPhysicalDevice physical;
    VkPhysicalDeviceProperties props;
    VkPhysicalDeviceMemoryProperties mem_props;
    VkDevice device;
    u32 queue_family;
    VkQueue queue;

    VkSwapchainKHR swapchain;
    VkFormat swap_format;
    VkExtent2D swap_extent;
    u32 swap_image_count;
    VkImage swap_images[8];
    VkImageView swap_views[8];
    bool swapchain_srgb;

    VkRenderPass hdr_rp, shadow_rp, bloom_rp, ldr_rp;
    VkPipeline sky_pipeline, scene_pipeline, shadow_pipeline;
    VkPipeline bright_pipeline, blur_pipeline, post_pipeline;
    VkPipelineLayout scene_layout, sky_layout, shadow_layout, post_layout_a, post_layout_b;
    VkDescriptorSetLayout global_dsl, material_dsl, post_dsl_a, post_dsl_b;
    VkDescriptorPool pool_frame, pool_material;

    VkCommandPool cmd_pool;
    VkCommandBuffer upload_cmd;

    HrFrameRes frames[HR_VK_FRAMES];
    HrTargets targets[HR_VK_FRAMES];

    HrGpuMesh meshes[HR_VK_MAX_MESHES];
    int mesh_count;
    HrGpuMaterial materials[HR_MAT_COUNT];

    HrGpuTexture tex_albedo[HR_TEX_PRESET_COUNT];
    HrGpuTexture tex_normal[HR_TEX_PRESET_COUNT];
    HrGpuTexture tex_orm[HR_TEX_PRESET_COUNT];

    HR_DA(HrDrawCmd) draws;
    HR_DA(HrBatch) batches;

    u32 instance_count;
    u32 draw_calls;
    u32 triangle_count;

    HrFrameDesc frame;
    bool frame_active;
    u32 frame_index;
    u32 image_index;
    u64 frame_counter;

    double cpu_ms, gpu_ms;
    bool timestamp_valid;
    float timestamp_period;
    u64 cpu_start_ns;
    bool capture_pending;
    char capture_path[512];

    bool framebuffer_dirty;
    int window_w, window_h;
};

/* vk_common.c */
VkCommandBuffer hr_vk_begin_upload(HrRenderer* r);
void hr_vk_end_upload(HrRenderer* r, VkCommandBuffer cmd);
void hr_vk_image_barrier(VkCommandBuffer cmd, VkImage image, VkImageAspectFlags aspect,
                         VkImageLayout old_layout, VkImageLayout new_layout,
                         VkAccessFlags src_access, VkAccessFlags dst_access,
                         VkPipelineStageFlags src_stage, VkPipelineStageFlags dst_stage,
                         u32 layer_count);
u32 hr_vk_memory_type(HrRenderer* r, u32 type_bits, VkMemoryPropertyFlags props);
HrGpuBuffer hr_vk_create_buffer(HrRenderer* r, VkDeviceSize size, VkBufferUsageFlags usage,
                               VkMemoryPropertyFlags props, bool map);
void hr_vk_destroy_buffer(HrRenderer* r, HrGpuBuffer* b);
HrGpuBuffer hr_vk_create_staging(HrRenderer* r, const void* data, VkDeviceSize size);
void hr_vk_create_image(HrRenderer* r, VkImage* image, VkDeviceMemory* mem, u32 w, u32 h,
                        u32 mips, u32 layers, VkFormat format,
                        VkImageUsageFlags usage);
void hr_vk_destroy_image(HrRenderer* r, VkImage* image, VkDeviceMemory* mem);
VkImageView hr_vk_create_view(HrRenderer* r, VkImage image, VkFormat format,
                              VkImageAspectFlags aspect, u32 mips, u32 layers,
                              VkImageViewType type);
VkShaderModule hr_vk_load_shader(HrRenderer* r, const char* name);
VkPipeline hr_vk_create_pipeline(HrRenderer* r, VkPipelineLayout layout, VkRenderPass rp,
                                 const char* vs, const char* fs,
                                 const VkVertexInputBindingDescription* bindings, u32 binding_count,
                                 const VkVertexInputAttributeDescription* attrs, u32 attr_count,
                                 bool depth_test, bool depth_write, bool shadow_pass);
void hr_vk_write_bmp(const char* path, const void* pixels, u32 width, u32 height, bool bgra);

/* vk_targets.c */
void hr_vk_destroy_targets(HrRenderer* r, HrTargets* t);
void hr_vk_create_targets(HrRenderer* r, HrTargets* t, u32 hdr_w, u32 hdr_h, u32 shadow_size);
void hr_vk_update_frame_descriptors(HrRenderer* r, HrFrameRes* f, HrTargets* t);
void hr_vk_create_materials(HrRenderer* r);
void hr_vk_create_textures(HrRenderer* r, VkCommandBuffer cmd, HrGpuBuffer* staging, u32* count);

/* vk_frame.c */
bool hr_vk_init_frames(HrRenderer* r);
void hr_vk_shutdown_frames(HrRenderer* r);

/* vk_device.c */
void hr_vk_create_swapchain(HrRenderer* r);
void hr_vk_destroy_swapchain(HrRenderer* r);
bool hr_vk_recreate_swapchain(HrRenderer* r);

#endif
