#include "vk_internal.h"

#include <stdio.h>

VkCommandBuffer hr_vk_begin_upload(HrRenderer* r) {
    VkCommandBufferBeginInfo bi = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VK_CHECK(vkBeginCommandBuffer(r->upload_cmd, &bi));
    return r->upload_cmd;
}

void hr_vk_end_upload(HrRenderer* r, VkCommandBuffer cmd) {
    VK_CHECK(vkEndCommandBuffer(cmd));
    VkSubmitInfo si = { VK_STRUCTURE_TYPE_SUBMIT_INFO };
    si.commandBufferCount = 1;
    si.pCommandBuffers = &cmd;
    VK_CHECK(vkQueueSubmit(r->queue, 1, &si, VK_NULL_HANDLE));
    VK_CHECK(vkQueueWaitIdle(r->queue));
}

void hr_vk_image_barrier(VkCommandBuffer cmd, VkImage image, VkImageAspectFlags aspect,
                         VkImageLayout old_layout, VkImageLayout new_layout,
                         VkAccessFlags src_access, VkAccessFlags dst_access,
                         VkPipelineStageFlags src_stage, VkPipelineStageFlags dst_stage,
                         u32 layer_count) {
    VkImageMemoryBarrier b = { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
    b.oldLayout = old_layout;
    b.newLayout = new_layout;
    b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.image = image;
    b.subresourceRange.aspectMask = aspect;
    b.subresourceRange.baseMipLevel = 0;
    b.subresourceRange.levelCount = VK_REMAINING_MIP_LEVELS;
    b.subresourceRange.baseArrayLayer = 0;
    b.subresourceRange.layerCount = layer_count ? layer_count : VK_REMAINING_ARRAY_LAYERS;
    b.srcAccessMask = src_access;
    b.dstAccessMask = dst_access;
    vkCmdPipelineBarrier(cmd, src_stage, dst_stage, 0, 0, NULL, 0, NULL, 1, &b);
}

u32 hr_vk_memory_type(HrRenderer* r, u32 type_bits, VkMemoryPropertyFlags props) {
    for (u32 i = 0; i < r->mem_props.memoryTypeCount; ++i) {
        if ((type_bits & (1u << i)) &&
            (r->mem_props.memoryTypes[i].propertyFlags & props) == props) return i;
    }
    HR_FATAL("vulkan", "no suitable memory type (bits 0x%x flags 0x%x)", type_bits, props);
    return 0;
}

HrGpuBuffer hr_vk_create_buffer(HrRenderer* r, VkDeviceSize size, VkBufferUsageFlags usage,
                               VkMemoryPropertyFlags props, bool map) {
    HrGpuBuffer b;
    memset(&b, 0, sizeof(b));
    b.size = size;
    VkBufferCreateInfo bi = { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bi.size = size;
    bi.usage = usage;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VK_CHECK(vkCreateBuffer(r->device, &bi, NULL, &b.buffer));
    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(r->device, b.buffer, &req);
    VkMemoryAllocateInfo ai = { VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = hr_vk_memory_type(r, req.memoryTypeBits, props);
    VK_CHECK(vkAllocateMemory(r->device, &ai, NULL, &b.memory));
    VK_CHECK(vkBindBufferMemory(r->device, b.buffer, b.memory, 0));
    if (map) VK_CHECK(vkMapMemory(r->device, b.memory, 0, VK_WHOLE_SIZE, 0, &b.mapped));
    return b;
}

void hr_vk_destroy_buffer(HrRenderer* r, HrGpuBuffer* b) {
    if (b->mapped) { vkUnmapMemory(r->device, b->memory); b->mapped = NULL; }
    if (b->buffer) vkDestroyBuffer(r->device, b->buffer, NULL);
    if (b->memory) vkFreeMemory(r->device, b->memory, NULL);
    memset(b, 0, sizeof(*b));
}

HrGpuBuffer hr_vk_create_staging(HrRenderer* r, const void* data, VkDeviceSize size) {
    HrGpuBuffer b = hr_vk_create_buffer(r, size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                        VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true);
    memcpy(b.mapped, data, (size_t)size);
    return b;
}

void hr_vk_create_image(HrRenderer* r, VkImage* image, VkDeviceMemory* mem, u32 w, u32 h,
                        u32 mips, u32 layers, VkFormat format, VkImageUsageFlags usage) {
    VkImageCreateInfo ci = { VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
    ci.imageType = VK_IMAGE_TYPE_2D;
    ci.format = format;
    ci.extent.width = w;
    ci.extent.height = h;
    ci.extent.depth = 1;
    ci.mipLevels = mips;
    ci.arrayLayers = layers;
    ci.samples = VK_SAMPLE_COUNT_1_BIT;
    ci.tiling = VK_IMAGE_TILING_OPTIMAL;
    ci.usage = usage;
    ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    VK_CHECK(vkCreateImage(r->device, &ci, NULL, image));
    VkMemoryRequirements req;
    vkGetImageMemoryRequirements(r->device, *image, &req);
    VkMemoryAllocateInfo ai = { VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = hr_vk_memory_type(r, req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    VK_CHECK(vkAllocateMemory(r->device, &ai, NULL, mem));
    VK_CHECK(vkBindImageMemory(r->device, *image, *mem, 0));
}

void hr_vk_destroy_image(HrRenderer* r, VkImage* image, VkDeviceMemory* mem) {
    if (*image) vkDestroyImage(r->device, *image, NULL);
    if (*mem) vkFreeMemory(r->device, *mem, NULL);
    *image = VK_NULL_HANDLE;
    *mem = VK_NULL_HANDLE;
}

VkImageView hr_vk_create_view(HrRenderer* r, VkImage image, VkFormat format,
                              VkImageAspectFlags aspect, u32 mips, u32 layers,
                              VkImageViewType type) {
    VkImageViewCreateInfo ci = { VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
    ci.image = image;
    ci.viewType = type;
    ci.format = format;
    ci.subresourceRange.aspectMask = aspect;
    ci.subresourceRange.baseMipLevel = 0;
    ci.subresourceRange.levelCount = mips;
    ci.subresourceRange.baseArrayLayer = 0;
    ci.subresourceRange.layerCount = layers;
    VkImageView view;
    VK_CHECK(vkCreateImageView(r->device, &ci, NULL, &view));
    return view;
}

VkShaderModule hr_vk_load_shader(HrRenderer* r, const char* name) {
    const HrShaderBlob* blob = NULL;
    for (int i = 0; i < hr_shader_blob_count; ++i)
        if (strcmp(hr_shader_blobs[i].name, name) == 0) { blob = &hr_shader_blobs[i]; break; }
    if (!blob) HR_FATAL("vulkan", "shader '%s' not embedded", name);
    VkShaderModuleCreateInfo ci = { VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
    ci.codeSize = (size_t)blob->word_count * 4u;
    ci.pCode = blob->words;
    VkShaderModule m;
    VK_CHECK(vkCreateShaderModule(r->device, &ci, NULL, &m));
    return m;
}

VkPipeline hr_vk_create_pipeline(HrRenderer* r, VkPipelineLayout layout, VkRenderPass rp,
                                 const char* vs_name, const char* fs_name,
                                 const VkVertexInputBindingDescription* bindings, u32 binding_count,
                                 const VkVertexInputAttributeDescription* attrs, u32 attr_count,
                                 bool depth_test, bool depth_write, bool shadow_pass) {
    VkShaderModule vs = hr_vk_load_shader(r, vs_name);
    VkShaderModule fs = fs_name ? hr_vk_load_shader(r, fs_name) : VK_NULL_HANDLE;

    VkPipelineShaderStageCreateInfo stages[2];
    memset(stages, 0, sizeof(stages));
    stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vs;
    stages[0].pName = "main";
    u32 stage_count = 1;
    if (fs) {
        stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = fs;
        stages[1].pName = "main";
        stage_count = 2;
    }

    VkPipelineVertexInputStateCreateInfo vi = { VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
    vi.vertexBindingDescriptionCount = binding_count;
    vi.pVertexBindingDescriptions = bindings;
    vi.vertexAttributeDescriptionCount = attr_count;
    vi.pVertexAttributeDescriptions = attrs;

    VkPipelineInputAssemblyStateCreateInfo ia = { VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo vp = { VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
    vp.viewportCount = 1;
    vp.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rs = { VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
    rs.depthClampEnable = VK_FALSE;
    rs.rasterizerDiscardEnable = VK_FALSE;
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode = VK_CULL_MODE_NONE;
    rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rs.depthBiasEnable = shadow_pass ? VK_TRUE : VK_FALSE;
    rs.depthBiasConstantFactor = 1.25f;
    rs.depthBiasSlopeFactor = 2.0f;
    rs.lineWidth = 1.0f;

    VkPipelineMultisampleStateCreateInfo ms = { VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo ds = { VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO };
    ds.depthTestEnable = depth_test ? VK_TRUE : VK_FALSE;
    ds.depthWriteEnable = depth_write ? VK_TRUE : VK_FALSE;
    ds.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
    ds.depthBoundsTestEnable = VK_FALSE;
    ds.stencilTestEnable = VK_FALSE;

    VkPipelineColorBlendAttachmentState blend = {0};
    blend.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                           VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    VkPipelineColorBlendStateCreateInfo cb = { VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
    cb.attachmentCount = shadow_pass ? 0u : 1u;
    cb.pAttachments = shadow_pass ? NULL : &blend;

    VkDynamicState dyn[2] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    VkPipelineDynamicStateCreateInfo dy = { VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
    dy.dynamicStateCount = 2;
    dy.pDynamicStates = dyn;

    VkGraphicsPipelineCreateInfo ci = { VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
    ci.stageCount = stage_count;
    ci.pStages = stages;
    ci.pVertexInputState = &vi;
    ci.pInputAssemblyState = &ia;
    ci.pViewportState = &vp;
    ci.pRasterizationState = &rs;
    ci.pMultisampleState = &ms;
    ci.pDepthStencilState = &ds;
    ci.pColorBlendState = shadow_pass ? NULL : &cb;
    ci.pDynamicState = &dy;
    ci.layout = layout;
    ci.renderPass = rp;
    ci.subpass = 0;

    VkPipeline p;
    VK_CHECK(vkCreateGraphicsPipelines(r->device, VK_NULL_HANDLE, 1, &ci, NULL, &p));
    vkDestroyShaderModule(r->device, vs, NULL);
    if (fs) vkDestroyShaderModule(r->device, fs, NULL);
    return p;
}

void hr_vk_write_bmp(const char* path, const void* pixels, u32 width, u32 height, bool bgra) {
    FILE* f = fopen(path, "wb");
    if (!f) { HR_ERROR("renderer", "cannot open screenshot '%s'", path); return; }
    u32 row = width * 3u;
    u32 pad = (4u - (row % 4u)) % 4u;
    u32 data_size = (row + pad) * height;
    u8 header[54] = {0};
    header[0] = 'B'; header[1] = 'M';
    u32 total = 54u + data_size;
    memcpy(header + 2, &total, 4);
    u32 offset = 54;
    memcpy(header + 10, &offset, 4);
    u32 dib = 40;
    memcpy(header + 14, &dib, 4);
    memcpy(header + 18, &width, 4);
    memcpy(header + 22, &height, 4);
    u16 planes = 1, bpp = 24;
    memcpy(header + 26, &planes, 2);
    memcpy(header + 28, &bpp, 2);
    memcpy(header + 34, &data_size, 4);
    fwrite(header, 1, 54, f);

    const u8* src = (const u8*)pixels;
    u8* line = (u8*)hr_alloc(row + pad);
    for (u32 y = 0; y < height; ++y) {
        const u8* s = src + (usize)(height - 1u - y) * width * 4u;
        for (u32 x = 0; x < width; ++x) {
            if (bgra) {
                line[x * 3 + 0] = s[x * 4 + 0];
                line[x * 3 + 1] = s[x * 4 + 1];
                line[x * 3 + 2] = s[x * 4 + 2];
            } else {
                line[x * 3 + 0] = s[x * 4 + 2];
                line[x * 3 + 1] = s[x * 4 + 1];
                line[x * 3 + 2] = s[x * 4 + 0];
            }
        }
        memset(line + row, 0, pad);
        fwrite(line, 1, row + pad, f);
    }
    hr_free(line);
    fclose(f);
    HR_INFO("renderer", "screenshot written: %s (%ux%u)", path, width, height);
}
