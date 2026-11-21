#include "vk_internal.h"

static void upload_texture(HrRenderer* r, VkCommandBuffer cmd, HrGpuTexture* tex,
                           const HrImage* img, bool srgb, HrGpuBuffer* staging_out) {
    VkFormat format = srgb ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_R8G8B8A8_UNORM;
    int levels = 1;
    int m = img->width > img->height ? img->width : img->height;
    while ((1 << levels) < m) levels++;

    hr_vk_create_image(r, &tex->image, &tex->memory, (u32)img->width, (u32)img->height,
                       (u32)levels, 1, format,
                       VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
                       VK_IMAGE_USAGE_TRANSFER_DST_BIT);
    tex->format = format;
    tex->width = img->width;
    tex->height = img->height;
    tex->mips = levels;

    VkDeviceSize bytes = (VkDeviceSize)img->width * (VkDeviceSize)img->height * 4u;
    HrGpuBuffer staging = hr_vk_create_staging(r, img->rgba, bytes);
    *staging_out = staging;

    hr_vk_image_barrier(cmd, tex->image, VK_IMAGE_ASPECT_COLOR_BIT,
                        VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        0, VK_ACCESS_TRANSFER_WRITE_BIT,
                        VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 1);
    VkBufferImageCopy region;
    memset(&region, 0, sizeof(region));
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.layerCount = 1;
    region.imageExtent.width = (u32)img->width;
    region.imageExtent.height = (u32)img->height;
    region.imageExtent.depth = 1;
    vkCmdCopyBufferToImage(cmd, staging.buffer, tex->image,
                           VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
    hr_vk_image_barrier(cmd, tex->image, VK_IMAGE_ASPECT_COLOR_BIT,
                        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL,
                        VK_ACCESS_TRANSFER_WRITE_BIT,
                        VK_ACCESS_TRANSFER_READ_BIT | VK_ACCESS_TRANSFER_WRITE_BIT,
                        VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 1);

    for (int i = 1; i < levels; ++i) {
        VkImageBlit blit;
        memset(&blit, 0, sizeof(blit));
        blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        blit.srcSubresource.mipLevel = (u32)(i - 1);
        blit.srcSubresource.layerCount = 1;
        blit.srcOffsets[1].x = HR_MAX(1, img->width >> (i - 1));
        blit.srcOffsets[1].y = HR_MAX(1, img->height >> (i - 1));
        blit.srcOffsets[1].z = 1;
        blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        blit.dstSubresource.mipLevel = (u32)i;
        blit.dstSubresource.layerCount = 1;
        blit.dstOffsets[1].x = HR_MAX(1, img->width >> i);
        blit.dstOffsets[1].y = HR_MAX(1, img->height >> i);
        blit.dstOffsets[1].z = 1;
        vkCmdBlitImage(cmd, tex->image, VK_IMAGE_LAYOUT_GENERAL,
                       tex->image, VK_IMAGE_LAYOUT_GENERAL, 1, &blit, VK_FILTER_LINEAR);
        hr_vk_image_barrier(cmd, tex->image, VK_IMAGE_ASPECT_COLOR_BIT,
                            VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL,
                            VK_ACCESS_TRANSFER_WRITE_BIT,
                            VK_ACCESS_TRANSFER_READ_BIT | VK_ACCESS_TRANSFER_WRITE_BIT,
                            VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 1);
    }
    hr_vk_image_barrier(cmd, tex->image, VK_IMAGE_ASPECT_COLOR_BIT,
                        VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                        VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                        VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 1);

    tex->view = hr_vk_create_view(r, tex->image, format, VK_IMAGE_ASPECT_COLOR_BIT,
                                  (u32)levels, 1, VK_IMAGE_VIEW_TYPE_2D);
    VkSamplerCreateInfo si = { VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    si.magFilter = VK_FILTER_LINEAR;
    si.minFilter = VK_FILTER_LINEAR;
    si.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    si.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    si.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    si.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    si.anisotropyEnable = VK_TRUE;
    si.maxAnisotropy = r->props.limits.maxSamplerAnisotropy > 8.0f ? 8.0f : r->props.limits.maxSamplerAnisotropy;
    si.maxLod = (float)(levels - 1);
    si.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
    VK_CHECK(vkCreateSampler(r->device, &si, NULL, &tex->sampler));
}

void hr_vk_create_textures(HrRenderer* r, VkCommandBuffer cmd, HrGpuBuffer* staging, u32* count) {
    int sizes[HR_TEX_PRESET_COUNT];
    for (int i = 0; i < HR_TEX_PRESET_COUNT; ++i) sizes[i] = 256;
    sizes[HR_TEX_ASPHALT] = 512;
    sizes[HR_TEX_GRASS] = 512;
    sizes[HR_TEX_ROCK] = 384;
    for (int i = 0; i < HR_TEX_PRESET_COUNT; ++i) {
        HrTextureSet set;
        hr_texture_set_generate(&set, (HrTexturePreset)i, sizes[i], 0x51a7u + (u32)i * 97u);
        upload_texture(r, cmd, &r->tex_albedo[i], &set.albedo, true, &staging[(*count)++]);
        upload_texture(r, cmd, &r->tex_normal[i], &set.normal, false, &staging[(*count)++]);
        upload_texture(r, cmd, &r->tex_orm[i], &set.orm, false, &staging[(*count)++]);
        hr_image_free(&set.albedo);
        hr_image_free(&set.normal);
        hr_image_free(&set.orm);
    }
}

static void update_material_set(HrRenderer* r, HrGpuMaterial* m) {
    VkDescriptorBufferInfo ubo = { m->ubo, 0, sizeof(HrMaterialUbo) };
    VkDescriptorImageInfo imgs[4] = {
        { m->albedo->sampler, m->albedo->view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL },
        { m->normal->sampler, m->normal->view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL },
        { m->orm->sampler, m->orm->view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL },
        { m->detail->sampler, m->detail->view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL },
    };
    VkWriteDescriptorSet w[5];
    memset(w, 0, sizeof(w));
    for (int i = 0; i < 5; ++i) w[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    w[0].dstSet = m->set;
    w[0].dstBinding = 0;
    w[0].descriptorCount = 1;
    w[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    w[0].pBufferInfo = &ubo;
    for (int i = 0; i < 4; ++i) {
        w[i + 1].dstSet = m->set;
        w[i + 1].dstBinding = (u32)i + 1;
        w[i + 1].descriptorCount = 1;
        w[i + 1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w[i + 1].pImageInfo = &imgs[i];
    }
    vkUpdateDescriptorSets(r->device, 5, w, 0, NULL);
}

static int preset_texture(HrMaterialPreset mat) {
    switch (mat) {
        case HR_MAT_TERRAIN: return HR_TEX_GRASS;
        case HR_MAT_ROAD: return HR_TEX_ASPHALT;
        case HR_MAT_CAR_PAINT: return HR_TEX_PAINT;
        case HR_MAT_GLASS: return HR_TEX_GLASS;
        case HR_MAT_RUBBER: return HR_TEX_ASPHALT;
        case HR_MAT_METAL: return HR_TEX_METAL;
        case HR_MAT_FOLIAGE: return HR_TEX_FOLIAGE;
        case HR_MAT_TRUNK: return HR_TEX_TRUNK;
        case HR_MAT_ROCK: return HR_TEX_ROCK;
        case HR_MAT_CONCRETE: return HR_TEX_CONCRETE;
        case HR_MAT_LIGHT_WHITE: return HR_TEX_GLASS;
        case HR_MAT_LIGHT_RED: return HR_TEX_GLASS;
        case HR_MAT_SIGN: return HR_TEX_CONCRETE;
        case HR_MAT_WATER: return HR_TEX_GLASS;
        case HR_MAT_WINDOW: return HR_TEX_GLASS;
        default: return HR_TEX_CONCRETE;
    }
}

void hr_vk_create_materials(HrRenderer* r) {
    VkDescriptorSetLayout layouts[HR_MAT_COUNT];
    for (int i = 0; i < HR_MAT_COUNT; ++i) layouts[i] = r->material_dsl;
    VkDescriptorSet sets[HR_MAT_COUNT];
    VkDescriptorSetAllocateInfo ai = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
    ai.descriptorPool = r->pool_material;
    ai.descriptorSetCount = HR_MAT_COUNT;
    ai.pSetLayouts = layouts;
    VK_CHECK(vkAllocateDescriptorSets(r->device, &ai, sets));

    for (int i = 0; i < HR_MAT_COUNT; ++i) {
        HrGpuMaterial* m = &r->materials[i];
        memset(m, 0, sizeof(*m));
        m->set = sets[i];
        HrGpuBuffer ubo = hr_vk_create_buffer(r, sizeof(HrMaterialUbo),
                                              VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                              VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true);
        m->ubo = ubo.buffer;
        m->ubo_mem = ubo.memory;
        m->ubo_map = ubo.mapped;
        int t = preset_texture((HrMaterialPreset)i);
        m->albedo = &r->tex_albedo[t];
        m->normal = &r->tex_normal[t];
        m->orm = &r->tex_orm[t];
        m->detail = &r->tex_albedo[HR_TEX_ROCK];

        HrMaterialUbo mu;
        memset(&mu, 0, sizeof(mu));
        mu.base_color = v4(1, 1, 1, 1);
        mu.params = v4(1.0f, 1.0f, 0.0f, 0.0f);
        mu.uv_scale = v4(1, 1, 1, 1);
        switch ((HrMaterialPreset)i) {
            case HR_MAT_TERRAIN:
                mu.params = v4(1.0f, 1.0f, 0.0f, 1.0f);
                mu.uv_scale = v4(1, 1, 0.45f, 1.0f);
                break;
            case HR_MAT_ROAD:
                mu.params = v4(1.0f, 1.0f, 0.0f, 2.0f);
                mu.uv_scale = v4(1, 1, 0.32f, 1.0f);
                break;
            case HR_MAT_CAR_PAINT:
                mu.params = v4(1.0f, 1.0f, 0.0f, 4.0f);
                mu.uv_scale = v4(2.0f, 2.0f, 0.35f, 1.0f);
                break;
            case HR_MAT_GLASS:
                mu.params = v4(0.0f, 0.85f, 0.0f, 64.0f);
                mu.uv_scale = v4(1, 1, 0.05f, 1.0f);
                break;
            case HR_MAT_RUBBER:
                mu.base_color = v4(0.16f, 0.16f, 0.16f, 1);
                mu.params = v4(0.0f, 1.4f, 0.0f, 0.0f);
                mu.uv_scale = v4(2, 2, 0.6f, 1.0f);
                break;
            case HR_MAT_METAL:
                mu.params = v4(1.0f, 1.0f, 0.0f, 0.0f);
                mu.uv_scale = v4(2, 2, 0.5f, 1.0f);
                break;
            case HR_MAT_FOLIAGE:
                mu.params = v4(0.0f, 1.0f, 0.0f, 16.0f);
                mu.uv_scale = v4(1, 1, 0.55f, 1.0f);
                break;
            case HR_MAT_TRUNK:
                mu.params = v4(0.0f, 1.0f, 0.0f, 0.0f);
                mu.uv_scale = v4(1, 1, 0.8f, 1.0f);
                break;
            case HR_MAT_ROCK:
                mu.params = v4(0.0f, 1.0f, 0.0f, 0.0f);
                mu.uv_scale = v4(1, 1, 1.0f, 1.0f);
                break;
            case HR_MAT_CONCRETE:
                mu.uv_scale = v4(2.5f, 2.5f, 0.28f, 1.0f);
                break;
            case HR_MAT_LIGHT_WHITE:
                mu.base_color = v4(1.0f, 0.92f, 0.75f, 1);
                mu.params = v4(0.0f, 0.25f, 3.5f, 8.0f);
                mu.uv_scale = v4(1, 1, 0.05f, 1.0f);
                break;
            case HR_MAT_LIGHT_RED:
                mu.base_color = v4(1.0f, 0.02f, 0.01f, 1);
                mu.params = v4(0.0f, 0.3f, 2.0f, 8.0f);
                mu.uv_scale = v4(1, 1, 0.05f, 1.0f);
                break;
            case HR_MAT_SIGN:
                mu.base_color = v4(0.85f, 0.85f, 0.85f, 1);
                mu.uv_scale = v4(1, 1, 0.5f, 1.0f);
                break;
            case HR_MAT_WATER:
                mu.base_color = v4(0.02f, 0.06f, 0.08f, 1);
                mu.params = v4(0.0f, 0.08f, 0.0f, 32.0f);
                mu.uv_scale = v4(1, 1, 0.0f, 1.0f);
                break;
            case HR_MAT_WINDOW:
                mu.base_color = v4(1.0f, 0.68f, 0.34f, 1);
                mu.params = v4(0.0f, 0.4f, 1.8f, 8.0f);
                mu.uv_scale = v4(1, 1, 0.05f, 1.0f);
                break;
            default: break;
        }
        memcpy(m->ubo_map, &mu, sizeof(mu));
        update_material_set(r, m);
    }
}

void hr_vk_destroy_targets(HrRenderer* r, HrTargets* t) {
    if (t->hdr_fb) vkDestroyFramebuffer(r->device, t->hdr_fb, NULL);
    if (t->bloom_a_fb) vkDestroyFramebuffer(r->device, t->bloom_a_fb, NULL);
    if (t->bloom_b_fb) vkDestroyFramebuffer(r->device, t->bloom_b_fb, NULL);
    if (t->ldr_fb) vkDestroyFramebuffer(r->device, t->ldr_fb, NULL);
    for (u32 i = 0; i < HR_CASCADE_COUNT; ++i) {
        if (t->shadow_fbs[i]) vkDestroyFramebuffer(r->device, t->shadow_fbs[i], NULL);
        if (t->shadow_layer_views[i]) vkDestroyImageView(r->device, t->shadow_layer_views[i], NULL);
    }
    if (t->hdr_view) vkDestroyImageView(r->device, t->hdr_view, NULL);
    if (t->depth_view) vkDestroyImageView(r->device, t->depth_view, NULL);
    if (t->bloom_a_view) vkDestroyImageView(r->device, t->bloom_a_view, NULL);
    if (t->bloom_b_view) vkDestroyImageView(r->device, t->bloom_b_view, NULL);
    if (t->ldr_view) vkDestroyImageView(r->device, t->ldr_view, NULL);
    if (t->shadow_view) vkDestroyImageView(r->device, t->shadow_view, NULL);
    hr_vk_destroy_image(r, &t->hdr, &t->hdr_mem);
    hr_vk_destroy_image(r, &t->depth, &t->depth_mem);
    hr_vk_destroy_image(r, &t->bloom_a, &t->bloom_a_mem);
    hr_vk_destroy_image(r, &t->bloom_b, &t->bloom_b_mem);
    hr_vk_destroy_image(r, &t->ldr, &t->ldr_mem);
    hr_vk_destroy_image(r, &t->shadow, &t->shadow_mem);
    hr_vk_destroy_buffer(r, &t->capture);
    memset(t, 0, sizeof(*t));
}

void hr_vk_create_targets(HrRenderer* r, HrTargets* t, u32 hdr_w, u32 hdr_h, u32 shadow_size) {
    hr_vk_destroy_targets(r, t);
    t->hdr_width = hdr_w;
    t->hdr_height = hdr_h;
    t->shadow_size = shadow_size;
    t->ldr_width = r->swap_extent.width;
    t->ldr_height = r->swap_extent.height;

    hr_vk_create_image(r, &t->hdr, &t->hdr_mem, hdr_w, hdr_h, 1, 1,
                       VK_FORMAT_R16G16B16A16_SFLOAT,
                       VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
    hr_vk_create_image(r, &t->depth, &t->depth_mem, hdr_w, hdr_h, 1, 1,
                       VK_FORMAT_D32_SFLOAT,
                       VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
    u32 bw = HR_MAX(1u, hdr_w / 2), bh = HR_MAX(1u, hdr_h / 2);
    hr_vk_create_image(r, &t->bloom_a, &t->bloom_a_mem, bw, bh, 1, 1,
                       VK_FORMAT_R16G16B16A16_SFLOAT,
                       VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
    hr_vk_create_image(r, &t->bloom_b, &t->bloom_b_mem, bw, bh, 1, 1,
                       VK_FORMAT_R16G16B16A16_SFLOAT,
                       VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
    hr_vk_create_image(r, &t->ldr, &t->ldr_mem, t->ldr_width, t->ldr_height, 1, 1,
                       r->swap_format,
                       VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT);
    hr_vk_create_image(r, &t->shadow, &t->shadow_mem, shadow_size, shadow_size, 1, HR_CASCADE_COUNT,
                       VK_FORMAT_D32_SFLOAT,
                       VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);

    t->hdr_view = hr_vk_create_view(r, t->hdr, VK_FORMAT_R16G16B16A16_SFLOAT,
                                    VK_IMAGE_ASPECT_COLOR_BIT, 1, 1, VK_IMAGE_VIEW_TYPE_2D);
    t->depth_view = hr_vk_create_view(r, t->depth, VK_FORMAT_D32_SFLOAT,
                                      VK_IMAGE_ASPECT_DEPTH_BIT, 1, 1, VK_IMAGE_VIEW_TYPE_2D);
    t->bloom_a_view = hr_vk_create_view(r, t->bloom_a, VK_FORMAT_R16G16B16A16_SFLOAT,
                                        VK_IMAGE_ASPECT_COLOR_BIT, 1, 1, VK_IMAGE_VIEW_TYPE_2D);
    t->bloom_b_view = hr_vk_create_view(r, t->bloom_b, VK_FORMAT_R16G16B16A16_SFLOAT,
                                        VK_IMAGE_ASPECT_COLOR_BIT, 1, 1, VK_IMAGE_VIEW_TYPE_2D);
    t->ldr_view = hr_vk_create_view(r, t->ldr, r->swap_format,
                                    VK_IMAGE_ASPECT_COLOR_BIT, 1, 1, VK_IMAGE_VIEW_TYPE_2D);
    t->shadow_view = hr_vk_create_view(r, t->shadow, VK_FORMAT_D32_SFLOAT,
                                       VK_IMAGE_ASPECT_DEPTH_BIT, 1, HR_CASCADE_COUNT,
                                       VK_IMAGE_VIEW_TYPE_2D_ARRAY);
    for (u32 i = 0; i < HR_CASCADE_COUNT; ++i)
        t->shadow_layer_views[i] = hr_vk_create_view_layer(r, t->shadow, VK_FORMAT_D32_SFLOAT,
                                                           VK_IMAGE_ASPECT_DEPTH_BIT, i);

    VkImageView attachments[2] = { t->hdr_view, t->depth_view };
    VkFramebufferCreateInfo fi = { VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
    fi.renderPass = r->hdr_rp;
    fi.attachmentCount = 2;
    fi.pAttachments = attachments;
    fi.width = hdr_w;
    fi.height = hdr_h;
    fi.layers = 1;
    VK_CHECK(vkCreateFramebuffer(r->device, &fi, NULL, &t->hdr_fb));

    fi.renderPass = r->bloom_rp;
    fi.attachmentCount = 1;
    fi.width = bw;
    fi.height = bh;
    fi.pAttachments = &t->bloom_a_view;
    VK_CHECK(vkCreateFramebuffer(r->device, &fi, NULL, &t->bloom_a_fb));
    fi.pAttachments = &t->bloom_b_view;
    VK_CHECK(vkCreateFramebuffer(r->device, &fi, NULL, &t->bloom_b_fb));

    fi.renderPass = r->ldr_rp;
    fi.width = t->ldr_width;
    fi.height = t->ldr_height;
    fi.pAttachments = &t->ldr_view;
    VK_CHECK(vkCreateFramebuffer(r->device, &fi, NULL, &t->ldr_fb));

    fi.renderPass = r->shadow_rp;
    fi.width = shadow_size;
    fi.height = shadow_size;
    fi.layers = 1;
    for (u32 i = 0; i < HR_CASCADE_COUNT; ++i) {
        fi.pAttachments = &t->shadow_layer_views[i];
        VK_CHECK(vkCreateFramebuffer(r->device, &fi, NULL, &t->shadow_fbs[i]));
    }

    t->capture = hr_vk_create_buffer(r, (VkDeviceSize)t->ldr_width * t->ldr_height * 4u,
                                     VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                     VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true);
}

void hr_vk_update_frame_descriptors(HrRenderer* r, HrFrameRes* f, HrTargets* t) {
    VkDescriptorBufferInfo ubo = { f->ubo, 0, sizeof(HrGlobals) };
    VkDescriptorImageInfo shadow = { r->shadow_sampler, t->shadow_view,
                                     VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
    VkWriteDescriptorSet w[2];
    memset(w, 0, sizeof(w));
    for (int i = 0; i < 2; ++i) w[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    w[0].dstSet = f->global_set;
    w[0].dstBinding = 0;
    w[0].descriptorCount = 1;
    w[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    w[0].pBufferInfo = &ubo;
    w[1].dstSet = f->global_set;
    w[1].dstBinding = 1;
    w[1].descriptorCount = 1;
    w[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    w[1].pImageInfo = &shadow;
    vkUpdateDescriptorSets(r->device, 2, w, 0, NULL);

    VkDescriptorImageInfo hdr = { r->tex_albedo[HR_TEX_ROCK].sampler, t->hdr_view,
                                  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
    VkDescriptorImageInfo bloom_a = { r->tex_albedo[HR_TEX_ROCK].sampler, t->bloom_a_view,
                                      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
    VkDescriptorImageInfo bloom_b = { r->tex_albedo[HR_TEX_ROCK].sampler, t->bloom_b_view,
                                      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
    VkDescriptorImageInfo depth = { r->shadow_sampler, t->depth_view,
                                    VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL };
    VkWriteDescriptorSet post[6];
    memset(post, 0, sizeof(post));
    for (int i = 0; i < 6; ++i) post[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    post[0].dstSet = f->bright_set;
    post[0].dstBinding = 0;
    post[0].descriptorCount = 1;
    post[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    post[0].pImageInfo = &hdr;
    post[1].dstSet = f->blurh_set;
    post[1].dstBinding = 0;
    post[1].descriptorCount = 1;
    post[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    post[1].pImageInfo = &bloom_a;
    post[2].dstSet = f->blurv_set;
    post[2].dstBinding = 0;
    post[2].descriptorCount = 1;
    post[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    post[2].pImageInfo = &bloom_b;
    post[3].dstSet = f->post_set;
    post[3].dstBinding = 0;
    post[3].descriptorCount = 1;
    post[3].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    post[3].pImageInfo = &hdr;
    post[4].dstSet = f->post_set;
    post[4].dstBinding = 1;
    post[4].descriptorCount = 1;
    post[4].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    post[4].pImageInfo = &bloom_a;
    post[5].dstSet = f->post_set;
    post[5].dstBinding = 2;
    post[5].descriptorCount = 1;
    post[5].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    post[5].pImageInfo = &depth;
    vkUpdateDescriptorSets(r->device, 6, post, 0, NULL);
}
