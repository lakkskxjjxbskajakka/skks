#include "vk_internal.h"

#include <stdlib.h>

static u32 render_scale_extent(HrRenderer* r, u32* out_w, u32* out_h) {
    float s = r->frame.render_scale;
    if (!(s > 0.25f)) s = 1.0f;
    if (s > 1.0f) s = 1.0f;
    u32 w = (u32)((float)r->swap_extent.width * s);
    u32 h = (u32)((float)r->swap_extent.height * s);
    if (w < 64) w = 64;
    if (h < 64) h = 64;
    *out_w = w;
    *out_h = h;
    return 0;
}

static u32 target_shadow_size(HrRenderer* r) {
    int s = r->frame.shadow_resolution;
    if (s < 512) s = 512;
    if (s > 4096) s = 4096;
    return (u32)s;
}

static int cmp_draw(const void* a, const void* b) {
    const HrDrawCmd* x = (const HrDrawCmd*)a;
    const HrDrawCmd* y = (const HrDrawCmd*)b;
    if (x->material != y->material) return x->material < y->material ? -1 : 1;
    if (x->mesh != y->mesh) return x->mesh < y->mesh ? -1 : 1;
    return 0;
}

bool hr_vk_init_frames(HrRenderer* r) {
    VkCommandBufferAllocateInfo cai = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    cai.commandPool = r->cmd_pool;
    cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;

    VkSemaphoreCreateInfo si = { VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
    VkFenceCreateInfo fi = { VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };

    u32 hdr_w, hdr_h;
    render_scale_extent(r, &hdr_w, &hdr_h);
    u32 shadow = target_shadow_size(r);

    for (u32 i = 0; i < HR_VK_FRAMES; ++i) {
        HrFrameRes* f = &r->frames[i];
        memset(f, 0, sizeof(*f));
        cai.commandBufferCount = 1;
        VK_CHECK(vkAllocateCommandBuffers(r->device, &cai, &f->cmd));
        VK_CHECK(vkCreateSemaphore(r->device, &si, NULL, &f->image_available));
        VK_CHECK(vkCreateSemaphore(r->device, &si, NULL, &f->render_finished));
        VK_CHECK(vkCreateFence(r->device, &fi, NULL, &f->fence));

        HrGpuBuffer ubo = hr_vk_create_buffer(r, sizeof(HrGlobals), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                              VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true);
        f->ubo = ubo.buffer;
        f->ubo_mem = ubo.memory;
        f->ubo_map = ubo.mapped;

        f->instances = hr_vk_create_buffer(r, (VkDeviceSize)HR_VK_MAX_INSTANCES * sizeof(HrInstanceGpu),
                                           VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                                           VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                           VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, true);

        if (r->props.limits.timestampComputeAndGraphics) {
            VkQueryPoolCreateInfo qi = { VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO };
            qi.queryType = VK_QUERY_TYPE_TIMESTAMP;
            qi.queryCount = 2;
            VK_CHECK(vkCreateQueryPool(r->device, &qi, NULL, &f->query_pool));
        }

        VkDescriptorSetLayout layouts[5] = {
            r->global_dsl, r->post_dsl_a, r->post_dsl_a, r->post_dsl_a, r->post_dsl_b
        };
        VkDescriptorSet sets[5];
        VkDescriptorSetAllocateInfo ai = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        ai.descriptorPool = r->pool_frame;
        ai.descriptorSetCount = 5;
        ai.pSetLayouts = layouts;
        VK_CHECK(vkAllocateDescriptorSets(r->device, &ai, sets));
        f->global_set = sets[0];
        f->bright_set = sets[1];
        f->blurh_set = sets[2];
        f->blurv_set = sets[3];
        f->post_set = sets[4];

        hr_vk_create_targets(r, &r->targets[i], hdr_w, hdr_h, shadow);
    }

    r->timestamp_valid = r->props.limits.timestampComputeAndGraphics != 0;
    r->timestamp_period = r->props.limits.timestampPeriod;

    for (u32 i = 0; i < HR_VK_FRAMES; ++i)
        hr_vk_update_frame_descriptors(r, &r->frames[i], &r->targets[i]);
    r->framebuffer_dirty = false;
    return true;
}

void hr_vk_shutdown_frames(HrRenderer* r) {
    if (!r->device) return;
    for (u32 i = 0; i < HR_VK_FRAMES; ++i) {
        HrFrameRes* f = &r->frames[i];
        hr_vk_destroy_targets(r, &r->targets[i]);
        hr_vk_destroy_buffer(r, &f->instances);
        if (f->ubo_mem) {
            if (f->ubo_map) vkUnmapMemory(r->device, f->ubo_mem);
            vkFreeMemory(r->device, f->ubo_mem, NULL);
        }
        if (f->ubo) vkDestroyBuffer(r->device, f->ubo, NULL);
        if (f->query_pool) vkDestroyQueryPool(r->device, f->query_pool, NULL);
        if (f->fence) vkDestroyFence(r->device, f->fence, NULL);
        if (f->image_available) vkDestroySemaphore(r->device, f->image_available, NULL);
        if (f->render_finished) vkDestroySemaphore(r->device, f->render_finished, NULL);
    }
}

int hr_renderer_upload_mesh(HrRenderer* r, const HrMeshData* mesh, const char* name) {
    if (!r || !mesh || mesh->vertex_count == 0 || mesh->index_count == 0) return -1;
    if (r->mesh_count >= HR_VK_MAX_MESHES) {
        HR_WARN("renderer", "mesh limit reached (%d)", HR_VK_MAX_MESHES);
        return -1;
    }
    HrGpuMesh* m = &r->meshes[r->mesh_count];
    memset(m, 0, sizeof(*m));
    m->index_count = mesh->index_count;
    m->center = mesh->bounds_center;
    m->radius = mesh->bounds_radius;
    snprintf(m->name, sizeof(m->name), "%s", name ? name : mesh->name ? mesh->name : "mesh");

    VkDeviceSize vsize = (VkDeviceSize)mesh->vertex_count * sizeof(HrVertex);
    VkDeviceSize isz = (VkDeviceSize)mesh->index_count * sizeof(u32);
    HrGpuBuffer vs = hr_vk_create_staging(r, mesh->vertices, vsize);
    HrGpuBuffer is = hr_vk_create_staging(r, mesh->indices, isz);
    m->vbo = hr_vk_create_buffer(r, vsize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, false);
    m->ibo = hr_vk_create_buffer(r, isz, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, false);

    VkCommandBuffer cmd = hr_vk_begin_upload(r);
    VkBufferCopy vc = { 0, 0, vsize };
    vkCmdCopyBuffer(cmd, vs.buffer, m->vbo.buffer, 1, &vc);
    VkBufferCopy ic = { 0, 0, isz };
    vkCmdCopyBuffer(cmd, is.buffer, m->ibo.buffer, 1, &ic);
    VkMemoryBarrier mb = { VK_STRUCTURE_TYPE_MEMORY_BARRIER };
    mb.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    mb.dstAccessMask = VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT | VK_ACCESS_INDEX_READ_BIT;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_VERTEX_INPUT_BIT, 0, 1, &mb, 0, NULL, 0, NULL);
    hr_vk_end_upload(r, cmd);
    hr_vk_destroy_buffer(r, &vs);
    hr_vk_destroy_buffer(r, &is);

    int idx = r->mesh_count++;
    HR_DEBUG("renderer", "mesh '%s': %u verts, %u tris", m->name, mesh->vertex_count, mesh->index_count / 3);
    return idx;
}

static void ensure_targets(HrRenderer* r) {
    u32 hdr_w, hdr_h;
    render_scale_extent(r, &hdr_w, &hdr_h);
    u32 shadow = target_shadow_size(r);
    bool need = r->framebuffer_dirty;
    for (u32 i = 0; i < HR_VK_FRAMES; ++i) {
        HrTargets* t = &r->targets[i];
        if (!t->hdr || t->hdr_width != hdr_w || t->hdr_height != hdr_h ||
            t->shadow_size != shadow || t->ldr_width != r->swap_extent.width ||
            t->ldr_height != r->swap_extent.height) need = true;
    }
    if (!need) return;
    vkDeviceWaitIdle(r->device);
    for (u32 i = 0; i < HR_VK_FRAMES; ++i) {
        hr_vk_create_targets(r, &r->targets[i], hdr_w, hdr_h, shadow);
        hr_vk_update_frame_descriptors(r, &r->frames[i], &r->targets[i]);
    }
    r->framebuffer_dirty = false;
}

static void compute_csm(const HrFrameDesc* frame, float aspect, u32 shadow_size,
                        Mat4 out_vp[HR_CASCADE_COUNT], float out_splits[HR_CASCADE_COUNT],
                        Vec3* out_forward, Vec3* out_light_dir) {
    Vec3 cam = frame->camera_position;
    Vec3 fwd = v3_norm(v3_sub(frame->camera_target, cam));
    if (v3_len_sq(fwd) < 0.5f) fwd = v3(0, 0, -1);
    Vec3 up_ref = v3_norm(frame->camera_up);
    if (v3_len_sq(up_ref) < 0.5f) up_ref = v3(0, 1, 0);
    Vec3 right = v3_norm(v3_cross(fwd, up_ref));
    if (v3_len_sq(right) < 0.5f) right = v3(1, 0, 0);
    Vec3 up = v3_cross(right, fwd);

    Vec3 light = v3_norm(frame->sun_direction);
    if (v3_len_sq(light) < 0.5f) light = v3_norm(v3(-0.4f, -0.75f, -0.5f));
    Vec3 lup = fabsf(light.y) > 0.98f ? v3(0, 0, 1) : v3(0, 1, 0);
    Mat4 lrot = mat4_look_at(v3(0, 0, 0), light, lup);

    float near_plane = frame->near_plane > 0.0f ? frame->near_plane : 0.08f;
    float far_plane = frame->far_plane > near_plane ? frame->far_plane : 1500.0f;
    float shadow_dist = frame->shadow_distance > 5.0f ? frame->shadow_distance : 90.0f;
    if (far_plane > shadow_dist) far_plane = shadow_dist;
    if (far_plane <= near_plane) far_plane = near_plane + 1.0f;

    float fovy = frame->fov_deg * HR_DEG2RAD;
    float tan_y = tanf(HR_CLAMP(fovy, 0.1f, 2.8f) * 0.5f);
    float tan_x = tan_y * aspect;
    const float lambda = 0.82f;
    float prev = near_plane;

    for (u32 c = 0; c < HR_CASCADE_COUNT; ++c) {
        float p = (float)(c + 1) / (float)HR_CASCADE_COUNT;
        float log_split = near_plane * powf(far_plane / near_plane, p);
        float uni_split = near_plane + (far_plane - near_plane) * p;
        float split = lambda * log_split + (1.0f - lambda) * uni_split;
        if (split <= prev) split = prev + 0.05f;
        if (split > far_plane) split = far_plane;
        out_splits[c] = split;

        Vec3 lo = v3(1e30f, 1e30f, 1e30f), hi = v3(-1e30f, -1e30f, -1e30f);
        for (u32 di = 0; di < 2; ++di) {
            float d = di == 0 ? prev : split;
            for (int sy = -1; sy <= 1; sy += 2) {
                for (int sx = -1; sx <= 1; sx += 2) {
                    Vec3 pnt = v3_add(cam, v3_scale(fwd, d));
                    pnt = v3_add(pnt, v3_scale(right, (float)sx * tan_x * d));
                    pnt = v3_add(pnt, v3_scale(up, (float)sy * tan_y * d));
                    Vec3 lp = mat4_mul_point(lrot, pnt);
                    lo = v3_min(lo, lp);
                    hi = v3_max(hi, lp);
                }
            }
        }
        Vec3 center = v3_scale(v3_add(lo, hi), 0.5f);
        float radius = HR_MAX(hi.x - lo.x, hi.y - lo.y) * 0.5f + 0.01f;
        if (radius < 1.0f) radius = 1.0f;
        float texel = 2.0f * radius / (float)shadow_size;
        center.x = floorf(center.x / texel + 0.5f) * texel;
        center.y = floorf(center.y / texel + 0.5f) * texel;

        float near_d = HR_MAX(0.01f, -hi.z - 0.5f);
        float far_d = HR_MAX(near_d + 1.0f, -lo.z + 0.5f);
        Mat4 view = mat4_mul(mat4_translate(v3(-center.x, -center.y, 0.0f)), lrot);
        Mat4 proj = mat4_ortho_vk(-radius, radius, -radius, radius, near_d, far_d);
        out_vp[c] = mat4_mul(proj, view);
        prev = split;
    }
    *out_forward = fwd;
    *out_light_dir = light;
}

bool hr_renderer_begin_frame(HrRenderer* r, const HrFrameDesc* frame) {
    if (!r || !frame) return false;
    r->frame = *frame;
    r->draws.len = 0;
    r->frame_active = false;
    r->cpu_start_ns = hr_time_ns();

    int w, h;
    hr_window_get_framebuffer_size(r->window, &w, &h);
    if (w <= 0 || h <= 0) return false;
    if (w != r->window_w || h != r->window_h || r->framebuffer_dirty) {
        r->window_w = w;
        r->window_h = h;
        if (!hr_vk_recreate_swapchain(r)) return false;
    }
    r->frame_index = (u32)(r->frame_counter % HR_VK_FRAMES);
    HrFrameRes* f = &r->frames[r->frame_index];

    if (f->submitted) {
        VK_CHECK(vkWaitForFences(r->device, 1, &f->fence, VK_TRUE, UINT64_MAX));
        if (r->timestamp_valid && f->queries_ready) {
            u64 ts[2] = {0, 0};
            VkResult qr = vkGetQueryPoolResults(r->device, f->query_pool, 0, 2,
                                                sizeof(ts), ts, sizeof(u64),
                                                VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT);
            if (qr == VK_SUCCESS && ts[1] > ts[0]) {
                double ms = (double)(ts[1] - ts[0]) * (double)r->timestamp_period * 1e-6;
                r->gpu_ms = r->gpu_ms > 0.0 ? r->gpu_ms * 0.85 + ms * 0.15 : ms;
            }
            f->queries_ready = false;
        }
    }
    ensure_targets(r);

    VkResult ar = vkAcquireNextImageKHR(r->device, r->swapchain, UINT64_MAX,
                                        f->image_available, VK_NULL_HANDLE, &r->image_index);
    if (ar == VK_ERROR_OUT_OF_DATE_KHR) {
        hr_vk_recreate_swapchain(r);
        return false;
    } else if (ar != VK_SUCCESS && ar != VK_SUBOPTIMAL_KHR) {
        HR_ERROR("renderer", "vkAcquireNextImageKHR: %d", (int)ar);
        return false;
    }
    r->frame_active = true;
    return true;
}

void hr_renderer_draw(HrRenderer* r, int mesh, HrMaterialPreset material, Mat4 model, Vec4 tint) {
    if (!r || !r->frame_active) return;
    if (mesh < 0 || mesh >= r->mesh_count) return;
    if (material < 0 || material >= HR_MAT_COUNT) return;
    if (r->draws.len >= HR_VK_MAX_DRAWS) return;
    HrDrawCmd d;
    d.mesh = (u32)mesh;
    d.material = (u32)material;
    d.model = model;
    d.tint = tint;
    hr_da_push(&r->draws, d);
}

void hr_renderer_end_frame(HrRenderer* r) {
    if (!r || !r->frame_active) return;
    HrFrameRes* f = &r->frames[r->frame_index];
    HrTargets* t = &r->targets[r->frame_index];

    /* ---- sort and build instanced batches ---- */
    qsort(r->draws.items, r->draws.len, sizeof(HrDrawCmd), cmp_draw);
    r->batches.len = 0;
    r->instance_count = 0;
    r->triangle_count = 0;

    Mat4 view = mat4_look_at(r->frame.camera_position, r->frame.camera_target, r->frame.camera_up);
    float aspect = (float)t->hdr_width / (float)t->hdr_height;
    float fov = r->frame.fov_deg > 1.0f ? r->frame.fov_deg : 60.0f;
    float nearp = r->frame.near_plane > 0.001f ? r->frame.near_plane : 0.1f;
    float farp = r->frame.far_plane > nearp ? r->frame.far_plane : 1200.0f;
    Mat4 proj = mat4_perspective_vk(fov * HR_DEG2RAD, aspect, nearp, farp);
    Mat4 vp = mat4_mul(proj, view);
    Vec4 planes[6];
    hr_frustum_planes(vp, planes);

    HrInstanceGpu* inst = (HrInstanceGpu*)f->instances.mapped;
    HrBatch cur;
    memset(&cur, 0, sizeof(cur));
    cur.mesh = UINT32_MAX;
    cur.material = UINT32_MAX;
    bool have_cur = false;

    for (usize di = 0; di < r->draws.len; ++di) {
        HrDrawCmd* d = &r->draws.items[di];
        HrGpuMesh* mesh = &r->meshes[d->mesh];
        Vec3 center = mat4_mul_point(d->model, mesh->center);
        float sx = v3_len(v3(d->model.c[0][0], d->model.c[0][1], d->model.c[0][2]));
        float sy = v3_len(v3(d->model.c[1][0], d->model.c[1][1], d->model.c[1][2]));
        float sz = v3_len(v3(d->model.c[2][0], d->model.c[2][1], d->model.c[2][2]));
        float scale = HR_MAX(sx, HR_MAX(sy, sz));
        float radius = mesh->radius * scale;
        Vec3 rel = v3_sub(center, r->frame.camera_position);
        if (v3_len_sq(rel) > (farp + radius) * (farp + radius)) continue;
        if (!hr_sphere_in_frustum(planes, center, radius)) continue;

        if (d->mesh != cur.mesh || d->material != cur.material) {
            if (have_cur && cur.instance_count > 0) hr_da_push(&r->batches, cur);
            cur.mesh = d->mesh;
            cur.material = d->material;
            cur.first_instance = r->instance_count;
            cur.instance_count = 0;
            have_cur = true;
        }
        if (r->instance_count >= HR_VK_MAX_INSTANCES) continue;
        HrInstanceGpu* gi = &inst[r->instance_count++];
        gi->model = d->model;
        gi->color = d->tint;
        gi->params = v4(0, 0, 0, 0);
        cur.instance_count++;
    }
    if (have_cur && cur.instance_count > 0) hr_da_push(&r->batches, cur);

    r->draw_calls = (u32)r->batches.len;
    for (usize batch_i = 0; batch_i < r->batches.len; ++batch_i) {
        HrBatch* b = &r->batches.items[batch_i];
        r->triangle_count += (r->meshes[b->mesh].index_count / 3u) * b->instance_count;
    }

    /* ---- globals / cascaded shadow matrices ---- */
    Mat4 light_vps[HR_CASCADE_COUNT];
    float splits[HR_CASCADE_COUNT];
    Vec3 cam_fwd, sun;
    compute_csm(&r->frame, aspect, t->shadow_size, light_vps, splits, &cam_fwd, &sun);
    float shadow_dist = r->frame.shadow_distance > 5.0f ? r->frame.shadow_distance : 90.0f;

    HrGlobals* g = (HrGlobals*)f->ubo_map;
    memset(g, 0, sizeof(*g));
    g->view_proj = vp;
    g->inv_view_proj = mat4_invert(vp);
    for (u32 c = 0; c < HR_CASCADE_COUNT; ++c) g->light_view_proj[c] = light_vps[c];
    g->cam_pos_time = v4_from3(r->frame.camera_position, r->frame.time);
    g->cam_forward = v4_from3(cam_fwd, 0.0f);
    g->sun_dir_intensity = v4_from3(sun, r->frame.sun_intensity);
    g->sun_color_exposure = v4_from3(r->frame.sun_color, r->frame.exposure);
    g->ambient_sky = v4_from3(r->frame.ambient_sky, 0);
    g->ambient_ground = v4_from3(r->frame.ambient_ground, 0);
    g->fog_color_density = v4_from3(r->frame.fog_color, r->frame.fog_density);
    g->screen = v4((float)t->hdr_width, (float)t->hdr_height,
                   1.0f / (float)t->hdr_width, 1.0f / (float)t->hdr_height);
    g->shadow_params = v4(1.0f / (float)t->shadow_size, shadow_dist,
                          r->frame.shadows_enabled ? 1.0f : 0.0f, (float)HR_CASCADE_COUNT);
    g->cascade_splits = v4(splits[0], splits[1], splits[2], splits[3]);
    g->misc = v4(r->frame.time, 0, 0, 0);

    /* ---- record ---- */
    VK_CHECK(vkResetCommandBuffer(f->cmd, 0));
    VkCommandBufferBeginInfo bi = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VK_CHECK(vkBeginCommandBuffer(f->cmd, &bi));
    VkCommandBuffer cmd = f->cmd;

    if (f->query_pool) {
        vkCmdResetQueryPool(cmd, f->query_pool, 0, 2);
        vkCmdWriteTimestamp(cmd, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, f->query_pool, 0);
    }

    VkViewport vp_view = { 0, 0, 0, 0, 0, 1 };
    VkRect2D scissor;

    /* ---- cascaded shadow passes ---- */
    if (r->frame.shadows_enabled) {
        vp_view.width = (float)t->shadow_size;
        vp_view.height = (float)t->shadow_size;
        scissor.offset.x = 0; scissor.offset.y = 0;
        scissor.extent.width = t->shadow_size;
        scissor.extent.height = t->shadow_size;
        vkCmdSetViewport(cmd, 0, 1, &vp_view);
        vkCmdSetScissor(cmd, 0, 1, &scissor);

        VkClearValue clears[1];
        clears[0].depthStencil.depth = 1.0f;
        clears[0].depthStencil.stencil = 0;
        for (u32 cascade = 0; cascade < HR_CASCADE_COUNT; ++cascade) {
            VkRenderPassBeginInfo rp = { VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
            rp.renderPass = r->shadow_rp;
            rp.framebuffer = t->shadow_fbs[cascade];
            rp.renderArea = scissor;
            rp.clearValueCount = 1;
            rp.pClearValues = clears;
            vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r->shadow_pipeline);
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r->shadow_layout,
                                    0, 1, &f->global_set, 0, NULL);
            vkCmdPushConstants(cmd, r->shadow_layout, VK_SHADER_STAGE_VERTEX_BIT,
                               0, sizeof(cascade), &cascade);
            for (usize batch_i = 0; batch_i < r->batches.len; ++batch_i) {
                HrBatch* b = &r->batches.items[batch_i];
                HrGpuMesh* mesh = &r->meshes[b->mesh];
                VkBuffer vb[2] = { mesh->vbo.buffer, f->instances.buffer };
                VkDeviceSize off[2] = { 0, 0 };
                vkCmdBindVertexBuffers(cmd, 0, 2, vb, off);
                vkCmdBindIndexBuffer(cmd, mesh->ibo.buffer, 0, VK_INDEX_TYPE_UINT32);
                vkCmdDrawIndexed(cmd, mesh->index_count, b->instance_count, 0, 0, b->first_instance);
            }
            vkCmdEndRenderPass(cmd);
        }
        hr_vk_image_barrier(cmd, t->shadow, VK_IMAGE_ASPECT_DEPTH_BIT,
                            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                            VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                            VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
                            VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, HR_CASCADE_COUNT);
    }

    /* ---- HDR pass: sky + scene ---- */
    vp_view.width = (float)t->hdr_width;
    vp_view.height = (float)t->hdr_height;
    scissor.extent.width = t->hdr_width;
    scissor.extent.height = t->hdr_height;
    vkCmdSetViewport(cmd, 0, 1, &vp_view);
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    VkClearValue hdr_clears[2];
    hdr_clears[0].color.float32[0] = 0.0f;
    hdr_clears[0].color.float32[1] = 0.0f;
    hdr_clears[0].color.float32[2] = 0.0f;
    hdr_clears[0].color.float32[3] = 1.0f;
    hdr_clears[1].depthStencil.depth = 1.0f;
    hdr_clears[1].depthStencil.stencil = 0;
    VkRenderPassBeginInfo rp = { VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
    rp.renderPass = r->hdr_rp;
    rp.framebuffer = t->hdr_fb;
    rp.renderArea = scissor;
    rp.clearValueCount = 2;
    rp.pClearValues = hdr_clears;
    vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r->sky_pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r->sky_layout,
                            0, 1, &f->global_set, 0, NULL);
    vkCmdDraw(cmd, 3, 1, 0, 0);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r->scene_pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r->scene_layout,
                            0, 1, &f->global_set, 0, NULL);
    for (usize batch_i = 0; batch_i < r->batches.len; ++batch_i) {
        HrBatch* b = &r->batches.items[batch_i];
        HrGpuMesh* mesh = &r->meshes[b->mesh];
        VkBuffer vb[2] = { mesh->vbo.buffer, f->instances.buffer };
        VkDeviceSize off[2] = { 0, 0 };
        vkCmdBindVertexBuffers(cmd, 0, 2, vb, off);
        vkCmdBindIndexBuffer(cmd, mesh->ibo.buffer, 0, VK_INDEX_TYPE_UINT32);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r->scene_layout,
                                1, 1, &r->materials[b->material].set, 0, NULL);
        vkCmdDrawIndexed(cmd, mesh->index_count, b->instance_count, 0, 0, b->first_instance);
    }
    vkCmdEndRenderPass(cmd);

    hr_vk_image_barrier(cmd, t->hdr, VK_IMAGE_ASPECT_COLOR_BIT,
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 1);
    hr_vk_image_barrier(cmd, t->depth, VK_IMAGE_ASPECT_DEPTH_BIT,
                        VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL,
                        VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL,
                        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                        VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
                        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 1);

    /* ---- bloom chain ---- */
    u32 bw = HR_MAX(1u, t->hdr_width / 2u), bh = HR_MAX(1u, t->hdr_height / 2u);
    scissor.extent.width = bw;
    scissor.extent.height = bh;
    vp_view.width = (float)bw;
    vp_view.height = (float)bh;
    vkCmdSetViewport(cmd, 0, 1, &vp_view);
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    VkClearValue black = {0};
    black.color.float32[3] = 1.0f;
    VkRenderPassBeginInfo brp = { VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
    brp.renderPass = r->bloom_rp;
    brp.renderArea = scissor;
    brp.clearValueCount = 1;
    brp.pClearValues = &black;

    brp.framebuffer = t->bloom_a_fb;
    vkCmdBeginRenderPass(cmd, &brp, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r->bright_pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r->post_layout_a,
                            0, 1, &f->bright_set, 0, NULL);
    Vec4 bright_pc = v4(0.95f, 0.45f, r->frame.bloom_enabled ? 0.55f : 0.0f, 0);
    vkCmdPushConstants(cmd, r->post_layout_a, VK_SHADER_STAGE_FRAGMENT_BIT, 0, 16, &bright_pc);
    vkCmdDraw(cmd, 3, 1, 0, 0);
    vkCmdEndRenderPass(cmd);
    hr_vk_image_barrier(cmd, t->bloom_a, VK_IMAGE_ASPECT_COLOR_BIT,
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 1);

    brp.framebuffer = t->bloom_b_fb;
    vkCmdBeginRenderPass(cmd, &brp, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r->blur_pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r->post_layout_a,
                            0, 1, &f->blurh_set, 0, NULL);
    Vec4 blurh_pc = v4(1.0f / (float)bw, 0, 0, 0);
    vkCmdPushConstants(cmd, r->post_layout_a, VK_SHADER_STAGE_FRAGMENT_BIT, 0, 16, &blurh_pc);
    vkCmdDraw(cmd, 3, 1, 0, 0);
    vkCmdEndRenderPass(cmd);
    hr_vk_image_barrier(cmd, t->bloom_b, VK_IMAGE_ASPECT_COLOR_BIT,
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 1);

    brp.framebuffer = t->bloom_a_fb;
    vkCmdBeginRenderPass(cmd, &brp, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r->blur_pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r->post_layout_a,
                            0, 1, &f->blurv_set, 0, NULL);
    Vec4 blurv_pc = v4(0, 1.0f / (float)bh, 0, 0);
    vkCmdPushConstants(cmd, r->post_layout_a, VK_SHADER_STAGE_FRAGMENT_BIT, 0, 16, &blurv_pc);
    vkCmdDraw(cmd, 3, 1, 0, 0);
    vkCmdEndRenderPass(cmd);
    hr_vk_image_barrier(cmd, t->bloom_a, VK_IMAGE_ASPECT_COLOR_BIT,
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 1);

    /* ---- tone map + present target ---- */
    scissor.extent.width = t->ldr_width;
    scissor.extent.height = t->ldr_height;
    vp_view.width = (float)t->ldr_width;
    vp_view.height = (float)t->ldr_height;
    vkCmdSetViewport(cmd, 0, 1, &vp_view);
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    VkRenderPassBeginInfo lrp = { VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
    lrp.renderPass = r->ldr_rp;
    lrp.framebuffer = t->ldr_fb;
    lrp.renderArea = scissor;
    lrp.clearValueCount = 1;
    lrp.pClearValues = &black;
    vkCmdBeginRenderPass(cmd, &lrp, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r->post_pipeline);
    VkDescriptorSet post_sets[2] = { f->global_set, f->post_set };
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r->post_layout_b,
                            0, 2, post_sets, 0, NULL);
    Vec4 p0 = v4(r->frame.exposure, r->frame.bloom_enabled ? r->frame.bloom_intensity : 0.0f,
                 r->frame.saturation, r->frame.contrast);
    Vec4 p1 = v4(r->frame.vignette, r->frame.grain, r->frame.time,
                 r->frame.fxaa_enabled ? 1.0f : 0.0f);
    Vec4 p2 = v4(r->swapchain_srgb ? 0.0f : 1.0f, 0, 0, 0);
    Vec4 p3 = v4(nearp, farp, 0.85f, 0.0f);
    Vec4 post_pc[4] = { p0, p1, p2, p3 };
    vkCmdPushConstants(cmd, r->post_layout_b, VK_SHADER_STAGE_FRAGMENT_BIT, 0, 64, post_pc);
    vkCmdDraw(cmd, 3, 1, 0, 0);
    vkCmdEndRenderPass(cmd);

    hr_vk_image_barrier(cmd, t->ldr, VK_IMAGE_ASPECT_COLOR_BIT,
                        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                        VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT,
                        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                        VK_PIPELINE_STAGE_TRANSFER_BIT, 1);

    if (r->frame.screenshot_path && r->frame.screenshot_path[0]) {
        VkBufferImageCopy region;
        memset(&region, 0, sizeof(region));
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.layerCount = 1;
        region.imageExtent.width = t->ldr_width;
        region.imageExtent.height = t->ldr_height;
        region.imageExtent.depth = 1;
        vkCmdCopyImageToBuffer(cmd, t->ldr, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                               t->capture.buffer, 1, &region);
        r->capture_pending = true;
        snprintf(r->capture_path, sizeof(r->capture_path), "%s", r->frame.screenshot_path);
    }

    VkImage swap_img = r->swap_images[r->image_index];
    hr_vk_image_barrier(cmd, swap_img, VK_IMAGE_ASPECT_COLOR_BIT,
                        VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        0, VK_ACCESS_TRANSFER_WRITE_BIT,
                        VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 1);
    VkImageBlit blit;
    memset(&blit, 0, sizeof(blit));
    blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    blit.srcSubresource.layerCount = 1;
    blit.srcOffsets[1].x = (int)t->ldr_width;
    blit.srcOffsets[1].y = (int)t->ldr_height;
    blit.srcOffsets[1].z = 1;
    blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    blit.dstSubresource.layerCount = 1;
    blit.dstOffsets[1].x = (int)r->swap_extent.width;
    blit.dstOffsets[1].y = (int)r->swap_extent.height;
    blit.dstOffsets[1].z = 1;
    vkCmdBlitImage(cmd, t->ldr, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                   swap_img, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit, VK_FILTER_LINEAR);
    hr_vk_image_barrier(cmd, swap_img, VK_IMAGE_ASPECT_COLOR_BIT,
                        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                        VK_ACCESS_TRANSFER_WRITE_BIT, 0,
                        VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, 1);

    if (f->query_pool) vkCmdWriteTimestamp(cmd, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, f->query_pool, 1);
    VK_CHECK(vkEndCommandBuffer(cmd));

    VkPipelineStageFlags wait_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
    VkSubmitInfo submit = { VK_STRUCTURE_TYPE_SUBMIT_INFO };
    submit.waitSemaphoreCount = 1;
    submit.pWaitSemaphores = &f->image_available;
    submit.pWaitDstStageMask = &wait_stage;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &cmd;
    submit.signalSemaphoreCount = 1;
    submit.pSignalSemaphores = &f->render_finished;

    VK_CHECK(vkResetFences(r->device, 1, &f->fence));
    VK_CHECK(vkQueueSubmit(r->queue, 1, &submit, f->fence));
    f->submitted = true;
    f->queries_ready = f->query_pool != VK_NULL_HANDLE;

    VkPresentInfoKHR present = { VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
    present.waitSemaphoreCount = 1;
    present.pWaitSemaphores = &f->render_finished;
    present.swapchainCount = 1;
    present.pSwapchains = &r->swapchain;
    present.pImageIndices = &r->image_index;
    VkResult pr = vkQueuePresentKHR(r->queue, &present);
    if (pr == VK_ERROR_OUT_OF_DATE_KHR || pr == VK_SUBOPTIMAL_KHR) r->framebuffer_dirty = true;

    if (r->capture_pending) {
        VK_CHECK(vkWaitForFences(r->device, 1, &f->fence, VK_TRUE, UINT64_MAX));
        bool bgra = r->swap_format == VK_FORMAT_B8G8R8A8_SRGB ||
                    r->swap_format == VK_FORMAT_B8G8R8A8_UNORM;
        hr_vk_write_bmp(r->capture_path, t->capture.mapped,
                        t->ldr_width, t->ldr_height, bgra);
        r->capture_pending = false;
    }

    r->frame_counter++;
    r->frame_active = false;
    r->cpu_ms = (double)(hr_time_ns() - r->cpu_start_ns) * 1e-6;
}

void hr_renderer_get_stats(HrRenderer* r, HrRendererStats* out) {
    if (!r || !out) return;
    memset(out, 0, sizeof(*out));
    HrTargets* t = &r->targets[r->frame_index];
    out->cpu_frame_ms = r->cpu_ms;
    out->gpu_frame_ms = r->gpu_ms;
    out->draw_calls = r->draw_calls;
    out->instances = r->instance_count;
    out->triangles = r->triangle_count;
    out->hdr_width = t->hdr_width;
    out->hdr_height = t->hdr_height;
    out->output_width = r->swap_extent.width;
    out->output_height = r->swap_extent.height;
    out->frame_index = r->frame_counter;
    out->backend = "Vulkan 1.1";
    out->device_name = r->props.deviceName;
    out->vsync = r->vsync;
}
