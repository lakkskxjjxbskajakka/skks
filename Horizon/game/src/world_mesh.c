/* Horizon Engine - procedural environment: race track, terrain, trees, rocks. */
#include "world_internal.h"
#include "horizon/memory.h"

static const Vec3 k_track_control[] = {
    {-20.0f,  2.0f, -170.0f},
    { 60.0f,  1.2f, -150.0f},
    {125.0f,  2.8f,  -95.0f},
    {145.0f,  0.4f,  -20.0f},
    {128.0f, -1.2f,   55.0f},
    { 80.0f,  1.6f,  118.0f},
    { 12.0f,  3.2f,  150.0f},
    {-62.0f,  2.0f,  140.0f},
    {-122.0f, -0.6f,   95.0f},
    {-150.0f,  1.0f,   25.0f},
    {-132.0f,  2.6f,  -55.0f},
    {-80.0f,  0.2f, -125.0f},
};

static Vec3 catmull_rom(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3, float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    Vec3 a = v3_scale(p1, 2.0f);
    Vec3 b = v3_scale(v3_sub(p2, p0), t);
    Vec3 c = v3_add(v3_sub(v3_scale(p0, 2.0f), v3_scale(p1, 5.0f)),
                    v3_sub(v3_scale(p2, 4.0f), p3));
    Vec3 d = v3_add(v3_sub(v3_neg(p0), p2), v3_add(v3_scale(p1, 3.0f), v3_neg(p3)));
    return v3_scale(v3_add(a, v3_add(v3_scale(b, t),
                    v3_add(v3_scale(c, t2), v3_scale(d, t3)))), 0.5f);
}

static float terrain_raw_height(float x, float z) {
    float base = hr_fbm2(x * 0.0042f, z * 0.0042f, 5, 7u) * 15.0f
               + hr_fbm2(x * 0.021f, z * 0.021f, 3, 19u) * 2.4f;
    float ridge = 1.0f - fabsf(hr_fbm2(x * 0.0052f + 3.7f, z * 0.0052f - 1.3f, 5, 31u));
    float r = sqrtf(x * x + z * z);
    float mountain = hr_smoothstep(230.0f, 540.0f, r) * (8.0f + ridge * ridge * 62.0f);
    return base + mountain;
}

int hr_road_nearest(HrWorld* w, float x, float z, float* out_distance) {
    int best = 0;
    float best_d = 1e30f;
    for (int i = 0; i < w->road_count; ++i) {
        float dx = w->road[i].pos.x - x;
        float dz = w->road[i].pos.z - z;
        float d = dx * dx + dz * dz;
        if (d < best_d) { best_d = d; best = i; }
    }
    if (out_distance) *out_distance = sqrtf(best_d);
    return best;
}

float hr_ground_height(HrWorld* w, float x, float z) {
    float h = terrain_raw_height(x, z);
    float d = 0.0f;
    int idx = hr_road_nearest(w, x, z, &d);
    if (d < 34.0f) {
        float blend = 1.0f - hr_smoothstep(7.0f, 30.0f, d);
        h = hr_lerp(h, w->road[idx].pos.y - 0.12f, blend);
    }
    return h;
}

Vec3 hr_road_point(const HrWorld* w, float t) {
    float f = t - floorf(t);
    int idx = (int)(f * (float)w->road_count) % w->road_count;
    if (idx < 0) idx += w->road_count;
    return w->road[idx].pos;
}

static void build_road_samples(HrWorld* w) {
    const int n = HR_ROAD_SAMPLES;
    const int nc = HR_ARRAY_COUNT(k_track_control);
    w->road = (HrRoadSample*)hr_calloc((usize)n, sizeof(HrRoadSample));
    HR_ASSERT(w->road);
    w->road_count = n;

    for (int i = 0; i < n; ++i) {
        float t = (float)i / (float)n * (float)nc;
        int seg = (int)t;
        float u = t - (float)seg;
        Vec3 p0 = k_track_control[(seg - 1 + nc) % nc];
        Vec3 p1 = k_track_control[seg % nc];
        Vec3 p2 = k_track_control[(seg + 1) % nc];
        Vec3 p3 = k_track_control[(seg + 2) % nc];
        w->road[i].pos = catmull_rom(p0, p1, p2, p3, u);
    }

    float total = 0.0f;
    for (int i = 0; i < n; ++i) {
        Vec3 prev = w->road[(i - 1 + n) % n].pos;
        Vec3 next = w->road[(i + 1) % n].pos;
        w->road[i].tangent = v3_norm(v3_sub(next, prev));
        w->road[i].right = v3_norm(v3_cross(v3(0, 1, 0), w->road[i].tangent));
        w->road[i].distance = total;
        Vec3 step = v3_sub(w->road[(i + 1) % n].pos, w->road[i].pos);
        total += v3_len(step);
    }
    w->road_length = total;

    for (int i = 0; i < n; ++i) {
        Vec3 a = w->road[(i - 1 + n) % n].tangent;
        Vec3 b = w->road[(i + 1) % n].tangent;
        float ang = acosf(HR_CLAMP(v3_dot(a, b), -1.0f, 1.0f));
        float ds = 0.0f;
        for (int k = -1; k <= 1; k += 2) {
            int j = (i + k + n) % n;
            ds += v3_dist(w->road[i].pos, w->road[j].pos) * 0.5f;
        }
        w->road[i].curvature = ang / HR_MAX(ds, 0.01f);
    }
}

typedef struct TerrainJobArg {
    HrWorld* w;
    HrMeshData* mesh;
    int segments;
    float size;
} TerrainJobArg;

static void terrain_row_job(void* arg_ptr, u32 begin, u32 end) {
    TerrainJobArg* arg = (TerrainJobArg*)arg_ptr;
    int seg = arg->segments;
    float half = arg->size * 0.5f;
    float step = arg->size / (float)seg;
    for (u32 r = begin; r < end; ++r) {
        float z = -half + step * (float)r;
        for (int c = 0; c <= seg; ++c) {
            float x = -half + step * (float)c;
            HrVertex* v = &arg->mesh->vertices[(usize)r * (usize)(seg + 1) + (usize)c];
            v->position = v3(x, hr_ground_height(arg->w, x, z), z);
            v->normal = v3(0, 1, 0);
            v->tangent = v4(1, 0, 0, 1);
            v->uv = v2(x / 16.0f, z / 16.0f);
        }
    }
}

static void build_terrain(HrWorld* w, HrMeshData* mesh) {
    const int seg = HR_TERRAIN_SEGMENTS;
    const u32 verts = (u32)(seg + 1) * (u32)(seg + 1);
    const u32 quads = (u32)seg * (u32)seg;
    hr_mesh_init(mesh, "terrain");
    mesh->vertices = (HrVertex*)hr_alloc(sizeof(HrVertex) * verts);
    mesh->vertex_capacity = verts;
    mesh->vertex_count = verts;
    mesh->indices = (u32*)hr_alloc(sizeof(u32) * (usize)quads * 6u);
    mesh->index_capacity = quads * 6u;
    mesh->index_count = quads * 6u;
    HR_ASSERT(mesh->vertices && mesh->indices);

    TerrainJobArg arg = { w, mesh, seg, HR_TERRAIN_SIZE };
    hr_jobs_parallel_for(w->jobs, (u32)(seg + 1), 8, terrain_row_job, &arg);

    u32 k = 0;
    for (int r = 0; r < seg; ++r) {
        for (int c = 0; c < seg; ++c) {
            u32 a = (u32)r * (u32)(seg + 1) + (u32)c;
            u32 b = a + 1;
            u32 d = a + (u32)(seg + 1);
            u32 e = d + 1;
            mesh->indices[k++] = a; mesh->indices[k++] = d; mesh->indices[k++] = e;
            mesh->indices[k++] = a; mesh->indices[k++] = e; mesh->indices[k++] = b;
        }
    }
    hr_mesh_recompute_normals(mesh);
    hr_mesh_finalize(mesh);
    w->mesh_terrain = hr_renderer_upload_mesh(w->renderer, mesh, "terrain");
    hr_mesh_free(mesh);
}

static void build_road_mesh(HrWorld* w) {
    HrMeshData road, shoulder;
    hr_mesh_init(&road, "road");
    hr_mesh_init(&shoulder, "shoulder");
    int n = w->road_count;
    const float hw = HR_ROAD_HALF_WIDTH;
    const float sw = HR_SHOULDER_WIDTH;
    for (int i = 0; i <= n; ++i) {
        const HrRoadSample* s = &w->road[i % n];
        Vec3 l = v3_add(s->pos, v3_scale(s->right, -hw));
        Vec3 r = v3_add(s->pos, v3_scale(s->right, hw));
        l.y += 0.055f;
        r.y += 0.055f;
        float v = s->distance / 9.0f;
        hr_mesh_vertex(&road, l, v3(0, 1, 0), v2(0.0f, v));
        hr_mesh_vertex(&road, r, v3(0, 1, 0), v2(1.0f, v));

        Vec3 sl = v3_add(s->pos, v3_scale(s->right, -(hw + sw)));
        Vec3 sr = v3_add(s->pos, v3_scale(s->right, (hw + sw)));
        sl.y += 0.025f;
        sr.y += 0.025f;
        Vec3 el = l; el.y -= 0.03f;
        Vec3 er = r; er.y -= 0.03f;
        hr_mesh_vertex(&shoulder, sl, v3(0, 1, 0), v2(0.0f, v));
        hr_mesh_vertex(&shoulder, el, v3(0, 1, 0), v2(0.3f, v));
        hr_mesh_vertex(&shoulder, er, v3(0, 1, 0), v2(0.7f, v));
        hr_mesh_vertex(&shoulder, sr, v3(0, 1, 0), v2(1.0f, v));
    }
    for (int i = 0; i < n; ++i) {
        u32 a = (u32)i * 2;
        u32 b = a + 2;
        hr_mesh_quad(&road, a, a + 1, b + 1, b);
        u32 c = (u32)i * 4;
        u32 d = c + 4;
        hr_mesh_quad(&shoulder, c, c + 1, d + 1, d);
        hr_mesh_quad(&shoulder, c + 2, c + 3, d + 3, d + 2);
    }
    hr_mesh_finalize(&road);
    hr_mesh_finalize(&shoulder);
    w->mesh_road = hr_renderer_upload_mesh(w->renderer, &road, "road");
    w->mesh_shoulder = hr_renderer_upload_mesh(w->renderer, &shoulder, "shoulder");
    hr_mesh_free(&road);
    hr_mesh_free(&shoulder);
}

static void cone_mesh(HrMeshData* m, float base_y, float radius, float height, int segments) {
    Vec3 apex = v3(0.0f, base_y + height, 0.0f);
    u32 base = m->vertex_count;
    for (int i = 0; i <= segments; ++i) {
        float a = (float)i / (float)segments * HR_TAU;
        float cx = cosf(a), sz = sinf(a);
        Vec3 p = v3(cx * radius, base_y, sz * radius);
        Vec3 n = v3_norm(v3(cx * height, radius * 0.75f, sz * height));
        hr_mesh_vertex(m, p, n, v2((float)i / (float)segments, 0.0f));
    }
    u32 apex_idx = hr_mesh_vertex(m, apex, v3(0, 1, 0), v2(0.5f, 1.0f));
    for (int i = 0; i < segments; ++i) {
        hr_mesh_triangle(m, base + (u32)i, apex_idx, base + (u32)i + 1);
    }
}

static void build_vegetation_meshes(HrWorld* w) {
    HrMeshData trunk, pine, leaf;
    hr_mesh_init(&trunk, "trunk");
    hr_mesh_cylinder(&trunk, v3(0, 0, 0), 0.24f, 2.8f, 7, true);
    hr_mesh_finalize(&trunk);
    w->mesh_trunk = hr_renderer_upload_mesh(w->renderer, &trunk, "trunk");
    hr_mesh_free(&trunk);

    hr_mesh_init(&pine, "pine");
    cone_mesh(&pine, 1.35f, 2.30f, 2.7f, 9);
    cone_mesh(&pine, 2.55f, 1.95f, 2.7f, 9);
    cone_mesh(&pine, 3.75f, 1.60f, 2.6f, 9);
    cone_mesh(&pine, 4.85f, 1.20f, 2.4f, 9);
    cone_mesh(&pine, 5.85f, 0.78f, 2.1f, 9);
    hr_mesh_recompute_normals(&pine);
    hr_mesh_finalize(&pine);
    w->mesh_pine = hr_renderer_upload_mesh(w->renderer, &pine, "pine");
    hr_mesh_free(&pine);

    hr_mesh_init(&leaf, "leaf");
    hr_mesh_uv_sphere(&leaf, v3(0, 4.3f, 0), 2.0f, 6, 10);
    hr_mesh_uv_sphere(&leaf, v3(1.05f, 5.1f, 0.3f), 1.35f, 5, 9);
    hr_mesh_uv_sphere(&leaf, v3(-0.95f, 5.35f, -0.4f), 1.2f, 5, 9);
    hr_mesh_uv_sphere(&leaf, v3(0.15f, 5.9f, -0.1f), 0.95f, 5, 8);
    hr_mesh_finalize(&leaf);
    w->mesh_leaf = hr_renderer_upload_mesh(w->renderer, &leaf, "leaf");
    hr_mesh_free(&leaf);
}

static void build_rock_mesh(HrWorld* w) {
    HrMeshData rock;
    hr_mesh_init(&rock, "rock");
    hr_mesh_uv_sphere(&rock, v3(0, 0.35f, 0), 1.0f, 5, 7);
    for (u32 i = 0; i < rock.vertex_count; ++i) {
        Vec3 p = rock.vertices[i].position;
        float n = hr_value_noise3(p.x * 2.7f, p.y * 2.7f, p.z * 2.7f, 77u);
        Vec3 dir = v3_norm(v3(p.x, p.y - 0.35f, p.z));
        rock.vertices[i].position = v3_add(p, v3_scale(dir, n * 0.28f));
        rock.vertices[i].position.y = HR_MAX(rock.vertices[i].position.y, -0.05f);
    }
    hr_mesh_recompute_normals(&rock);
    hr_mesh_finalize(&rock);
    w->mesh_rock = hr_renderer_upload_mesh(w->renderer, &rock, "rock");
    hr_mesh_free(&rock);

    HrMeshData post;
    hr_mesh_init(&post, "post");
    hr_mesh_box(&post, v3(0, 0.55f, 0), v3(0.075f, 0.55f, 0.075f));
    hr_mesh_box(&post, v3(0, 0.8f, 0.07f), v3(0.11f, 0.12f, 0.03f));
    hr_mesh_finalize(&post);
    w->mesh_post = hr_renderer_upload_mesh(w->renderer, &post, "post");
    hr_mesh_free(&post);
}

static void place_props(HrWorld* w) {
    const int target_trees = 880;
    const int target_rocks = 120;
    w->trees = (HrTreeInstance*)hr_calloc((usize)target_trees, sizeof(HrTreeInstance));
    w->rocks = (HrRockInstance*)hr_calloc((usize)target_rocks, sizeof(HrRockInstance));
    HR_ASSERT(w->trees && w->rocks);

    u32 seed = 0x51ed2a11u;
    int placed = 0;
    for (int attempt = 0; attempt < target_trees * 12 && placed < target_trees; ++attempt) {
        seed = hr_hash_u32(seed + 0x9e3779b9u);
        float a = hr_hash_float(seed) * HR_TAU;
        float r = sqrtf(hr_hash_float(seed ^ 0x51u)) * 430.0f;
        float x = cosf(a) * r;
        float z = sinf(a) * r;
        float dist = 0.0f;
        hr_road_nearest(w, x, z, &dist);
        if (dist < 10.5f) continue;
        HrTreeInstance* t = &w->trees[placed];
        t->pos = v3(x, hr_ground_height(w, x, z), z);
        t->kind = (hr_hash_float(seed ^ 0x8811u) > 0.76f) ? 1 : 0;
        t->scale = 0.75f + hr_hash_float(seed ^ 0x1234u) * 0.95f;
        t->rotation = hr_hash_float(seed ^ 0xabcd) * HR_TAU;
        float warmth = hr_hash_float(seed ^ 0x777u);
        t->tint = v4(0.58f + warmth * 0.18f, 0.78f + hr_hash_float(seed ^ 0x999u) * 0.22f,
                     0.52f + (1.0f - warmth) * 0.16f, 1.0f);
        placed++;
    }
    w->tree_count = placed;

    placed = 0;
    seed = 0x2f13b7c9u;
    for (int attempt = 0; attempt < target_rocks * 20 && placed < target_rocks; ++attempt) {
        seed = hr_hash_u32(seed + 0x85ebca6bu);
        float a = hr_hash_float(seed) * HR_TAU;
        float r = sqrtf(hr_hash_float(seed ^ 0x33u)) * 330.0f;
        float x = cosf(a) * r;
        float z = sinf(a) * r;
        float dist = 0.0f;
        hr_road_nearest(w, x, z, &dist);
        if (dist < 7.5f) continue;
        HrRockInstance* rock = &w->rocks[placed];
        rock->pos = v3(x, hr_ground_height(w, x, z) - 0.1f, z);
        float s = 0.35f + hr_hash_float(seed ^ 0x44u) * 1.9f;
        rock->scale = v3(s * (0.8f + hr_hash_float(seed ^ 0xa1u) * 0.6f), s * 0.7f,
                         s * (0.8f + hr_hash_float(seed ^ 0xb2u) * 0.6f));
        rock->rotation = hr_hash_float(seed ^ 0xc3u) * HR_TAU;
        rock->tint = v4(0.62f + hr_hash_float(seed ^ 0xd4u) * 0.25f, 0.6f, 0.56f, 1.0f);
        placed++;
    }
    w->rock_count = placed;

    int post_cap = w->road_count / 8 * 2 + 8;
    w->posts = (HrPostInstance*)hr_calloc((usize)post_cap, sizeof(HrPostInstance));
    HR_ASSERT(w->posts);
    int pc = 0;
    for (int i = 0; i < w->road_count; i += 8) {
        const HrRoadSample* s = &w->road[i];
        float yaw = atan2f(s->tangent.x, s->tangent.z);
        for (int side = -1; side <= 1; side += 2) {
            Vec3 p = v3_add(s->pos, v3_scale(s->right, (HR_ROAD_HALF_WIDTH + 3.2f) * (float)side));
            p.y = hr_ground_height(w, p.x, p.z);
            w->posts[pc].pos = p;
            w->posts[pc].rotation = yaw;
            pc++;
        }
    }
    w->post_count = pc;
}

void hr_world_build_environment(HrWorld* w) {
    build_road_samples(w);

    HrMeshData terrain;
    build_terrain(w, &terrain);
    build_road_mesh(w);
    build_vegetation_meshes(w);
    build_rock_mesh(w);
    place_props(w);

    u32 verts = (u32)(HR_TERRAIN_SEGMENTS + 1) * (u32)(HR_TERRAIN_SEGMENTS + 1);
    u32 tris = (u32)HR_TERRAIN_SEGMENTS * (u32)HR_TERRAIN_SEGMENTS * 2u;
    HR_INFO("world", "terrain %u verts, %u tris; %d trees, %d rocks, %d posts; track %.0f m",
            verts, tris, w->tree_count, w->rock_count, w->post_count, w->road_length);
}
