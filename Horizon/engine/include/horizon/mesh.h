/* Horizon Engine - CPU mesh construction helpers. */
#ifndef HORIZON_MESH_H
#define HORIZON_MESH_H

#include "horizon/math.h"

typedef struct HrVertex {
    Vec3 position;
    Vec3 normal;
    Vec4 tangent;
    Vec2 uv;
} HrVertex;

typedef struct HrMeshData {
    HrVertex* vertices;
    u32 vertex_count;
    u32 vertex_capacity;
    u32* indices;
    u32 index_count;
    u32 index_capacity;
    Vec3 bounds_min;
    Vec3 bounds_max;
    Vec3 bounds_center;
    float bounds_radius;
    const char* name;
} HrMeshData;

void hr_mesh_init(HrMeshData* m, const char* name);
void hr_mesh_free(HrMeshData* m);
u32  hr_mesh_vertex(HrMeshData* m, Vec3 p, Vec3 n, Vec2 uv);
void hr_mesh_triangle(HrMeshData* m, u32 a, u32 b, u32 c);
void hr_mesh_quad(HrMeshData* m, u32 a, u32 b, u32 c, u32 d);
void hr_mesh_recompute_normals(HrMeshData* m);
void hr_mesh_compute_tangents(HrMeshData* m);
void hr_mesh_finalize(HrMeshData* m);

/* primitives (normals/uvs provided, no tangent until finalize) */
void hr_mesh_box(HrMeshData* m, Vec3 center, Vec3 half_extents);
void hr_mesh_cylinder(HrMeshData* m, Vec3 center, float radius, float height,
                      int segments, bool capped);
void hr_mesh_uv_sphere(HrMeshData* m, Vec3 center, float radius, int rings, int segments);
void hr_mesh_plane(HrMeshData* m, Vec3 center, Vec2 size, Vec2 uv_scale);

#endif /* HORIZON_MESH_H */
