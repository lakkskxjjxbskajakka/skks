/* Horizon Engine - binary asset pack, mesh serialization, cache and async load. */
#ifndef HORIZON_ASSETS_H
#define HORIZON_ASSETS_H

#include "horizon/core.h"
#include "horizon/mesh.h"
#include "horizon/jobs.h"

#include <stdatomic.h>

#define HR_ASSET_MAGIC 0x53415248u /* 'HRAS' */
#define HR_ASSET_VERSION 1u

typedef enum HrAssetType {
    HR_ASSET_BLOB = 0,
    HR_ASSET_MESH = 1,
    HR_ASSET_TEXTURE = 2,
    HR_ASSET_MATERIAL = 3,
    HR_ASSET_SCENE = 4
} HrAssetType;

typedef struct HrAssetEntry {
    u32 type;
    u32 name_hash;
    u32 name_offset;
    u32 flags;
    u64 data_offset;
    u64 data_size;
    u32 checksum;
    u32 reserved;
} HrAssetEntry;

typedef struct HrAssetPack {
    u8* data;
    usize size;
    HrAssetEntry* entries;
    u32 entry_count;
    char* strings;
    usize string_size;
    bool owns_data;
    bool building;
} HrAssetPack;

typedef enum HrAssetLoadState {
    HR_ASSET_LOAD_IDLE = 0,
    HR_ASSET_LOAD_LOADING,
    HR_ASSET_LOAD_READY,
    HR_ASSET_LOAD_FAILED
} HrAssetLoadState;

typedef struct HrAsyncAsset {
    _Atomic int state;
    char path[512];
    void* data;
    usize size;
} HrAsyncAsset;

/* Pack builder / IO */
bool hr_asset_pack_begin(HrAssetPack* pack);
bool hr_asset_pack_add(HrAssetPack* pack, HrAssetType type, const char* name,
                       const void* data, usize size);
bool hr_asset_pack_write(const HrAssetPack* pack, const char* path);
void hr_asset_pack_free(HrAssetPack* pack);

bool hr_asset_pack_open(HrAssetPack* pack, const char* path);
bool hr_asset_pack_open_memory(HrAssetPack* pack, const void* data, usize size, bool take_ownership);
const HrAssetEntry* hr_asset_pack_find(const HrAssetPack* pack, const char* name);
const void* hr_asset_pack_data(const HrAssetPack* pack, const HrAssetEntry* entry, usize* out_size);

/* Mesh serialization (little-endian binary blob). */
bool hr_mesh_serialize(const HrMeshData* mesh, void** out_data, usize* out_size);
bool hr_mesh_deserialize(const void* data, usize size, HrMeshData* out_mesh);

/* Async file load through job system (caller owns returned data via hr_free). */
void hr_asset_async_init(HrAsyncAsset* a, const char* path);
bool hr_asset_async_submit(HrAsyncAsset* a, HrJobSystem* jobs);
HrAssetLoadState hr_asset_async_state(const HrAsyncAsset* a);
bool hr_asset_async_take(HrAsyncAsset* a, void** out_data, usize* out_size);
void hr_asset_async_destroy(HrAsyncAsset* a);

u32 hr_asset_fnv1a(const void* data, usize size);

#endif /* HORIZON_ASSETS_H */
