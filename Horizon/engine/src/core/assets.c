/* Horizon Engine - asset pack format, mesh serialization, async file loading. */
#include "horizon/assets.h"
#include "horizon/memory.h"

#include <stdio.h>

typedef struct HrPackBuilder {
    HrAssetEntry* entries;
    u32 count;
    u32 cap;
    u8* data;
    usize data_len;
    usize data_cap;
    char* strings;
    usize strings_len;
    usize strings_cap;
} HrPackBuilder;

typedef struct HrAssetFileHeader {
    u32 magic;
    u32 version;
    u32 count;
    u32 flags;
    u64 toc_offset;
    u64 strings_offset;
    u64 data_offset;
    u64 total_size;
} HrAssetFileHeader;

u32 hr_asset_fnv1a(const void* data, usize size) {
    const u8* p = (const u8*)data;
    u32 h = 2166136261u;
    for (usize i = 0; i < size; ++i) {
        h ^= (u32)p[i];
        h *= 16777619u;
    }
    return h;
}

static bool buf_grow_u8(u8** buf, usize* cap, usize need) {
    if (need <= *cap) return true;
    usize ncap = *cap ? *cap * 2u : 4096u;
    while (ncap < need) ncap *= 2u;
    u8* n = (u8*)hr_realloc(*buf, ncap);
    if (!n) return false;
    *buf = n;
    *cap = ncap;
    return true;
}

static bool buf_grow_char(char** buf, usize* cap, usize need) {
    if (need <= *cap) return true;
    usize ncap = *cap ? *cap * 2u : 1024u;
    while (ncap < need) ncap *= 2u;
    char* n = (char*)hr_realloc(*buf, ncap);
    if (!n) return false;
    *buf = n;
    *cap = ncap;
    return true;
}

static HrPackBuilder* builder_get(HrAssetPack* pack) {
    return (HrPackBuilder*)pack->data; /* abused during build; cleared on write/free */
}

bool hr_asset_pack_begin(HrAssetPack* pack) {
    if (!pack) return false;
    memset(pack, 0, sizeof(*pack));
    HrPackBuilder* b = (HrPackBuilder*)hr_calloc(1, sizeof(HrPackBuilder));
    if (!b) return false;
    /* reserve string table offset 0 as empty sentinel */
    buf_grow_char(&b->strings, &b->strings_cap, 1);
    b->strings[0] = '\0';
    b->strings_len = 1;
    pack->data = (u8*)b;
    pack->owns_data = true;
    return true;
}

bool hr_asset_pack_add(HrAssetPack* pack, HrAssetType type, const char* name,
                       const void* data, usize size) {
    if (!pack || !name) return false;
    HrPackBuilder* b = builder_get(pack);
    if (!b || pack->entry_count == 0xffffffffu) return false;

    if (b->count >= b->cap) {
        u32 ncap = b->cap ? b->cap * 2u : 8u;
        HrAssetEntry* ne = (HrAssetEntry*)hr_realloc(b->entries, ncap * sizeof(HrAssetEntry));
        if (!ne) return false;
        b->entries = ne;
        b->cap = ncap;
    }

    usize name_len = strlen(name) + 1;
    if (!buf_grow_char(&b->strings, &b->strings_cap, b->strings_len + name_len)) return false;
    u32 name_offset = (u32)b->strings_len;
    memcpy(b->strings + b->strings_len, name, name_len);
    b->strings_len += name_len;

    usize aligned = HR_ALIGN_UP(b->data_len, 16);
    usize need = aligned + size;
    if (!buf_grow_u8(&b->data, &b->data_cap, need)) return false;
    if (aligned > b->data_len) memset(b->data + b->data_len, 0, aligned - b->data_len);
    if (size > 0 && data) memcpy(b->data + aligned, data, size);
    b->data_len = need;

    HrAssetEntry* e = &b->entries[b->count++];
    memset(e, 0, sizeof(*e));
    e->type = (u32)type;
    e->name_hash = hr_asset_fnv1a(name, strlen(name));
    e->name_offset = name_offset;
    e->data_offset = (u64)aligned;
    e->data_size = (u64)size;
    e->checksum = hr_asset_fnv1a(data, size);
    return true;
}

bool hr_asset_pack_write(const HrAssetPack* pack, const char* path) {
    if (!pack || !path) return false;
    const HrPackBuilder* b = (const HrPackBuilder*)pack->data;
    if (!b) return false;

    HrAssetFileHeader h;
    memset(&h, 0, sizeof(h));
    h.magic = HR_ASSET_MAGIC;
    h.version = HR_ASSET_VERSION;
    h.count = b->count;
    h.toc_offset = (u64)sizeof(HrAssetFileHeader);
    h.strings_offset = h.toc_offset + (u64)b->count * (u64)sizeof(HrAssetEntry);
    h.data_offset = HR_ALIGN_UP((usize)h.strings_offset + b->strings_len, 16);
    h.total_size = (u64)h.data_offset + (u64)b->data_len;

    FILE* f = fopen(path, "wb");
    if (!f) return false;
    bool ok = true;
    ok = ok && fwrite(&h, sizeof(h), 1, f) == 1;
    if (b->count > 0) ok = ok && fwrite(b->entries, sizeof(HrAssetEntry), b->count, f) == b->count;
    if (b->strings_len > 0) ok = ok && fwrite(b->strings, 1, b->strings_len, f) == b->strings_len;
    usize pad = (usize)h.data_offset - ((usize)h.strings_offset + b->strings_len);
    if (pad > 0) {
        u8 zeros[16] = {0};
        ok = ok && fwrite(zeros, 1, pad, f) == pad;
    }
    if (b->data_len > 0) ok = ok && fwrite(b->data, 1, b->data_len, f) == b->data_len;
    fclose(f);
    return ok;
}

static void builder_free(HrPackBuilder* b) {
    if (!b) return;
    hr_free(b->entries);
    hr_free(b->data);
    hr_free(b->strings);
    hr_free(b);
}

void hr_asset_pack_free(HrAssetPack* pack) {
    if (!pack) return;
    if (pack->owns_data) {
        /* could be builder or loaded buffer; distinguish by entries pointer being inside? */
        /* builder stores HrPackBuilder* in pack->data and owns_data true, entries==NULL */
        if (pack->entries == NULL && pack->data != NULL) {
            builder_free((HrPackBuilder*)pack->data);
        } else {
            hr_free(pack->data);
        }
    }
    hr_free(pack->entries);
    hr_free(pack->strings);
    memset(pack, 0, sizeof(*pack));
}

bool hr_asset_pack_open(HrAssetPack* pack, const char* path) {
    if (!pack || !path) return false;
    memset(pack, 0, sizeof(*pack));
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fclose(f); return false; }
    usize size = (usize)sz;
    u8* data = (u8*)hr_alloc(size);
    if (!data) { fclose(f); return false; }
    bool ok = fread(data, 1, size, f) == size;
    fclose(f);
    if (!ok) { hr_free(data); return false; }
    ok = hr_asset_pack_open_memory(pack, data, size, true);
    if (!ok) { hr_free(data); memset(pack, 0, sizeof(*pack)); }
    return ok;
}

bool hr_asset_pack_open_memory(HrAssetPack* pack, const void* data, usize size, bool take_ownership) {
    if (!pack || !data || size < sizeof(HrAssetFileHeader)) return false;
    const HrAssetFileHeader* h = (const HrAssetFileHeader*)data;
    if (h->magic != HR_ASSET_MAGIC || h->version != HR_ASSET_VERSION) return false;
    if (h->toc_offset + (u64)h->count * sizeof(HrAssetEntry) > size) return false;
    if (h->strings_offset > size || h->data_offset > size || h->total_size > size) return false;

    memset(pack, 0, sizeof(*pack));
    pack->size = size;
    pack->entry_count = h->count;
    if (h->count > 0) {
        pack->entries = (HrAssetEntry*)hr_alloc(sizeof(HrAssetEntry) * h->count);
        if (!pack->entries) return false;
        memcpy(pack->entries, (const u8*)data + h->toc_offset, sizeof(HrAssetEntry) * h->count);
    }
    usize str_size = (usize)h->data_offset - (usize)h->strings_offset;
    if (str_size > 0) {
        pack->strings = (char*)hr_alloc(str_size);
        if (!pack->strings) { hr_asset_pack_free(pack); return false; }
        memcpy(pack->strings, (const u8*)data + h->strings_offset, str_size);
        pack->string_size = str_size;
    }
    if (take_ownership) {
        pack->data = (u8*)data;
        pack->owns_data = true;
    } else {
        pack->data = (u8*)hr_alloc(size);
        if (!pack->data) { hr_asset_pack_free(pack); return false; }
        memcpy(pack->data, data, size);
        pack->owns_data = true;
    }
    return true;
}

const HrAssetEntry* hr_asset_pack_find(const HrAssetPack* pack, const char* name) {
    if (!pack || !name) return NULL;
    u32 hash = hr_asset_fnv1a(name, strlen(name));
    for (u32 i = 0; i < pack->entry_count; ++i) {
        const HrAssetEntry* e = &pack->entries[i];
        if (e->name_hash != hash) continue;
        if (e->name_offset < pack->string_size) {
            if (strcmp(pack->strings + e->name_offset, name) == 0) return e;
        }
    }
    return NULL;
}

const void* hr_asset_pack_data(const HrAssetPack* pack, const HrAssetEntry* entry, usize* out_size) {
    if (!pack || !entry || !pack->data) return NULL;
    u64 end = entry->data_offset + entry->data_size;
    if (end > (u64)pack->size) return NULL;
    if (out_size) *out_size = (usize)entry->data_size;
    return pack->data + entry->data_offset;
}

/* ------------------------------------------------------------ mesh blob */
#define HR_MESH_BLOB_MAGIC 0x48524d53u /* 'HRMS' */
#define HR_MESH_BLOB_VER 1u

typedef struct HrMeshBlobHeader {
    u32 magic;
    u32 version;
    u32 vertex_count;
    u32 index_count;
    float bounds_min[3];
    float bounds_max[3];
    float bounds_center[3];
    float bounds_radius;
} HrMeshBlobHeader;

bool hr_mesh_serialize(const HrMeshData* mesh, void** out_data, usize* out_size) {
    if (!mesh || !out_data || !out_size) return false;
    usize vbytes = (usize)mesh->vertex_count * sizeof(HrVertex);
    usize ibytes = (usize)mesh->index_count * sizeof(u32);
    usize total = sizeof(HrMeshBlobHeader) + vbytes + ibytes;
    u8* buf = (u8*)hr_alloc(total);
    if (!buf) return false;
    HrMeshBlobHeader h;
    memset(&h, 0, sizeof(h));
    h.magic = HR_MESH_BLOB_MAGIC;
    h.version = HR_MESH_BLOB_VER;
    h.vertex_count = mesh->vertex_count;
    h.index_count = mesh->index_count;
    h.bounds_min[0] = mesh->bounds_min.x; h.bounds_min[1] = mesh->bounds_min.y; h.bounds_min[2] = mesh->bounds_min.z;
    h.bounds_max[0] = mesh->bounds_max.x; h.bounds_max[1] = mesh->bounds_max.y; h.bounds_max[2] = mesh->bounds_max.z;
    h.bounds_center[0] = mesh->bounds_center.x; h.bounds_center[1] = mesh->bounds_center.y; h.bounds_center[2] = mesh->bounds_center.z;
    h.bounds_radius = mesh->bounds_radius;
    memcpy(buf, &h, sizeof(h));
    if (vbytes) memcpy(buf + sizeof(h), mesh->vertices, vbytes);
    if (ibytes) memcpy(buf + sizeof(h) + vbytes, mesh->indices, ibytes);
    *out_data = buf;
    *out_size = total;
    return true;
}

bool hr_mesh_deserialize(const void* data, usize size, HrMeshData* out_mesh) {
    if (!data || !out_mesh || size < sizeof(HrMeshBlobHeader)) return false;
    const HrMeshBlobHeader* h = (const HrMeshBlobHeader*)data;
    if (h->magic != HR_MESH_BLOB_MAGIC || h->version != HR_MESH_BLOB_VER) return false;
    usize vbytes = (usize)h->vertex_count * sizeof(HrVertex);
    usize ibytes = (usize)h->index_count * sizeof(u32);
    if (sizeof(HrMeshBlobHeader) + vbytes + ibytes > size) return false;
    hr_mesh_init(out_mesh, "asset_mesh");
    if (h->vertex_count) {
        out_mesh->vertices = (HrVertex*)hr_alloc(vbytes);
        if (!out_mesh->vertices) { hr_mesh_free(out_mesh); return false; }
        memcpy(out_mesh->vertices, (const u8*)data + sizeof(HrMeshBlobHeader), vbytes);
        out_mesh->vertex_count = h->vertex_count;
        out_mesh->vertex_capacity = h->vertex_count;
    }
    if (h->index_count) {
        out_mesh->indices = (u32*)hr_alloc(ibytes);
        if (!out_mesh->indices) { hr_mesh_free(out_mesh); return false; }
        memcpy(out_mesh->indices, (const u8*)data + sizeof(HrMeshBlobHeader) + vbytes, ibytes);
        out_mesh->index_count = h->index_count;
        out_mesh->index_capacity = h->index_count;
    }
    out_mesh->bounds_min = v3(h->bounds_min[0], h->bounds_min[1], h->bounds_min[2]);
    out_mesh->bounds_max = v3(h->bounds_max[0], h->bounds_max[1], h->bounds_max[2]);
    out_mesh->bounds_center = v3(h->bounds_center[0], h->bounds_center[1], h->bounds_center[2]);
    out_mesh->bounds_radius = h->bounds_radius;
    return true;
}

/* --------------------------------------------------------------- async load */

static void async_load_job(void* arg) {
    HrAsyncAsset* a = (HrAsyncAsset*)arg;
    FILE* f = fopen(a->path, "rb");
    if (!f) { atomic_store(&a->state, HR_ASSET_LOAD_FAILED); return; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fclose(f); atomic_store(&a->state, HR_ASSET_LOAD_FAILED); return; }
    void* data = hr_alloc((usize)sz);
    if (!data) { fclose(f); atomic_store(&a->state, HR_ASSET_LOAD_FAILED); return; }
    bool ok = fread(data, 1, (usize)sz, f) == (usize)sz;
    fclose(f);
    if (!ok) { hr_free(data); atomic_store(&a->state, HR_ASSET_LOAD_FAILED); return; }
    a->data = data;
    a->size = (usize)sz;
    atomic_store(&a->state, HR_ASSET_LOAD_READY);
}

void hr_asset_async_init(HrAsyncAsset* a, const char* path) {
    if (!a) return;
    memset(a, 0, sizeof(*a));
    atomic_init(&a->state, HR_ASSET_LOAD_IDLE);
    if (path) snprintf(a->path, sizeof(a->path), "%s", path);
}

bool hr_asset_async_submit(HrAsyncAsset* a, HrJobSystem* jobs) {
    if (!a || !jobs) return false;
    atomic_store(&a->state, HR_ASSET_LOAD_LOADING);
    if (!hr_jobs_submit(jobs, async_load_job, a)) {
        atomic_store(&a->state, HR_ASSET_LOAD_FAILED);
        return false;
    }
    return true;
}

HrAssetLoadState hr_asset_async_state(const HrAsyncAsset* a) {
    if (!a) return HR_ASSET_LOAD_FAILED;
    return (HrAssetLoadState)atomic_load(&((HrAsyncAsset*)a)->state);
}

bool hr_asset_async_take(HrAsyncAsset* a, void** out_data, usize* out_size) {
    if (!a || !out_data || !out_size) return false;
    if (atomic_load(&a->state) != HR_ASSET_LOAD_READY) return false;
    *out_data = a->data;
    *out_size = a->size;
    a->data = NULL;
    a->size = 0;
    atomic_store(&a->state, HR_ASSET_LOAD_IDLE);
    return true;
}

void hr_asset_async_destroy(HrAsyncAsset* a) {
    if (!a) return;
    hr_free(a->data);
    a->data = NULL;
    a->size = 0;
    atomic_store(&a->state, HR_ASSET_LOAD_IDLE);
}
