/* Horizon Engine - allocators. All hot-path allocations are explicit. */
#ifndef HORIZON_MEMORY_H
#define HORIZON_MEMORY_H

#include "horizon/core.h"

void* hr_alloc(usize size);
void* hr_calloc(usize count, usize size);
void* hr_realloc(void* ptr, usize size);
void  hr_free(void* ptr);
usize hr_mem_used(void);
usize hr_mem_peak(void);
usize hr_mem_allocs(void);

/* Arena: bump allocation, reset frees everything at once. */
typedef struct HrArena {
    u8* base;
    usize capacity;
    usize used;
    usize peak;
    const char* name;
} HrArena;

bool  hr_arena_init(HrArena* a, usize capacity, const char* name);
void  hr_arena_destroy(HrArena* a);
void* hr_arena_alloc(HrArena* a, usize size, usize align);
void* hr_arena_alloc_zero(HrArena* a, usize size, usize align);
void  hr_arena_reset(HrArena* a);
usize hr_arena_mark(const HrArena* a);
void  hr_arena_rewind(HrArena* a, usize mark);

#define HR_ARENA_ALLOC(a, T) ((T*)hr_arena_alloc((a), sizeof(T), _Alignof(T)))
#define HR_ARENA_ALLOCN(a, T, n) ((T*)hr_arena_alloc((a), sizeof(T) * (usize)(n), _Alignof(T)))

/* Pool: fixed-size objects, O(1) alloc/free, no fragmentation. */
typedef struct HrPool {
    u8* base;
    usize object_size;
    u32 capacity;
    u32 live;
    u32* free_list;
    u32 free_count;
    const char* name;
} HrPool;

bool  hr_pool_init(HrPool* p, usize object_size, u32 capacity, const char* name);
void  hr_pool_destroy(HrPool* p);
void* hr_pool_alloc(HrPool* p);
void  hr_pool_free(HrPool* p, void* ptr);
void  hr_pool_reset(HrPool* p);

/* Free-list heap: variable size, coalescing, first fit. */
typedef struct HrFreeBlock HrFreeBlock;
typedef struct HrFreeList {
    u8* base;
    usize capacity;
    usize used;
    HrFreeBlock* head;
    const char* name;
} HrFreeList;

bool  hr_freelist_init(HrFreeList* fl, usize capacity, const char* name);
void  hr_freelist_destroy(HrFreeList* fl);
void* hr_freelist_alloc(HrFreeList* fl, usize size, usize align);
void  hr_freelist_free(HrFreeList* fl, void* ptr);
usize hr_freelist_used(const HrFreeList* fl);
usize hr_freelist_capacity(const HrFreeList* fl);
u32   hr_freelist_blocks(const HrFreeList* fl);

#endif /* HORIZON_MEMORY_H */
