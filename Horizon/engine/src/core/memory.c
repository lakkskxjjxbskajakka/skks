#include "horizon/memory.h"
#include <stdatomic.h>

/* ------------------------------------------------------------- heap tracking */
typedef struct HrHeapHeader {
    u64 magic;
    usize size;
    usize pad;
} HrHeapHeader;

#define HR_HEAP_MAGIC 0x48525a4e4d454d31ull
#define HR_HEAP_HDR ((usize)sizeof(HrHeapHeader))

static _Atomic usize g_used = 0;
static _Atomic usize g_peak = 0;
static _Atomic usize g_allocs = 0;

static void* heap_ptr_from_user(void* p) { return p ? (u8*)p - HR_HEAP_HDR : NULL; }

void* hr_alloc(usize size) {
    if (!size) return NULL;
    HrHeapHeader* h = (HrHeapHeader*)malloc(HR_HEAP_HDR + size);
    if (!h) return NULL;
    h->magic = HR_HEAP_MAGIC;
    h->size = size;
    h->pad = 0;
    usize used = atomic_fetch_add_explicit(&g_used, size, memory_order_relaxed) + size;
    usize peak = atomic_load_explicit(&g_peak, memory_order_relaxed);
    while (used > peak && !atomic_compare_exchange_weak(&g_peak, &peak, used)) {}
    atomic_fetch_add_explicit(&g_allocs, 1, memory_order_relaxed);
    return (u8*)h + HR_HEAP_HDR;
}

void* hr_calloc(usize count, usize size) {
    if (count && size > (usize)-1 / count) return NULL;
    usize total = count * size;
    void* p = hr_alloc(total);
    if (p) memset(p, 0, total);
    return p;
}

void hr_free(void* ptr) {
    if (!ptr) return;
    HrHeapHeader* h = (HrHeapHeader*)heap_ptr_from_user(ptr);
    if (h->magic != HR_HEAP_MAGIC) {
        HR_FATAL("memory", "corrupt or double free at %p", ptr);
    }
    h->magic = 0;
    atomic_fetch_sub_explicit(&g_used, h->size, memory_order_relaxed);
    free(h);
}

void* hr_realloc(void* ptr, usize size) {
    if (!ptr) return hr_alloc(size);
    if (!size) { hr_free(ptr); return NULL; }
    HrHeapHeader* h = (HrHeapHeader*)heap_ptr_from_user(ptr);
    if (h->magic != HR_HEAP_MAGIC) HR_FATAL("memory", "realloc: corrupt pointer %p", ptr);
    usize old = h->size;
    void* np = hr_alloc(size);
    if (!np) return NULL;
    memcpy(np, ptr, old < size ? old : size);
    hr_free(ptr);
    return np;
}

usize hr_mem_used(void) { return atomic_load_explicit(&g_used, memory_order_relaxed); }
usize hr_mem_peak(void) { return atomic_load_explicit(&g_peak, memory_order_relaxed); }
usize hr_mem_allocs(void) { return atomic_load_explicit(&g_allocs, memory_order_relaxed); }

/* -------------------------------------------------------------------- arena */
bool hr_arena_init(HrArena* a, usize capacity, const char* name) {
    memset(a, 0, sizeof(*a));
    a->base = (u8*)hr_alloc(capacity);
    if (!a->base) return false;
    a->capacity = capacity;
    a->name = name;
    return true;
}

void hr_arena_destroy(HrArena* a) {
    if (!a) return;
    hr_free(a->base);
    memset(a, 0, sizeof(*a));
}

void* hr_arena_alloc(HrArena* a, usize size, usize align) {
    if (!size) return NULL;
    if (align < 1) align = 1;
    usize start = HR_ALIGN_UP(a->used, align);
    if (start > a->capacity || size > a->capacity - start) {
        HR_ERROR("memory", "arena '%s' out of memory (%zu + %zu > %zu)",
                 a->name ? a->name : "?", start, size, a->capacity);
        return NULL;
    }
    void* p = a->base + start;
    a->used = start + size;
    if (a->used > a->peak) a->peak = a->used;
    return p;
}

void* hr_arena_alloc_zero(HrArena* a, usize size, usize align) {
    void* p = hr_arena_alloc(a, size, align);
    if (p) memset(p, 0, size);
    return p;
}

void hr_arena_reset(HrArena* a) { a->used = 0; }
usize hr_arena_mark(const HrArena* a) { return a->used; }
void hr_arena_rewind(HrArena* a, usize mark) { a->used = mark <= a->capacity ? mark : a->capacity; }

/* --------------------------------------------------------------------- pool */
bool hr_pool_init(HrPool* p, usize object_size, u32 capacity, const char* name) {
    memset(p, 0, sizeof(*p));
    usize stride = HR_ALIGN_UP(object_size < sizeof(u32) ? sizeof(u32) : object_size, 16);
    p->base = (u8*)hr_alloc(stride * capacity);
    p->free_list = (u32*)hr_alloc(sizeof(u32) * capacity);
    if (!p->base || !p->free_list) {
        hr_free(p->base);
        hr_free(p->free_list);
        memset(p, 0, sizeof(*p));
        return false;
    }
    p->object_size = stride;
    p->capacity = capacity;
    p->free_count = capacity;
    p->name = name;
    for (u32 i = 0; i < capacity; ++i) p->free_list[i] = capacity - 1 - i;
    return true;
}

void hr_pool_destroy(HrPool* p) {
    if (!p) return;
    hr_free(p->base);
    hr_free(p->free_list);
    memset(p, 0, sizeof(*p));
}

void* hr_pool_alloc(HrPool* p) {
    if (!p->free_count) {
        HR_ERROR("memory", "pool '%s' exhausted", p->name ? p->name : "?");
        return NULL;
    }
    u32 idx = p->free_list[--p->free_count];
    p->live++;
    return p->base + (usize)idx * p->object_size;
}

void hr_pool_free(HrPool* p, void* ptr) {
    if (!ptr) return;
    usize off = (u8*)ptr - p->base;
    HR_ASSERT(off % p->object_size == 0 && off / p->object_size < p->capacity);
    p->free_list[p->free_count++] = (u32)(off / p->object_size);
    p->live--;
}

void hr_pool_reset(HrPool* p) {
    p->free_count = p->capacity;
    p->live = 0;
    for (u32 i = 0; i < p->capacity; ++i) p->free_list[i] = p->capacity - 1 - i;
}

/* ---------------------------------------------------------------- free list */
struct HrFreeBlock {
    usize size;
    struct HrFreeBlock* prev;
    struct HrFreeBlock* next;
    u32 free;
    u32 magic;
};

#define HR_FL_MAGIC 0x46524c42u
#define HR_FL_HDR ((usize)HR_ALIGN_UP(sizeof(struct HrFreeBlock), 16))

static usize fl_payload(usize size) { return size < 16 ? 16 : HR_ALIGN_UP(size, 16); }

bool hr_freelist_init(HrFreeList* fl, usize capacity, const char* name) {
    memset(fl, 0, sizeof(*fl));
    fl->base = (u8*)hr_alloc(capacity);
    if (!fl->base) return false;
    fl->capacity = capacity;
    fl->name = name;
    struct HrFreeBlock* b = (struct HrFreeBlock*)fl->base;
    b->size = capacity - HR_FL_HDR;
    b->prev = NULL;
    b->next = NULL;
    b->free = 1;
    b->magic = HR_FL_MAGIC;
    fl->head = b;
    return true;
}

void hr_freelist_destroy(HrFreeList* fl) {
    if (!fl) return;
    hr_free(fl->base);
    memset(fl, 0, sizeof(*fl));
}

void* hr_freelist_alloc(HrFreeList* fl, usize size, usize align) {
    if (align < 16) align = 16;
    usize need = fl_payload(size);
    for (struct HrFreeBlock* b = fl->head; b; b = b->next) {
        if (!b->free) continue;
        usize payload = (usize)((u8*)b + HR_FL_HDR);
        usize aligned = HR_ALIGN_UP(payload, align);
        usize extra = aligned - payload;
        if (b->size < need + extra) continue;
        if (extra > 0 && extra < HR_FL_HDR + 16) continue;
        usize remain = b->size - extra - need;
        if (remain > 0 && remain < HR_FL_HDR + 16) {
            /* absorb leftover that is too small to become a block */
            need += remain;
            remain = 0;
        }
        struct HrFreeBlock* ab;
        if (extra > 0) {
            ab = (struct HrFreeBlock*)((u8*)payload + extra - HR_FL_HDR);
            ab->free = 0;
            ab->magic = HR_FL_MAGIC;
            ab->prev = b;
            ab->next = NULL;
            b->size = extra - HR_FL_HDR;
            b->next = ab;
        } else {
            ab = b;
        }
        ab->size = need;
        if (remain) {
            struct HrFreeBlock* nb = (struct HrFreeBlock*)((u8*)ab + HR_FL_HDR + need);
            nb->size = remain - HR_FL_HDR;
            nb->free = 1;
            nb->magic = HR_FL_MAGIC;
            nb->prev = ab;
            nb->next = b->next;
            if (nb->next) nb->next->prev = nb;
            ab->next = nb;
        } else {
            ab->next = b->next;
            if (ab->next) ab->next->prev = ab;
        }
        fl->used += ab->size + HR_FL_HDR;
        return (u8*)ab + HR_FL_HDR;
    }
    HR_ERROR("memory", "freelist '%s' cannot allocate %zu bytes", fl->name ? fl->name : "?", size);
    return NULL;
}

void hr_freelist_free(HrFreeList* fl, void* ptr) {
    if (!ptr) return;
    struct HrFreeBlock* b = (struct HrFreeBlock*)((u8*)ptr - HR_FL_HDR);
    if (b->magic != HR_FL_MAGIC || b->free) HR_FATAL("memory", "freelist: invalid free %p", ptr);
    b->free = 1;
    fl->used -= b->size + HR_FL_HDR;
    if (b->next && b->next->free) {
        b->size += HR_FL_HDR + b->next->size;
        b->next = b->next->next;
        if (b->next) b->next->prev = b;
    }
    if (b->prev && b->prev->free) {
        struct HrFreeBlock* p = b->prev;
        p->size += HR_FL_HDR + b->size;
        p->next = b->next;
        if (p->next) p->next->prev = p;
    }
}

usize hr_freelist_used(const HrFreeList* fl) { return fl->used; }
usize hr_freelist_capacity(const HrFreeList* fl) { return fl->capacity; }
u32 hr_freelist_blocks(const HrFreeList* fl) {
    u32 n = 0;
    for (struct HrFreeBlock* b = fl->head; b; b = b->next) n++;
    return n;
}
