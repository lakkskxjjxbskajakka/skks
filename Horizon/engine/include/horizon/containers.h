/* Horizon Engine - minimal, explicit dynamic containers. */
#ifndef HORIZON_CONTAINERS_H
#define HORIZON_CONTAINERS_H

#include "horizon/core.h"
#include "horizon/memory.h"

#define HR_DA(T) struct { T* items; usize len; usize cap; }

#define hr_dynarray(T) struct { T* items; usize len; usize cap; }

#define hr_da_init(da) do { (da)->items = NULL; (da)->len = 0; (da)->cap = 0; } while (0)
#define hr_da_free(da) do { hr_free((da)->items); hr_da_init(da); } while (0)
#define hr_da_reserve(da, n) do { \
    if ((usize)(n) > (da)->cap) { \
        usize newcap = (da)->cap ? (da)->cap * 2 : 8; \
        while (newcap < (usize)(n)) newcap *= 2; \
        void* np = hr_realloc((da)->items, newcap * sizeof(*(da)->items)); \
        HR_ASSERT(np); \
        (da)->items = np; \
        (da)->cap = newcap; \
    } \
} while (0)

#define hr_da_push(da, value) do { \
    hr_da_reserve((da), (da)->len + 1); \
    (da)->items[(da)->len++] = (value); \
} while (0)

#define hr_da_pop(da) ((da)->items[--(da)->len])
#define hr_da_clear(da) ((da)->len = 0)

#define hr_da_len(da) ((da).len)

#endif /* HORIZON_CONTAINERS_H */
