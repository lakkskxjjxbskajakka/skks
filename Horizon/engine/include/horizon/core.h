/* Horizon Engine - core types, logging, timing, profiling. MIT licensed. */
#ifndef HORIZON_CORE_H
#define HORIZON_CORE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_MSC_VER)
#  define HR_INLINE static __forceinline
#  define HR_LIKELY(x) (x)
#  define HR_UNLIKELY(x) (x)
#  define HR_RESTRICT __restrict
#else
#  define HR_INLINE static inline __attribute__((always_inline))
#  define HR_LIKELY(x) __builtin_expect(!!(x), 1)
#  define HR_UNLIKELY(x) __builtin_expect(!!(x), 0)
#  define HR_RESTRICT __restrict__
#endif

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t   i8;
typedef int16_t  i16;
typedef int32_t  i32;
typedef int64_t  i64;
typedef size_t   usize;
typedef ptrdiff_t isize;

#define HR_MIN(a, b) ((a) < (b) ? (a) : (b))
#define HR_MAX(a, b) ((a) > (b) ? (a) : (b))
#define HR_CLAMP(x, lo, hi) HR_MIN(HR_MAX((x), (lo)), (hi))
#define HR_ALIGN_UP(x, a) (((x) + ((a) - 1)) & ~((usize)(a) - 1))
#define HR_ARRAY_COUNT(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define HR_KIB(n) ((usize)(n) * 1024u)
#define HR_MIB(n) (HR_KIB(n) * 1024u)
#define HR_GIB(n) (HR_MIB(n) * 1024u)
#define HR_UNUSED(x) ((void)(x))
#define HR_ENTITY_INVALID ((HrEntity)0)

/* ------------------------------------------------------------------ logging */
typedef enum HrLogLevel {
    HR_LOG_TRACE = 0,
    HR_LOG_DEBUG_LEVEL = 1,
    HR_LOG_INFO_LEVEL = 2,
    HR_LOG_WARN = 3,
    HR_LOG_ERROR = 4,
    HR_LOG_FATAL = 5
} HrLogLevel;

void hr_log_set_level(int level);
int  hr_log_get_level(void);
void hr_log_write(int level, const char* tag, const char* fmt, ...);

#define HR_LOG_TRACE_(tag, ...) hr_log_write(HR_LOG_TRACE, tag, __VA_ARGS__)
#define HR_DEBUG(tag, ...)      hr_log_write(HR_LOG_DEBUG_LEVEL, tag, __VA_ARGS__)
#define HR_INFO(tag, ...)       hr_log_write(HR_LOG_INFO_LEVEL, tag, __VA_ARGS__)
#define HR_WARN(tag, ...)       hr_log_write(HR_LOG_WARN, tag, __VA_ARGS__)
#define HR_ERROR(tag, ...)      hr_log_write(HR_LOG_ERROR, tag, __VA_ARGS__)
#define HR_FATAL(tag, ...)      do { hr_log_write(HR_LOG_FATAL, tag, __VA_ARGS__); abort(); } while (0)
#define HR_ASSERT(x) do { if (HR_UNLIKELY(!(x))) { \
    hr_log_write(HR_LOG_FATAL, "assert", "%s:%d: assertion failed: %s", __FILE__, __LINE__, #x); abort(); } } while (0)

/* --------------------------------------------------------------------- time */
u64  hr_time_ns(void);
double hr_time_sec(void);
void hr_sleep_ms(u32 ms);

/* ------------------------------------------------------------------ profiler */
void hr_prof_frame_begin(void);
void hr_prof_frame_end(void);
void hr_prof_zone_begin(const char* name);
void hr_prof_zone_end(void);
void hr_prof_report(FILE* out);
double hr_prof_zone_ms(const char* name);
extern int hr_prof_enabled;

typedef struct HrProfileZone {
    const char* name;
    u64 start_ns;
} HrProfileZone;

HR_INLINE HrProfileZone hr_prof_scope(const char* name) {
    HrProfileZone z = { name, 0 };
    if (hr_prof_enabled) { z.start_ns = hr_time_ns(); hr_prof_zone_begin(name); }
    return z;
}
HR_INLINE void hr_prof_scope_end(HrProfileZone* z) {
    if (hr_prof_enabled && z->start_ns) hr_prof_zone_end();
}

#endif /* HORIZON_CORE_H */
