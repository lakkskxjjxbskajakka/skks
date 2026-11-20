#include "horizon/core.h"

#include <stdarg.h>
#include <time.h>

#if defined(_WIN32)
#  include <windows.h>
#endif

static int g_log_level = HR_LOG_INFO_LEVEL;

void hr_log_set_level(int level) { g_log_level = level; }
int hr_log_get_level(void) { return g_log_level; }

void hr_log_write(int level, const char* tag, const char* fmt, ...) {
    if (level < g_log_level) return;
    static const char* names[] = { "TRACE", "DEBUG", "INFO ", "WARN ", "ERROR", "FATAL" };
    FILE* out = (level >= HR_LOG_WARN) ? stderr : stdout;
    fprintf(out, "[%s][%-5s] ", names[HR_CLAMP(level, 0, 5)], tag ? tag : "core");
    va_list ap;
    va_start(ap, fmt);
    vfprintf(out, fmt, ap);
    va_end(ap);
    fputc('\n', out);
    fflush(out);
}

/* ---------------------------------------------------------------- time */
u64 hr_time_ns(void) {
#if defined(_WIN32)
    static LARGE_INTEGER freq;
    static int once = 0;
    if (!once) { QueryPerformanceFrequency(&freq); once = 1; }
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    return (u64)((double)c.QuadPart * 1e9 / (double)freq.QuadPart);
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (u64)ts.tv_sec * 1000000000ull + (u64)ts.tv_nsec;
#endif
}

double hr_time_sec(void) { return (double)hr_time_ns() * 1e-9; }

void hr_sleep_ms(u32 ms) {
#if defined(_WIN32)
    Sleep(ms);
#else
    struct timespec ts;
    ts.tv_sec = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    nanosleep(&ts, NULL);
#endif
}

/* ------------------------------------------------------------ profiler */
int hr_prof_enabled = 1;

#define HR_PROF_MAX_SLOTS 64
#define HR_PROF_STACK 32

typedef struct HrProfSlot {
    const char* name;
    double frame_ms;
    double total_ms;
    double min_ms;
    double max_ms;
    u32 calls;
    u32 total_calls;
} HrProfSlot;

typedef struct HrProfiler {
    HrProfSlot slots[HR_PROF_MAX_SLOTS];
    int slot_count;
    struct { const char* name; u64 start; int slot; } stack[HR_PROF_STACK];
    int stack_depth;
    u64 frame_start;
    u64 total_start;
    u64 frames;
} HrProfiler;

static HrProfiler g_prof;

static int prof_slot(const char* name) {
    for (int i = 0; i < g_prof.slot_count; ++i)
        if (g_prof.slots[i].name == name) return i;
    if (g_prof.slot_count >= HR_PROF_MAX_SLOTS) return -1;
    int i = g_prof.slot_count++;
    memset(&g_prof.slots[i], 0, sizeof(g_prof.slots[i]));
    g_prof.slots[i].name = name;
    g_prof.slots[i].min_ms = 1e30;
    return i;
}

void hr_prof_frame_begin(void) {
    g_prof.frame_start = hr_time_ns();
    if (g_prof.total_start == 0) g_prof.total_start = g_prof.frame_start;
    g_prof.stack_depth = 0;
}

void hr_prof_frame_end(void) {
    u64 now = hr_time_ns();
    double frame_ms = (double)(now - g_prof.frame_start) * 1e-6;
    if (hr_prof_enabled) {
        for (int i = 0; i < g_prof.slot_count; ++i) {
            HrProfSlot* s = &g_prof.slots[i];
            s->total_ms += s->frame_ms;
            s->total_calls += s->calls;
            if (s->calls && s->frame_ms < s->min_ms) s->min_ms = s->frame_ms;
            if (s->frame_ms > s->max_ms) s->max_ms = s->frame_ms;
            s->frame_ms = 0.0;
            s->calls = 0;
        }
    }
    g_prof.frames++;
    (void)frame_ms;
}

void hr_prof_zone_begin(const char* name) {
    if (!hr_prof_enabled) return;
    if (g_prof.stack_depth >= HR_PROF_STACK) return;
    int s = prof_slot(name);
    if (s < 0) return;
    g_prof.stack[g_prof.stack_depth].name = name;
    g_prof.stack[g_prof.stack_depth].start = hr_time_ns();
    g_prof.stack[g_prof.stack_depth].slot = s;
    g_prof.stack_depth++;
}

void hr_prof_zone_end(void) {
    if (!hr_prof_enabled) return;
    if (g_prof.stack_depth <= 0) return;
    g_prof.stack_depth--;
    u64 now = hr_time_ns();
    u64 d = now - g_prof.stack[g_prof.stack_depth].start;
    HrProfSlot* s = &g_prof.slots[g_prof.stack[g_prof.stack_depth].slot];
    s->frame_ms += (double)d * 1e-6;
    s->calls++;
}

double hr_prof_zone_ms(const char* name) {
    for (int i = 0; i < g_prof.slot_count; ++i) {
        if (g_prof.slots[i].name == name) return g_prof.slots[i].frame_ms;
    }
    return 0.0;
}

void hr_prof_report(FILE* out) {
    if (!out) out = stdout;
    double elapsed = (g_prof.total_start && g_prof.frames)
        ? (double)(hr_time_ns() - g_prof.total_start) * 1e-9 : 0.0;
    fprintf(out, "--- Horizon profiler (%llu frames, %.2fs) ---\n",
            (unsigned long long)g_prof.frames, elapsed);
    for (int i = 0; i < g_prof.slot_count; ++i) {
        HrProfSlot* s = &g_prof.slots[i];
        if (!s->total_calls) continue;
        fprintf(out, "  %-24s avg %8.3f ms  min %8.3f  max %8.3f  calls %llu\n",
                s->name, s->total_ms / (double)g_prof.frames,
                s->min_ms == 1e30 ? 0.0 : s->min_ms, s->max_ms,
                (unsigned long long)s->total_calls);
    }
}
