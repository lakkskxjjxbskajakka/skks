#include "horizon/jobs.h"
#include "horizon/memory.h"

#include <pthread.h>
#include <unistd.h>
#include <stdatomic.h>
#include <time.h>

#if defined(_WIN32)
#  include <windows.h>
#endif

typedef struct HrTask {
    HrJobRangeFn range_fn;
    HrJobFn fn;
    void* arg;
    u32 begin;
    u32 end;
} HrTask;

#define HR_JOB_QUEUE_CAP 65536u

struct HrJobSystem {
    pthread_t threads[64];
    u32 worker_count;
    pthread_mutex_t lock;
    pthread_cond_t work_cv;
    pthread_cond_t done_cv;
    HrTask queue[HR_JOB_QUEUE_CAP];
    u32 head;
    u32 tail;
    atomic_uint pending;
    atomic_bool stop;
    atomic_uint active;
};

static u32 g_hw_workers = 0;

static u32 detect_cores(void) {
#if defined(_WIN32)
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    return si.dwNumberOfProcessors ? si.dwNumberOfProcessors : 1;
#else
    long n = sysconf(_SC_NPROCESSORS_ONLN);
    return n > 0 ? (u32)n : 1u;
#endif
}

static bool queue_push(HrJobSystem* js, HrTask t) {
    bool ok = false;
    pthread_mutex_lock(&js->lock);
    u32 next = (js->tail + 1u) % HR_JOB_QUEUE_CAP;
    if (next != js->head) {
        js->queue[js->tail] = t;
        js->tail = next;
        ok = true;
        pthread_cond_signal(&js->work_cv);
    }
    pthread_mutex_unlock(&js->lock);
    return ok;
}

static bool queue_pop(HrJobSystem* js, HrTask* out) {
    bool ok = false;
    pthread_mutex_lock(&js->lock);
    if (js->head != js->tail) {
        *out = js->queue[js->head];
        js->head = (js->head + 1u) % HR_JOB_QUEUE_CAP;
        ok = true;
    }
    pthread_mutex_unlock(&js->lock);
    return ok;
}

static void run_task(HrTask* t) {
    if (t->range_fn) t->range_fn(t->arg, t->begin, t->end);
    else if (t->fn) t->fn(t->arg);
}

static void* worker_main(void* user) {
    HrJobSystem* js = (HrJobSystem*)user;
    for (;;) {
        HrTask t;
        memset(&t, 0, sizeof(t));
        bool have = false;
        pthread_mutex_lock(&js->lock);
        while (!have && !atomic_load_explicit(&js->stop, memory_order_relaxed)) {
            if (js->head != js->tail) {
                t = js->queue[js->head];
                js->head = (js->head + 1u) % HR_JOB_QUEUE_CAP;
                have = true;
            } else {
                pthread_cond_wait(&js->work_cv, &js->lock);
            }
        }
        pthread_mutex_unlock(&js->lock);
        if (!have && atomic_load_explicit(&js->stop, memory_order_relaxed)) break;
        run_task(&t);
        if (atomic_fetch_sub_explicit(&js->pending, 1, memory_order_acq_rel) == 1) {
            pthread_mutex_lock(&js->lock);
            pthread_cond_broadcast(&js->done_cv);
            pthread_mutex_unlock(&js->lock);
        }
    }
    return NULL;
}

bool hr_jobs_init(HrJobSystem** out, u32 worker_count) {
    HrJobSystem* js = (HrJobSystem*)hr_calloc(1, sizeof(HrJobSystem));
    if (!js) return false;
    pthread_mutex_init(&js->lock, NULL);
    pthread_cond_init(&js->work_cv, NULL);
    pthread_cond_init(&js->done_cv, NULL);
    atomic_init(&js->pending, 0);
    atomic_init(&js->stop, false);
    atomic_init(&js->active, 0);

    if (worker_count == 0) {
        u32 hw = detect_cores();
        worker_count = hw > 1 ? hw - 1 : 0;
    }
    if (worker_count > 63) worker_count = 63;
    js->worker_count = worker_count;
    g_hw_workers = detect_cores();
    for (u32 i = 0; i < worker_count; ++i) {
        if (pthread_create(&js->threads[i], NULL, worker_main, js) != 0) {
            HR_WARN("jobs", "failed to start worker %u", i);
            js->worker_count = i;
            break;
        }
    }
    *out = js;
    HR_INFO("jobs", "scheduler ready: %u workers + caller (%u hw threads)",
            js->worker_count, g_hw_workers);
    return true;
}

void hr_jobs_shutdown(HrJobSystem* js) {
    if (!js) return;
    hr_jobs_wait_idle(js);
    atomic_store(&js->stop, true);
    pthread_mutex_lock(&js->lock);
    pthread_cond_broadcast(&js->work_cv);
    pthread_mutex_unlock(&js->lock);
    for (u32 i = 0; i < js->worker_count; ++i) pthread_join(js->threads[i], NULL);
    pthread_mutex_destroy(&js->lock);
    pthread_cond_destroy(&js->work_cv);
    pthread_cond_destroy(&js->done_cv);
    hr_free(js);
}

u32 hr_jobs_worker_count(const HrJobSystem* js) { return js ? js->worker_count : 0; }

void hr_jobs_parallel_for(HrJobSystem* js, u32 count, u32 batch_size,
                          HrJobRangeFn fn, void* arg) {
    if (!js || count == 0 || !fn) return;
    if (batch_size == 0) batch_size = 1;
    u32 tasks = (count + batch_size - 1u) / batch_size;
    if (tasks == 1) { fn(arg, 0, count); return; }

    atomic_fetch_add_explicit(&js->pending, tasks, memory_order_relaxed);
    for (u32 i = 0; i < tasks; ++i) {
        HrTask t;
        t.range_fn = fn;
        t.fn = NULL;
        t.arg = arg;
        t.begin = i * batch_size;
        t.end = HR_MIN(count, t.begin + batch_size);
        if (!queue_push(js, t)) {
            run_task(&t);
            if (atomic_fetch_sub_explicit(&js->pending, 1, memory_order_acq_rel) == 1) {
                pthread_mutex_lock(&js->lock);
                pthread_cond_broadcast(&js->done_cv);
                pthread_mutex_unlock(&js->lock);
            }
        }
    }

    /* caller helps drain the queue */
    for (;;) {
        if (atomic_load_explicit(&js->pending, memory_order_acquire) == 0) break;
        HrTask t;
        if (queue_pop(js, &t)) {
            run_task(&t);
            if (atomic_fetch_sub_explicit(&js->pending, 1, memory_order_acq_rel) == 1) {
                pthread_mutex_lock(&js->lock);
                pthread_cond_broadcast(&js->done_cv);
                pthread_mutex_unlock(&js->lock);
            }
        } else {
            pthread_mutex_lock(&js->lock);
            while (atomic_load_explicit(&js->pending, memory_order_acquire) != 0) {
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                ts.tv_nsec += 1000000L;
                if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
                pthread_cond_timedwait(&js->done_cv, &js->lock, &ts);
            }
            pthread_mutex_unlock(&js->lock);
            break;
        }
    }
}

bool hr_jobs_submit(HrJobSystem* js, HrJobFn fn, void* arg) {
    if (!js || !fn) return false;
    HrTask t;
    t.range_fn = NULL;
    t.fn = fn;
    t.arg = arg;
    t.begin = t.end = 0;
    atomic_fetch_add_explicit(&js->pending, 1, memory_order_relaxed);
    if (!queue_push(js, t)) {
        atomic_fetch_sub_explicit(&js->pending, 1, memory_order_relaxed);
        return false;
    }
    return true;
}

void hr_jobs_wait_idle(HrJobSystem* js) {
    if (!js) return;
    for (;;) {
        if (atomic_load_explicit(&js->pending, memory_order_acquire) == 0) return;
        HrTask t;
        if (queue_pop(js, &t)) {
            run_task(&t);
            if (atomic_fetch_sub_explicit(&js->pending, 1, memory_order_acq_rel) == 1) {
                pthread_mutex_lock(&js->lock);
                pthread_cond_broadcast(&js->done_cv);
                pthread_mutex_unlock(&js->lock);
            }
        } else {
            pthread_mutex_lock(&js->lock);
            if (atomic_load_explicit(&js->pending, memory_order_acquire) != 0)
                pthread_cond_wait(&js->done_cv, &js->lock);
            pthread_mutex_unlock(&js->lock);
        }
    }
}
