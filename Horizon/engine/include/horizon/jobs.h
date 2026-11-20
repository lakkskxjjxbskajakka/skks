/* Horizon Engine - task scheduler / parallel-for. */
#ifndef HORIZON_JOBS_H
#define HORIZON_JOBS_H

#include "horizon/core.h"

typedef struct HrJobSystem HrJobSystem;

typedef void (*HrJobRangeFn)(void* arg, u32 begin, u32 end);
typedef void (*HrJobFn)(void* arg);

bool hr_jobs_init(HrJobSystem** out, u32 worker_count);
void hr_jobs_shutdown(HrJobSystem* js);
u32  hr_jobs_worker_count(const HrJobSystem* js);

/* Splits [0,count) into batches and blocks until all batches complete.
   The calling thread participates, so a single-core machine still works. */
void hr_jobs_parallel_for(HrJobSystem* js, u32 count, u32 batch_size,
                          HrJobRangeFn fn, void* arg);

/* Fire-and-forget task; must be followed by hr_jobs_wait_idle before shutdown. */
bool hr_jobs_submit(HrJobSystem* js, HrJobFn fn, void* arg);
void hr_jobs_wait_idle(HrJobSystem* js);

#endif /* HORIZON_JOBS_H */
