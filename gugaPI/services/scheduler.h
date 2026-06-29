#ifndef SERVICES_SCHEDULER_H_
#define SERVICES_SCHEDULER_H_

#include <stdbool.h>
#include <stdint.h>

namespace services {

typedef void (*SchedulerTaskCallback)(void);
typedef uint8_t SchedulerTaskId;

enum SchedulerStatus {
    SCHEDULER_OK = 0,
    SCHEDULER_ERROR_FULL,
    SCHEDULER_ERROR_INVALID_ARG,
    SCHEDULER_ERROR_INVALID_ID
};

struct SchedulerTaskInfo {
    const char *name;
    uint32_t period_ms;
    uint32_t next_run_ms;
    bool enabled;
};

void Scheduler_Init(void);
SchedulerStatus Scheduler_AddTask(const char *name,
                                  SchedulerTaskCallback callback,
                                  uint32_t period_ms,
                                  uint32_t start_delay_ms,
                                  SchedulerTaskId *out_id);
SchedulerStatus Scheduler_EnableTask(SchedulerTaskId id, bool enabled);
SchedulerStatus Scheduler_GetTaskInfo(SchedulerTaskId id,
                                      SchedulerTaskInfo *out_info);
void Scheduler_Run(void);

} /* namespace services */

#endif /* SERVICES_SCHEDULER_H_ */