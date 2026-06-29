#include "services/scheduler.h"

#include "board/board_config.h"
#include "services/time.h"

namespace services {

struct SchedulerTaskSlot {
    const char *name;
    SchedulerTaskCallback callback;
    uint32_t period_ms;
    uint32_t next_run_ms;
    bool enabled;
    bool used;
};

static SchedulerTaskSlot g_tasks[BOARD_SCHEDULER_MAX_TASKS];

static bool IsDue(uint32_t now_ms, uint32_t target_ms)
{
    return ((int32_t) (now_ms - target_ms) >= 0);
}

void Scheduler_Init(void)
{
    for (uint32_t i = 0U; i < BOARD_SCHEDULER_MAX_TASKS; i++) {
        g_tasks[i].name = 0;
        g_tasks[i].callback = 0;
        g_tasks[i].period_ms = 0U;
        g_tasks[i].next_run_ms = 0U;
        g_tasks[i].enabled = false;
        g_tasks[i].used = false;
    }
}

SchedulerStatus Scheduler_AddTask(const char *name,
                                  SchedulerTaskCallback callback,
                                  uint32_t period_ms,
                                  uint32_t start_delay_ms,
                                  SchedulerTaskId *out_id)
{
    if ((callback == 0) || (period_ms == 0U)) {
        return SCHEDULER_ERROR_INVALID_ARG;
    }

    for (uint32_t i = 0U; i < BOARD_SCHEDULER_MAX_TASKS; i++) {
        if (!g_tasks[i].used) {
            g_tasks[i].name = name;
            g_tasks[i].callback = callback;
            g_tasks[i].period_ms = period_ms;
            g_tasks[i].next_run_ms = Time_Millis() + start_delay_ms;
            g_tasks[i].enabled = true;
            g_tasks[i].used = true;

            if (out_id != 0) {
                *out_id = (SchedulerTaskId) i;
            }

            return SCHEDULER_OK;
        }
    }

    return SCHEDULER_ERROR_FULL;
}

SchedulerStatus Scheduler_EnableTask(SchedulerTaskId id, bool enabled)
{
    if ((id >= BOARD_SCHEDULER_MAX_TASKS) || (!g_tasks[id].used)) {
        return SCHEDULER_ERROR_INVALID_ID;
    }

    g_tasks[id].enabled = enabled;
    g_tasks[id].next_run_ms = Time_Millis() + g_tasks[id].period_ms;
    return SCHEDULER_OK;
}

SchedulerStatus Scheduler_GetTaskInfo(SchedulerTaskId id,
                                      SchedulerTaskInfo *out_info)
{
    if ((out_info == 0) ||
        (id >= BOARD_SCHEDULER_MAX_TASKS) ||
        (!g_tasks[id].used)) {
        return SCHEDULER_ERROR_INVALID_ID;
    }

    out_info->name = g_tasks[id].name;
    out_info->period_ms = g_tasks[id].period_ms;
    out_info->next_run_ms = g_tasks[id].next_run_ms;
    out_info->enabled = g_tasks[id].enabled;

    return SCHEDULER_OK;
}

void Scheduler_Run(void)
{
    const uint32_t now = Time_Millis();

    for (uint32_t i = 0U; i < BOARD_SCHEDULER_MAX_TASKS; i++) {
        if (!g_tasks[i].used || !g_tasks[i].enabled) {
            continue;
        }

        if (IsDue(now, g_tasks[i].next_run_ms)) {
            g_tasks[i].next_run_ms += g_tasks[i].period_ms;
            g_tasks[i].callback();
        }
    }
}

} /* namespace services */