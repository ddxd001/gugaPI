#include "services/scheduler.h"

#include "board/board_config.h"
#include "services/time.h"

namespace services {

struct SchedulerTaskSlot {
    const char *name;
    SchedulerTaskCallback callback;
    uint32_t period_ms;
    uint32_t next_run_ms;
    uint32_t run_count;
    uint32_t last_runtime_us;
    uint32_t max_runtime_us;
    uint32_t last_late_ms;
    uint32_t max_late_ms;
    uint32_t timeout_count;
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
        g_tasks[i].run_count = 0U;
        g_tasks[i].last_runtime_us = 0U;
        g_tasks[i].max_runtime_us = 0U;
        g_tasks[i].last_late_ms = 0U;
        g_tasks[i].max_late_ms = 0U;
        g_tasks[i].timeout_count = 0U;
        g_tasks[i].enabled = false;
        g_tasks[i].used = false;
    }
}

void ResetTaskStats(SchedulerTaskSlot *task)
{
    if (task == 0) {
        return;
    }

    task->run_count = 0U;
    task->last_runtime_us = 0U;
    task->max_runtime_us = 0U;
    task->last_late_ms = 0U;
    task->max_late_ms = 0U;
    task->timeout_count = 0U;
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
            ResetTaskStats(&g_tasks[i]);
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
    out_info->run_count = g_tasks[id].run_count;
    out_info->last_runtime_us = g_tasks[id].last_runtime_us;
    out_info->max_runtime_us = g_tasks[id].max_runtime_us;
    out_info->last_late_ms = g_tasks[id].last_late_ms;
    out_info->max_late_ms = g_tasks[id].max_late_ms;
    out_info->timeout_count = g_tasks[id].timeout_count;
    out_info->enabled = g_tasks[id].enabled;

    return SCHEDULER_OK;
}

SchedulerStatus Scheduler_ResetTaskStats(SchedulerTaskId id)
{
    if ((id >= BOARD_SCHEDULER_MAX_TASKS) || (!g_tasks[id].used)) {
        return SCHEDULER_ERROR_INVALID_ID;
    }

    ResetTaskStats(&g_tasks[id]);
    return SCHEDULER_OK;
}

void Scheduler_ResetAllStats(void)
{
    for (uint32_t i = 0U; i < BOARD_SCHEDULER_MAX_TASKS; i++) {
        if (g_tasks[i].used) {
            ResetTaskStats(&g_tasks[i]);
        }
    }
}

void Scheduler_Run(void)
{
    for (uint32_t i = 0U; i < BOARD_SCHEDULER_MAX_TASKS; i++) {
        if (!g_tasks[i].used || !g_tasks[i].enabled) {
            continue;
        }

        const uint32_t now = Time_Millis();
        if (IsDue(now, g_tasks[i].next_run_ms)) {
            const uint32_t scheduled_ms = g_tasks[i].next_run_ms;
            const uint32_t late_ms = now - scheduled_ms;
            const uint32_t start_us = Time_Micros();
            g_tasks[i].next_run_ms += g_tasks[i].period_ms;
            g_tasks[i].callback();
            const uint32_t runtime_us = Time_Micros() - start_us;

            g_tasks[i].run_count++;
            g_tasks[i].last_runtime_us = runtime_us;
            if (runtime_us > g_tasks[i].max_runtime_us) {
                g_tasks[i].max_runtime_us = runtime_us;
            }

            g_tasks[i].last_late_ms = late_ms;
            if (late_ms > g_tasks[i].max_late_ms) {
                g_tasks[i].max_late_ms = late_ms;
            }

            if (runtime_us >
                (g_tasks[i].period_ms * 1000U)) {
                g_tasks[i].timeout_count++;
            }
        }
    }
}

} /* namespace services */
