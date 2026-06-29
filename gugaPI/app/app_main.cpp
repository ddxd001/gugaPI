#include "app/app_main.h"

#include "services/fault.h"
#include "services/scheduler.h"
#include "services/time.h"

namespace {

void App_HeartbeatTask(void)
{
    /*
     * Add board LED heartbeat or periodic health checks here after the
     * corresponding driver is available.
     */
}

} /* namespace */

namespace app {

static AppState g_appState = {
    APP_MODE_IDLE,
    0U
};

void App_Init(void)
{
    g_appState.mode = APP_MODE_RUNNING;
    g_appState.uptime_ms = 0U;

    if (services::Scheduler_AddTask("heartbeat",
                                    App_HeartbeatTask,
                                    1000U,
                                    1000U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
}

void App_Run(void)
{
    g_appState.uptime_ms = services::Time_Millis();

    if (services::Fault_Get() != services::FAULT_NONE) {
        g_appState.mode = APP_MODE_FAULT;
    }
}

const AppState *App_GetState(void)
{
    return &g_appState;
}

} /* namespace app */