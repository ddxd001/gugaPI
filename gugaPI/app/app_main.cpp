#include "app/app_main.h"

#include "board/board_buzzer.h"
#include "board/board_led.h"
#include "config/feature_config.h"
#include "services/fault.h"
#include "services/scheduler.h"
#include "services/time.h"

namespace {

void App_StatusLedTestTask(void)
{
#if FEATURE_ENABLE_STATUS_LED && FEATURE_ENABLE_LED_TEST
    if (board::Board_StatusLedIsReady()) {
        (void) board::Board_StatusLedToggle();
    }
#endif
}

void App_BuzzerTestOnTask(void)
{
#if FEATURE_ENABLE_BUZZER && FEATURE_ENABLE_BUZZER_TEST
    if (board::Board_BuzzerIsReady()) {
        (void) board::Board_BuzzerOn();
    }
#endif
}

void App_BuzzerTestOffTask(void)
{
#if FEATURE_ENABLE_BUZZER && FEATURE_ENABLE_BUZZER_TEST
    if (board::Board_BuzzerIsReady()) {
        (void) board::Board_BuzzerOff();
    }
#endif
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

#if FEATURE_ENABLE_STATUS_LED && FEATURE_ENABLE_LED_TEST
    if (services::Scheduler_AddTask("status_led_test",
                                    App_StatusLedTestTask,
                                    500U,
                                    500U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif

#if FEATURE_ENABLE_BUZZER && FEATURE_ENABLE_BUZZER_TEST
    if (services::Scheduler_AddTask("buzzer_test_on",
                                    App_BuzzerTestOnTask,
                                    1000U,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }

    if (services::Scheduler_AddTask("buzzer_test_off",
                                    App_BuzzerTestOffTask,
                                    1000U,
                                    200U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif
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