#include "app/app_main.h"

#include "board/board_button.h"

#include "board/board_buzzer.h"
#include "board/board_led.h"
#include "config/feature_config.h"
#include "drivers/common/driver_status.h"
#include "services/debug_uart.h"
#include "services/fault.h"
#include "services/scheduler.h"
#include "services/time.h"

namespace {

#if (FEATURE_ENABLE_STATUS_LED && FEATURE_ENABLE_LED_TEST) || \
    (FEATURE_ENABLE_BUZZER && FEATURE_ENABLE_BUZZER_TEST)
const uint32_t TEST_TASK_PERIOD_MS = 10U;

enum OutputTestState {
    OUTPUT_TEST_START_ON = 0,
    OUTPUT_TEST_WAIT_ON,
    OUTPUT_TEST_START_OFF,
    OUTPUT_TEST_WAIT_OFF
};
#endif

#if FEATURE_ENABLE_STATUS_LED && FEATURE_ENABLE_LED_TEST
const uint32_t STATUS_LED_ON_TIME_MS = 500U;
const uint32_t STATUS_LED_OFF_TIME_MS = 500U;

static OutputTestState g_statusLedState = OUTPUT_TEST_START_ON;
static uint32_t g_statusLedStateStartMs = 0U;

void App_StatusLedTestTask(void)
{
    if (!board::Board_StatusLedIsReady()) {
        return;
    }

    const uint32_t now = services::Time_Millis();

    switch (g_statusLedState) {
    case OUTPUT_TEST_START_ON:
        (void) board::Board_StatusLedOn();
        g_statusLedStateStartMs = now;
        g_statusLedState = OUTPUT_TEST_WAIT_ON;
        break;

    case OUTPUT_TEST_WAIT_ON:
        if (services::Time_HasElapsed(g_statusLedStateStartMs,
                                      STATUS_LED_ON_TIME_MS)) {
            g_statusLedState = OUTPUT_TEST_START_OFF;
        }
        break;

    case OUTPUT_TEST_START_OFF:
        (void) board::Board_StatusLedOff();
        g_statusLedStateStartMs = now;
        g_statusLedState = OUTPUT_TEST_WAIT_OFF;
        break;

    case OUTPUT_TEST_WAIT_OFF:
        if (services::Time_HasElapsed(g_statusLedStateStartMs,
                                      STATUS_LED_OFF_TIME_MS)) {
            g_statusLedState = OUTPUT_TEST_START_ON;
        }
        break;

    default:
        g_statusLedState = OUTPUT_TEST_START_OFF;
        break;
    }
}
#endif

#if FEATURE_ENABLE_BUZZER && FEATURE_ENABLE_BUZZER_TEST
const uint32_t BUZZER_ON_TIME_MS = 200U;
const uint32_t BUZZER_OFF_TIME_MS = 800U;

static OutputTestState g_buzzerState = OUTPUT_TEST_START_ON;
static uint32_t g_buzzerStateStartMs = 0U;

void App_BuzzerTestTask(void)
{
    if (!board::Board_BuzzerIsReady()) {
        return;
    }

    const uint32_t now = services::Time_Millis();

    switch (g_buzzerState) {
    case OUTPUT_TEST_START_ON:
        (void) board::Board_BuzzerOn();
        g_buzzerStateStartMs = now;
        g_buzzerState = OUTPUT_TEST_WAIT_ON;
        break;

    case OUTPUT_TEST_WAIT_ON:
        if (services::Time_HasElapsed(g_buzzerStateStartMs,
                                      BUZZER_ON_TIME_MS)) {
            g_buzzerState = OUTPUT_TEST_START_OFF;
        }
        break;

    case OUTPUT_TEST_START_OFF:
        (void) board::Board_BuzzerOff();
        g_buzzerStateStartMs = now;
        g_buzzerState = OUTPUT_TEST_WAIT_OFF;
        break;

    case OUTPUT_TEST_WAIT_OFF:
        if (services::Time_HasElapsed(g_buzzerStateStartMs,
                                      BUZZER_OFF_TIME_MS)) {
            g_buzzerState = OUTPUT_TEST_START_ON;
        }
        break;

    default:
        g_buzzerState = OUTPUT_TEST_START_OFF;
        break;
    }
}
#endif

#if FEATURE_ENABLE_DEBUG_UART && FEATURE_ENABLE_UART_COUNTER_TEST
const uint32_t DEBUG_UART_COUNTER_PERIOD_MS = 1000U;
static uint32_t g_debugUartCounter = 0U;

void App_DebugUartCounterTask(void)
{
    if (!services::DebugUart_IsReady()) {
        return;
    }

    services::DebugUart_WriteLineUInt32(g_debugUartCounter);
    g_debugUartCounter++;
}
#endif

#if FEATURE_ENABLE_BUTTONS
const uint32_t BUTTON_SCAN_PERIOD_MS = 5U;

void App_ButtonScanTask(void)
{
    if (board::Board_ButtonsUpdate(services::Time_Millis()) !=
        drivers::DRIVER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
}
#endif
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
#if FEATURE_ENABLE_BUTTONS
    if (services::Scheduler_AddTask("buttons",
                                    App_ButtonScanTask,
                                    BUTTON_SCAN_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif


#if FEATURE_ENABLE_STATUS_LED && FEATURE_ENABLE_LED_TEST
    if (services::Scheduler_AddTask("status_led_test",
                                    App_StatusLedTestTask,
                                    TEST_TASK_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif

#if FEATURE_ENABLE_BUZZER && FEATURE_ENABLE_BUZZER_TEST
    if (services::Scheduler_AddTask("buzzer_test",
                                    App_BuzzerTestTask,
                                    TEST_TASK_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif

#if FEATURE_ENABLE_DEBUG_UART && FEATURE_ENABLE_UART_COUNTER_TEST
    if (services::Scheduler_AddTask("debug_uart_count",
                                    App_DebugUartCounterTask,
                                    DEBUG_UART_COUNTER_PERIOD_MS,
                                    0U,
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