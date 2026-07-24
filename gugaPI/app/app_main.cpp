#include "app/app_main.h"

#include "app/app_grayscale.h"
#include "app/app_imu.h"
#include "app/app_shell.h"
#include "app/action.h"
#include "app/chassis.h"
#include "app/config_store.h"
#include "app/heading.h"
#include "app/linefollow.h"
#include "board/board_button.h"

#include "board/board_buzzer.h"
#include "board/board_led.h"
#include "config/feature_config.h"
#include "drivers/common/driver_status.h"
#include "services/debug_uart.h"
#include "services/fault.h"
#include "services/log.h"
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

#if FEATURE_ENABLE_MOTOR_DRIVER
const uint32_t CHASSIS_SERVICE_PERIOD_MS = 100U;
const uint32_t CHASSIS_FEEDBACK_PERIOD_MS = 20U;

static services::SchedulerTaskId g_chassisTaskId = 0U;
static bool g_chassisTaskRegistered = false;
static bool g_faultStopHandled = false;

void App_ChassisServiceTask(void)
{
    (void) app::Chassis_Service();
}

/* Periodic feedback: keep ChassisState.actual_rpm / encoder fresh at 20 ms so
 * any future heading/position controller reads current state without each
 * caller having to do its own on-demand I2C round-trip. Read-only telemetry,
 * left running during fault for diagnostics. */
void App_ChassisFeedbackTask(void)
{
    (void) app::Chassis_Update();
}
#endif

#if FEATURE_ENABLE_IMU && FEATURE_ENABLE_MOTOR_DRIVER
const uint32_t HEADING_PERIOD_MS = 50U;

void App_HeadingTask(void)
{
    app::Heading_Update();
}

const uint32_t ACTION_PERIOD_MS = 50U;

void App_ActionTask(void)
{
    app::ActionRunner_Update();
}
#endif

#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_MOTOR_DRIVER
const uint32_t LINEFOLLOW_PERIOD_MS = 50U;

void App_LineFollowTask(void)
{
    app::LF_Update();
}
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
const uint32_t BUTTON_OLED_SELECT_PERIOD_MS = 10U;
const uint32_t BUTTON_OLED_SELECT_DISPLAY_PERIOD_MS = 100U;
static bool g_buttonOledLastPressed[board::BOARD_BUTTON_COUNT] = {
    false,
    false,
    false
};

void App_ButtonScanTask(void)
{
    if (board::Board_ButtonsUpdate(services::Time_Millis()) !=
        drivers::DRIVER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
}

#if FEATURE_ENABLE_OLED
void App_ButtonOledSelectTask(void)
{
    bool pressed[board::BOARD_BUTTON_COUNT] = { false, false, false };
    for (uint32_t i = 0U; i < (uint32_t) board::BOARD_BUTTON_COUNT; i++) {
        const board::BoardButtonId id = (board::BoardButtonId) i;
        pressed[i] = board::Board_ButtonIsPressed(id);
    }

#if FEATURE_ENABLE_INA219
    /* Skip button 1 OLED in competition modes (button 1 is used for
     * competition start/stop by the competition status task). */
    if ((app::App_GetState()->mode != app::APP_MODE_COMPETITION_ARMED) &&
        (app::App_GetState()->mode != app::APP_MODE_COMPETITION_RUNNING) &&
        pressed[board::BOARD_BUTTON_1] &&
        (!g_buttonOledLastPressed[board::BOARD_BUTTON_1])) {
        const drivers::DriverStatus status = app::AppShell_EnableIna219Oled(
            BUTTON_OLED_SELECT_DISPLAY_PERIOD_MS);
#if FEATURE_ENABLE_BUTTON_OLED_LOG
        LOG_INFO(status == drivers::DRIVER_OK ?
                 "button1: ina219 oled on" :
                 "button1: ina219 oled failed");
#else
        (void) status;
#endif
    }
#endif

#if FEATURE_ENABLE_GY931
    if (pressed[board::BOARD_BUTTON_2] &&
        (!g_buttonOledLastPressed[board::BOARD_BUTTON_2])) {
        const drivers::DriverStatus status = app::AppShell_EnableGy931Oled(
            BUTTON_OLED_SELECT_DISPLAY_PERIOD_MS);
#if FEATURE_ENABLE_BUTTON_OLED_LOG
        LOG_INFO(status == drivers::DRIVER_OK ?
                 "button2: gy931 oled on" :
                 "button2: gy931 oled failed");
#else
        (void) status;
#endif
    }
#endif

#if FEATURE_ENABLE_GRAYSCALE
    if (pressed[board::BOARD_BUTTON_3] &&
        (!g_buttonOledLastPressed[board::BOARD_BUTTON_3])) {
        const drivers::DriverStatus status = app::AppShell_EnableGrayOled(
            BUTTON_OLED_SELECT_DISPLAY_PERIOD_MS);
#if FEATURE_ENABLE_BUTTON_OLED_LOG
        LOG_INFO(status == drivers::DRIVER_OK ?
                 "button3: gray oled on" :
                 "button3: gray oled failed");
#else
        (void) status;
#endif
    }
#endif

    for (uint32_t i = 0U; i < (uint32_t) board::BOARD_BUTTON_COUNT; i++) {
        g_buttonOledLastPressed[i] = pressed[i];
    }
}
#endif
#endif

#if FEATURE_ENABLE_BUTTONS && FEATURE_ENABLE_MOTOR_DRIVER && \
    FEATURE_ENABLE_BUTTON_CHASSIS_TEST
const uint32_t BUTTON_CHASSIS_TEST_PERIOD_MS = 10U;
const uint32_t BUTTON_CHASSIS_TEST_RUN_MS = 1000U;
const int32_t BUTTON_CHASSIS_TEST_RPM = 5;

static bool g_buttonChassisTestActive = false;
static uint32_t g_buttonChassisTestStartMs = 0U;

void App_ButtonChassisTestTask(void)
{
    const uint32_t now = services::Time_Millis();

    if ((!g_buttonChassisTestActive) &&
        board::Board_ButtonWasPressed(board::BOARD_BUTTON_1)) {
        if (app::Chassis_SetWheelRpm(BUTTON_CHASSIS_TEST_RPM,
                                     BUTTON_CHASSIS_TEST_RPM) ==
            drivers::DRIVER_OK) {
            g_buttonChassisTestActive = true;
            g_buttonChassisTestStartMs = now;
        }
    }

    if (g_buttonChassisTestActive &&
        services::Time_HasElapsed(g_buttonChassisTestStartMs,
                                  BUTTON_CHASSIS_TEST_RUN_MS)) {
        (void) app::Chassis_Stop();
        g_buttonChassisTestActive = false;
    }
}
#endif

#if FEATURE_ENABLE_STATUS_LED || FEATURE_ENABLE_BUZZER
const uint32_t COMP_STATUS_PERIOD_MS = 10U;

/* Competition status task: button 1 start/stop + LED/buzzer indication.
 * LED: ARMED = slow blink (1 Hz), RUNNING = solid on, FAULT = fast blink (5 Hz).
 * Buzzer: FAULT only. Button 1 toggles competition in ARMED/RUNNING modes. */
void App_CompetitionStatusTask(void)
{
    const uint32_t now = services::Time_Millis();
    const app::AppMode mode = app::App_GetState()->mode;

    /* Button 1: start/stop competition (edge-triggered, only in comp modes) */
    if ((mode == app::APP_MODE_COMPETITION_ARMED) ||
        (mode == app::APP_MODE_COMPETITION_RUNNING)) {
        if (board::Board_ButtonWasPressed(board::BOARD_BUTTON_1)) {
            if (mode == app::APP_MODE_COMPETITION_ARMED) {
                (void) app::App_CompetitionStart();
            } else {
                (void) app::App_CompetitionStop();
            }
        }
    }

    /* LED status indication */
#if FEATURE_ENABLE_STATUS_LED
    if (!board::Board_StatusLedIsReady()) {
        /* skip if LED not initialized */
    } else if (mode == app::APP_MODE_FAULT) {
        if (((now / 100U) & 1U) == 0U) {
            (void) board::Board_StatusLedOn();
        } else {
            (void) board::Board_StatusLedOff();
        }
    } else if (mode == app::APP_MODE_COMPETITION_ARMED) {
        if (((now / 500U) & 1U) == 0U) {
            (void) board::Board_StatusLedOn();
        } else {
            (void) board::Board_StatusLedOff();
        }
    } else if (mode == app::APP_MODE_COMPETITION_RUNNING) {
        (void) board::Board_StatusLedOn();
    }
#endif

    /* Buzzer: fault alert only */
#if FEATURE_ENABLE_BUZZER
    if (board::Board_BuzzerIsReady()) {
        if (mode == app::APP_MODE_FAULT) {
            (void) board::Board_BuzzerOn();
        } else {
            (void) board::Board_BuzzerOff();
        }
    }
#endif
}
#endif
} /* namespace */

namespace app {

static AppState g_appState = {
#if FEATURE_PROFILE_COMPETITION
    APP_MODE_COMPETITION_ARMED,
#else
    APP_MODE_RUNNING,
#endif
    0U
};

void App_Init(void)
{
#if FEATURE_PROFILE_COMPETITION
    g_appState.mode = APP_MODE_COMPETITION_ARMED;
#else
    g_appState.mode = APP_MODE_RUNNING;
#endif
    g_appState.uptime_ms = 0U;
#if FEATURE_ENABLE_MOTOR_DRIVER
    (void) ConfigStore_Load();
    (void) Chassis_Init();
    if (services::Scheduler_AddTask("chassis",
                                    App_ChassisServiceTask,
                                    CHASSIS_SERVICE_PERIOD_MS,
                                    0U,
                                    &g_chassisTaskId) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    } else {
        g_chassisTaskRegistered = true;
    }
    if (services::Scheduler_AddTask("chassis_fb",
                                    App_ChassisFeedbackTask,
                                    CHASSIS_FEEDBACK_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif
#if FEATURE_ENABLE_IMU && FEATURE_ENABLE_MOTOR_DRIVER
    Heading_Init();
    if (services::Scheduler_AddTask("heading",
                                    App_HeadingTask,
                                    HEADING_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
    ActionRunner_Init();
    if (services::Scheduler_AddTask("action",
                                    App_ActionTask,
                                    ACTION_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif
#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_MOTOR_DRIVER
    app::LF_Init();
    if (services::Scheduler_AddTask("linefollow",
                                    App_LineFollowTask,
                                    LINEFOLLOW_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif
#if FEATURE_ENABLE_IMU
    App_ImuInit();
    if (services::Scheduler_AddTask("imu",
                                    App_ImuUpdate,
                                    10U,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif
#if FEATURE_ENABLE_GRAYSCALE
    App_GrayscaleInit();
    if (services::Scheduler_AddTask("grayscale",
                                    App_GrayscaleUpdate,
                                    10U,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif
#if FEATURE_ENABLE_BUTTONS
    if (services::Scheduler_AddTask("buttons",
                                    App_ButtonScanTask,
                                    BUTTON_SCAN_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif
#if FEATURE_ENABLE_BUTTONS && FEATURE_ENABLE_OLED
    if (services::Scheduler_AddTask("button_oled",
                                    App_ButtonOledSelectTask,
                                    BUTTON_OLED_SELECT_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif
#if FEATURE_ENABLE_BUTTONS && FEATURE_ENABLE_MOTOR_DRIVER && \
    FEATURE_ENABLE_BUTTON_CHASSIS_TEST
    if (services::Scheduler_AddTask("button_chassis",
                                    App_ButtonChassisTestTask,
                                    BUTTON_CHASSIS_TEST_PERIOD_MS,
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

#if FEATURE_ENABLE_STATUS_LED || FEATURE_ENABLE_BUZZER
    if (services::Scheduler_AddTask("comp_status",
                                    App_CompetitionStatusTask,
                                    COMP_STATUS_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif
}

void App_Run(void)
{
    g_appState.uptime_ms = services::Time_Millis();

    if (services::Fault_HasFault()) {
        g_appState.mode = APP_MODE_FAULT;
#if FEATURE_ENABLE_MOTOR_DRIVER
        if (!g_faultStopHandled) {
            g_faultStopHandled = true;
#if FEATURE_ENABLE_IMU && FEATURE_ENABLE_MOTOR_DRIVER
            (void) Heading_Stop();
#endif
#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_MOTOR_DRIVER
            (void) LF_Stop();
#endif
            (void) Chassis_Stop();
            if (g_chassisTaskRegistered) {
                (void) services::Scheduler_EnableTask(g_chassisTaskId, false);
            }
        }
#endif
        return;
    }

    if (g_faultStopHandled) {
        g_faultStopHandled = false;
    }

#if FEATURE_ENABLE_MOTOR_DRIVER
    switch (g_appState.mode) {
    case APP_MODE_COMPETITION_ARMED:
        if (g_chassisTaskRegistered) {
            (void) services::Scheduler_EnableTask(g_chassisTaskId, false);
        }
        break;

    case APP_MODE_COMPETITION_RUNNING:
        if (g_chassisTaskRegistered) {
            (void) services::Scheduler_EnableTask(g_chassisTaskId, true);
        }
        if (!ActionRunner_GetState()->running) {
            g_appState.mode = APP_MODE_COMPETITION_ARMED;
            (void) Chassis_Stop();
        }
        break;

    default:
        g_appState.mode = APP_MODE_RUNNING;
        if (g_chassisTaskRegistered) {
            (void) services::Scheduler_EnableTask(g_chassisTaskId, true);
        }
        break;
    }
#else
    g_appState.mode = APP_MODE_RUNNING;
#endif
}

const AppState *App_GetState(void)
{
    return &g_appState;
}

drivers::DriverStatus App_CompetitionArm(void)
{
    if (g_appState.mode != APP_MODE_RUNNING) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (services::Fault_HasFault()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
#if FEATURE_ENABLE_MOTOR_DRIVER
    (void) Chassis_Stop();
#endif
    g_appState.mode = APP_MODE_COMPETITION_ARMED;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus App_CompetitionStart(void)
{
    if (g_appState.mode != APP_MODE_COMPETITION_ARMED) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (services::Fault_HasFault()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (ActionRunner_GetState()->count == 0U) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    const drivers::DriverStatus status = ActionRunner_Start();
    if (status == drivers::DRIVER_OK) {
        g_appState.mode = APP_MODE_COMPETITION_RUNNING;
    }
    return status;
}

drivers::DriverStatus App_CompetitionStop(void)
{
    if (g_appState.mode != APP_MODE_COMPETITION_RUNNING) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    (void) ActionRunner_Cancel();
    (void) Chassis_Stop();
    g_appState.mode = APP_MODE_COMPETITION_ARMED;
    return drivers::DRIVER_OK;
}

} /* namespace app */
