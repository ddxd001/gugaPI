#include "app/app_main.h"

#include "app/app_grayscale.h"
#include "app/app_imu.h"
#include "app/app_lora.h"
#include "app/action.h"
#include "app/chassis.h"
#include "app/config_store.h"
#include "app/heading.h"
#include "app/linefollow.h"
#include "board/board.h"
#include "board/board_button.h"

#include "board/board_buzzer.h"
#include "board/board_led.h"
#include "board/board_oled.h"
#include "config/feature_config.h"
#include "drivers/common/driver_status.h"
#include "services/debug_uart.h"
#include "services/fault.h"
#include "services/log.h"
#include "services/scheduler.h"
#include "services/shell.h"
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
static bool g_chassisTaskEnabled = false;
static bool g_faultStopHandled = false;

void SetChassisTaskEnabled(bool enabled)
{
    if ((!g_chassisTaskRegistered) ||
        (g_chassisTaskEnabled == enabled)) {
        return;
    }

    if (services::Scheduler_EnableTask(g_chassisTaskId, enabled) !=
        services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
        return;
    }

    g_chassisTaskEnabled = enabled;
}

void App_ChassisServiceTask(void)
{
    const drivers::DriverStatus status = app::Chassis_Service();
    if ((status != drivers::DRIVER_OK) &&
        (status != drivers::DRIVER_ERROR_BUSY)) {
        services::Fault_Set(services::FAULT_DRIVER_TIMEOUT);
    }
}

/* Periodic feedback: keep ChassisState.actual_rpm / encoder fresh at 20 ms so
 * any future heading/position controller reads current state without each
 * caller having to do its own on-demand I2C round-trip. Read-only telemetry,
 * left running during fault for diagnostics. */
void App_ChassisFeedbackTask(void)
{
    const drivers::DriverStatus status = app::Chassis_Update();
    if (status != drivers::DRIVER_OK) {
        services::Fault_Set(services::FAULT_DRIVER_TIMEOUT);
    }
}
#endif

#if FEATURE_ENABLE_INA219 && FEATURE_ENABLE_MOTOR_DRIVER
static bool g_powerInhibitHandled = false;
#endif

#if FEATURE_ENABLE_GRAYSCALE
/* One mux channel per invocation: 2 ms gives an approximately 16 ms frame
 * while retaining margin over the 200 us mux settle + 125 us ADC sample. */
const uint32_t GRAYSCALE_PERIOD_MS = 2U;
#endif

#if FEATURE_ENABLE_IMU
/* Match the ICM45686 200 Hz output data rate. The 1 MHz SPI burst takes
 * roughly 120 us on the wire, so a 5 ms task keeps ample scheduler margin. */
const uint32_t IMU_PERIOD_MS = 5U;
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
const uint32_t LINEFOLLOW_PERIOD_MS = 20U;

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

#if FEATURE_ENABLE_BUTTON_EVENT_LOG
void WriteButtonEventName(const char *name, bool *first)
{
    if (!(*first)) {
        services::Shell_WriteString(",");
    }
    services::Shell_WriteString(name);
    *first = false;
}

bool WriteButtonGeneratedEvents(board::BoardButtonId id, uint32_t events)
{
    if (events == drivers::BUTTON_EVENT_NONE) {
        return false;
    }

    /* Start on a fresh line because the interactive prompt may already be
     * visible.  Writes are queued by DebugUart and do not block on TX. */
    services::Shell_WriteString("\r\nbutton event ");
    services::Shell_WriteString(board::Board_ButtonGetName(id));
    services::Shell_WriteString(" types=");

    bool first = true;
    if ((events & drivers::BUTTON_EVENT_PRESSED) != 0U) {
        WriteButtonEventName("pressed", &first);
    }
    if ((events & drivers::BUTTON_EVENT_RELEASED) != 0U) {
        WriteButtonEventName("released", &first);
    }
    if ((events & drivers::BUTTON_EVENT_SHORT_PRESSED) != 0U) {
        WriteButtonEventName("short", &first);
    }
    if ((events & drivers::BUTTON_EVENT_LONG_PRESSED) != 0U) {
        WriteButtonEventName("long", &first);
    }

    services::Shell_WriteString(" held_ms=");
    services::Shell_WriteUInt32(board::Board_ButtonGetPressDurationMs(id));
    services::Shell_WriteString("\r\n");
    return true;
}

void WriteAllButtonGeneratedEvents(void)
{
    bool wrote_event = false;
    for (uint32_t i = 0U; i < (uint32_t) board::BOARD_BUTTON_COUNT; i++) {
        const board::BoardButtonId id = (board::BoardButtonId) i;
        if (WriteButtonGeneratedEvents(
                id,
                board::Board_ButtonGetGeneratedEvents(id))) {
            wrote_event = true;
        }
    }
    if (wrote_event) {
        services::Shell_PrintPrompt();
    }
}
#endif

void App_ButtonScanTask(void)
{
    if (board::Board_ButtonsUpdate(services::Time_Millis()) !=
        drivers::DRIVER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
        return;
    }

#if FEATURE_ENABLE_BUTTON_EVENT_LOG
    WriteAllButtonGeneratedEvents();
#endif
}
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

#if FEATURE_ENABLE_STATUS_LED || FEATURE_ENABLE_BUZZER || FEATURE_ENABLE_OLED
const uint32_t COMP_STATUS_PERIOD_MS = 10U;
#if FEATURE_ENABLE_OLED
const uint32_t FAULT_OLED_REFRESH_PERIOD_MS = 500U;
static services::FaultCode g_faultOledCode = services::FAULT_NONE;
static uint32_t g_faultOledLastAttemptMs = 0U;

const char *FaultCodeText(services::FaultCode code)
{
    switch (code) {
    case services::FAULT_ASSERT:
        return "ASSERT";
    case services::FAULT_DRIVER_INIT:
        return "DRIVER INIT";
    case services::FAULT_DRIVER_TIMEOUT:
        return "DRIVER TIMEOUT";
    case services::FAULT_SENSOR_LOST:
        return "SENSOR LOST";
    case services::FAULT_UART_OVERFLOW:
        return "UART OVERFLOW";
    case services::FAULT_UNKNOWN:
        return "UNKNOWN";
    case services::FAULT_NONE:
    default:
        return "NONE";
    }
}

void App_FaultOledUpdate(uint32_t now)
{
    const services::FaultCode code = services::Fault_Get();
    const bool code_changed = (code != g_faultOledCode);
    if ((!code_changed) &&
        (!services::Time_HasElapsed(g_faultOledLastAttemptMs,
                                    FAULT_OLED_REFRESH_PERIOD_MS))) {
        return;
    }

    g_faultOledCode = code;
    g_faultOledLastAttemptMs = now;
    if (!board::Board_OledIsReady()) {
        return;
    }

    drivers::DriverStatus status = board::Board_OledClear();
    if (status == drivers::DRIVER_OK) {
        status = board::Board_OledWriteText(0U, 0U, "SYSTEM FAULT");
    }
    if (status == drivers::DRIVER_OK) {
        status = board::Board_OledWriteText(1U, 0U, FaultCodeText(code));
    }
    if (status == drivers::DRIVER_OK) {
        if (code == services::FAULT_DRIVER_INIT) {
            status = board::Board_OledWriteText(2U, 0U, "FAILED:");
            const char *failed_driver = board::Board_GetFailedDriver();
            if ((status == drivers::DRIVER_OK) && (failed_driver != 0)) {
                status = board::Board_OledWriteText(
                    2U,
                    8U,
                    failed_driver);
            }
        } else {
            status = board::Board_OledWriteText(
                2U,
                0U,
                "MOTORS STOPPED");
        }
    }
    if (status == drivers::DRIVER_OK) {
        (void) board::Board_OledWriteText(3U, 0U, "RESET TO RECOVER");
    }
}
#endif

/* Competition status task: button 1 start/stop + LED/OLED indication.
 * LED: ARMED = slow blink (1 Hz), RUNNING = solid on, FAULT = fast blink (5 Hz).
 * OLED: latched fault type. Buzzer remains silent in every mode.
 * Button 1 toggles competition in ARMED/RUNNING modes. */
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

    /* The buzzer is intentionally not used for fault reporting. */
#if FEATURE_ENABLE_BUZZER
    if (board::Board_BuzzerIsReady()) {
        (void) board::Board_BuzzerOff();
    }
#endif

#if FEATURE_ENABLE_OLED
    if (mode == app::APP_MODE_FAULT) {
        App_FaultOledUpdate(now);
    } else {
        g_faultOledCode = services::FAULT_NONE;
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
    /* Immediately silence buzzer and LED before any tasks run, to prevent
     * brief alarm from default GPIO state on power-up. */
#if FEATURE_ENABLE_STATUS_LED
    if (board::Board_StatusLedIsReady()) {
        (void) board::Board_StatusLedOff();
    }
#endif
#if FEATURE_ENABLE_BUZZER
    if (board::Board_BuzzerIsReady()) {
        (void) board::Board_BuzzerOff();
    }
#endif

#if FEATURE_PROFILE_COMPETITION
    g_appState.mode = APP_MODE_COMPETITION_ARMED;
#else
    g_appState.mode = APP_MODE_RUNNING;
#endif
    g_appState.uptime_ms = 0U;
#if FEATURE_ENABLE_MOTOR_DRIVER || FEATURE_ENABLE_INA219 || \
    FEATURE_ENABLE_GRAYSCALE
    const drivers::DriverStatus config_status = ConfigStore_Load();
    const ConfigStoreStatus *config_store_status = ConfigStore_GetStatus();
    if ((config_status != drivers::DRIVER_OK) &&
        (config_store_status != 0)) {
        if (config_store_status->load_outcome ==
            CONFIG_LOAD_DEFAULTS_IO_ERROR) {
            LOG_WARN("config load I/O failed; defaults active");
        } else {
            LOG_WARN("config invalid or unavailable; defaults active");
        }
    }
#endif

#if FEATURE_ENABLE_INA219
    if (App_Ina219Init() != drivers::DRIVER_OK) {
        LOG_ERROR("INA219 protection init failed");
        services::Fault_Set(services::FAULT_DRIVER_INIT);
    }
#if FEATURE_ENABLE_MOTOR_DRIVER
    g_powerInhibitHandled = false;
#endif
#endif

#if FEATURE_ENABLE_LORA
    App_LoraProtocolInit();
#endif

#if FEATURE_ENABLE_MOTOR_DRIVER
    const drivers::DriverStatus chassis_status = Chassis_Init();
    if (chassis_status != drivers::DRIVER_OK) {
        LOG_ERROR("chassis init failed; motion inhibited");
        services::Fault_Set(services::FAULT_DRIVER_INIT);
    }
    if (services::Scheduler_AddTask("chassis",
                                    App_ChassisServiceTask,
                                    CHASSIS_SERVICE_PERIOD_MS,
                                    0U,
                                    &g_chassisTaskId) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    } else {
        g_chassisTaskRegistered = true;
        /* Scheduler_AddTask creates tasks in the enabled state. */
        g_chassisTaskEnabled = true;
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
                                    IMU_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif
#if FEATURE_ENABLE_GRAYSCALE
    App_GrayscaleInit();
    if (services::Scheduler_AddTask("grayscale",
                                    App_GrayscaleUpdate,
                                    GRAYSCALE_PERIOD_MS,
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

#if FEATURE_ENABLE_STATUS_LED || FEATURE_ENABLE_BUZZER || FEATURE_ENABLE_OLED
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

#if FEATURE_ENABLE_INA219
    App_Ina219Run();
#if FEATURE_ENABLE_MOTOR_DRIVER
    if (App_Ina219MotionInhibitRequested()) {
        if (!g_powerInhibitHandled) {
            g_powerInhibitHandled = true;
            (void) Chassis_Stop();
        }
    } else {
        g_powerInhibitHandled = false;
    }
#endif
#endif

#if FEATURE_ENABLE_LORA
    App_LoraProtocolRun();
#endif

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
            SetChassisTaskEnabled(false);
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
        SetChassisTaskEnabled(false);
        break;

    case APP_MODE_COMPETITION_RUNNING:
        SetChassisTaskEnabled(true);
        if (!ActionRunner_GetState()->running) {
            g_appState.mode = APP_MODE_COMPETITION_ARMED;
            (void) Chassis_Stop();
        }
        break;

    default:
        g_appState.mode = APP_MODE_RUNNING;
        SetChassisTaskEnabled(true);
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
