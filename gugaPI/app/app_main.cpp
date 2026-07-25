#include "app/app_main.h"

#include "app/app_grayscale.h"
#include "app/app_imu.h"
#include "app/app_ina219.h"
#include "app/app_lora.h"
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
    if (pressed[board::BOARD_BUTTON_1] &&
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
        /* Stop the chassis once on transition into fault and suppress the
         * motion/lease task so a faulted system cannot keep driving. The
         * MotorDriver watchdog also times out (lease no longer refreshed)
         * and coasts both wheels as a backstop. */
        if (!g_faultStopHandled) {
            g_faultStopHandled = true;
            (void) Chassis_Stop();
            if (g_chassisTaskRegistered) {
                (void) services::Scheduler_EnableTask(g_chassisTaskId, false);
            }
        }
#endif
    } else {
        g_appState.mode = APP_MODE_RUNNING;
#if FEATURE_ENABLE_MOTOR_DRIVER
        if (g_faultStopHandled) {
            g_faultStopHandled = false;
            if (g_chassisTaskRegistered) {
                (void) services::Scheduler_EnableTask(g_chassisTaskId, true);
            }
        }
#endif
    }
}

const AppState *App_GetState(void)
{
    return &g_appState;
}

} /* namespace app */
