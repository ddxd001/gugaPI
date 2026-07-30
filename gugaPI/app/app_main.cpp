#include "app/app_main.h"

#include "app/app_grayscale.h"
#include "app/app_imu.h"
#include "app/app_infrared_sensor.h"
#include "app/app_jyme02_can.h"
#include "app/app_large_timer.h"
#include "app/app_can_bus.h"
#include "app/app_lora.h"
#include "app/ball_vision.h"
#include "app/ball_balance.h"
#include "app/app_shell.h"
#include "app/action.h"
#include "app/chassis.h"
#include "app/config_store.h"
#include "app/dm_g6220_controller.h"
#include "app/heading.h"
#include "app/linefollow.h"
#include "app/line_sensor.h"
#include "app/mode_switch_chord.h"
#include "app/road_event_controller.h"
#include "app/seq_store.h"
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
const uint8_t CHASSIS_FEEDBACK_FAULT_THRESHOLD = 3U;

static services::SchedulerTaskId g_chassisTaskId = 0U;
static bool g_chassisTaskRegistered = false;
static bool g_chassisTaskEnabled = false;
static bool g_faultStopHandled = false;
static uint8_t g_chassisFeedbackFailureStreak = 0U;

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
    if (status == drivers::DRIVER_OK) {
        g_chassisFeedbackFailureStreak = 0U;
        return;
    }

    if (g_chassisFeedbackFailureStreak <
        CHASSIS_FEEDBACK_FAULT_THRESHOLD) {
        g_chassisFeedbackFailureStreak++;
        if (g_chassisFeedbackFailureStreak ==
            CHASSIS_FEEDBACK_FAULT_THRESHOLD) {
            services::Fault_Set(services::FAULT_DRIVER_TIMEOUT);
        }
    }
}
#endif

#if FEATURE_ENABLE_INA219 && FEATURE_ENABLE_MOTOR_DRIVER
static bool g_powerInhibitHandled = false;
#endif

#if FEATURE_ENABLE_GRAYSCALE
/* One mux channel per invocation: 1 ms gives an approximately 8 ms frame
 * while retaining margin over the 200 us mux settle + 125 us ADC sample. */
const uint32_t GRAYSCALE_PERIOD_MS = 1U;
#endif

#if FEATURE_ENABLE_IMU
/* Match the ICM45686 200 Hz output data rate. The 1 MHz SPI burst takes
 * roughly 120 us on the wire, so a 5 ms task keeps ample scheduler margin. */
const uint32_t IMU_PERIOD_MS = 5U;
#endif

#if FEATURE_ENABLE_IMU && FEATURE_ENABLE_MOTOR_DRIVER
/* Heading TURN needs lower command latency than the general action
 * sequencer. Consume the newest 200 Hz IMU sample every 10 ms without
 * changing the scheduler implementation. */
const uint32_t HEADING_PERIOD_MS = 10U;

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
/* The real UART sensor publishes a complete frame in about 1.30 ms. Polling
 * the unified sequence at 2 ms keeps last-byte-to-wheel-command latency below
 * 5 ms without running PID or chassis code inside the UART ISR. Duplicate
 * sequence numbers remain no-ops for the slower 8-channel ADC source. */
const uint32_t LINEFOLLOW_PERIOD_MS = 2U;

void App_LineFollowTask(void)
{
#if FEATURE_ENABLE_IMU
    if (app::LineSensor_IsRoadCapable()) {
        app::RoadEventController_Update();
    }
#endif
    app::LF_Update();
}
#endif

#if FEATURE_ENABLE_INFRARED_LINE_SENSOR
void App_InfraredLineTask(void)
{
    app::App_InfraredSensorUpdate();
#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_MOTOR_DRIVER
    /* IR3 frames are already decoded in this foreground task. Let an active
     * line follower consume the newly published sequence immediately instead
     * of waiting for the independent 2 ms safety/control task. LF_Update()
     * ignores duplicate sequence numbers, so the regular task remains the
     * watchdog path without issuing a second wheel command. */
    const app::LFState *linefollow = app::LF_GetState();
    const app::AppInfraredSensorData *infrared =
        app::App_InfraredSensorGetData();
    if ((app::LineSensor_GetSource() == app::LINE_SENSOR_IR3) &&
        (linefollow->mode == app::LF_FOLLOW) &&
        (infrared->frame.sequence != linefollow->last_sequence)) {
        app::LF_Update();
    }
#endif
}
#endif

#if FEATURE_ENABLE_BALL_VISION
const uint32_t BALL_VISION_PERIOD_MS = 2U;

void App_BallVisionTask(void)
{
    app::BallVision_Update();
}
#endif

#if FEATURE_ENABLE_BALL_BALANCE
const uint32_t BALL_BALANCE_PERIOD_MS = 10U;

void App_BallBalanceTask(void)
{
    app::BallBalance_Update();
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

#if FEATURE_ENABLE_CAN
const uint32_t CAN_WATCH_PERIOD_MS = 10U;

void App_CanReceiveTask(void)
{
    app::AppCanBus_Update();
    app::AppShell_CanWatchUpdate();
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

#if FEATURE_ENABLE_BUTTONS || FEATURE_ENABLE_STATUS_LED || \
    FEATURE_ENABLE_BUZZER || FEATURE_ENABLE_OLED
const uint32_t COMP_STATUS_PERIOD_MS = 10U;
#if FEATURE_ENABLE_BUTTONS
const uint32_t MODE_SWITCH_CHORD_HOLD_MS = 1000U;
static app::ModeSwitchChordState g_modeSwitchChord = {
    false, false, 0U
};
#endif
#if FEATURE_ENABLE_OLED
const uint32_t FAULT_OLED_REFRESH_PERIOD_MS = 500U;
const uint32_t COMP_OLED_MIN_REFRESH_MS = 100U;
static services::FaultCode g_faultOledCode = services::FAULT_NONE;
static uint32_t g_faultOledLastAttemptMs = 0U;
static uint32_t g_compOledLastAttemptMs = 0U;
static app::AppMode g_compOledLastMode = app::APP_MODE_IDLE;
static uint8_t g_compOledLastSlot = 0xFFU;
static uint8_t g_compOledLastCount = 0xFFU;
static uint8_t g_compOledLastStep = 0xFFU;
static uint32_t g_compOledLastSecond = 0xFFFFFFFFU;
static app::CompetitionResult g_compOledLastResult =
    app::COMP_RESULT_LOAD_ERROR;
static bool g_compOledLastValid = false;
static bool g_compOledLastAnyValid = false;

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
    case services::FAULT_DM_TIMEOUT:
        return "DM TIMEOUT";
    case services::FAULT_DM_MOTOR:
        return "DM FAULT";
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

char *OledAppendText(char *cursor, const char *end, const char *text)
{
    while ((cursor < end) && (*text != '\0')) {
        *cursor++ = *text++;
    }
    *cursor = '\0';
    return cursor;
}

char *OledAppendUInt(char *cursor, const char *end, uint32_t value)
{
    char reverse[10];
    uint8_t count = 0U;
    do {
        reverse[count++] = static_cast<char>('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (count < sizeof(reverse)));

    while ((count > 0U) && (cursor < end)) {
        *cursor++ = reverse[--count];
    }
    *cursor = '\0';
    return cursor;
}

drivers::DriverStatus CompetitionOledWriteLine(
    uint8_t row,
    const char *prefix,
    uint32_t value,
    const char *suffix)
{
    char line[22];
    char *cursor = line;
    const char *end = &line[21];
    line[0] = '\0';
    cursor = OledAppendText(cursor, end, prefix);
    cursor = OledAppendUInt(cursor, end, value);
    (void) OledAppendText(cursor, end, suffix);
    return board::Board_OledWriteText(row, 0U, line);
}

const char *CompetitionResultText(app::CompetitionResult result)
{
    switch (result) {
    case app::COMP_RESULT_DONE:
        return "COMP DONE";
    case app::COMP_RESULT_FAILED:
        return "COMP FAILED";
    case app::COMP_RESULT_STOPPED:
        return "COMP STOPPED";
    case app::COMP_RESULT_LOAD_ERROR:
        return "COMP LOAD ERROR";
    case app::COMP_RESULT_NONE:
    default:
        return "COMP READY";
    }
}

drivers::DriverStatus CompetitionOledRender(
    app::AppMode mode,
    const app::CompetitionState *state,
    const app::ActionRunnerState *runner,
    uint32_t elapsed_seconds)
{
    drivers::DriverStatus status = board::Board_OledClear();
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    if (mode == app::APP_MODE_COMPETITION_RUNNING) {
        status = CompetitionOledWriteLine(
            0U, "RUN TASK ", state->selected_slot, "");
        if (status == drivers::DRIVER_OK) {
            const uint32_t step =
                (runner->current < runner->count) ?
                    static_cast<uint32_t>(runner->current) + 1U :
                    static_cast<uint32_t>(runner->count);
            char line[22];
            char *cursor = line;
            const char *end = &line[21];
            line[0] = '\0';
            cursor = OledAppendText(cursor, end, "STEP ");
            cursor = OledAppendUInt(cursor, end, step);
            cursor = OledAppendText(cursor, end, "/");
            (void) OledAppendUInt(cursor, end, runner->count);
            status = board::Board_OledWriteText(1U, 0U, line);
        }
        if (status == drivers::DRIVER_OK) {
            status = CompetitionOledWriteLine(
                2U, "TIME ", elapsed_seconds, "s");
        }
        if (status == drivers::DRIVER_OK) {
            status = board::Board_OledWriteText(3U, 0U, "B2 STOP");
        }
        return status;
    }

    if ((state->result == app::COMP_RESULT_DONE) ||
        (state->result == app::COMP_RESULT_FAILED) ||
        (state->result == app::COMP_RESULT_STOPPED)) {
        status = board::Board_OledWriteText(
            0U, 0U, CompetitionResultText(state->result));
        if (status == drivers::DRIVER_OK) {
            status = CompetitionOledWriteLine(
                1U, "TASK ", state->selected_slot, "");
        }
        if (status == drivers::DRIVER_OK) {
            const uint32_t step =
                (runner->current < runner->count) ?
                    static_cast<uint32_t>(runner->current) + 1U :
                    static_cast<uint32_t>(runner->count);
            char line[22];
            char *cursor = line;
            const char *end = &line[21];
            line[0] = '\0';
            cursor = OledAppendText(cursor, end, "STEP ");
            cursor = OledAppendUInt(cursor, end, step);
            cursor = OledAppendText(cursor, end, "/");
            (void) OledAppendUInt(cursor, end, runner->count);
            status = board::Board_OledWriteText(2U, 0U, line);
        }
        if (status == drivers::DRIVER_OK) {
            status = board::Board_OledWriteText(3U, 0U, "READY IN 2S");
        }
        return status;
    }

    const char *title = "COMP READY";
    if (state->result == app::COMP_RESULT_LOAD_ERROR) {
        title = CompetitionResultText(state->result);
    } else if (!state->any_valid_slot) {
        title = "COMP NO TASK";
    }
    status = board::Board_OledWriteText(0U, 0U, title);
    if (status == drivers::DRIVER_OK) {
        status = CompetitionOledWriteLine(
            1U, "TASK ", state->selected_slot, "");
    }
    if (status == drivers::DRIVER_OK) {
        if (state->result == app::COMP_RESULT_LOAD_ERROR) {
            status = CompetitionOledWriteLine(
                2U,
                "LOAD ERROR ",
                static_cast<uint32_t>(state->last_status),
                "");
        } else if (state->slot_valid) {
            status = CompetitionOledWriteLine(
                2U, "STEPS ", state->instruction_count, " READY");
        } else {
            status = board::Board_OledWriteText(2U, 0U, "EMPTY");
        }
    }
    if (status == drivers::DRIVER_OK) {
        status = board::Board_OledWriteText(
            3U, 0U, "<B1 B2 START B3>");
    }
    return status;
}

void App_CompetitionOledUpdate(uint32_t now, app::AppMode mode)
{
    if ((mode != app::APP_MODE_COMPETITION_ARMED) &&
        (mode != app::APP_MODE_COMPETITION_RUNNING)) {
        return;
    }
    if (!board::Board_OledIsReady()) {
        return;
    }
    if (!services::Time_HasElapsed(g_compOledLastAttemptMs,
                                   COMP_OLED_MIN_REFRESH_MS)) {
        return;
    }

    const app::CompetitionState *state = app::App_CompetitionGetState();
    const app::ActionRunnerState *runner = app::ActionRunner_GetState();
    const uint32_t elapsed_seconds =
        (mode == app::APP_MODE_COMPETITION_RUNNING) ?
            ((now - runner->seq_start_ms) / 1000U) : 0U;
    const uint8_t step =
        (mode == app::APP_MODE_COMPETITION_RUNNING) ?
            runner->current : 0U;

    const bool changed =
        (mode != g_compOledLastMode) ||
        (state->selected_slot != g_compOledLastSlot) ||
        (state->instruction_count != g_compOledLastCount) ||
        (step != g_compOledLastStep) ||
        (elapsed_seconds != g_compOledLastSecond) ||
        (state->result != g_compOledLastResult) ||
        (state->slot_valid != g_compOledLastValid) ||
        (state->any_valid_slot != g_compOledLastAnyValid);
    if (!changed) {
        return;
    }

    g_compOledLastAttemptMs = now;
    if (CompetitionOledRender(mode, state, runner, elapsed_seconds) !=
        drivers::DRIVER_OK) {
        return;
    }

    g_compOledLastMode = mode;
    g_compOledLastSlot = state->selected_slot;
    g_compOledLastCount = state->instruction_count;
    g_compOledLastStep = step;
    g_compOledLastSecond = elapsed_seconds;
    g_compOledLastResult = state->result;
    g_compOledLastValid = state->slot_valid;
    g_compOledLastAnyValid = state->any_valid_slot;
}
#endif

/* Competition status task: button selection/start/stop + LED/OLED indication.
 * LED: ARMED = slow blink (1 Hz), RUNNING = solid on, FAULT = fast blink (5 Hz).
 * OLED: competition selection/progress, with latched faults taking priority.
 * LED2/LED3 and buzzer are owned by ActionRunner while a sequence runs. */
void App_CompetitionStatusTask(void)
{
    const uint32_t now = services::Time_Millis();
    app::AppMode mode = app::App_GetState()->mode;

    /* B1+B3 is a system chord and takes priority while active. Otherwise,
     * ARMED uses B1/B3 for slot selection and B2 to start. RUNNING leaves
     * individual B1/B3 events available to ActionRunner. */
#if FEATURE_ENABLE_BUTTONS
    if (mode == app::APP_MODE_FAULT) {
        app::ModeSwitchChord_Reset(&g_modeSwitchChord);
    } else {
        const bool chord_was_active =
            app::ModeSwitchChord_IsActive(&g_modeSwitchChord);
        const app::ModeSwitchChordEvent chord_event =
            app::ModeSwitchChord_Update(
                &g_modeSwitchChord,
                board::Board_ButtonIsPressed(board::BOARD_BUTTON_1),
                board::Board_ButtonIsPressed(board::BOARD_BUTTON_3),
                now,
                MODE_SWITCH_CHORD_HOLD_MS);

        if (chord_was_active ||
            app::ModeSwitchChord_IsActive(&g_modeSwitchChord) ||
            (chord_event != app::MODE_SWITCH_CHORD_NONE)) {
            (void) board::Board_ButtonTakeEvents(
                board::BOARD_BUTTON_1, drivers::BUTTON_EVENT_ALL);
            (void) board::Board_ButtonTakeEvents(
                board::BOARD_BUTTON_3, drivers::BUTTON_EVENT_ALL);
        }

        if ((chord_event == app::MODE_SWITCH_CHORD_STARTED) &&
            ((mode == app::APP_MODE_COMPETITION_RUNNING) ||
             (mode == app::APP_MODE_RUNNING))) {
            (void) app::App_EmergencyStop();
            mode = app::App_GetState()->mode;
        }

        if (chord_event == app::MODE_SWITCH_CHORD_TRIGGERED) {
            mode = app::App_GetState()->mode;
            if (mode == app::APP_MODE_COMPETITION_ARMED) {
                (void) app::App_DebugModeEnter();
            } else if (mode == app::APP_MODE_RUNNING) {
                (void) app::App_CompetitionArm();
            }
            mode = app::App_GetState()->mode;
        }
    }

    if (mode == app::APP_MODE_COMPETITION_ARMED) {
        const uint32_t select_mask =
            drivers::BUTTON_EVENT_PRESSED |
            drivers::BUTTON_EVENT_SHORT_PRESSED;
        const uint32_t button1_events = board::Board_ButtonTakeEvents(
            board::BOARD_BUTTON_1, select_mask);
        const uint32_t button3_events = board::Board_ButtonTakeEvents(
            board::BOARD_BUTTON_3, select_mask);
        const uint32_t button2_events = board::Board_ButtonTakeEvents(
            board::BOARD_BUTTON_2, select_mask);

        const app::CompetitionState *state =
            app::App_CompetitionGetState();
        if ((button1_events & drivers::BUTTON_EVENT_SHORT_PRESSED) != 0U) {
            const uint8_t previous = static_cast<uint8_t>(
                (state->selected_slot + app::SEQ_SLOT_COUNT - 1U) %
                app::SEQ_SLOT_COUNT);
            (void) app::App_CompetitionSelect(previous);
        }
        if ((button3_events & drivers::BUTTON_EVENT_SHORT_PRESSED) != 0U) {
            state = app::App_CompetitionGetState();
            const uint8_t next = static_cast<uint8_t>(
                (state->selected_slot + 1U) % app::SEQ_SLOT_COUNT);
            (void) app::App_CompetitionSelect(next);
        }
        if ((button2_events & drivers::BUTTON_EVENT_SHORT_PRESSED) != 0U) {
            (void) app::App_CompetitionStart();
        }
    } else if (mode == app::APP_MODE_COMPETITION_RUNNING) {
        const uint32_t button2_events = board::Board_ButtonTakeEvents(
            board::BOARD_BUTTON_2,
            drivers::BUTTON_EVENT_PRESSED |
            drivers::BUTTON_EVENT_SHORT_PRESSED);
        if ((button2_events & drivers::BUTTON_EVENT_SHORT_PRESSED) != 0U) {
            (void) app::App_CompetitionStop();
        }
    }
#endif

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
    } else {
        (void) board::Board_StatusLedOff();
    }
#endif

#if FEATURE_ENABLE_OLED
    if (mode == app::APP_MODE_FAULT) {
        App_FaultOledUpdate(now);
    } else if (app::App_LargeTimerOwnsDisplay()) {
        /* The independently controlled large timer may be used while a
         * competition sequence is running. Invalidate the cached page so it
         * is redrawn immediately after the timer releases the OLED. */
        g_compOledLastSlot = 0xFFU;
    } else {
        g_faultOledCode = services::FAULT_NONE;
        App_CompetitionOledUpdate(now, mode);
    }
#endif
}
#endif
} /* namespace */

namespace app {

static AppState g_appState = {
    APP_MODE_COMPETITION_ARMED,
    0U
};

static CompetitionState g_competitionState = {
    0U,
    false,
    false,
    0U,
    COMP_RESULT_NONE,
    drivers::DRIVER_OK
};
static uint32_t g_competitionResultStartMs = 0U;
static const uint32_t kCompetitionResultHoldMs = 2000U;

static void CompetitionClearButtonEvents(void)
{
#if FEATURE_ENABLE_BUTTONS
    for (uint32_t index = 0U;
         index < static_cast<uint32_t>(board::BOARD_BUTTON_COUNT);
         index++) {
        (void) board::Board_ButtonTakeEvents(
            static_cast<board::BoardButtonId>(index),
            drivers::BUTTON_EVENT_ALL);
    }
#endif
}

static void CompetitionSetResult(CompetitionResult result,
                                 drivers::DriverStatus status)
{
    g_competitionState.result = result;
    g_competitionState.last_status = status;
    g_competitionResultStartMs = services::Time_Millis();
}

static drivers::DriverStatus CompetitionRefreshMetadata(void)
{
    SeqSlotInfo selected_info = { false, 0U };
    const drivers::DriverStatus selected_status = SeqStore_GetInfo(
        g_competitionState.selected_slot, &selected_info);
    if (selected_status == drivers::DRIVER_OK) {
        g_competitionState.slot_valid = selected_info.valid;
        g_competitionState.instruction_count = selected_info.count;
    } else {
        g_competitionState.slot_valid = false;
        g_competitionState.instruction_count = 0U;
    }

    g_competitionState.any_valid_slot = false;
    for (uint8_t slot = 0U; slot < SEQ_SLOT_COUNT; slot++) {
        SeqSlotInfo info = { false, 0U };
        const drivers::DriverStatus status = SeqStore_GetInfo(slot, &info);
        if (status != drivers::DRIVER_OK) {
            continue;
        }
        if (info.valid) {
            g_competitionState.any_valid_slot = true;
        }
    }

    if (selected_status != drivers::DRIVER_OK) {
        return selected_status;
    }
    return drivers::DRIVER_OK;
}

static void CompetitionSelectInitialSlot(void)
{
    uint8_t selected_slot = 0U;
    bool found_valid = false;
    drivers::DriverStatus scan_status = drivers::DRIVER_OK;

    for (uint8_t slot = 0U; slot < SEQ_SLOT_COUNT; slot++) {
        SeqSlotInfo info = { false, 0U };
        const drivers::DriverStatus status = SeqStore_GetInfo(slot, &info);
        if (status != drivers::DRIVER_OK) {
            scan_status = status;
            continue;
        }
        if (info.valid) {
            selected_slot = slot;
            found_valid = true;
            break;
        }
    }

    g_competitionState.selected_slot = selected_slot;
    const drivers::DriverStatus refresh_status =
        CompetitionRefreshMetadata();
    if ((!found_valid) &&
        ((scan_status != drivers::DRIVER_OK) ||
         (refresh_status != drivers::DRIVER_OK))) {
        CompetitionSetResult(
            COMP_RESULT_LOAD_ERROR,
            (refresh_status != drivers::DRIVER_OK) ?
                refresh_status : scan_status);
    } else {
        CompetitionSetResult(COMP_RESULT_NONE, drivers::DRIVER_OK);
    }
}

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
#if FEATURE_ENABLE_OLED
    App_LargeTimerInit();
#endif

    g_appState.mode = APP_MODE_COMPETITION_ARMED;
    g_appState.uptime_ms = 0U;
#if FEATURE_ENABLE_BUTTONS
    ModeSwitchChord_Reset(&g_modeSwitchChord);
#endif
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
    g_chassisFeedbackFailureStreak = 0U;
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
#endif
#if FEATURE_ENABLE_FRAM && FEATURE_ENABLE_IMU && FEATURE_ENABLE_MOTOR_DRIVER
    if (SeqStore_Init() != drivers::DRIVER_OK) {
        LOG_WARN("sequence store unavailable or migration incomplete");
    }
#endif
    CompetitionSelectInitialSlot();
    AppShell_DisableOledStreams();
#if FEATURE_ENABLE_INFRARED_LINE_SENSOR
    app::App_InfraredSensorInit();
    if (services::Scheduler_AddTask("infrared_line",
                                    App_InfraredLineTask,
                                    1U,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif
#if FEATURE_ENABLE_BALL_VISION
    app::BallVision_Init();
    if (services::Scheduler_AddTask("ball_vision",
                                    App_BallVisionTask,
                                    BALL_VISION_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif
#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_MOTOR_DRIVER
    app::LineSensor_Init();
    app::LF_Init();
#if FEATURE_ENABLE_IMU
    app::RoadEventController_Init();
#endif
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

#if FEATURE_ENABLE_CAN
    app::AppCanBus_Init();
    if (services::Scheduler_AddTask("can_rx",
                                    App_CanReceiveTask,
                                    CAN_WATCH_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif
#if FEATURE_ENABLE_BALL_BALANCE
    app::BallBalance_Init();
    if (services::Scheduler_AddTask("ball_balance",
                                    App_BallBalanceTask,
                                    BALL_BALANCE_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif

#if FEATURE_ENABLE_BUTTONS || FEATURE_ENABLE_STATUS_LED || \
    FEATURE_ENABLE_BUZZER || FEATURE_ENABLE_OLED
    if (services::Scheduler_AddTask("comp_status",
                                    App_CompetitionStatusTask,
                                    COMP_STATUS_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif
#if FEATURE_ENABLE_IMU && FEATURE_ENABLE_MOTOR_DRIVER
    /* Register after button/competition handling so the B1+B3 system chord
     * can consume its events before ActionRunner evaluates button sources. */
    if (services::Scheduler_AddTask("action",
                                    App_ActionTask,
                                    ACTION_PERIOD_MS,
                                    0U,
                                    0) != services::SCHEDULER_OK) {
        services::Fault_Set(services::FAULT_UNKNOWN);
    }
#endif

#if FEATURE_ENABLE_MOTOR_DRIVER
    (void) Chassis_Stop();
    SetChassisTaskEnabled(false);
#endif
}

void App_Run(void)
{
    g_appState.uptime_ms = services::Time_Millis();

#if FEATURE_ENABLE_OLED
    /* Progress the OLED DMA state machine before other shared-I2C clients. */
    (void) board::Board_OledService();
    /* The large timer only updates the framebuffer when a displayed tenth
     * changes. Scheduler_Run has already serviced every control task. */
    App_LargeTimerUpdate();
#endif

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
#if FEATURE_ENABLE_BALL_BALANCE
        BallBalance_EmergencyStop();
#endif
#if FEATURE_ENABLE_DM_G6220_CAN
        DmG6220Controller_EmergencyDisable();
#endif
#if FEATURE_ENABLE_MOTOR_DRIVER
        if (!g_faultStopHandled) {
            g_faultStopHandled = true;
            (void) ActionRunner_Cancel();
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

    if ((g_appState.mode == APP_MODE_COMPETITION_ARMED) &&
        ((g_competitionState.result == COMP_RESULT_DONE) ||
         (g_competitionState.result == COMP_RESULT_FAILED) ||
         (g_competitionState.result == COMP_RESULT_STOPPED)) &&
        services::Time_HasElapsed(g_competitionResultStartMs,
                                  kCompetitionResultHoldMs)) {
        CompetitionSetResult(COMP_RESULT_NONE, drivers::DRIVER_OK);
    }

#if FEATURE_ENABLE_MOTOR_DRIVER
    switch (g_appState.mode) {
    case APP_MODE_COMPETITION_ARMED:
        SetChassisTaskEnabled(false);
        break;

    case APP_MODE_COMPETITION_RUNNING:
        SetChassisTaskEnabled(true);
        if (!ActionRunner_GetState()->running) {
            const ActionRunnerState *runner = ActionRunner_GetState();
            CompetitionSetResult(
                runner->last_success ?
                    COMP_RESULT_DONE : COMP_RESULT_FAILED,
                runner->last_success ?
                    drivers::DRIVER_OK : drivers::DRIVER_ERROR);
            g_appState.mode = APP_MODE_COMPETITION_ARMED;
            CompetitionClearButtonEvents();
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

const CompetitionState *App_CompetitionGetState(void)
{
    return &g_competitionState;
}

drivers::DriverStatus App_CompetitionSelect(uint8_t slot)
{
    if (slot >= SEQ_SLOT_COUNT) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (g_appState.mode != APP_MODE_COMPETITION_ARMED) {
        return drivers::DRIVER_ERROR_BUSY;
    }

    g_competitionState.selected_slot = slot;
    const drivers::DriverStatus status = CompetitionRefreshMetadata();
    if (status != drivers::DRIVER_OK) {
        CompetitionSetResult(COMP_RESULT_LOAD_ERROR, status);
        return status;
    }

    CompetitionSetResult(COMP_RESULT_NONE, drivers::DRIVER_OK);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus App_CompetitionRefreshSelection(void)
{
    const drivers::DriverStatus status = CompetitionRefreshMetadata();
    if (status != drivers::DRIVER_OK) {
        CompetitionSetResult(COMP_RESULT_LOAD_ERROR, status);
        return status;
    }
    if (g_competitionState.result == COMP_RESULT_LOAD_ERROR) {
        CompetitionSetResult(COMP_RESULT_NONE, drivers::DRIVER_OK);
    }
    return drivers::DRIVER_OK;
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
    SetChassisTaskEnabled(false);
#endif
    (void) ActionRunner_Cancel();
#if FEATURE_ENABLE_BALL_BALANCE
    BallBalance_EmergencyStop();
#endif
#if FEATURE_ENABLE_DM_G6220_CAN
    DmG6220Controller_EmergencyDisable();
#endif
    CompetitionSelectInitialSlot();
    AppShell_DisableOledStreams();
    CompetitionClearButtonEvents();
    g_appState.mode = APP_MODE_COMPETITION_ARMED;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus App_DebugModeEnter(void)
{
    if (g_appState.mode != APP_MODE_COMPETITION_ARMED) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (services::Fault_HasFault()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    const drivers::DriverStatus stop_status = App_EmergencyStop();
    if (stop_status != drivers::DRIVER_OK) {
        return stop_status;
    }
    if (services::Fault_HasFault()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    CompetitionClearButtonEvents();
#if FEATURE_ENABLE_OLED
    (void) board::Board_OledClear();
    g_compOledLastMode = APP_MODE_IDLE;
#endif
#if FEATURE_ENABLE_STATUS_LED
    if (board::Board_StatusLedIsReady()) {
        (void) board::Board_StatusLedOff();
    }
#endif
#if FEATURE_ENABLE_MOTOR_DRIVER
    SetChassisTaskEnabled(true);
#endif
    g_appState.mode = APP_MODE_RUNNING;
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

    const drivers::DriverStatus refresh_status =
        CompetitionRefreshMetadata();
    if (refresh_status != drivers::DRIVER_OK) {
        CompetitionSetResult(COMP_RESULT_LOAD_ERROR, refresh_status);
        return refresh_status;
    }
    if (!g_competitionState.slot_valid) {
        g_competitionState.last_status =
            drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    const drivers::DriverStatus load_status = SeqStore_Load(
        g_competitionState.selected_slot);
    if (load_status != drivers::DRIVER_OK) {
        CompetitionSetResult(COMP_RESULT_LOAD_ERROR, load_status);
        return load_status;
    }

    ActionValidationResult validation;
    const drivers::DriverStatus validation_status =
        ActionRunner_ValidateCompetition(&validation);
    if (validation_status != drivers::DRIVER_OK) {
        CompetitionSetResult(COMP_RESULT_LOAD_ERROR, validation_status);
        return validation_status;
    }

    const drivers::DriverStatus status = ActionRunner_Start();
    if (status == drivers::DRIVER_OK) {
#if FEATURE_ENABLE_MOTOR_DRIVER
        SetChassisTaskEnabled(true);
#endif
        CompetitionSetResult(COMP_RESULT_NONE, drivers::DRIVER_OK);
        g_appState.mode = APP_MODE_COMPETITION_RUNNING;
    } else {
        g_competitionState.last_status = status;
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
    SetChassisTaskEnabled(false);
    CompetitionSetResult(COMP_RESULT_STOPPED, drivers::DRIVER_OK);
    CompetitionClearButtonEvents();
    g_appState.mode = APP_MODE_COMPETITION_ARMED;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus App_EmergencyStop(void)
{
    drivers::DriverStatus first_error = drivers::DRIVER_OK;

    const auto record_error = [&first_error](drivers::DriverStatus status) {
        if ((first_error == drivers::DRIVER_OK) &&
            (status != drivers::DRIVER_OK)) {
            first_error = status;
        }
    };

#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_MOTOR_DRIVER && \
    FEATURE_ENABLE_IMU
    record_error(RoadEventController_Cancel());
#endif
#if FEATURE_ENABLE_IMU && FEATURE_ENABLE_MOTOR_DRIVER
    record_error(ActionRunner_Cancel());
    record_error(Heading_Stop());
#endif
#if FEATURE_ENABLE_BALL_BALANCE
    BallBalance_EmergencyStop();
#endif
#if FEATURE_ENABLE_DM_G6220_CAN
    DmG6220Controller_EmergencyDisable();
#endif
#if FEATURE_ENABLE_GRAYSCALE && FEATURE_ENABLE_MOTOR_DRIVER
    record_error(LF_Stop());
#endif
#if FEATURE_ENABLE_MOTOR_DRIVER
    /* Release raw Shell position/speed leases even if chassis initialization
     * failed, then always make the final direct MotorDriver stop attempt. */
    Chassis_ReleaseAllMotorCommands();
    record_error(Chassis_Stop());
#endif

    if (g_appState.mode == APP_MODE_COMPETITION_RUNNING) {
        CompetitionSetResult(COMP_RESULT_STOPPED, first_error);
        CompetitionClearButtonEvents();
        g_appState.mode = APP_MODE_COMPETITION_ARMED;
    } else if ((g_appState.mode != APP_MODE_COMPETITION_ARMED) &&
               (g_appState.mode != APP_MODE_FAULT)) {
        g_appState.mode = APP_MODE_IDLE;
    }

    return first_error;
}

} /* namespace app */
