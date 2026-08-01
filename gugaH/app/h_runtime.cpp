#include "app/h_runtime.h"

#include "board/board_button.h"
#include "board/board_ball_vision.h"
#include "board/board_buzzer.h"
#include "board/board_fram.h"
#include "board/board_led.h"
#include "board/board_oled.h"
#include "board/board_pins.h"
#include "control/chassis.h"
#include "control/dm_actuator.h"
#include "control/sensor_hub.h"
#include "services/debug_uart.h"
#include "services/shell.h"
#include "services/time.h"

namespace gugah {
namespace {

HConfig g_config = {};
HAppState g_state = {};
HButtonEvents g_button_events = {};
MotionCommand g_last_motion = {};
uint32_t g_last_motion_send_ms = 0U;
bool g_hardware_fault = false;
bool g_chassis_fault = false;
bool g_chassis_command_failing = false;
uint32_t g_chassis_command_failure_ms = 0U;
bool g_telemetry = false;
bool g_suppress_buttons = false;
bool g_b1_start_armed = false;
bool g_config_save_pending = false;
bool g_manual_chassis = false;
int16_t g_manual_left_rpm = 0;
int16_t g_manual_right_rpm = 0;
bool g_manual_ball = false;
BallState g_manual_ball_state = {};
bool g_ready_hold_enabled = true;
bool g_ready_hold_active = false;
uint32_t g_ready_hold_retry_ms = 0U;
bool g_dm_bench_control = false;
uint32_t g_buzzer_off_ms = 0U;
uint32_t g_last_vision_print_ms = 0U;
bool g_vision_trace = false;
uint8_t g_gray_calibration_status = 0U;
static const uint32_t kBuzzerPulseMs = 80U;

void UpdateBallControl2ms(uint32_t now_ms)
{
    const HAppInput task_input = HRuntime_GetInput(now_ms);
    const BallOutput task_output =
        HApp_UpdateBall2ms(&g_state, &task_input, &g_config);
    if (task_output.command_valid) {
        (void)DmActuator_SetPosition(task_output.dm_target_mrad);
    }
    if (task_output.stop_chassis) {
        HRuntime_ChassisStopTest();
    }

    const int16_t ready_target_0p1mm =
        HApp_ReadyBallTarget0p1mm(&g_state);
    if (g_ready_hold_enabled && !g_manual_ball &&
        !g_dm_bench_control &&
        (g_state.run_state == H_STATE_READY) &&
        (static_cast<int32_t>(
             now_ms - g_ready_hold_retry_ms) >= 0)) {
        const HAppInput app_input = HRuntime_GetInput(now_ms);
        BallInput input = {};
        input.now_ms = now_ms;
        input.vision = app_input.vision;
        input.imu = app_input.imu;
        input.dm = app_input.dm;
        g_manual_ball = Ball_StartHold(
            &g_manual_ball_state, ready_target_0p1mm,
            &input, &g_config);
        g_ready_hold_active = g_manual_ball;
    }
    if (g_manual_ball && (g_state.run_state == H_STATE_READY)) {
        BallInput input = {};
        const HAppInput app_input = HRuntime_GetInput(now_ms);
        input.now_ms = now_ms;
        input.vision = app_input.vision;
        input.imu = app_input.imu;
        input.dm = app_input.dm;
        if (g_ready_hold_active &&
            (g_manual_ball_state.target_position_0p1mm !=
             ready_target_0p1mm)) {
            /* B2/B3 retargets the already-running READY controller.  Start()
             * preserves its observer and beam reference, so a 1 mm target
             * step does not cause a DM position discontinuity. */
            (void)Ball_StartHold(
                &g_manual_ball_state, ready_target_0p1mm,
                &input, &g_config);
        }
        const BallOutput output =
            Ball_Update(&g_manual_ball_state, &input, &g_config);
        if (output.command_valid) {
            (void)DmActuator_SetPosition(output.dm_target_mrad);
        }
        if (output.stop_chassis) {
            HRuntime_ChassisStopTest();
            if (g_ready_hold_active) {
                g_manual_ball = false;
                g_ready_hold_active = false;
                Ball_Init(&g_manual_ball_state);
                g_ready_hold_retry_ms = now_ms + 250U;
            }
        }
    }
}

bool FramRead(uint16_t address, uint8_t *data, uint16_t length)
{
    return board::Board_FramRead(address, data, length) ==
           drivers::DRIVER_OK;
}

bool FramWrite(uint16_t address,
               const uint8_t *data,
               uint16_t length)
{
    return board::Board_FramWrite(address, data, length) ==
           drivers::DRIVER_OK;
}

bool MotionChanged(const MotionCommand &left,
                   const MotionCommand &right)
{
    return (left.mode != right.mode) ||
           (left.left_rpm != right.left_rpm) ||
           (left.right_rpm != right.right_rpm) ||
           (left.left_position_count != right.left_position_count) ||
           (left.right_position_count != right.right_position_count);
}

void ApplyMotion(const MotionCommand &command, uint32_t now_ms)
{
    if (command.mode == MOTION_COMMAND_NONE) {
        return;
    }
    const bool changed = MotionChanged(command, g_last_motion);
    const uint32_t refresh_ms =
        (command.mode == MOTION_COMMAND_STOP) ? 100U : 20U;
    if ((command.mode != MOTION_COMMAND_SPEED) && !changed &&
        ((now_ms - g_last_motion_send_ms) < refresh_ms)) {
        return;
    }
    drivers::DriverStatus status = drivers::DRIVER_OK;
    if (command.mode == MOTION_COMMAND_SPEED) {
        status = Chassis_SetWheelRpm(
            command.left_rpm, command.right_rpm, now_ms);
    } else if (command.mode == MOTION_COMMAND_POSITION) {
        status = Chassis_SetWheelPosition(
            command.left_position_count,
            command.right_position_count, now_ms);
    } else {
        status = Chassis_Stop(now_ms);
    }
    if (status != drivers::DRIVER_OK) {
        if (!g_chassis_command_failing) {
            g_chassis_command_failing = true;
            g_chassis_command_failure_ms = now_ms;
        } else if ((now_ms - g_chassis_command_failure_ms) >= 100U) {
            g_chassis_fault = true;
        }
        (void)Chassis_Stop(now_ms);
    } else {
        g_chassis_command_failing = false;
    }
    g_last_motion = command;
    g_last_motion_send_ms = now_ms;
}

HButtonEvents TakeButtonEvents(void)
{
    HButtonEvents events = g_button_events;
    g_button_events = {};
    const bool any_down =
        board::Board_ButtonIsPressed(board::BOARD_BUTTON_1) ||
        board::Board_ButtonIsPressed(board::BOARD_BUTTON_2) ||
        board::Board_ButtonIsPressed(board::BOARD_BUTTON_3);
    if (g_suppress_buttons) {
        if (!any_down) {
            g_suppress_buttons = false;
        }
        events = {};
    } else {
        events.any_pressed = any_down || events.any_pressed;
    }
    return events;
}

void WriteUnsigned(uint32_t value)
{
    services::Shell_WriteInt(static_cast<int32_t>(value));
}

void PrintCsv(uint32_t now_ms)
{
    const drivers::GrayscaleProcessedData *line =
        SensorHub_GetLine();
    const ChassisFeedback chassis = Chassis_GetFeedback();
    const VisionFeedback vision = SensorHub_GetVision(now_ms);
    const ImuFeedback imu = SensorHub_GetImu();
    const DmFeedback dm = DmActuator_GetFeedback();
    const BallState *ball = HRuntime_GetActiveBallState();
    services::Shell_Write("T,");
    WriteUnsigned(now_ms);
    services::Shell_Write(",");
    services::Shell_WriteInt(g_state.run_state);
    services::Shell_Write(",");
    services::Shell_WriteInt(line != 0 ? line->line_position : 0);
    services::Shell_Write(",");
    services::Shell_WriteInt(line != 0 ? line->active_mask : 0);
    services::Shell_Write(",");
    services::Shell_WriteInt(chassis.left_rpm);
    services::Shell_Write(",");
    services::Shell_WriteInt(chassis.right_rpm);
    services::Shell_Write(",");
    services::Shell_WriteInt(g_state.course.distance_mm);
    services::Shell_Write(",");
    services::Shell_WriteInt(vision.frame.position_0p1mm);
    services::Shell_Write(",");
    services::Shell_WriteInt(ball->estimated_velocity_0p1mm_s);
    services::Shell_Write(",");
    services::Shell_WriteInt(ball->target_velocity_0p1mm_s);
    services::Shell_Write(",");
    services::Shell_WriteInt(ball->velocity_error_0p1mm_s);
    services::Shell_Write(",");
    services::Shell_WriteInt(ball->position_error_0p1mm);
    services::Shell_Write(",");
    services::Shell_WriteInt(ball->beam_target_mdeg);
    services::Shell_Write(",");
    services::Shell_WriteInt(imu.beam_mdeg);
    services::Shell_Write(",");
    services::Shell_WriteInt(ball->imu_beam_compensation_mdeg);
    services::Shell_Write(",");
    services::Shell_WriteInt(dm.position_mrad);
    services::Shell_Write(",");
    services::Shell_WriteInt(
        static_cast<int32_t>(Chassis_GetErrorCount() +
                             DmActuator_GetErrorCount() +
                             SensorHub_GetErrorCount()));
    services::Shell_Write(",");
    services::Shell_WriteInt(
        g_state.course.line_curve_factor_permille);
    services::Shell_Write(",");
    services::Shell_WriteInt(
        g_state.course.line_control.last_correction_rpm);
    services::Shell_Write(",");
    services::Shell_WriteInt(
        g_state.course.line_control.filtered_derivative_per_s);
    services::Shell_Write(",");
    services::Shell_WriteInt(g_last_motion.left_rpm);
    services::Shell_Write(",");
    services::Shell_WriteInt(g_last_motion.right_rpm);
    services::Shell_Write("\r\n");
}

void AppendInt(char *buffer,
               uint8_t size,
               uint8_t *index,
               int32_t value)
{
    char reversed[12] = {};
    uint8_t count = 0U;
    uint32_t magnitude = (value < 0)
        ? static_cast<uint32_t>(-(static_cast<int64_t>(value)))
        : static_cast<uint32_t>(value);
    do {
        reversed[count++] =
            static_cast<char>('0' + magnitude % 10U);
        magnitude /= 10U;
    } while ((magnitude != 0U) && (count < sizeof(reversed)));
    if ((value < 0) && (*index < (size - 1U))) {
        buffer[(*index)++] = '-';
    }
    while ((count != 0U) && (*index < (size - 1U))) {
        buffer[(*index)++] = reversed[--count];
    }
    buffer[*index] = '\0';
}

void AppendText(char *buffer,
                uint8_t size,
                uint8_t *index,
                const char *text)
{
    while ((*text != '\0') && (*index < (size - 1U))) {
        buffer[(*index)++] = *text++;
    }
    buffer[*index] = '\0';
}

void PrintVisionRx(uint32_t now_ms)
{
    const VisionFeedback vision = SensorHub_GetVision(now_ms);
    const drivers::BallVisionParserStats *stats =
        SensorHub_GetVisionStats();
    drivers::BallVisionFrame frame = {};
    const bool has_frame = SensorHub_GetLatestVisionFrame(&frame);
    char line[192] = {};
    uint8_t index = 0U;
#define APPEND_VISION_FIELD(label, value) \
    AppendText(line, sizeof(line), &index, label); \
    AppendInt(line, sizeof(line), &index, value)
    APPEND_VISION_FIELD("V,seq=", has_frame ? frame.sequence : -1);
    APPEND_VISION_FIELD(",fl=", has_frame ? frame.flags : 0);
    APPEND_VISION_FIELD(",x=",
                        has_frame ? frame.position_0p1mm : 0);
    APPEND_VISION_FIELD(",cf=", has_frame ? frame.confidence : 0);
    APPEND_VISION_FIELD(",src=",
                        has_frame ? frame.source_delay_ms : 0);
    APPEND_VISION_FIELD(",age=",
                        has_frame
                            ? static_cast<int32_t>(now_ms - frame.received_ms)
                            : -1);
    APPEND_VISION_FIELD(",on=", vision.communication_online ? 1 : 0);
    APPEND_VISION_FIELD(",use=", vision.ball_usable ? 1 : 0);
    APPEND_VISION_FIELD(",b=",
                        static_cast<int32_t>(stats->bytes_received));
    APPEND_VISION_FIELD(",vf=",
                        static_cast<int32_t>(stats->valid_frames));
    APPEND_VISION_FIELD(",crc=",
                        static_cast<int32_t>(stats->crc_errors));
    APPEND_VISION_FIELD(",hdr=",
                        static_cast<int32_t>(stats->header_errors));
    APPEND_VISION_FIELD(",pay=",
                        static_cast<int32_t>(stats->payload_errors));
    APPEND_VISION_FIELD(",irq=", static_cast<int32_t>(
        board::Board_BallVisionGetIrqCount()));
    APPEND_VISION_FIELD(",ue=", static_cast<int32_t>(
        board::Board_BallVisionGetUartErrors()));
    APPEND_VISION_FIELD(",dr=", static_cast<int32_t>(
        board::Board_BallVisionGetDroppedBytes()));
#undef APPEND_VISION_FIELD
    AppendText(line, sizeof(line), &index, "\r\n");
    (void)services::DebugUart_TryWriteData(
        reinterpret_cast<const uint8_t *>(line), index);
}

void OledLine(uint8_t row, const char *label, int32_t value)
{
    char text[22] = {};
    uint8_t index = 0U;
    while ((*label != '\0') && (index < sizeof(text) - 1U)) {
        text[index++] = *label++;
    }
    AppendInt(text, sizeof(text), &index, value);
    while (index < sizeof(text) - 1U) {
        text[index++] = ' ';
    }
    text[index] = '\0';
    (void)board::Board_OledWriteText(row, 0U, text);
}

void OledText(uint8_t row, const char *text)
{
    char padded[22] = {};
    uint8_t index = 0U;
    while ((*text != '\0') && (index < sizeof(padded) - 1U)) {
        padded[index++] = *text++;
    }
    while (index < sizeof(padded) - 1U) {
        padded[index++] = ' ';
    }
    padded[index] = '\0';
    (void)board::Board_OledWriteText(row, 0U, padded);
}

void OledTime(uint8_t row, uint32_t elapsed_ms)
{
    char text[22] = {};
    uint8_t index = 0U;
    /* Round to the nearest 0.1 s and format without floating point. */
    const uint32_t deciseconds = (elapsed_ms + 50U) / 100U;
    AppendText(text, sizeof(text), &index, "TIME ");
    AppendInt(text, sizeof(text), &index,
              static_cast<int32_t>(deciseconds / 10U));
    AppendText(text, sizeof(text), &index, ".");
    AppendInt(text, sizeof(text), &index,
              static_cast<int32_t>(deciseconds % 10U));
    AppendText(text, sizeof(text), &index, "s");
    OledText(row, text);
}

void OledSignedMillimeters(uint8_t row,
                           const char *label,
                           int32_t value_0p1mm)
{
    char text[22] = {};
    uint8_t index = 0U;
    AppendText(text, sizeof(text), &index, label);
    if (value_0p1mm < 0) {
        AppendText(text, sizeof(text), &index, "-");
        value_0p1mm = -value_0p1mm;
    }
    AppendInt(text, sizeof(text), &index, value_0p1mm / 10);
    AppendText(text, sizeof(text), &index, ".");
    AppendInt(text, sizeof(text), &index, value_0p1mm % 10);
    AppendText(text, sizeof(text), &index, "mm");
    OledText(row, text);
}

void OledResult(void)
{
    char text[22] = {};
    uint8_t index = 0U;
    text[index++] = 'E';
    AppendInt(text, sizeof(text), &index,
              g_state.maximum_ball_error_0p1mm);
    text[index++] = ' ';
    const char *reason = (g_state.run_state == H_STATE_PASS)
        ? "pass"
        : ((g_state.run_state == H_STATE_ABORTED)
            ? "aborted" : HApp_FailureText(g_state.failure));
    while ((*reason != '\0') && (index < sizeof(text) - 1U)) {
        text[index++] = *reason++;
    }
    text[index] = '\0';
    OledText(3U, text);
}

} /* namespace */

void HRuntime_Init(void)
{
    (void)board::Board_LedsInit();
    (void)board::Board_BuzzerInit();
    const bool buttons_ok =
        board::Board_ButtonsInit() == drivers::DRIVER_OK;
    const bool fram_ok =
        (board::Board_FramInit() == drivers::DRIVER_OK) &&
        (board::Board_FramSelfTest() == drivers::DRIVER_OK);
    const bool loaded = fram_ok && HConfig_Load(&g_config, FramRead);
    if (!loaded) {
        HConfig_Defaults(&g_config);
    }
    const bool oled_ok =
        board::Board_OledInit() == drivers::DRIVER_OK;

    const bool chassis_ok =
        Chassis_Init(&g_config, services::Time_Millis()) ==
        drivers::DRIVER_OK;
    const bool sensors_ok = SensorHub_Init(&g_config);
    const bool dm_transport_ok =
        DmActuator_Init() == drivers::DRIVER_OK;
    if (dm_transport_ok && buttons_ok && sensors_ok) {
        (void)DmActuator_Enable();
    }

    HApp_Init(&g_state);
    Ball_Init(&g_manual_ball_state);
    g_manual_ball = false;
    g_ready_hold_enabled = true;
    g_ready_hold_active = false;
    g_ready_hold_retry_ms = 0U;
    g_b1_start_armed = false;
    g_config_save_pending = false;
    g_gray_calibration_status = 0U;
    g_hardware_fault = !buttons_ok || !oled_ok;
    g_chassis_fault = !chassis_ok;
    g_chassis_command_failing = false;
    g_dm_bench_control = false;
    g_last_motion.mode = MOTION_COMMAND_NONE;
    (void)board::Board_StatusLedOn();
    services::Shell_WriteLine(loaded
        ? "config: FRAM"
        : "config: conservative defaults");
}

void HRuntime_Service(void)
{
    const uint32_t now_ms = services::Time_Millis();
    SensorHub_ServiceVision(now_ms, &g_config);
    DmActuator_Update(now_ms);
    services::Shell_Process();
    services::DebugUart_Pump();
    (void)board::Board_OledService();
    if (g_config_save_pending && !HApp_IsRunning(&g_state) &&
        !board::Board_OledHasPendingFlush() &&
        HConfig_Save(&g_config, FramWrite, FramRead)) {
        g_config_save_pending = false;
    }
}

void HRuntime_Update1ms(uint32_t now_ms)
{
    if ((g_buzzer_off_ms != 0U) &&
        (static_cast<int32_t>(
             now_ms - g_buzzer_off_ms) >= 0)) {
        (void)board::Board_BuzzerOff();
        g_buzzer_off_ms = 0U;
    }
    (void)board::Board_ButtonsUpdate(now_ms);
    SensorHub_Update1ms(now_ms, &g_config);
    const bool running = HApp_IsRunning(&g_state);
    const uint32_t b1 = board::Board_ButtonTakeEvents(
        board::BOARD_BUTTON_1, drivers::BUTTON_EVENT_ALL);
    const uint32_t b2 = board::Board_ButtonTakeEvents(
        board::BOARD_BUTTON_2, drivers::BUTTON_EVENT_ALL);
    const uint32_t b3 = board::Board_ButtonTakeEvents(
        board::BOARD_BUTTON_3, drivers::BUTTON_EVENT_ALL);
    g_button_events.b1_pressed |=
        (b1 & drivers::BUTTON_EVENT_PRESSED) != 0U;
    g_button_events.b1_short |=
        (b1 & drivers::BUTTON_EVENT_SHORT_PRESSED) != 0U;
    const bool ready = g_state.run_state == H_STATE_READY;
    if (ready &&
        ((b1 & drivers::BUTTON_EVENT_LONG_PRESSED) != 0U)) {
        /* Arm at the long-press threshold, but expose the start event only
         * after the debounced B1 release. */
        g_b1_start_armed = true;
    }
    if ((b1 & drivers::BUTTON_EVENT_RELEASED) != 0U) {
        if (ready && g_b1_start_armed) {
            g_button_events.b1_long = true;
        }
        g_b1_start_armed = false;
    } else if (!ready) {
        g_b1_start_armed = false;
    }
    g_button_events.b2_decrement |=
        (b2 & drivers::BUTTON_EVENT_SHORT_PRESSED) != 0U;
    g_button_events.b3_increment |=
        (b3 & drivers::BUTTON_EVENT_SHORT_PRESSED) != 0U;
    g_button_events.any_pressed |= running &&
        (((b1 | b2 | b3) & drivers::BUTTON_EVENT_PRESSED) != 0U);
    const bool calibration_adjustment_active =
        (g_state.run_state == H_STATE_RUNNING) &&
        ((g_state.selected_problem == H_PROBLEM_CAL_ZERO) ||
         (g_state.selected_problem == H_PROBLEM_CAL_H2_LOOP));
    if (((g_state.run_state == H_STATE_READY) ||
         calibration_adjustment_active) &&
        ((now_ms % 100U) == 0U)) {
        g_button_events.b2_decrement |=
            board::Board_ButtonIsPressed(board::BOARD_BUTTON_2) &&
            (board::Board_ButtonGetPressDurationMs(
                 board::BOARD_BUTTON_2) >= BOARD_BUTTON_LONG_PRESS_MS);
        g_button_events.b3_increment |=
            board::Board_ButtonIsPressed(board::BOARD_BUTTON_3) &&
            (board::Board_ButtonGetPressDurationMs(
                 board::BOARD_BUTTON_3) >= BOARD_BUTTON_LONG_PRESS_MS);
    }
}

void HRuntime_Update2ms(uint32_t now_ms)
{
    DmActuator_Update(now_ms);
    HAppInput input = HRuntime_GetInput(now_ms);
    input.line_frame_new = SensorHub_TakeLineFrameReady();
    input.buttons = TakeButtonEvents();
    if (input.buttons.b1_long &&
        (g_state.run_state == H_STATE_READY)) {
        HRuntime_ChassisStopTest();
        HRuntime_BallStopTest();
    }
    const HRunState before = g_state.run_state;
    const HAppOutput output =
        HApp_Update(&g_state, &input, &g_config);
    if (output.gray_calibrate_white ||
        output.gray_calibrate_black) {
        const bool captured = HConfig_CaptureGrayscaleSurface(
            &g_config,
            SensorHub_GetGrayscaleRaw(),
            output.gray_calibrate_white);
        g_gray_calibration_status = captured
            ? (output.gray_calibrate_white ? 1U : 2U) : 3U;
    }
    if (output.ball_zero_delta_0p1mm != 0) {
        int32_t zero = static_cast<int32_t>(
            g_config.ball_zero_offset_0p1mm) +
            output.ball_zero_delta_0p1mm;
        if (zero < -H_CONFIG_BALL_ZERO_LIMIT_0P1MM) {
            zero = -H_CONFIG_BALL_ZERO_LIMIT_0P1MM;
        } else if (zero > H_CONFIG_BALL_ZERO_LIMIT_0P1MM) {
            zero = H_CONFIG_BALL_ZERO_LIMIT_0P1MM;
        }
        g_config.ball_zero_offset_0p1mm =
            static_cast<int16_t>(zero);
    }
    if (output.h2_loop_delta_mm != 0) {
        int32_t offset = static_cast<int32_t>(
            g_config.h2_loop_offset_mm) +
            output.h2_loop_delta_mm;
        if (offset < -H_CONFIG_H2_LOOP_OFFSET_LIMIT_MM) {
            offset = -H_CONFIG_H2_LOOP_OFFSET_LIMIT_MM;
        } else if (offset > H_CONFIG_H2_LOOP_OFFSET_LIMIT_MM) {
            offset = H_CONFIG_H2_LOOP_OFFSET_LIMIT_MM;
        }
        g_config.h2_loop_offset_mm =
            static_cast<int16_t>(offset);
    }
    if (output.save_config) {
        g_config_save_pending = true;
    }
    if ((before != H_STATE_READY) &&
        (g_state.run_state == H_STATE_READY)) {
        g_ready_hold_enabled = true;
        g_ready_hold_active = false;
        g_ready_hold_retry_ms = now_ms;
        g_suppress_buttons = true;
    }
    if ((before == H_STATE_READY) &&
        (g_state.run_state == H_STATE_RUNNING)) {
        if (g_state.selected_problem == H_PROBLEM_CAL_GRAY) {
            g_gray_calibration_status = 0U;
        }
        g_suppress_buttons = true;
        (void)board::Board_BuzzerOn();
        g_buzzer_off_ms = now_ms + kBuzzerPulseMs;
    }
    ApplyMotion(output.motion, now_ms);
    if (output.ball.command_valid) {
        /* A busy TX buffer is retried by the next 10 ms ball update. */
        (void)DmActuator_SetPosition(output.ball.dm_target_mrad);
    }
    const bool became_terminal =
        (before != H_STATE_PASS) &&
        (before != H_STATE_FAIL) &&
        (before != H_STATE_ABORTED) &&
        ((g_state.run_state == H_STATE_PASS) ||
         (g_state.run_state == H_STATE_FAIL) ||
         (g_state.run_state == H_STATE_ABORTED));
    if (output.buzzer_pulse) {
        (void)board::Board_BuzzerOn();
        g_buzzer_off_ms = now_ms + kBuzzerPulseMs;
    } else if (became_terminal) {
        (void)board::Board_BuzzerOff();
        g_buzzer_off_ms = 0U;
    }
    UpdateBallControl2ms(now_ms);
}

void HRuntime_Update5ms(uint32_t now_ms)
{
    SensorHub_Update5ms(now_ms, &g_config);
    (void)Chassis_Update(now_ms);
    if (g_manual_chassis && (g_state.run_state == H_STATE_READY)) {
        (void)Chassis_SetWheelRpm(
            g_manual_left_rpm, g_manual_right_rpm, now_ms);
    }
}

void HRuntime_Update50ms(uint32_t now_ms)
{
    if (g_vision_trace &&
        ((now_ms - g_last_vision_print_ms) >= 100U)) {
        g_last_vision_print_ms = now_ms;
        PrintVisionRx(now_ms);
    }
    if (g_state.selected_problem == H_PROBLEM_CAL_ZERO) {
        OledText(0U, "CAL ZERO");
    } else if (g_state.selected_problem == H_PROBLEM_CAL_H2_LOOP) {
        OledText(0U, "CAL H2 LOOP");
    } else if (g_state.selected_problem == H_PROBLEM_CAL_GRAY) {
        OledText(0U, "CAL GRAY");
    } else {
        OledLine(0U, "H", g_state.selected_problem);
    }
    if (g_state.run_state == H_STATE_READY) {
        if (g_state.selected_problem == H_PROBLEM_CAL_ZERO) {
            OledSignedMillimeters(
                1U, "ZERO ", g_config.ball_zero_offset_0p1mm);
            OledText(2U, g_config_save_pending
                ? "SAVING FRAM" : "HOLD B1 TO START");
        } else if (g_state.selected_problem ==
                   H_PROBLEM_CAL_H2_LOOP) {
            OledLine(1U, "OFFSET mm ",
                     g_config.h2_loop_offset_mm);
            OledText(2U, g_config_save_pending
                ? "SAVING FRAM" : "HOLD B1 TO START");
        } else if (g_state.selected_problem ==
                   H_PROBLEM_CAL_GRAY) {
            OledText(1U, "B2=WHITE B3=BLACK");
            OledText(2U, g_config_save_pending
                ? "SAVING FRAM" : "HOLD B1 TO START");
        } else {
            OledLine(1U, "TARGET mm ",
                     g_state.h6_target_0p1mm / 10);
            OledLine(2U, "READY ", g_hardware_fault ? 0 : 1);
        }
    } else if (HApp_IsRunning(&g_state)) {
        if (g_state.selected_problem == H_PROBLEM_CAL_ZERO) {
            OledSignedMillimeters(
                1U, "ZERO ", g_config.ball_zero_offset_0p1mm);
            OledText(2U, "B2- B3+ B1 EXIT");
        } else if (g_state.selected_problem ==
                   H_PROBLEM_CAL_H2_LOOP) {
            OledLine(1U, "OFFSET mm ",
                     g_config.h2_loop_offset_mm);
            OledText(2U, "B2- B3+ B1 EXIT");
        } else if (g_state.selected_problem ==
                   H_PROBLEM_CAL_GRAY) {
            OledText(1U, "B2 CAP WHITE");
            OledText(2U, "B3 CAP BLACK");
        } else {
            OledTime(1U, now_ms - g_state.run_start_ms);
        }
        if (g_state.selected_problem == H_PROBLEM_3) {
            OledLine(2U, "H3 STAGE ", g_state.h3_stage);
        } else if ((g_state.selected_problem != H_PROBLEM_CAL_ZERO) &&
                   (g_state.selected_problem !=
                    H_PROBLEM_CAL_H2_LOOP) &&
                   (g_state.selected_problem != H_PROBLEM_CAL_GRAY)) {
            OledLine(2U, "DIST mm ", g_state.course.distance_mm);
        }
    } else {
        OledText(1U, HApp_StateText(g_state.run_state));
        OledTime(2U, g_state.result_time_ms);
        OledResult();
    }
    if ((g_state.run_state == H_STATE_READY) ||
        HApp_IsRunning(&g_state)) {
        if (g_state.selected_problem == H_PROBLEM_CAL_H2_LOOP) {
            OledLine(3U, "H2 STOP mm ",
                     static_cast<int32_t>(g_config.lap_distance_mm) +
                         g_config.h2_loop_offset_mm);
        } else if (g_state.selected_problem == H_PROBLEM_CAL_GRAY) {
            const char *status = "B1 EXIT";
            if (g_gray_calibration_status == 1U) {
                status = "WHITE OK";
            } else if (g_gray_calibration_status == 2U) {
                status = "BLACK OK";
            } else if (g_gray_calibration_status == 3U) {
                status = "CAPTURE ERR";
            }
            OledText(3U, status);
        } else {
            const BallState *ball = HRuntime_GetActiveBallState();
            OledLine(3U, "BALL e0.1 ",
                     HApp_IsRunning(&g_state)
                        ? g_state.ball.position_error_0p1mm
                        : ball->position_error_0p1mm);
        }
    }
    if (g_telemetry) {
        PrintCsv(now_ms);
    }
    (void)board::Board_LedSet(
        board::BOARD_LED_ID_2, g_state.run_state == H_STATE_RUNNING);
    (void)board::Board_LedSet(
        board::BOARD_LED_ID_3, g_state.run_state == H_STATE_FAIL);
}

HAppState *HRuntime_GetState(void)
{
    return &g_state;
}

const BallState *HRuntime_GetActiveBallState(void)
{
    return ((g_state.run_state == H_STATE_READY) && g_manual_ball)
        ? &g_manual_ball_state : &g_state.ball;
}

HConfig *HRuntime_GetConfig(void)
{
    return &g_config;
}

HAppInput HRuntime_GetInput(uint32_t now_ms)
{
    HAppInput input = {};
    input.now_ms = now_ms;
    input.hardware_fault = g_hardware_fault;
    input.chassis_fault = g_chassis_fault;
    input.line = SensorHub_GetLine();
    input.chassis = Chassis_GetFeedback();
    input.vision = SensorHub_GetVision(now_ms);
    input.imu = SensorHub_GetImu();
    input.dm = DmActuator_GetFeedback();
    return input;
}

bool HRuntime_Start(void)
{
    HRuntime_ChassisStopTest();
    HRuntime_BallStopTest();
    HAppInput input = HRuntime_GetInput(services::Time_Millis());
    const bool started = HApp_Start(&g_state, &input, &g_config);
    if (started) {
        g_suppress_buttons = true;
    }
    return started;
}

void HRuntime_Abort(void)
{
    const HAppOutput output = HApp_Abort(&g_state);
    ApplyMotion(output.motion, services::Time_Millis());
}

void HRuntime_Select(HProblem problem)
{
    if ((g_state.run_state == H_STATE_READY) &&
        (problem >= H_PROBLEM_2) &&
        (problem <= H_PROBLEM_CAL_GRAY)) {
        g_state.selected_problem = problem;
    }
}

bool HRuntime_SaveConfig(void)
{
    return !HApp_IsRunning(&g_state) &&
           !board::Board_OledHasPendingFlush() &&
           HConfig_Save(&g_config, FramWrite, FramRead);
}

void HRuntime_DefaultConfig(void)
{
    if (!HApp_IsRunning(&g_state)) {
        HConfig_Defaults(&g_config);
    }
}

void HRuntime_SetTelemetry(bool enabled)
{
    g_telemetry = enabled;
}

bool HRuntime_GetTelemetry(void)
{
    return g_telemetry;
}

void HRuntime_SetVisionTrace(bool enabled)
{
    g_vision_trace = enabled;
}

bool HRuntime_GetVisionTrace(void)
{
    return g_vision_trace;
}

void HRuntime_PrintStatus(void)
{
    services::Shell_Write("state=");
    services::Shell_Write(HApp_StateText(g_state.run_state));
    services::Shell_Write(" H=");
    services::Shell_WriteInt(g_state.selected_problem);
    services::Shell_Write(" fail=");
    services::Shell_Write(HApp_FailureText(g_state.failure));
    services::Shell_Write(" time_ms=");
    WriteUnsigned(g_state.result_time_ms);
    services::Shell_Write(" max_ball_0p1mm=");
    services::Shell_WriteInt(g_state.maximum_ball_error_0p1mm);
    services::Shell_Write(" h3_stage=");
    services::Shell_WriteInt(g_state.h3_stage);
    services::Shell_Write(" ball_target0.1=");
    services::Shell_WriteInt(g_state.active_ball_target_0p1mm);
    services::Shell_Write(" ball_zero0.1=");
    services::Shell_WriteInt(g_config.ball_zero_offset_0p1mm);
    services::Shell_Write(" h2_loop_mm=");
    services::Shell_WriteInt(g_config.h2_loop_offset_mm);
    services::Shell_Write(" h2_stop_mm=");
    services::Shell_WriteInt(
        static_cast<int32_t>(g_config.lap_distance_mm) +
        g_config.h2_loop_offset_mm);
    services::Shell_Write(" config_save_pending=");
    services::Shell_WriteInt(g_config_save_pending ? 1 : 0);
    services::Shell_Write(" course_phase=");
    services::Shell_WriteInt(g_state.course.phase);
    services::Shell_Write(" distance_mm=");
    services::Shell_WriteInt(g_state.course.distance_mm);
    services::Shell_Write(" course_pass_ms=");
    WriteUnsigned(g_state.course.pass_ms);
    services::Shell_Write(" h4_yaw_target=");
    services::Shell_WriteInt(g_state.course.h4_target_yaw_mdeg);
    services::Shell_Write(" h4_yaw_error=");
    services::Shell_WriteInt(g_state.course.h4_yaw_error_mdeg);
    services::Shell_Write(" h4_corr=");
    services::Shell_WriteInt(g_state.course.h4_heading_correction_rpm);
    services::Shell_Write(" h4_accel=");
    services::Shell_WriteInt(
        g_state.course.h4_commanded_accel_mm_s2);
    services::Shell_Write(" h5_accel=");
    services::Shell_WriteInt(
        g_state.course.h5_commanded_accel_mm_s2);
    services::Shell_Write(" h5_curve=");
    services::Shell_WriteInt(g_state.course.h5_curve_mode);
    services::Shell_Write(" line_curve_permille=");
    services::Shell_WriteInt(
        g_state.course.line_curve_factor_permille);
    services::Shell_Write(" line_corr=");
    services::Shell_WriteInt(
        g_state.course.line_control.last_correction_rpm);
    services::Shell_Write(" line_d_filtered=");
    services::Shell_WriteInt(
        g_state.course.line_control.filtered_derivative_per_s);
    services::Shell_Write(" h5_limit_rpm=");
    services::Shell_WriteInt(
        g_state.course.h5_speed_limit_millirpm / 1000);
    services::Shell_Write(" err_chassis=");
    WriteUnsigned(Chassis_GetErrorCount());
    services::Shell_Write(" err_dm=");
    WriteUnsigned(DmActuator_GetErrorCount());
    services::Shell_Write(" err_sensor=");
    WriteUnsigned(SensorHub_GetErrorCount());
    services::Shell_Write("\r\n");
}

void HRuntime_ClearFault(void)
{
    if (!HApp_IsRunning(&g_state)) {
        const HProblem selected = g_state.selected_problem;
        const int16_t target = g_state.h6_target_0p1mm;
        HRuntime_ChassisStopTest();
        HRuntime_BallStopTest();
        const bool buttons_ok =
            board::Board_ButtonIsReady(board::BOARD_BUTTON_1) &&
            board::Board_ButtonIsReady(board::BOARD_BUTTON_2) &&
            board::Board_ButtonIsReady(board::BOARD_BUTTON_3);
        const bool chassis_ok =
            Chassis_Init(&g_config, services::Time_Millis()) ==
            drivers::DRIVER_OK;
        const bool oled_ok = board::Board_OledIsReady() ||
            (board::Board_OledInit() == drivers::DRIVER_OK);
        g_hardware_fault = !buttons_ok || !oled_ok;
        g_chassis_fault = !chassis_ok;
        g_chassis_command_failing = false;
        HApp_Init(&g_state);
        g_state.selected_problem = selected;
        g_state.h6_target_0p1mm = target;
        g_ready_hold_enabled = true;
        g_ready_hold_active = false;
        g_ready_hold_retry_ms = services::Time_Millis();
    }
}

bool HRuntime_ChassisTest(int16_t left_rpm, int16_t right_rpm)
{
    if ((g_state.run_state != H_STATE_READY) ||
        (left_rpm < -60) || (left_rpm > 60) ||
        (right_rpm < -60) || (right_rpm > 60)) {
        return false;
    }
    g_manual_left_rpm = left_rpm;
    g_manual_right_rpm = right_rpm;
    g_manual_chassis = true;
    return Chassis_SetWheelRpm(
        left_rpm, right_rpm, services::Time_Millis()) ==
        drivers::DRIVER_OK;
}

void HRuntime_ChassisStopTest(void)
{
    g_manual_chassis = false;
    g_manual_left_rpm = 0;
    g_manual_right_rpm = 0;
    (void)Chassis_Stop(services::Time_Millis());
}

bool HRuntime_BallHold(int16_t target_0p1mm)
{
    if ((g_state.run_state != H_STATE_READY) ||
        (target_0p1mm < -1000) || (target_0p1mm > 1000)) {
        return false;
    }
    const HAppInput app_input =
        HRuntime_GetInput(services::Time_Millis());
    BallInput input = {};
    input.now_ms = app_input.now_ms;
    input.vision = app_input.vision;
    input.imu = app_input.imu;
    input.dm = app_input.dm;
    g_ready_hold_enabled = false;
    g_ready_hold_active = false;
    g_manual_ball = Ball_StartHold(
        &g_manual_ball_state, target_0p1mm, &input, &g_config);
    return g_manual_ball;
}

bool HRuntime_BallMove(int16_t target_0p1mm)
{
    if ((g_state.run_state != H_STATE_READY) ||
        (target_0p1mm < -1000) || (target_0p1mm > 1000)) {
        return false;
    }
    const HAppInput app_input =
        HRuntime_GetInput(services::Time_Millis());
    BallInput input = {};
    input.now_ms = app_input.now_ms;
    input.vision = app_input.vision;
    input.imu = app_input.imu;
    input.dm = app_input.dm;
    g_ready_hold_enabled = false;
    g_ready_hold_active = false;
    g_manual_ball = Ball_StartMove(
        &g_manual_ball_state, target_0p1mm, 10000U,
        &input, &g_config);
    return g_manual_ball;
}

void HRuntime_BallStopTest(void)
{
    g_manual_ball = false;
    g_ready_hold_enabled = false;
    g_ready_hold_active = false;
    g_dm_bench_control = false;
    Ball_Level(&g_manual_ball_state);
    (void)DmActuator_HoldCurrent();
}

bool HRuntime_DmBenchTakeControl(void)
{
    if (g_state.run_state != H_STATE_READY) {
        return false;
    }
    g_manual_ball = false;
    g_ready_hold_enabled = false;
    g_ready_hold_active = false;
    g_dm_bench_control = true;
    return true;
}

void HRuntime_DmBenchReleaseControl(void)
{
    g_dm_bench_control = false;
    g_ready_hold_enabled = true;
    g_ready_hold_active = false;
    g_ready_hold_retry_ms = services::Time_Millis();
    if ((g_state.run_state == H_STATE_READY) && DmActuator_IsReady()) {
        (void)DmActuator_HoldCurrent();
    }
}

bool HRuntime_DmBenchHasControl(void)
{
    return g_dm_bench_control;
}

} /* namespace gugah */
