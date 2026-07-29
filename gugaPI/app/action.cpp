#include "app/action.h"

#include "app/app_grayscale.h"
#include "app/app_imu.h"
#include "app/app_main.h"
#include "app/chassis.h"
#include "app/dm_g6220_controller.h"
#include "app/heading.h"
#include "app/linefollow.h"
#include "app/line_sensor.h"
#include "app/road_event_controller.h"
#include "board/board_buzzer.h"
#include "board/board_button.h"
#include "board/board_led.h"
#include "config/feature_config.h"
#include "drivers/common/driver_status.h"
#include "services/fault.h"
#include "services/time.h"

namespace app {
namespace {

static const uint8_t kMaxInstrs = 64U;
static const uint32_t kSequenceTimeoutMs = 300000U; /* whole-sequence cap */
static const int32_t kMaxLoopCount = 1000;
static const int32_t kDmMaxVelocityMradS = 20000;
static const uint32_t kSensorMaxAgeMs = 200U;
static const uint32_t kChassisFeedbackMaxAgeMs = 100U;
static const uint16_t kConditionTickMs = 50U;
static const uint16_t kConditionTimeoutMask = 0x03FFU;
static const uint16_t kConditionStableMask = 0x7C00U;
static const uint16_t kConditionWaitMask = 0x8000U;

ActionRunnerState g_state;
uint16_t g_loopIterations[kMaxInstrs];
bool g_preserveRoadMotion = false;

struct ConditionRuntime {
    bool prepared;
    bool true_pending;
    uint32_t true_since_ms;
    int32_t start_left_count;
    int32_t start_right_count;
    uint32_t start_road_event_sequence;
    bool value_valid;
    int32_t value;
    bool comparison_true;
};

ConditionRuntime g_conditionRuntime;

struct OutputTimer {
    bool active;
    uint32_t start_ms;
    uint32_t duration_ms;
};

OutputTimer g_led2Timer;
OutputTimer g_led3Timer;
OutputTimer g_buzzerTimer;

enum InstrResult {
    INSTR_RUNNING = 0,
    INSTR_SUCCESS,
    INSTR_FALSE,
    INSTR_TIMEOUT,
    INSTR_UNAVAILABLE,
    INSTR_ROUTE_UNAVAILABLE,
    INSTR_ROUTE_REACQUIRE_FAILED,
    INSTR_FAULT
};

bool IsButtonPressedSource(ActionConditionSource source)
{
    return (source == ACT_SOURCE_BUTTON1_PRESSED) ||
           (source == ACT_SOURCE_BUTTON2_PRESSED) ||
           (source == ACT_SOURCE_BUTTON3_PRESSED);
}

bool IsButtonSource(ActionConditionSource source)
{
    return (source >= ACT_SOURCE_BUTTON1_LEVEL) &&
           (source <= ACT_SOURCE_BUTTON3_PRESSED);
}

bool IsBooleanSource(ActionConditionSource source)
{
    return IsButtonSource(source) ||
           (source == ACT_SOURCE_LINE_DETECTED) ||
           (source == ACT_SOURCE_IMU_VALID) ||
           (source == ACT_SOURCE_CHASSIS_FEEDBACK_VALID) ||
           (source == ACT_SOURCE_CHASSIS_INITIALIZED);
}

bool IsEnumSource(ActionConditionSource source)
{
    return (source == ACT_SOURCE_ROAD_TYPE) ||
           (source == ACT_SOURCE_ROAD_EVENT_TYPE) ||
           (source == ACT_SOURCE_HEADING_MODE) ||
           (source == ACT_SOURCE_LINEFOLLOW_MODE) ||
           (source == ACT_SOURCE_APP_MODE);
}

bool IsCompareAllowed(ActionConditionSource source, ActionCompareOp compare)
{
    if (source == ACT_SOURCE_ROAD_EVENT_PATHS) {
        return (compare == ACT_COMPARE_CONTAINS) ||
               (compare == ACT_COMPARE_NOT_CONTAINS);
    }
    if (IsBooleanSource(source) || IsEnumSource(source) ||
        (source == ACT_SOURCE_CONSTANT)) {
        return (compare == ACT_COMPARE_EQ) || (compare == ACT_COMPARE_NE);
    }
    return (compare <= ACT_COMPARE_GE);
}

board::BoardButtonId ButtonIdForSource(ActionConditionSource source)
{
    if ((source == ACT_SOURCE_BUTTON2_LEVEL) ||
        (source == ACT_SOURCE_BUTTON2_PRESSED)) {
        return board::BOARD_BUTTON_2;
    }
    if ((source == ACT_SOURCE_BUTTON3_LEVEL) ||
        (source == ACT_SOURCE_BUTTON3_PRESSED)) {
        return board::BOARD_BUTTON_3;
    }
    return board::BOARD_BUTTON_1;
}

void ResetConditionRuntime(void)
{
    g_conditionRuntime.prepared = false;
    g_conditionRuntime.true_pending = false;
    g_conditionRuntime.true_since_ms = 0U;
    g_conditionRuntime.start_left_count = 0;
    g_conditionRuntime.start_right_count = 0;
    g_conditionRuntime.start_road_event_sequence = 0U;
    g_conditionRuntime.value_valid = false;
    g_conditionRuntime.value = 0;
    g_conditionRuntime.comparison_true = false;
}

void ResetLoopRuntime(void)
{
    for (uint8_t i = 0U; i < kMaxInstrs; i++) {
        g_loopIterations[i] = 0U;
    }
}

int32_t EncoderDeltaToMillimeters(int32_t delta_counts,
                                  uint32_t wheel_radius_um,
                                  uint32_t counts_per_rev)
{
    if ((wheel_radius_um == 0U) || (counts_per_rev == 0U)) {
        return 0;
    }
    static const int64_t kPiNumerator = 3141593LL;
    static const int64_t kPiDenominator = 1000000LL;
    const int64_t numerator =
        static_cast<int64_t>(delta_counts) * 2LL * kPiNumerator *
        static_cast<int64_t>(wheel_radius_um);
    const int64_t denominator =
        kPiDenominator * static_cast<int64_t>(counts_per_rev) * 1000LL;
    if (numerator >= 0) {
        return static_cast<int32_t>((numerator + denominator / 2LL) /
                                    denominator);
    }
    return static_cast<int32_t>((numerator - denominator / 2LL) /
                                denominator);
}

bool IsLineSensorFresh(const LineSensorSnapshot *data)
{
    return (data != 0) && data->valid && data->fresh;
}

bool IsImuFresh(const AppImuData *data, uint32_t now)
{
    return (data != 0) && data->valid &&
           ((now - data->last_update_ms) <= kSensorMaxAgeMs);
}

bool IsChassisFeedbackFresh(const ChassisState *state, uint32_t now)
{
    return (state != 0) && state->initialized &&
           (state->feedback_sequence != 0U) &&
           (state->last_feedback_status == drivers::DRIVER_OK) &&
           ((now - state->last_feedback_ms) <= kChassisFeedbackMaxAgeMs);
}

bool CompareValue(int32_t actual, ActionCompareOp compare, int32_t expected)
{
    switch (compare) {
    case ACT_COMPARE_EQ: return actual == expected;
    case ACT_COMPARE_NE: return actual != expected;
    case ACT_COMPARE_LT: return actual < expected;
    case ACT_COMPARE_LE: return actual <= expected;
    case ACT_COMPARE_GT: return actual > expected;
    case ACT_COMPARE_GE: return actual >= expected;
    case ACT_COMPARE_CONTAINS:
        return (actual & expected) == expected;
    case ACT_COMPARE_NOT_CONTAINS:
        return (actual & expected) != expected;
    default:
        return false;
    }
}

/* Stop every motion primitive so no state bleeds into the next instruction
 * (fixes drive->wait continuing to drive, drive->follow dual-commanding). */
void StopAll(void)
{
    (void) RoadEventController_Cancel();
    g_preserveRoadMotion = false;
}

void StopPreservedRoadMotionForCurrent(void)
{
    if (!g_state.running || !g_preserveRoadMotion ||
        (g_state.current >= g_state.count)) {
        return;
    }
    const ActionOp next_op = g_state.instrs[g_state.current].op;
    if ((next_op != ACT_OP_ROAD_NAV) && (next_op != ACT_OP_LOOP)) {
        StopAll();
    }
}

void ClearOutputTimer(OutputTimer *timer)
{
    timer->active = false;
    timer->start_ms = 0U;
    timer->duration_ms = 0U;
}

void StartOutputTimer(OutputTimer *timer, int32_t duration_ms)
{
    if (duration_ms > 0) {
        timer->active = true;
        timer->start_ms = services::Time_Millis();
        timer->duration_ms = static_cast<uint32_t>(duration_ms);
    } else {
        ClearOutputTimer(timer);
    }
}

void StopSequenceOutputs(void)
{
#if FEATURE_ENABLE_STATUS_LED
    if (board::Board_LedIsReady(board::BOARD_LED_ID_2)) {
        (void) board::Board_LedOff(board::BOARD_LED_ID_2);
    }
    if (board::Board_LedIsReady(board::BOARD_LED_ID_3)) {
        (void) board::Board_LedOff(board::BOARD_LED_ID_3);
    }
#endif
#if FEATURE_ENABLE_BUZZER
    if (board::Board_BuzzerIsReady()) {
        (void) board::Board_BuzzerOff();
    }
#endif
    ClearOutputTimer(&g_led2Timer);
    ClearOutputTimer(&g_led3Timer);
    ClearOutputTimer(&g_buzzerTimer);
}

void UpdateSequenceOutputs(void)
{
#if FEATURE_ENABLE_STATUS_LED
    if (g_led2Timer.active &&
        services::Time_HasElapsed(g_led2Timer.start_ms,
                                  g_led2Timer.duration_ms)) {
        (void) board::Board_LedOff(board::BOARD_LED_ID_2);
        ClearOutputTimer(&g_led2Timer);
    }
    if (g_led3Timer.active &&
        services::Time_HasElapsed(g_led3Timer.start_ms,
                                  g_led3Timer.duration_ms)) {
        (void) board::Board_LedOff(board::BOARD_LED_ID_3);
        ClearOutputTimer(&g_led3Timer);
    }
#endif
#if FEATURE_ENABLE_BUZZER
    if (g_buzzerTimer.active &&
        services::Time_HasElapsed(g_buzzerTimer.start_ms,
                                  g_buzzerTimer.duration_ms)) {
        (void) board::Board_BuzzerOff();
        ClearOutputTimer(&g_buzzerTimer);
    }
#endif
}

#if FEATURE_ENABLE_STATUS_LED
bool LedTargetReady(int32_t target)
{
    return (((target != 0) && (target != 2)) ||
            board::Board_LedIsReady(board::BOARD_LED_ID_2)) &&
           (((target != 0) && (target != 3)) ||
            board::Board_LedIsReady(board::BOARD_LED_ID_3));
}

bool ApplyLedOutput(const Instr *instr)
{
    if (!LedTargetReady(instr->param1)) {
        return false;
    }

    const board::BoardLedId first =
        (instr->param1 == 3) ? board::BOARD_LED_ID_3 :
                              board::BOARD_LED_ID_2;
    const board::BoardLedId last =
        (instr->param1 == 2) ? board::BOARD_LED_ID_2 :
                              board::BOARD_LED_ID_3;
    for (uint8_t raw_id = static_cast<uint8_t>(first);
         raw_id <= static_cast<uint8_t>(last);
         raw_id++) {
        const board::BoardLedId id =
            static_cast<board::BoardLedId>(raw_id);
        OutputTimer *timer = (id == board::BOARD_LED_ID_2) ?
            &g_led2Timer : &g_led3Timer;
        drivers::DriverStatus status = drivers::DRIVER_OK;
        if (instr->op == ACT_OP_LED_ON) {
            status = board::Board_LedOn(id);
        } else if (instr->op == ACT_OP_LED_OFF) {
            status = board::Board_LedOff(id);
        } else {
            status = board::Board_LedToggle(id);
        }
        if (status != drivers::DRIVER_OK) {
            return false;
        }
        if (board::Board_LedIsOn(id)) {
            StartOutputTimer(timer, instr->param2);
        } else {
            ClearOutputTimer(timer);
        }
    }
    return true;
}
#endif

#if FEATURE_ENABLE_BUZZER
bool ApplyBuzzerOutput(const Instr *instr)
{
    if (!board::Board_BuzzerIsReady()) {
        return false;
    }
    drivers::DriverStatus status = drivers::DRIVER_OK;
    if (instr->op == ACT_OP_BUZZER_ON) {
        status = board::Board_BuzzerOn();
    } else if (instr->op == ACT_OP_BUZZER_OFF) {
        status = board::Board_BuzzerOff();
    } else {
        status = board::Board_BuzzerToggle();
    }
    if (status != drivers::DRIVER_OK) {
        return false;
    }
    if (board::Board_BuzzerIsOn()) {
        StartOutputTimer(&g_buzzerTimer, instr->param2);
    } else {
        ClearOutputTimer(&g_buzzerTimer);
    }
    return true;
}
#endif

void FinishSequence(void)
{
    g_state.running = false;
    g_state.last_success = true;
    g_state.last_status = drivers::DRIVER_OK;
    g_state.result = ACT_RUN_SUCCESS;
    g_state.failure_reason = ACT_FAIL_NONE;
    g_state.failure_index = ACT_NEXT;
    StopAll();
#if FEATURE_ENABLE_DM_G6220_CAN
    DmG6220Controller_EmergencyDisable();
#endif
    StopSequenceOutputs();
    ResetConditionRuntime();
    ResetLoopRuntime();
    g_preserveRoadMotion = false;
}

void AbortSequence(void)
{
    g_state.running = false;
    g_state.last_success = false;
    if (g_state.result == ACT_RUN_RUNNING) {
        g_state.result = ACT_RUN_ABORTED;
    }
    StopAll();
#if FEATURE_ENABLE_DM_G6220_CAN
    DmG6220Controller_EmergencyDisable();
#endif
    StopSequenceOutputs();
    ResetConditionRuntime();
    ResetLoopRuntime();
    g_preserveRoadMotion = false;
}

/* Jump after an instruction completes. success path: ACT_NEXT = next instr,
 * else goto index. timeout path: ACT_NEXT = abort, else goto index (recovery). */
void Goto(uint8_t target, bool success)
{
    g_state.instr_start_ms = 0U;
    g_state.instr_started = false;
    if (success) {
        if ((target == ACT_NEXT) || (target >= g_state.count)) {
            g_state.current++;
            if (g_state.current >= g_state.count) {
                FinishSequence();
            }
        } else {
            g_state.current = target;
        }
    } else {
        if ((target == ACT_NEXT) || (target >= g_state.count)) {
            AbortSequence();
        } else {
            g_state.current = target;
        }
    }
}

bool EvalCond(ActionCond cond)
{
    switch (cond) {
    case ACT_COND_HEADING_REACHED:
        return (Heading_GetState()->mode == HEADING_IDLE);
    case ACT_COND_LINE_DETECTED:
        return LF_IsLineDetected();
    case ACT_COND_LINE_LOST:
        return !LF_IsLineDetected();
    case ACT_COND_BUTTON:
        return board::Board_ButtonWasPressed(board::BOARD_BUTTON_1);
    case ACT_COND_DISTANCE_REACHED:
        return (Heading_GetState()->mode == HEADING_IDLE);
    case ACT_COND_IMMEDIATE:
        return true;
    case ACT_COND_TIMEOUT:
    default:
        return false;   /* handled by the elapsed-time check in EvalInstr */
    }
}

bool ReadConditionValue(const ActionConditionConfig &condition,
                        bool wait_for_new_event,
                        int32_t *value)
{
    if (value == 0) {
        return false;
    }
    const uint32_t now = services::Time_Millis();
    const LineSensorSnapshot *line_sensor = LineSensor_GetSnapshot();
    const AppImuData *imu = App_ImuGetData();
    const ChassisState *chassis = Chassis_GetState();

    switch (condition.source) {
    case ACT_SOURCE_CONSTANT:
        *value = 1;
        return true;
    case ACT_SOURCE_BUTTON1_LEVEL:
    case ACT_SOURCE_BUTTON2_LEVEL:
    case ACT_SOURCE_BUTTON3_LEVEL: {
        const board::BoardButtonId id = ButtonIdForSource(condition.source);
        if (!board::Board_ButtonIsReady(id)) {
            return false;
        }
        *value = board::Board_ButtonIsPressed(id) ? 1 : 0;
        return true;
    }
    case ACT_SOURCE_BUTTON1_PRESSED:
    case ACT_SOURCE_BUTTON2_PRESSED:
    case ACT_SOURCE_BUTTON3_PRESSED: {
        const board::BoardButtonId id = ButtonIdForSource(condition.source);
        if (!board::Board_ButtonIsReady(id)) {
            return false;
        }
        *value =
            (board::Board_ButtonTakeEvents(
                 id, drivers::BUTTON_EVENT_PRESSED) != 0U) ? 1 : 0;
        return true;
    }
    case ACT_SOURCE_LINE_DETECTED:
        if (!IsLineSensorFresh(line_sensor)) {
            return false;
        }
        *value = LF_IsLineDetected() ? 1 : 0;
        return true;
    case ACT_SOURCE_LINE_POSITION_MPOS:
        if (!IsLineSensorFresh(line_sensor) ||
            !line_sensor->position_valid) {
            return false;
        }
        *value = line_sensor->line_position;
        return true;
    case ACT_SOURCE_LINE_CONFIDENCE:
        if (!IsLineSensorFresh(line_sensor)) {
            return false;
        }
        *value = line_sensor->position_confidence;
        return true;
    case ACT_SOURCE_ROAD_TYPE:
        if (!IsLineSensorFresh(line_sensor) ||
            !line_sensor->road_capable) {
            return false;
        }
        *value = static_cast<int32_t>(line_sensor->road_type);
        return true;
    case ACT_SOURCE_ROAD_EVENT_TYPE:
        if (!IsLineSensorFresh(line_sensor) ||
            !line_sensor->road_capable) {
            return false;
        }
        *value = (wait_for_new_event &&
                  (line_sensor->road_event_sequence ==
                   g_conditionRuntime.start_road_event_sequence)) ?
            static_cast<int32_t>(GRAYSCALE_ROAD_UNKNOWN) :
            static_cast<int32_t>(line_sensor->road_event_type);
        return true;
    case ACT_SOURCE_ROAD_EVENT_PATHS:
        if (!IsLineSensorFresh(line_sensor) ||
            !line_sensor->road_capable) {
            return false;
        }
        *value = (wait_for_new_event &&
                  (line_sensor->road_event_sequence ==
                   g_conditionRuntime.start_road_event_sequence)) ?
            0 : static_cast<int32_t>(line_sensor->road_event_paths);
        return true;
    case ACT_SOURCE_IMU_VALID:
        *value = IsImuFresh(imu, now) ? 1 : 0;
        return true;
    case ACT_SOURCE_IMU_YAW_MDEG:
    case ACT_SOURCE_IMU_PITCH_MDEG:
    case ACT_SOURCE_IMU_ROLL_MDEG:
    case ACT_SOURCE_IMU_GYRO_Z_MDPS:
        if (!IsImuFresh(imu, now)) {
            return false;
        }
        if (condition.source == ACT_SOURCE_IMU_YAW_MDEG) {
            *value = imu->yaw_mdeg;
        } else if (condition.source == ACT_SOURCE_IMU_PITCH_MDEG) {
            *value = imu->pitch_mdeg;
        } else if (condition.source == ACT_SOURCE_IMU_ROLL_MDEG) {
            *value = imu->roll_mdeg;
        } else {
            *value = imu->gyro_mdps[2];
        }
        return true;
    case ACT_SOURCE_CHASSIS_FEEDBACK_VALID:
        *value = IsChassisFeedbackFresh(chassis, now) ? 1 : 0;
        return true;
    case ACT_SOURCE_LEFT_ACTUAL_RPM:
    case ACT_SOURCE_RIGHT_ACTUAL_RPM:
    case ACT_SOURCE_LEFT_DISTANCE_MM:
    case ACT_SOURCE_RIGHT_DISTANCE_MM:
    case ACT_SOURCE_AVERAGE_DISTANCE_MM: {
        if (!IsChassisFeedbackFresh(chassis, now)) {
            return false;
        }
        if (condition.source == ACT_SOURCE_LEFT_ACTUAL_RPM) {
            *value = chassis->left.actual_rpm;
            return true;
        }
        if (condition.source == ACT_SOURCE_RIGHT_ACTUAL_RPM) {
            *value = chassis->right.actual_rpm;
            return true;
        }
        const int32_t left_mm = EncoderDeltaToMillimeters(
            chassis->left.encoder_count -
                g_conditionRuntime.start_left_count,
            chassis->config.wheel_radius_um,
            chassis->config.left_counts_per_rev);
        const int32_t right_mm = EncoderDeltaToMillimeters(
            chassis->right.encoder_count -
                g_conditionRuntime.start_right_count,
            chassis->config.wheel_radius_um,
            chassis->config.right_counts_per_rev);
        if (condition.source == ACT_SOURCE_LEFT_DISTANCE_MM) {
            *value = left_mm;
        } else if (condition.source == ACT_SOURCE_RIGHT_DISTANCE_MM) {
            *value = right_mm;
        } else {
            *value = static_cast<int32_t>(
                (static_cast<int64_t>(left_mm) + right_mm) / 2LL);
        }
        return true;
    }
    case ACT_SOURCE_CHASSIS_INITIALIZED:
        *value = ((chassis != 0) && chassis->initialized) ? 1 : 0;
        return true;
    case ACT_SOURCE_HEADING_MODE:
        *value = static_cast<int32_t>(Heading_GetState()->mode);
        return true;
    case ACT_SOURCE_LINEFOLLOW_MODE:
        *value = static_cast<int32_t>(LF_GetState()->mode);
        return true;
    case ACT_SOURCE_APP_MODE:
        *value = static_cast<int32_t>(App_GetState()->mode);
        return true;
    case ACT_SOURCE_COUNT:
    default:
        return false;
    }
}

void PrepareCondition(const Instr *instr)
{
    ResetConditionRuntime();
    ActionConditionConfig condition;
    if (!ActionCondition_Decode(instr, &condition)) {
        return;
    }
    const ChassisState *chassis = Chassis_GetState();
    if (chassis != 0) {
        g_conditionRuntime.start_left_count =
            chassis->left.encoder_count;
        g_conditionRuntime.start_right_count =
            chassis->right.encoder_count;
    }
    const LineSensorSnapshot *line_sensor = LineSensor_GetSnapshot();
    if ((line_sensor != 0) && line_sensor->road_capable) {
        g_conditionRuntime.start_road_event_sequence =
            line_sensor->road_event_sequence;
    }
    const bool waits = (instr->op != ACT_OP_CONDITION) ||
                       (condition.mode == ACT_CONDITION_WAIT);
    if (waits && IsButtonPressedSource(condition.source)) {
        const board::BoardButtonId id =
            ButtonIdForSource(condition.source);
        (void) board::Board_ButtonTakeEvents(
            id, drivers::BUTTON_EVENT_PRESSED);
    }
    g_conditionRuntime.prepared = true;
}

InstrResult EvalCompareInstr(const Instr *instr, uint32_t now)
{
    ActionConditionConfig condition;
    if (!ActionCondition_Decode(instr, &condition)) {
        return INSTR_UNAVAILABLE;
    }
    const bool waits = (instr->op != ACT_OP_CONDITION) ||
                       (condition.mode == ACT_CONDITION_WAIT);
    int32_t value = 0;
    const bool valid = ReadConditionValue(condition, waits, &value);
    g_conditionRuntime.value_valid = valid;
    g_conditionRuntime.value = value;
    if (!valid) {
        g_conditionRuntime.comparison_true = false;
        return INSTR_UNAVAILABLE;
    }

    const bool matches =
        CompareValue(value, condition.compare, condition.value);
    g_conditionRuntime.comparison_true = matches;
    if (!waits) {
        return matches ? INSTR_SUCCESS : INSTR_FALSE;
    }

    if (matches) {
        if (condition.stable_ms == 0U) {
            return INSTR_SUCCESS;
        }
        if (!g_conditionRuntime.true_pending) {
            g_conditionRuntime.true_pending = true;
            g_conditionRuntime.true_since_ms = now;
        } else if ((now - g_conditionRuntime.true_since_ms) >=
                   condition.stable_ms) {
            return INSTR_SUCCESS;
        }
    } else {
        g_conditionRuntime.true_pending = false;
        g_conditionRuntime.true_since_ms = 0U;
    }

    const uint32_t elapsed = now - g_state.instr_start_ms;
    return (elapsed > condition.timeout_ms) ?
        INSTR_TIMEOUT : INSTR_RUNNING;
}

bool StartOp(const Instr *instr)
{
    switch (instr->op) {
    case ACT_OP_DRIVE:
        return (Heading_HoldStart(instr->param1) == drivers::DRIVER_OK);
    case ACT_OP_TURN:
        return (Heading_TurnStart(instr->param1) == drivers::DRIVER_OK);
    case ACT_OP_FOLLOW:
        return (LF_Start(instr->param1, instr->param2) == drivers::DRIVER_OK);
    case ACT_OP_ROAD_NAV:
        return (RoadEventController_StartRoute(
                    static_cast<RoadRoute>(instr->condition_value),
                    instr->param1,
                    static_cast<uint32_t>(instr->param2)) ==
                drivers::DRIVER_OK);
    case ACT_OP_DRIVE_IF:
        return (Heading_HoldStart(instr->param1) == drivers::DRIVER_OK);
    case ACT_OP_FOLLOW_IF: {
        ActionConditionConfig condition;
        return ActionCondition_Decode(instr, &condition) &&
               (LF_Start(instr->param1, condition.timeout_ms) ==
                drivers::DRIVER_OK);
    }
    case ACT_OP_DRIVE_MM:
        return (Heading_DistanceStart(instr->param1,
                                      instr->param2,
                                      0U) == drivers::DRIVER_OK);
    case ACT_OP_DM_POSITION:
#if FEATURE_ENABLE_DM_G6220_CAN
        return DmG6220Controller_StartPosition(
                   (instr->until == ACT_COND_DM_RELATIVE) ?
                       DM_POSITION_RELATIVE : DM_POSITION_ABSOLUTE,
                   instr->param1,
                   instr->param2,
                   static_cast<uint32_t>(instr->condition_value)) ==
               drivers::DRIVER_OK;
#else
        return false;
#endif
    case ACT_OP_DM_SPEED:
#if FEATURE_ENABLE_DM_G6220_CAN
        return DmG6220Controller_StartSpeed(instr->param1) ==
               drivers::DRIVER_OK;
#else
        return false;
#endif
    case ACT_OP_DM_DISABLE:
#if FEATURE_ENABLE_DM_G6220_CAN
        return DmG6220Controller_Disable() == drivers::DRIVER_OK;
#else
        return false;
#endif
    case ACT_OP_WAIT:
        return true;
    case ACT_OP_STOP:
        StopAll();
        return true;
    case ACT_OP_LED_ON:
    case ACT_OP_LED_OFF:
    case ACT_OP_LED_TOGGLE:
#if FEATURE_ENABLE_STATUS_LED
        return ApplyLedOutput(instr);
#else
        return false;
#endif
    case ACT_OP_BUZZER_ON:
    case ACT_OP_BUZZER_OFF:
    case ACT_OP_BUZZER_TOGGLE:
#if FEATURE_ENABLE_BUZZER
        return ApplyBuzzerOutput(instr);
#else
        return false;
#endif
    case ACT_OP_BRANCH:
    case ACT_OP_CONDITION:
    case ACT_OP_LOOP:
    case ACT_OP_END:
        return true;
    default:
        return false;
    }
}

InstrResult EvalInstr(const Instr *instr, uint32_t now)
{
    if ((instr->op == ACT_OP_STOP) || (instr->op == ACT_OP_END)) {
        return INSTR_SUCCESS;
    }
    if (instr->op == ACT_OP_BRANCH) {
        return EvalCond(instr->until) ? INSTR_SUCCESS : INSTR_TIMEOUT;
    }
    if (ActionCondition_IsCompareOp(instr->op)) {
        return EvalCompareInstr(instr, now);
    }
    if (instr->op == ACT_OP_ROAD_NAV) {
        const RoadRouteResult result =
            RoadEventController_GetState()->route_result;
        switch (result) {
        case ROAD_ROUTE_RESULT_RUNNING:
            return ((now - g_state.instr_start_ms) >
                    static_cast<uint32_t>(instr->param2))
                ? INSTR_TIMEOUT : INSTR_RUNNING;
        case ROAD_ROUTE_RESULT_SUCCESS:
            return INSTR_SUCCESS;
        case ROAD_ROUTE_RESULT_UNAVAILABLE:
            return INSTR_ROUTE_UNAVAILABLE;
        case ROAD_ROUTE_RESULT_REACQUIRE_FAILED:
            return INSTR_ROUTE_REACQUIRE_FAILED;
        case ROAD_ROUTE_RESULT_TIMEOUT:
            return INSTR_TIMEOUT;
        case ROAD_ROUTE_RESULT_IDLE:
        case ROAD_ROUTE_RESULT_CANCELLED:
        case ROAD_ROUTE_RESULT_CONTROL_ERROR:
        case ROAD_ROUTE_RESULT_FAULT:
        default:
            return INSTR_FAULT;
        }
    }
    if (instr->op == ACT_OP_DRIVE_MM) {
        return EvalCond(ACT_COND_DISTANCE_REACHED)
            ? INSTR_SUCCESS
            : INSTR_RUNNING;
    }
    if (instr->op == ACT_OP_DM_POSITION) {
#if FEATURE_ENABLE_DM_G6220_CAN
        const DmG6220OperationResult result =
            DmG6220Controller_GetState()->operation_result;
        if (result == DM_OPERATION_RUNNING) {
            return INSTR_RUNNING;
        }
        if (result == DM_OPERATION_SUCCESS) {
            return INSTR_SUCCESS;
        }
        if (result == DM_OPERATION_TARGET_TIMEOUT) {
            return INSTR_TIMEOUT;
        }
        return INSTR_FAULT;
#else
        return INSTR_FAULT;
#endif
    }
    if (instr->op == ACT_OP_DM_SPEED) {
#if FEATURE_ENABLE_DM_G6220_CAN
        const uint32_t elapsed = now - g_state.instr_start_ms;
        if (elapsed < static_cast<uint32_t>(instr->param2)) {
            return INSTR_RUNNING;
        }
        const DmG6220ControlState *state =
            DmG6220Controller_GetState();
        if (state->mode == DM_CONTROL_SPEED) {
            if (DmG6220Controller_StopSpeedAndHold() !=
                drivers::DRIVER_OK) {
                return INSTR_FAULT;
            }
            return INSTR_RUNNING;
        }
        if (state->operation_result == DM_OPERATION_RUNNING) {
            return INSTR_RUNNING;
        }
        if (state->operation_result == DM_OPERATION_SUCCESS) {
            return INSTR_SUCCESS;
        }
        if (state->operation_result == DM_OPERATION_STOP_TIMEOUT) {
            return INSTR_TIMEOUT;
        }
        return INSTR_FAULT;
#else
        return INSTR_FAULT;
#endif
    }
    if (instr->op == ACT_OP_DM_DISABLE) {
#if FEATURE_ENABLE_DM_G6220_CAN
        const DmG6220OperationResult result =
            DmG6220Controller_GetState()->operation_result;
        if (result == DM_OPERATION_RUNNING) {
            return INSTR_RUNNING;
        }
        return (result == DM_OPERATION_SUCCESS) ?
            INSTR_SUCCESS : INSTR_FAULT;
#else
        return INSTR_FAULT;
#endif
    }

    const uint32_t elapsed = now - g_state.instr_start_ms;
    if (instr->until == ACT_COND_TIMEOUT) {
        return (elapsed >= static_cast<uint32_t>(instr->param2)) ?
                   INSTR_SUCCESS : INSTR_RUNNING;
    }
    if (elapsed > static_cast<uint32_t>(instr->param2)) {
        return INSTR_TIMEOUT;
    }
    return EvalCond(instr->until) ? INSTR_SUCCESS : INSTR_RUNNING;
}

void SetValidationError(ActionValidationResult *result,
                        uint8_t index,
                        ActionValidationField field,
                        ActionValidationReason reason)
{
    if (result != 0) {
        result->valid = false;
        result->index = index;
        result->field = field;
        result->reason = reason;
    }
}

bool IsOneOf(ActionCond cond,
             ActionCond a,
             ActionCond b,
             ActionCond c,
             ActionCond d)
{
    return (cond == a) || (cond == b) || (cond == c) || (cond == d);
}

bool ConditionValueInRange(ActionConditionSource source,
                           int32_t value,
                           int32_t max_rpm)
{
    if (IsBooleanSource(source) || (source == ACT_SOURCE_CONSTANT)) {
        return (value == 0) || (value == 1);
    }
    switch (source) {
    case ACT_SOURCE_LINE_POSITION_MPOS:
        return (value >= -3500) && (value <= 3500);
    case ACT_SOURCE_LINE_CONFIDENCE:
        return (value >= 0) && (value <= 1000);
    case ACT_SOURCE_ROAD_TYPE:
    case ACT_SOURCE_ROAD_EVENT_TYPE:
        return (value >= static_cast<int32_t>(GRAYSCALE_ROAD_UNKNOWN)) &&
               (value <= static_cast<int32_t>(
                    GRAYSCALE_ROAD_RIGHT_CORNER));
    case ACT_SOURCE_ROAD_EVENT_PATHS:
        return (value >= 1) && (value <= 7);
    case ACT_SOURCE_IMU_YAW_MDEG:
        return (value >= -180000) && (value <= 180000);
    case ACT_SOURCE_IMU_PITCH_MDEG:
    case ACT_SOURCE_IMU_ROLL_MDEG:
        return (value >= 0) && (value <= 360000);
    case ACT_SOURCE_IMU_GYRO_Z_MDPS:
        return (value >= -2000000) && (value <= 2000000);
    case ACT_SOURCE_LEFT_ACTUAL_RPM:
    case ACT_SOURCE_RIGHT_ACTUAL_RPM:
        return (value >= -max_rpm) && (value <= max_rpm);
    case ACT_SOURCE_LEFT_DISTANCE_MM:
    case ACT_SOURCE_RIGHT_DISTANCE_MM:
    case ACT_SOURCE_AVERAGE_DISTANCE_MM:
        return (value >= -10000) && (value <= 10000);
    case ACT_SOURCE_HEADING_MODE:
        return (value >= static_cast<int32_t>(HEADING_IDLE)) &&
               (value <= static_cast<int32_t>(HEADING_ARC_TURN));
    case ACT_SOURCE_LINEFOLLOW_MODE:
        return (value >= static_cast<int32_t>(LF_IDLE)) &&
               (value <= static_cast<int32_t>(LF_FOLLOW));
    case ACT_SOURCE_APP_MODE:
        return (value >= static_cast<int32_t>(APP_MODE_IDLE)) &&
               (value <= static_cast<int32_t>(
                    APP_MODE_COMPETITION_RUNNING));
    case ACT_SOURCE_BUTTON1_LEVEL:
    case ACT_SOURCE_BUTTON1_PRESSED:
    case ACT_SOURCE_BUTTON2_LEVEL:
    case ACT_SOURCE_BUTTON2_PRESSED:
    case ACT_SOURCE_BUTTON3_LEVEL:
    case ACT_SOURCE_BUTTON3_PRESSED:
    case ACT_SOURCE_LINE_DETECTED:
    case ACT_SOURCE_IMU_VALID:
    case ACT_SOURCE_CHASSIS_FEEDBACK_VALID:
    case ACT_SOURCE_CHASSIS_INITIALIZED:
        return (value == 0) || (value == 1);
    case ACT_SOURCE_COUNT:
    default:
        return false;
    }
}

bool ValidateCompareInstr(const Instr *instr, int32_t max_rpm)
{
    ActionConditionConfig condition;
    if (!ActionCondition_Decode(instr, &condition) ||
        !IsCompareAllowed(condition.source, condition.compare) ||
        !ConditionValueInRange(condition.source, condition.value, max_rpm)) {
        return false;
    }
    if (IsButtonPressedSource(condition.source) &&
        (condition.stable_ms != 0U)) {
        return false;
    }
    if (instr->op == ACT_OP_CONDITION) {
        if (instr->param1 != 0) {
            return false;
        }
        if (condition.mode == ACT_CONDITION_IMMEDIATE) {
            return (condition.timeout_ms == 0U) &&
                   (condition.stable_ms == 0U);
        }
        return (condition.timeout_ms >= kConditionTickMs) &&
               (condition.timeout_ms <= 30000U);
    }
    if ((instr->param1 < -max_rpm) || (instr->param1 > max_rpm) ||
        (condition.mode != ACT_CONDITION_WAIT) ||
        (condition.timeout_ms < kConditionTickMs) ||
        (condition.timeout_ms > 30000U)) {
        return false;
    }
    return true;
}

bool ValidateInstr(const Instr *instr,
                   uint8_t index,
                   ActionValidationResult *result)
{
    const int32_t max_rpm = static_cast<int32_t>(
        Chassis_GetState()->config.max_wheel_rpm);
    if ((instr->op <= ACT_OP_NONE) ||
        (instr->op > ACT_OP_DM_DISABLE)) {
        SetValidationError(result, index, ACT_VALID_FIELD_OP,
                           ACT_VALID_UNKNOWN_OP);
        return false;
    }
    if ((!ActionCondition_IsCompareOp(instr->op)) &&
        ((instr->until < ACT_COND_TIMEOUT) ||
         (instr->until > ACT_COND_DM_RELATIVE))) {
        SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                           ACT_VALID_WRONG_CONDITION);
        return false;
    }

    const bool led_op = (instr->op >= ACT_OP_LED_ON) &&
                        (instr->op <= ACT_OP_LED_TOGGLE);
    const bool buzzer_op = (instr->op >= ACT_OP_BUZZER_ON) &&
                           (instr->op <= ACT_OP_BUZZER_TOGGLE);
    const bool output_off = (instr->op == ACT_OP_LED_OFF) ||
                            (instr->op == ACT_OP_BUZZER_OFF);
    if (instr->op == ACT_OP_DRIVE) {
        if ((instr->param1 < -max_rpm) || (instr->param1 > max_rpm)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM1,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if (!IsOneOf(instr->until, ACT_COND_TIMEOUT,
                     ACT_COND_LINE_DETECTED, ACT_COND_LINE_LOST,
                     ACT_COND_BUTTON)) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return false;
        }
        if ((instr->param2 <= 0) || (instr->param2 > 30000)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM2,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
    } else if (instr->op == ACT_OP_DRIVE_MM) {
        if ((instr->param1 < -10000) || (instr->param1 > 10000)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM1,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if (instr->param1 == 0) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM1,
                               ACT_VALID_MUST_BE_NONZERO);
            return false;
        }
        if ((instr->param2 <= 0) || (instr->param2 > max_rpm)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM2,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if (instr->until != ACT_COND_DISTANCE_REACHED) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return false;
        }
    } else if (instr->op == ACT_OP_TURN) {
        if ((instr->param1 < -180) || (instr->param1 > 180)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM1,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if ((instr->param2 <= 0) || (instr->param2 > 30000)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM2,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if (instr->until != ACT_COND_HEADING_REACHED) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return false;
        }
    } else if (instr->op == ACT_OP_FOLLOW) {
        if ((instr->param1 < -max_rpm) || (instr->param1 > max_rpm)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM1,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if ((instr->param2 <= 0) || (instr->param2 > 30000)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM2,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if (!IsOneOf(instr->until, ACT_COND_TIMEOUT, ACT_COND_LINE_LOST,
                     ACT_COND_BUTTON, ACT_COND_LINE_DETECTED)) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return false;
        }
    } else if (instr->op == ACT_OP_WAIT) {
        if (instr->param1 != 0) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM1,
                               ACT_VALID_MUST_BE_ZERO);
            return false;
        }
        if ((instr->param2 < 0) || (instr->param2 > 30000)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM2,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if (!IsOneOf(instr->until, ACT_COND_TIMEOUT,
                     ACT_COND_LINE_DETECTED, ACT_COND_LINE_LOST,
                     ACT_COND_BUTTON)) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return false;
        }
    } else if ((instr->op == ACT_OP_STOP) || (instr->op == ACT_OP_END)) {
        if (instr->param1 != 0) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM1,
                               ACT_VALID_MUST_BE_ZERO);
            return false;
        }
        if (instr->param2 != 0) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM2,
                               ACT_VALID_MUST_BE_ZERO);
            return false;
        }
        if (instr->until != ACT_COND_IMMEDIATE) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return false;
        }
    } else if (instr->op == ACT_OP_BRANCH) {
        if (instr->param1 != 0) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM1,
                               ACT_VALID_MUST_BE_ZERO);
            return false;
        }
        if (instr->param2 != 0) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM2,
                               ACT_VALID_MUST_BE_ZERO);
            return false;
        }
        if (!IsOneOf(instr->until, ACT_COND_LINE_DETECTED,
                     ACT_COND_LINE_LOST, ACT_COND_BUTTON,
                     ACT_COND_IMMEDIATE) &&
            (instr->until != ACT_COND_TIMEOUT)) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return false;
        }
    } else if (instr->op == ACT_OP_LOOP) {
        if ((instr->param1 < 1) || (instr->param1 > kMaxLoopCount)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM1,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if (instr->param2 != 0) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM2,
                               ACT_VALID_MUST_BE_ZERO);
            return false;
        }
        if (instr->until != ACT_COND_IMMEDIATE) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return false;
        }
    } else if (instr->op == ACT_OP_ROAD_NAV) {
        if ((instr->param1 < 1) || (instr->param1 > max_rpm)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM1,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if ((instr->param2 < 50) || (instr->param2 > 30000) ||
            ((instr->param2 % 50) != 0)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM2,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if (instr->until != ACT_COND_IMMEDIATE) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return false;
        }
        if ((instr->condition_value <
             static_cast<int32_t>(ROAD_ROUTE_LEFT)) ||
            (instr->condition_value >=
             static_cast<int32_t>(ROAD_ROUTE_COUNT))) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
    } else if (instr->op == ACT_OP_DM_POSITION) {
        if ((instr->param1 < -drivers::DM_G6220_POSITION_LIMIT_MRAD) ||
            (instr->param1 > drivers::DM_G6220_POSITION_LIMIT_MRAD)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM1,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if ((instr->param2 <= 0) ||
            (instr->param2 > kDmMaxVelocityMradS)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM2,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if ((instr->until != ACT_COND_DM_ABSOLUTE) &&
            (instr->until != ACT_COND_DM_RELATIVE)) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return false;
        }
        if ((instr->condition_value < 50) ||
            (instr->condition_value > 30000) ||
            ((instr->condition_value % 50) != 0)) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
    } else if (instr->op == ACT_OP_DM_SPEED) {
        if ((instr->param1 == 0) ||
            (instr->param1 < -kDmMaxVelocityMradS) ||
            (instr->param1 > kDmMaxVelocityMradS)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM1,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if ((instr->param2 < 50) || (instr->param2 > 30000) ||
            ((instr->param2 % 50) != 0)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM2,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if ((instr->until != ACT_COND_IMMEDIATE) ||
            (instr->condition_value != 0)) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return false;
        }
    } else if (instr->op == ACT_OP_DM_DISABLE) {
        if ((instr->param1 != 0) || (instr->param2 != 0)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM1,
                               ACT_VALID_MUST_BE_ZERO);
            return false;
        }
        if ((instr->until != ACT_COND_IMMEDIATE) ||
            (instr->condition_value != 0)) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return false;
        }
    } else if (led_op || buzzer_op) {
        if (led_op && (instr->param1 != 0) &&
            (instr->param1 != 2) && (instr->param1 != 3)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM1,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if (buzzer_op && (instr->param1 != 0)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM1,
                               ACT_VALID_MUST_BE_ZERO);
            return false;
        }
        if ((instr->param2 < 0) || (instr->param2 > 30000) ||
            ((instr->param2 > 0) && (instr->param2 < 50))) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM2,
                               ACT_VALID_OUT_OF_RANGE);
            return false;
        }
        if (output_off && (instr->param2 != 0)) {
            SetValidationError(result, index, ACT_VALID_FIELD_PARAM2,
                               ACT_VALID_MUST_BE_ZERO);
            return false;
        }
        if (instr->until != ACT_COND_IMMEDIATE) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return false;
        }
    } else if (ActionCondition_IsCompareOp(instr->op)) {
        if (!ValidateCompareInstr(instr, max_rpm)) {
            SetValidationError(result, index, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return false;
        }
    }
    return true;
}

} /* namespace */

bool ActionCondition_IsCompareOp(ActionOp op)
{
    return (op == ACT_OP_CONDITION) ||
           (op == ACT_OP_DRIVE_IF) ||
           (op == ACT_OP_FOLLOW_IF);
}

bool ActionCondition_Decode(const Instr *instr,
                            ActionConditionConfig *condition)
{
    if ((instr == 0) || (condition == 0) ||
        !ActionCondition_IsCompareOp(instr->op)) {
        return false;
    }
    const uint8_t raw = static_cast<uint8_t>(instr->until);
    const uint8_t source_raw = static_cast<uint8_t>(raw >> 3U);
    const uint8_t compare_raw = static_cast<uint8_t>(raw & 0x07U);
    if ((source_raw >= static_cast<uint8_t>(ACT_SOURCE_COUNT)) ||
        (compare_raw > static_cast<uint8_t>(ACT_COMPARE_NOT_CONTAINS))) {
        return false;
    }
    const uint16_t timing = static_cast<uint16_t>(instr->param2);
    const uint16_t timeout_ticks =
        static_cast<uint16_t>(timing & kConditionTimeoutMask);
    const uint16_t stable_ticks =
        static_cast<uint16_t>((timing & kConditionStableMask) >> 10U);
    condition->source =
        static_cast<ActionConditionSource>(source_raw);
    condition->compare = static_cast<ActionCompareOp>(compare_raw);
    condition->value = instr->condition_value;
    condition->mode =
        ((timing & kConditionWaitMask) != 0U) ?
            ACT_CONDITION_WAIT : ACT_CONDITION_IMMEDIATE;
    condition->timeout_ms =
        static_cast<uint16_t>(timeout_ticks * kConditionTickMs);
    condition->stable_ms =
        static_cast<uint16_t>(stable_ticks * kConditionTickMs);
    if ((instr->op == ACT_OP_DRIVE_IF) ||
        (instr->op == ACT_OP_FOLLOW_IF)) {
        condition->mode = ACT_CONDITION_WAIT;
    }
    return true;
}

const char *ActionCondition_SourceText(ActionConditionSource source)
{
    switch (source) {
    case ACT_SOURCE_CONSTANT: return "constant";
    case ACT_SOURCE_BUTTON1_LEVEL: return "button1_level";
    case ACT_SOURCE_BUTTON1_PRESSED: return "button1_pressed";
    case ACT_SOURCE_BUTTON2_LEVEL: return "button2_level";
    case ACT_SOURCE_BUTTON2_PRESSED: return "button2_pressed";
    case ACT_SOURCE_BUTTON3_LEVEL: return "button3_level";
    case ACT_SOURCE_BUTTON3_PRESSED: return "button3_pressed";
    case ACT_SOURCE_LINE_DETECTED: return "line_detected";
    case ACT_SOURCE_LINE_POSITION_MPOS: return "line_position";
    case ACT_SOURCE_LINE_CONFIDENCE: return "line_confidence";
    case ACT_SOURCE_ROAD_TYPE: return "road_type";
    case ACT_SOURCE_ROAD_EVENT_TYPE: return "road_event_type";
    case ACT_SOURCE_ROAD_EVENT_PATHS: return "road_event_paths";
    case ACT_SOURCE_IMU_VALID: return "imu_valid";
    case ACT_SOURCE_IMU_YAW_MDEG: return "imu_yaw";
    case ACT_SOURCE_IMU_PITCH_MDEG: return "imu_pitch";
    case ACT_SOURCE_IMU_ROLL_MDEG: return "imu_roll";
    case ACT_SOURCE_IMU_GYRO_Z_MDPS: return "imu_gyro_z";
    case ACT_SOURCE_CHASSIS_FEEDBACK_VALID: return "feedback_valid";
    case ACT_SOURCE_LEFT_ACTUAL_RPM: return "left_rpm";
    case ACT_SOURCE_RIGHT_ACTUAL_RPM: return "right_rpm";
    case ACT_SOURCE_LEFT_DISTANCE_MM: return "left_distance";
    case ACT_SOURCE_RIGHT_DISTANCE_MM: return "right_distance";
    case ACT_SOURCE_AVERAGE_DISTANCE_MM: return "average_distance";
    case ACT_SOURCE_CHASSIS_INITIALIZED: return "chassis_initialized";
    case ACT_SOURCE_HEADING_MODE: return "heading_mode";
    case ACT_SOURCE_LINEFOLLOW_MODE: return "linefollow_mode";
    case ACT_SOURCE_APP_MODE: return "app_mode";
    case ACT_SOURCE_COUNT:
    default: return "?";
    }
}

const char *ActionCondition_CompareText(ActionCompareOp compare)
{
    switch (compare) {
    case ACT_COMPARE_EQ: return "eq";
    case ACT_COMPARE_NE: return "ne";
    case ACT_COMPARE_LT: return "lt";
    case ACT_COMPARE_LE: return "le";
    case ACT_COMPARE_GT: return "gt";
    case ACT_COMPARE_GE: return "ge";
    case ACT_COMPARE_CONTAINS: return "contains";
    case ACT_COMPARE_NOT_CONTAINS: return "not_contains";
    default: return "?";
    }
}

void ActionRunner_Init(void)
{
    g_state.count = 0U;
    g_state.current = 0U;
    g_state.running = false;
    g_state.last_success = false;
    g_state.seq_start_ms = 0U;
    g_state.instr_start_ms = 0U;
    g_state.instr_started = false;
    g_state.last_status = drivers::DRIVER_OK;
    g_state.result = ACT_RUN_IDLE;
    g_state.failure_reason = ACT_FAIL_NONE;
    g_state.failure_index = ACT_NEXT;
    ClearOutputTimer(&g_led2Timer);
    ClearOutputTimer(&g_led3Timer);
    ClearOutputTimer(&g_buzzerTimer);
    ResetConditionRuntime();
    ResetLoopRuntime();
    g_preserveRoadMotion = false;
    for (uint8_t i = 0U; i < kMaxInstrs; i++) {
        g_state.instrs[i].op = ACT_OP_NONE;
        g_state.instrs[i].condition_value = 0;
    }
}

drivers::DriverStatus ActionRunner_Clear(void)
{
    if (g_state.running) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    g_state.count = 0U;
    g_state.current = 0U;
    g_state.last_success = false;
    g_state.instr_started = false;
    g_state.last_status = drivers::DRIVER_OK;
    g_state.result = ACT_RUN_IDLE;
    g_state.failure_reason = ACT_FAIL_NONE;
    g_state.failure_index = ACT_NEXT;
    ResetConditionRuntime();
    ResetLoopRuntime();
    g_preserveRoadMotion = false;
    StopSequenceOutputs();
    return drivers::DRIVER_OK;
}

drivers::DriverStatus ActionRunner_AddInstr(ActionOp op,
                                            int32_t param1,
                                            int32_t param2,
                                            ActionCond until,
                                            uint8_t on_success,
                                            uint8_t on_timeout)
{
    if (g_state.running) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (g_state.count >= kMaxInstrs) {
        return drivers::DRIVER_ERROR;
    }
    Instr candidate = {
        op, param1, param2, until, 0, on_success, on_timeout
    };
    ActionValidationResult validation = { true, 0U, ACT_VALID_FIELD_NONE,
                                          ACT_VALID_OK };
    if (!ValidateInstr(&candidate, g_state.count, &validation)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    Instr *instr = &g_state.instrs[g_state.count];
    instr->op = op;
    instr->param1 = param1;
    instr->param2 = param2;
    instr->until = until;
    instr->condition_value = 0;
    instr->on_success = on_success;
    instr->on_timeout = on_timeout;
    g_state.count++;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus ActionRunner_AddRoadNav(
    int32_t route,
    int32_t rpm,
    int32_t timeout_ms,
    uint8_t on_success,
    uint8_t on_failure)
{
    if (g_state.running) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (g_state.count >= kMaxInstrs) {
        return drivers::DRIVER_ERROR;
    }
    Instr candidate = {
        ACT_OP_ROAD_NAV,
        rpm,
        timeout_ms,
        ACT_COND_IMMEDIATE,
        route,
        on_success,
        on_failure
    };
    ActionValidationResult validation = {
        true, 0U, ACT_VALID_FIELD_NONE, ACT_VALID_OK
    };
    if (!ValidateInstr(&candidate, g_state.count, &validation)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    g_state.instrs[g_state.count] = candidate;
    g_state.count++;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus ActionRunner_AddDmPosition(
    bool relative,
    int32_t target_mrad,
    int32_t max_velocity_mrad_s,
    int32_t timeout_ms,
    uint8_t on_success,
    uint8_t on_failure)
{
    if (g_state.running) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (g_state.count >= kMaxInstrs) {
        return drivers::DRIVER_ERROR;
    }
    Instr candidate = {
        ACT_OP_DM_POSITION,
        target_mrad,
        max_velocity_mrad_s,
        relative ? ACT_COND_DM_RELATIVE : ACT_COND_DM_ABSOLUTE,
        timeout_ms,
        on_success,
        on_failure
    };
    ActionValidationResult validation = {
        true, 0U, ACT_VALID_FIELD_NONE, ACT_VALID_OK
    };
    if (!ValidateInstr(&candidate, g_state.count, &validation)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    g_state.instrs[g_state.count] = candidate;
    g_state.count++;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus ActionRunner_AddCompareInstr(
    ActionOp op,
    int32_t param1,
    const ActionConditionConfig *condition,
    uint8_t on_success,
    uint8_t on_failure)
{
    if ((condition == 0) || !ActionCondition_IsCompareOp(op) ||
        (condition->source >= ACT_SOURCE_COUNT) ||
        (condition->compare > ACT_COMPARE_NOT_CONTAINS) ||
        ((condition->timeout_ms % kConditionTickMs) != 0U) ||
        ((condition->stable_ms % kConditionTickMs) != 0U) ||
        (condition->timeout_ms > 30000U) ||
        (condition->stable_ms > 1000U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if ((op == ACT_OP_CONDITION) &&
        (condition->mode == ACT_CONDITION_IMMEDIATE) &&
        ((condition->timeout_ms != 0U) ||
         (condition->stable_ms != 0U))) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (((op == ACT_OP_DRIVE_IF) || (op == ACT_OP_FOLLOW_IF) ||
         (condition->mode == ACT_CONDITION_WAIT)) &&
        (condition->timeout_ms == 0U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (g_state.running) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (g_state.count >= kMaxInstrs) {
        return drivers::DRIVER_ERROR;
    }
    const uint16_t timeout_ticks =
        static_cast<uint16_t>(condition->timeout_ms / kConditionTickMs);
    const uint16_t stable_ticks =
        static_cast<uint16_t>(condition->stable_ms / kConditionTickMs);
    uint16_t timing = static_cast<uint16_t>(
        (timeout_ticks & kConditionTimeoutMask) |
        ((stable_ticks << 10U) & kConditionStableMask));
    if ((op == ACT_OP_CONDITION) &&
        (condition->mode == ACT_CONDITION_WAIT)) {
        timing = static_cast<uint16_t>(timing | kConditionWaitMask);
    }
    const uint8_t condition_spec = static_cast<uint8_t>(
        (static_cast<uint8_t>(condition->source) << 3U) |
        static_cast<uint8_t>(condition->compare));
    Instr candidate = {
        op,
        param1,
        static_cast<int32_t>(timing),
        static_cast<ActionCond>(condition_spec),
        condition->value,
        on_success,
        on_failure
    };
    ActionValidationResult validation = {
        true, 0U, ACT_VALID_FIELD_NONE, ACT_VALID_OK
    };
    if (!ValidateInstr(&candidate, g_state.count, &validation)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    g_state.instrs[g_state.count] = candidate;
    g_state.count++;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus ActionRunner_Validate(ActionValidationResult *result)
{
    if (result != 0) {
        result->valid = true;
        result->index = ACT_NEXT;
        result->field = ACT_VALID_FIELD_NONE;
        result->reason = ACT_VALID_OK;
    }
    if (g_state.count == 0U) {
        SetValidationError(result, ACT_NEXT, ACT_VALID_FIELD_TABLE,
                           ACT_VALID_EMPTY);
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (g_state.count > kMaxInstrs) {
        SetValidationError(result, ACT_NEXT, ACT_VALID_FIELD_TABLE,
                           ACT_VALID_TOO_MANY);
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    for (uint8_t i = 0U; i < g_state.count; i++) {
        const Instr *instr = &g_state.instrs[i];
        if (!ValidateInstr(instr, i, result)) {
            return drivers::DRIVER_ERROR_INVALID_ARG;
        }
        if (((instr->op == ACT_OP_LOOP) &&
             (instr->on_success == ACT_NEXT)) ||
            ((instr->on_success != ACT_NEXT) &&
             (instr->on_success >= g_state.count))) {
            SetValidationError(result, i, ACT_VALID_FIELD_ON_SUCCESS,
                               ACT_VALID_BAD_TARGET);
            return drivers::DRIVER_ERROR_INVALID_ARG;
        }
        if (((instr->op == ACT_OP_LOOP) &&
             (instr->on_timeout == ACT_NEXT)) ||
            ((instr->on_timeout != ACT_NEXT) &&
             (instr->on_timeout >= g_state.count))) {
            SetValidationError(result, i, ACT_VALID_FIELD_ON_TIMEOUT,
                               ACT_VALID_BAD_TARGET);
            return drivers::DRIVER_ERROR_INVALID_ARG;
        }
    }
    return drivers::DRIVER_OK;
}

drivers::DriverStatus ActionRunner_ValidateCompetition(
    ActionValidationResult *result)
{
    const drivers::DriverStatus status = ActionRunner_Validate(result);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    for (uint8_t i = 0U; i < g_state.count; i++) {
        const Instr *instr = &g_state.instrs[i];
        if (!ActionCondition_IsCompareOp(instr->op)) {
            continue;
        }
        ActionConditionConfig condition;
        if (!ActionCondition_Decode(instr, &condition)) {
            SetValidationError(result, i, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return drivers::DRIVER_ERROR_INVALID_ARG;
        }
        if ((condition.source == ACT_SOURCE_BUTTON2_LEVEL) ||
            (condition.source == ACT_SOURCE_BUTTON2_PRESSED)) {
            SetValidationError(result, i, ACT_VALID_FIELD_CONDITION,
                               ACT_VALID_WRONG_CONDITION);
            return drivers::DRIVER_ERROR_INVALID_ARG;
        }
    }
    return drivers::DRIVER_OK;
}

drivers::DriverStatus ActionRunner_Start(void)
{
    if (g_state.running) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    StopSequenceOutputs();
    ResetLoopRuntime();
    ActionValidationResult validation;
    if (ActionRunner_Validate(&validation) != drivers::DRIVER_OK) {
        g_state.last_status = drivers::DRIVER_ERROR_INVALID_ARG;
        g_state.result = ACT_RUN_INVALID;
        g_state.failure_reason = ACT_FAIL_INVALID;
        g_state.failure_index = validation.index;
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (services::Fault_HasFault()) {
        g_state.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        g_state.result = ACT_RUN_FAULT;
        g_state.failure_reason = ACT_FAIL_FAULT;
        g_state.failure_index = ACT_NEXT;
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    g_state.current = 0U;
    g_state.running = true;
    g_state.last_success = false;
    g_state.last_status = drivers::DRIVER_OK;
    g_state.result = ACT_RUN_RUNNING;
    g_state.failure_reason = ACT_FAIL_NONE;
    g_state.failure_index = ACT_NEXT;
    g_state.seq_start_ms = services::Time_Millis();
    g_state.instr_start_ms = 0U;
    g_state.instr_started = false;
    ResetConditionRuntime();
    g_preserveRoadMotion = false;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus ActionRunner_Cancel(void)
{
    g_state.result = ACT_RUN_CANCELLED;
    g_state.failure_reason = ACT_FAIL_CANCELLED;
    g_state.failure_index = g_state.running ? g_state.current : ACT_NEXT;
    g_state.last_status = drivers::DRIVER_OK;
    AbortSequence();
    return drivers::DRIVER_OK;
}

void ActionRunner_Update(void)
{
    UpdateSequenceOutputs();
    if (!g_state.running) {
        return;
    }

    const uint32_t now = services::Time_Millis();

    if (services::Fault_HasFault()) {
        g_state.result = ACT_RUN_FAULT;
        g_state.failure_reason = ACT_FAIL_FAULT;
        g_state.failure_index = g_state.current;
        g_state.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        AbortSequence();
        return;
    }
    if ((now - g_state.seq_start_ms) > kSequenceTimeoutMs) {
        g_state.result = ACT_RUN_TIMEOUT;
        g_state.failure_reason = ACT_FAIL_SEQUENCE_TIMEOUT;
        g_state.failure_index = g_state.current;
        g_state.last_status = drivers::DRIVER_ERROR_TIMEOUT;
        AbortSequence();
        return;
    }
    if (g_state.current >= g_state.count) {
        FinishSequence();
        return;
    }

    const Instr *instr = &g_state.instrs[g_state.current];
    if (g_preserveRoadMotion &&
        (instr->op != ACT_OP_ROAD_NAV) &&
        (instr->op != ACT_OP_LOOP)) {
        StopAll();
    }
    if (instr->op == ACT_OP_END) {
        FinishSequence();
        return;
    }

    if (!g_state.instr_started) {
        if (ActionCondition_IsCompareOp(instr->op)) {
            PrepareCondition(instr);
        }
        if (!StartOp(instr)) {
            g_state.last_success = false;
            g_state.failure_reason = ACT_FAIL_START;
            g_state.failure_index = g_state.current;
            g_state.last_status = drivers::DRIVER_ERROR;
            StopAll();
            StopSequenceOutputs();
            Goto(instr->on_timeout, false);
            return;
        }
        g_state.instr_start_ms = now;
        g_state.instr_started = true;
        if (instr->op == ACT_OP_ROAD_NAV) {
            g_preserveRoadMotion = false;
        }
    }

    if (instr->op == ACT_OP_LOOP) {
        const uint8_t loop_index = g_state.current;
        uint16_t *iteration = &g_loopIterations[loop_index];
        if (!g_preserveRoadMotion) {
            StopAll();
        }
        g_state.last_success = true;
        g_state.failure_reason = ACT_FAIL_NONE;
        g_state.failure_index = ACT_NEXT;
        g_state.last_status = drivers::DRIVER_OK;
        if (*iteration < static_cast<uint16_t>(instr->param1)) {
            (*iteration)++;
            Goto(instr->on_success, true);
        } else {
            *iteration = 0U;
            Goto(instr->on_timeout, true);
        }
        StopPreservedRoadMotionForCurrent();
        return;
    }

    const InstrResult r = EvalInstr(instr, now);
    if (r == INSTR_RUNNING) {
        return;
    }
    if (r == INSTR_FAULT) {
        g_state.result = ACT_RUN_FAULT;
        g_state.last_success = false;
        g_state.failure_reason = ACT_FAIL_FAULT;
        g_state.failure_index = g_state.current;
        g_state.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        AbortSequence();
        return;
    }
    if (r == INSTR_SUCCESS) {
        if (instr->op == ACT_OP_ROAD_NAV) {
            g_preserveRoadMotion = true;
        } else {
            StopAll();
        }
        g_state.last_success = true;
        g_state.failure_reason = ACT_FAIL_NONE;
        g_state.failure_index = ACT_NEXT;
        g_state.last_status = drivers::DRIVER_OK;
        Goto(instr->on_success, true);
        StopPreservedRoadMotionForCurrent();
    } else {
        StopAll();
        g_state.last_success = false;
        if (r == INSTR_ROUTE_UNAVAILABLE) {
            g_state.failure_reason = ACT_FAIL_ROUTE_UNAVAILABLE;
            g_state.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        } else if (r == INSTR_ROUTE_REACQUIRE_FAILED) {
            g_state.failure_reason = ACT_FAIL_ROUTE_REACQUIRE_FAILED;
            g_state.last_status = drivers::DRIVER_ERROR_TIMEOUT;
        } else if (r == INSTR_FALSE) {
            g_state.failure_reason = ACT_FAIL_CONDITION_FALSE;
            g_state.last_status = drivers::DRIVER_OK;
        } else if (r == INSTR_UNAVAILABLE) {
            g_state.failure_reason = ACT_FAIL_CONDITION_UNAVAILABLE;
            g_state.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        } else if (ActionCondition_IsCompareOp(instr->op)) {
            g_state.failure_reason = ACT_FAIL_CONDITION_TIMEOUT;
            g_state.last_status = drivers::DRIVER_ERROR_TIMEOUT;
        } else {
            g_state.failure_reason = ACT_FAIL_INSTR_TIMEOUT;
            g_state.last_status = drivers::DRIVER_ERROR_TIMEOUT;
        }
        g_state.failure_index = g_state.current;
        Goto(instr->on_timeout, false);
    }
}

const ActionRunnerState *ActionRunner_GetState(void)
{
    return &g_state;
}

} /* namespace app */
