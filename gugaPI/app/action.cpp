#include "app/action.h"

#include "app/chassis.h"
#include "app/heading.h"
#include "app/linefollow.h"
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
static const uint32_t kSequenceTimeoutMs = 60000U;  /* whole-sequence cap */

ActionRunnerState g_state;

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
    INSTR_TIMEOUT
};

/* Stop every motion primitive so no state bleeds into the next instruction
 * (fixes drive->wait continuing to drive, drive->follow dual-commanding). */
void StopAll(void)
{
    (void) Heading_Stop();
    (void) LF_Stop();
    (void) Chassis_Stop();
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
    StopSequenceOutputs();
}

void AbortSequence(void)
{
    g_state.running = false;
    g_state.last_success = false;
    if (g_state.result == ACT_RUN_RUNNING) {
        g_state.result = ACT_RUN_ABORTED;
    }
    StopAll();
    StopSequenceOutputs();
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

bool StartOp(const Instr *instr)
{
    switch (instr->op) {
    case ACT_OP_DRIVE:
        return (Heading_HoldStart(instr->param1) == drivers::DRIVER_OK);
    case ACT_OP_TURN:
        return (Heading_TurnStart(instr->param1) == drivers::DRIVER_OK);
    case ACT_OP_FOLLOW:
        return (LF_Start(instr->param1, instr->param2) == drivers::DRIVER_OK);
    case ACT_OP_DRIVE_MM:
        return (Heading_DistanceStart(instr->param1,
                                      instr->param2,
                                      0U) == drivers::DRIVER_OK);
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
    if (instr->op == ACT_OP_DRIVE_MM) {
        return EvalCond(ACT_COND_DISTANCE_REACHED)
            ? INSTR_SUCCESS
            : INSTR_RUNNING;
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

bool ValidateInstr(const Instr *instr,
                   uint8_t index,
                   ActionValidationResult *result)
{
    const int32_t max_rpm = static_cast<int32_t>(
        Chassis_GetState()->config.max_wheel_rpm);
    if ((instr->op <= ACT_OP_NONE) ||
        (instr->op > ACT_OP_BUZZER_TOGGLE)) {
        SetValidationError(result, index, ACT_VALID_FIELD_OP,
                           ACT_VALID_UNKNOWN_OP);
        return false;
    }
    if ((instr->until < ACT_COND_TIMEOUT) ||
        (instr->until > ACT_COND_DISTANCE_REACHED)) {
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
    }
    return true;
}

} /* namespace */

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
    for (uint8_t i = 0U; i < kMaxInstrs; i++) {
        g_state.instrs[i].op = ACT_OP_NONE;
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
    Instr candidate = { op, param1, param2, until, on_success, on_timeout };
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
    instr->on_success = on_success;
    instr->on_timeout = on_timeout;
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
        if ((instr->on_success != ACT_NEXT) &&
            (instr->on_success >= g_state.count)) {
            SetValidationError(result, i, ACT_VALID_FIELD_ON_SUCCESS,
                               ACT_VALID_BAD_TARGET);
            return drivers::DRIVER_ERROR_INVALID_ARG;
        }
        if ((instr->on_timeout != ACT_NEXT) &&
            (instr->on_timeout >= g_state.count)) {
            SetValidationError(result, i, ACT_VALID_FIELD_ON_TIMEOUT,
                               ACT_VALID_BAD_TARGET);
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
    if (instr->op == ACT_OP_END) {
        FinishSequence();
        return;
    }

    if (!g_state.instr_started) {
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
    }

    const InstrResult r = EvalInstr(instr, now);
    if (r == INSTR_RUNNING) {
        return;
    }
    StopAll();
    if (r == INSTR_SUCCESS) {
        g_state.last_success = true;
        Goto(instr->on_success, true);
    } else {
        g_state.last_success = false;
        g_state.failure_reason = ACT_FAIL_INSTR_TIMEOUT;
        g_state.failure_index = g_state.current;
        g_state.last_status = drivers::DRIVER_ERROR_TIMEOUT;
        Goto(instr->on_timeout, false);
    }
}

const ActionRunnerState *ActionRunner_GetState(void)
{
    return &g_state;
}

} /* namespace app */
