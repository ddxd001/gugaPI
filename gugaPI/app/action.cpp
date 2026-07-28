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
    StopAll();
    StopSequenceOutputs();
}

void AbortSequence(void)
{
    g_state.running = false;
    g_state.last_success = false;
    StopAll();
    StopSequenceOutputs();
}

/* Jump after an instruction completes. success path: ACT_NEXT = next instr,
 * else goto index. timeout path: ACT_NEXT = abort, else goto index (recovery). */
void Goto(uint8_t target, bool success)
{
    g_state.instr_start_ms = 0U;
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

} /* namespace */

void ActionRunner_Init(void)
{
    g_state.count = 0U;
    g_state.current = 0U;
    g_state.running = false;
    g_state.last_success = false;
    g_state.seq_start_ms = 0U;
    g_state.instr_start_ms = 0U;
    g_state.last_status = drivers::DRIVER_OK;
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
    if ((op <= ACT_OP_NONE) || (op > ACT_OP_BUZZER_TOGGLE)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if ((op == ACT_OP_DRIVE_MM) &&
        ((param1 == 0) || (param1 < -10000) || (param1 > 10000) ||
         (param2 <= 0) || (param2 > 1000) ||
         (until != ACT_COND_DISTANCE_REACHED))) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    const bool led_op = (op >= ACT_OP_LED_ON) &&
                        (op <= ACT_OP_LED_TOGGLE);
    const bool buzzer_op = (op >= ACT_OP_BUZZER_ON) &&
                           (op <= ACT_OP_BUZZER_TOGGLE);
    const bool output_off = (op == ACT_OP_LED_OFF) ||
                            (op == ACT_OP_BUZZER_OFF);
    if (led_op &&
        (((param1 != 0) && (param1 != 2) && (param1 != 3)) ||
         (param2 < 0) || (param2 > 30000) ||
         ((param2 > 0) && (param2 < 50)) ||
         (output_off && (param2 != 0)) ||
         (until != ACT_COND_IMMEDIATE))) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (buzzer_op &&
        ((param1 != 0) || (param2 < 0) || (param2 > 30000) ||
         ((param2 > 0) && (param2 < 50)) ||
         (output_off && (param2 != 0)) ||
         (until != ACT_COND_IMMEDIATE))) {
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

drivers::DriverStatus ActionRunner_Start(void)
{
    if (!FEATURE_ENABLE_DIFFERENTIAL_CHASSIS) {
        StopAll();
        return drivers::DRIVER_ERROR_UNSUPPORTED;
    }
    if (g_state.running) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    StopSequenceOutputs();
    if (g_state.count == 0U) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (services::Fault_HasFault()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    g_state.current = 0U;
    g_state.running = true;
    g_state.last_success = false;
    g_state.seq_start_ms = services::Time_Millis();
    g_state.instr_start_ms = 0U;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus ActionRunner_Cancel(void)
{
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
        AbortSequence();
        return;
    }
    if ((now - g_state.seq_start_ms) > kSequenceTimeoutMs) {
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

    if (g_state.instr_start_ms == 0U) {
        if (!StartOp(instr)) {
            StopAll();
            StopSequenceOutputs();
            Goto(instr->on_timeout, false);
            return;
        }
        g_state.instr_start_ms = now;
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
        Goto(instr->on_timeout, false);
    }
}

const ActionRunnerState *ActionRunner_GetState(void)
{
    return &g_state;
}

} /* namespace app */
