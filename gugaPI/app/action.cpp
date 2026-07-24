#include "app/action.h"

#include "app/chassis.h"
#include "app/heading.h"
#include "app/linefollow.h"
#include "board/board_button.h"
#include "drivers/common/driver_status.h"
#include "services/fault.h"
#include "services/time.h"

namespace app {
namespace {

static const uint8_t kMaxInstrs = 16U;
static const uint32_t kSequenceTimeoutMs = 60000U;  /* whole-sequence cap */

ActionRunnerState g_state;

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

void FinishSequence(void)
{
    g_state.running = false;
    g_state.last_success = true;
    StopAll();
}

void AbortSequence(void)
{
    g_state.running = false;
    g_state.last_success = false;
    StopAll();
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
    case ACT_OP_WAIT:
        return true;
    case ACT_OP_STOP:
        StopAll();
        return true;
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
    if (g_state.running) {
        return drivers::DRIVER_ERROR_BUSY;
    }
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
