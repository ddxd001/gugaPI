#ifndef APP_ACTION_H_
#define APP_ACTION_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace app {

/* Instruction-table interpreter: each instruction runs an action until a
 * condition (or safety timeout), then jumps to on_success / on_timeout. This
 * replaces the old linear timed-action sequencer with condition-driven
 * branching (follow-until-line-lost, turn-until-heading-reached, branch on
 * line/button, loop). */

enum ActionOp {
    ACT_OP_NONE = 0,
    ACT_OP_DRIVE,    /* drive straight (heading hold) until cond */
    ACT_OP_TURN,     /* turn to relative angle, until heading_reached */
    ACT_OP_FOLLOW,   /* line-follow until cond */
    ACT_OP_WAIT,      /* wait until cond (usually timeout) */
    ACT_OP_STOP,     /* stop chassis+heading+linefollow, immediate */
    ACT_OP_BRANCH,   /* no motion; jump on cond (true->on_success, false->on_timeout) */
    ACT_OP_END       /* finish the sequence (success) */
};

enum ActionCond {
    ACT_COND_TIMEOUT = 0,        /* complete when param2 ms elapsed */
    ACT_COND_HEADING_REACHED,   /* heading returned to IDLE (turn done) */
    ACT_COND_LINE_DETECTED,     /* grayscale sees the line */
    ACT_COND_LINE_LOST,         /* grayscale lost the line */
    ACT_COND_BUTTON,            /* button 1 pressed */
    ACT_COND_IMMEDIATE          /* always true (instant) */
};

/* on_success / on_timeout target. ACT_NEXT in on_success = next instruction;
 * ACT_NEXT in on_timeout = abort (fail). A real index 0..count-1 = goto. */
static const uint8_t ACT_NEXT = 0xFFU;

struct Instr {
    ActionOp op;
    int32_t param1;       /* DRIVE/FOLLOW: rpm; TURN: delta_deg */
    int32_t param2;       /* timeout / duration ms (safety) */
    ActionCond until;     /* success condition */
    uint8_t on_success;   /* ACT_NEXT or index */
    uint8_t on_timeout;   /* ACT_NEXT(=abort) or index */
};

struct ActionRunnerState {
    Instr instrs[16];
    uint8_t count;
    uint8_t current;
    bool running;
    bool last_success;    /* last instr result (for status) */
    uint32_t seq_start_ms;
    uint32_t instr_start_ms;
    drivers::DriverStatus last_status;
};

void ActionRunner_Init(void);
drivers::DriverStatus ActionRunner_Clear(void);
/* Append an instruction. op as text ("drive"/"turn"/...), until as text
 * ("timeout"/"heading_reached"/"line_detected"/"line_lost"/"button"/
 * "immediate"). on_success/on_timeout: index, or "next"/"abort" (=ACT_NEXT). */
drivers::DriverStatus ActionRunner_AddInstr(ActionOp op,
                                            int32_t param1,
                                            int32_t param2,
                                            ActionCond until,
                                            uint8_t on_success,
                                            uint8_t on_timeout);
drivers::DriverStatus ActionRunner_Start(void);
drivers::DriverStatus ActionRunner_Cancel(void);
void ActionRunner_Update(void);
const ActionRunnerState *ActionRunner_GetState(void);

} /* namespace app */

#endif /* APP_ACTION_H_ */
