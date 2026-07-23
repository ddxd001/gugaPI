#ifndef APP_ACTION_H_
#define APP_ACTION_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace app {

enum ActionType {
    ACTION_NONE = 0,
    ACTION_DRIVE,        /* timed straight drive with heading hold */
    ACTION_TURN,         /* turn by relative angle */
    ACTION_WAIT,         /* timed wait (motors stopped) */
    ACTION_STOP,         /* stop chassis + heading */
    ACTION_LINE_FOLLOW   /* timed line-follow drive */
};

enum ActionStatus {
    ACTION_PENDING = 0,
    ACTION_RUNNING,
    ACTION_DONE,
    ACTION_FAILED
};

struct Action {
    ActionType type;
    int32_t param1;       /* DRIVE: base_rpm; TURN: delta_deg; LINE_FOLLOW: base_rpm */
    uint32_t param2;      /* DRIVE/WAIT/LINE_FOLLOW: duration_ms */
    ActionStatus status;
    uint32_t start_ms;
};

struct ActionRunnerState {
    Action actions[16];
    uint8_t count;
    uint8_t current;
    bool running;
    ActionStatus overall;   /* ACTION_DONE on completion, ACTION_FAILED on abort */
    uint32_t seq_start_ms;
};

/* Sequential action executor. Builds a list of actions (drive/turn/wait/stop)
 * and runs them in order, advancing on success, aborting (stop chassis) on any
 * failure/timeout/fault. Built on the heading + chassis primitives; never
 * bypasses the chassis lease/watchdog. */
void ActionRunner_Init(void);
drivers::DriverStatus ActionRunner_Clear(void);
drivers::DriverStatus ActionRunner_AddDrive(int32_t rpm, uint32_t duration_ms);
drivers::DriverStatus ActionRunner_AddTurn(int32_t delta_deg);
drivers::DriverStatus ActionRunner_AddWait(uint32_t duration_ms);
drivers::DriverStatus ActionRunner_AddStop(void);
drivers::DriverStatus ActionRunner_AddLineFollow(int32_t base_rpm,
                                                 uint32_t duration_ms);
drivers::DriverStatus ActionRunner_Start(void);
drivers::DriverStatus ActionRunner_Cancel(void);
void ActionRunner_Update(void);
const ActionRunnerState *ActionRunner_GetState(void);

} /* namespace app */

#endif /* APP_ACTION_H_ */
