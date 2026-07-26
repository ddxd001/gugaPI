#ifndef APP_APP_MAIN_H_
#define APP_APP_MAIN_H_

#include "app/app_state.h"
#include "drivers/common/driver_status.h"

namespace app {

void App_Init(void);
void App_Run(void);
const AppState *App_GetState(void);

enum CompetitionResult {
    COMP_RESULT_NONE = 0,
    COMP_RESULT_DONE,
    COMP_RESULT_FAILED,
    COMP_RESULT_STOPPED,
    COMP_RESULT_LOAD_ERROR
};

struct CompetitionState {
    uint8_t selected_slot;
    bool slot_valid;
    bool any_valid_slot;
    uint8_t instruction_count;
    CompetitionResult result;
    drivers::DriverStatus last_status;
};

/* Competition mode: ARMED (safe selection) -> RUNNING -> ARMED.
 * Start loads the selected FRAM sequence and verifies its CRC.
 * Stop cancels the sequence, stops the chassis, and returns to ARMED. */
const CompetitionState *App_CompetitionGetState(void);
drivers::DriverStatus App_CompetitionSelect(uint8_t slot);
drivers::DriverStatus App_CompetitionRefreshSelection(void);
drivers::DriverStatus App_CompetitionArm(void);
drivers::DriverStatus App_CompetitionStart(void);
drivers::DriverStatus App_CompetitionStop(void);

} /* namespace app */

#endif /* APP_APP_MAIN_H_ */
