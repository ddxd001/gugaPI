#ifndef APP_APP_MAIN_H_
#define APP_APP_MAIN_H_

#include "app/app_state.h"
#include "drivers/common/driver_status.h"

namespace app {

void App_Init(void);
void App_Run(void);
const AppState *App_GetState(void);

/* Competition mode: ARMED (safe idle) → RUNNING (ActionRunner active) → ARMED.
 * Start requires a pre-loaded sequence (run add) and no fault.
 * Stop cancels the sequence and returns to ARMED.
 * Button 1 toggles start/stop in competition modes.
 * Arm enters competition mode from development (dev-running → armed). */
drivers::DriverStatus App_CompetitionArm(void);
drivers::DriverStatus App_CompetitionStart(void);
drivers::DriverStatus App_CompetitionStop(void);

} /* namespace app */

#endif /* APP_APP_MAIN_H_ */