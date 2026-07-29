#ifndef APP_APP_LARGE_TIMER_H_
#define APP_APP_LARGE_TIMER_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace app {

struct AppLargeTimerState {
    bool visible;
    bool running;
    bool saturated;
    uint32_t elapsed_ms;
    uint16_t displayed_tenths;
    uint32_t display_updates;
    drivers::DriverStatus last_display_status;
};

void App_LargeTimerInit(void);
/* Start always begins a new count from 0:00.0 and claims the OLED. */
drivers::DriverStatus App_LargeTimerStart(void);
/* Stop freezes the elapsed value and keeps the large display visible. */
drivers::DriverStatus App_LargeTimerStop(void);
drivers::DriverStatus App_LargeTimerResume(void);
drivers::DriverStatus App_LargeTimerReset(void);
/* Hide releases the OLED and stops the timer without changing parameters. */
drivers::DriverStatus App_LargeTimerHide(void);
void App_LargeTimerUpdate(void);
bool App_LargeTimerOwnsDisplay(void);
const AppLargeTimerState *App_LargeTimerGetState(void);

} /* namespace app */

#endif /* APP_APP_LARGE_TIMER_H_ */
