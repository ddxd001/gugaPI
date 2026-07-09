#ifndef APP_APP_SHELL_H_
#define APP_APP_SHELL_H_

#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace app {

void AppShell_RegisterCommands(void);
drivers::DriverStatus AppShell_EnableIna219Oled(uint32_t period_ms);
drivers::DriverStatus AppShell_EnableGy931Oled(uint32_t period_ms);
drivers::DriverStatus AppShell_EnableGrayOled(uint32_t period_ms);

} /* namespace app */

#endif /* APP_APP_SHELL_H_ */
