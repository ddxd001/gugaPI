#ifndef APP_APP_INA219_H_
#define APP_APP_INA219_H_

#include <stdbool.h>
#include <stdint.h>

#include "app/ina219_protection.h"

namespace app {

struct AppIna219Data {
    bool initialized;
    Ina219ProtectionState protection;
    uint32_t last_attempt_ms;
    uint32_t last_update_ms;
};

drivers::DriverStatus App_Ina219Init(void);
void App_Ina219Run(void);
drivers::DriverStatus App_Ina219ReloadConfig(void);
const AppIna219Data *App_Ina219GetData(void);
void App_Ina219ClearLatchedFaults(void);
bool App_Ina219MotionInhibitRequested(void);

} /* namespace app */

#endif /* APP_APP_INA219_H_ */
