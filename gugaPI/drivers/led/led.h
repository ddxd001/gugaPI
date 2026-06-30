#ifndef DRIVERS_LED_LED_H_
#define DRIVERS_LED_LED_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

struct LedConfig {
    GPIO_Regs *port;
    uint32_t pin;
    bool active_low;
    bool initial_on;
};

struct LedContext {
    const LedConfig *config;
    bool initialized;
    bool is_on;
};

DriverStatus Led_Init(LedContext *ctx, const LedConfig *config);
DriverStatus Led_Deinit(LedContext *ctx);
DriverStatus Led_Set(LedContext *ctx, bool on);
DriverStatus Led_On(LedContext *ctx);
DriverStatus Led_Off(LedContext *ctx);
DriverStatus Led_Toggle(LedContext *ctx);
bool Led_IsReady(const LedContext *ctx);
bool Led_IsOn(const LedContext *ctx);

} /* namespace drivers */

#endif /* DRIVERS_LED_LED_H_ */