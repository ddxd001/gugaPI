#ifndef DRIVERS_BUZZER_BUZZER_H_
#define DRIVERS_BUZZER_BUZZER_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

struct BuzzerConfig {
    GPIO_Regs *port;
    uint32_t pin;
    bool active_high;
    bool initial_on;
};

struct BuzzerContext {
    const BuzzerConfig *config;
    bool initialized;
    bool is_on;
};

DriverStatus Buzzer_Init(BuzzerContext *ctx, const BuzzerConfig *config);
DriverStatus Buzzer_Deinit(BuzzerContext *ctx);
DriverStatus Buzzer_Set(BuzzerContext *ctx, bool on);
DriverStatus Buzzer_On(BuzzerContext *ctx);
DriverStatus Buzzer_Off(BuzzerContext *ctx);
DriverStatus Buzzer_Toggle(BuzzerContext *ctx);
bool Buzzer_IsReady(const BuzzerContext *ctx);
bool Buzzer_IsOn(const BuzzerContext *ctx);

} /* namespace drivers */

#endif /* DRIVERS_BUZZER_BUZZER_H_ */