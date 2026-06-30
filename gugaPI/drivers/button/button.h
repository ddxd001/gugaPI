#ifndef DRIVERS_BUTTON_BUTTON_H_
#define DRIVERS_BUTTON_BUTTON_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

struct ButtonConfig {
    GPIO_Regs *port;
    uint32_t pin;
    bool active_low;
    uint32_t debounce_ms;
};

struct ButtonContext {
    const ButtonConfig *config;
    bool initialized;
    bool raw_pressed;
    bool debounced_pressed;
    bool last_raw_pressed;
    bool pressed_event;
    bool released_event;
    uint32_t last_change_ms;
};

DriverStatus Button_Init(ButtonContext *ctx, const ButtonConfig *config);
DriverStatus Button_Deinit(ButtonContext *ctx);
DriverStatus Button_Update(ButtonContext *ctx, uint32_t now_ms);
DriverStatus Button_ReadRaw(ButtonContext *ctx, bool *pressed);
bool Button_IsReady(const ButtonContext *ctx);
bool Button_IsPressed(const ButtonContext *ctx);
bool Button_WasPressed(ButtonContext *ctx);
bool Button_WasReleased(ButtonContext *ctx);

} /* namespace drivers */

#endif /* DRIVERS_BUTTON_BUTTON_H_ */
