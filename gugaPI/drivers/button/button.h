#ifndef DRIVERS_BUTTON_BUTTON_H_
#define DRIVERS_BUTTON_BUTTON_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

enum ButtonEvent : uint32_t {
    BUTTON_EVENT_NONE          = 0U,
    BUTTON_EVENT_PRESSED       = (1U << 0U),
    BUTTON_EVENT_RELEASED      = (1U << 1U),
    BUTTON_EVENT_SHORT_PRESSED = (1U << 2U),
    BUTTON_EVENT_LONG_PRESSED  = (1U << 3U),
    BUTTON_EVENT_ALL           = BUTTON_EVENT_PRESSED |
                                 BUTTON_EVENT_RELEASED |
                                 BUTTON_EVENT_SHORT_PRESSED |
                                 BUTTON_EVENT_LONG_PRESSED
};

struct ButtonConfig {
    GPIO_Regs *port;
    uint32_t pin;
    bool active_low;
    uint32_t debounce_ms;
    uint32_t long_press_ms;
};

struct ButtonContext {
    const ButtonConfig *config;
    bool initialized;
    bool raw_pressed;
    bool debounced_pressed;
    bool last_raw_pressed;
    bool press_timing_active;
    bool long_press_reported;
    uint32_t last_change_ms;
    uint32_t press_start_ms;
    uint32_t press_duration_ms;
    uint32_t pending_events;
    uint32_t generated_events;
};

DriverStatus Button_Init(ButtonContext *ctx, const ButtonConfig *config);
DriverStatus Button_Deinit(ButtonContext *ctx);
DriverStatus Button_Update(ButtonContext *ctx, uint32_t now_ms);
DriverStatus Button_ReadRaw(ButtonContext *ctx, bool *pressed);
bool Button_IsReady(const ButtonContext *ctx);
bool Button_IsPressed(const ButtonContext *ctx);
uint32_t Button_PeekEvents(const ButtonContext *ctx);
/* Events raised by the most recent Button_Update(). Unlike TakeEvents(),
 * this diagnostic snapshot never consumes the pending event bits. */
uint32_t Button_GetGeneratedEvents(const ButtonContext *ctx);
uint32_t Button_TakeEvents(ButtonContext *ctx, uint32_t event_mask);
uint32_t Button_GetPressDurationMs(const ButtonContext *ctx);
bool Button_WasPressed(ButtonContext *ctx);
bool Button_WasReleased(ButtonContext *ctx);
bool Button_WasShortPressed(ButtonContext *ctx);
bool Button_WasLongPressed(ButtonContext *ctx);

} /* namespace drivers */

#endif /* DRIVERS_BUTTON_BUTTON_H_ */
