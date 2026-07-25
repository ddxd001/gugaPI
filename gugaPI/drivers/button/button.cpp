#include "drivers/button/button.h"

namespace drivers {

namespace {

bool Button_ReadRawPressed(const ButtonConfig *config)
{
    const bool pin_high = ((DL_GPIO_readPins(config->port, config->pin) &
                            config->pin) != 0U);

    return config->active_low ? !pin_high : pin_high;
}

void Button_RaiseEvent(ButtonContext *ctx, uint32_t event)
{
    ctx->pending_events |= event;
    ctx->generated_events |= event;
}

} /* namespace */

DriverStatus Button_Init(ButtonContext *ctx, const ButtonConfig *config)
{
    if ((ctx == 0) || (config == 0) || (config->port == 0) ||
        (config->pin == 0U) || (config->long_press_ms == 0U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const bool raw_pressed = Button_ReadRawPressed(config);

    ctx->config = config;
    ctx->initialized = true;
    ctx->raw_pressed = raw_pressed;
    ctx->debounced_pressed = raw_pressed;
    ctx->last_raw_pressed = raw_pressed;
    ctx->press_timing_active = false;
    ctx->long_press_reported = false;
    ctx->last_change_ms = 0U;
    ctx->press_start_ms = 0U;
    ctx->press_duration_ms = 0U;
    ctx->pending_events = BUTTON_EVENT_NONE;
    ctx->generated_events = BUTTON_EVENT_NONE;

    return DRIVER_OK;
}

DriverStatus Button_Deinit(ButtonContext *ctx)
{
    if (ctx == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->config = 0;
    ctx->initialized = false;
    ctx->raw_pressed = false;
    ctx->debounced_pressed = false;
    ctx->last_raw_pressed = false;
    ctx->press_timing_active = false;
    ctx->long_press_reported = false;
    ctx->last_change_ms = 0U;
    ctx->press_start_ms = 0U;
    ctx->press_duration_ms = 0U;
    ctx->pending_events = BUTTON_EVENT_NONE;
    ctx->generated_events = BUTTON_EVENT_NONE;

    return DRIVER_OK;
}

DriverStatus Button_Update(ButtonContext *ctx, uint32_t now_ms)
{
    if ((ctx == 0) || (!ctx->initialized) || (ctx->config == 0)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    ctx->generated_events = BUTTON_EVENT_NONE;

    const bool raw_pressed = Button_ReadRawPressed(ctx->config);

    ctx->raw_pressed = raw_pressed;

    /* A button already held during initialization has no DOWN edge, but its
     * hold duration still starts at the first periodic update. */
    if (ctx->debounced_pressed && !ctx->press_timing_active) {
        ctx->press_timing_active = true;
        ctx->press_start_ms = now_ms;
        ctx->press_duration_ms = 0U;
        ctx->long_press_reported = false;
    }

    if (raw_pressed != ctx->last_raw_pressed) {
        ctx->last_raw_pressed = raw_pressed;
        ctx->last_change_ms = now_ms;
    }

    if ((raw_pressed != ctx->debounced_pressed) &&
        ((uint32_t) (now_ms - ctx->last_change_ms) >=
         ctx->config->debounce_ms)) {
        ctx->debounced_pressed = raw_pressed;

        if (raw_pressed) {
            ctx->press_timing_active = true;
            ctx->long_press_reported = false;
            ctx->press_start_ms = now_ms;
            ctx->press_duration_ms = 0U;
            Button_RaiseEvent(ctx, BUTTON_EVENT_PRESSED);
        } else {
            if (ctx->press_timing_active) {
                ctx->press_duration_ms = now_ms - ctx->press_start_ms;
            }
            Button_RaiseEvent(ctx, BUTTON_EVENT_RELEASED);
            if (!ctx->long_press_reported) {
                if (ctx->press_duration_ms >= ctx->config->long_press_ms) {
                    Button_RaiseEvent(ctx, BUTTON_EVENT_LONG_PRESSED);
                    ctx->long_press_reported = true;
                } else {
                    Button_RaiseEvent(ctx, BUTTON_EVENT_SHORT_PRESSED);
                }
            }
            ctx->press_timing_active = false;
        }
    }

    if (ctx->debounced_pressed && ctx->press_timing_active) {
        ctx->press_duration_ms = now_ms - ctx->press_start_ms;
        if ((!ctx->long_press_reported) &&
            (ctx->press_duration_ms >= ctx->config->long_press_ms)) {
            Button_RaiseEvent(ctx, BUTTON_EVENT_LONG_PRESSED);
            ctx->long_press_reported = true;
        }
    }

    return DRIVER_OK;
}

DriverStatus Button_ReadRaw(ButtonContext *ctx, bool *pressed)
{
    if ((ctx == 0) || (!ctx->initialized) || (ctx->config == 0)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    if (pressed == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    *pressed = Button_ReadRawPressed(ctx->config);
    return DRIVER_OK;
}

bool Button_IsReady(const ButtonContext *ctx)
{
    return ((ctx != 0) && ctx->initialized && (ctx->config != 0));
}

bool Button_IsPressed(const ButtonContext *ctx)
{
    if (!Button_IsReady(ctx)) {
        return false;
    }

    return ctx->debounced_pressed;
}

uint32_t Button_PeekEvents(const ButtonContext *ctx)
{
    if (!Button_IsReady(ctx)) {
        return BUTTON_EVENT_NONE;
    }

    return ctx->pending_events;
}

uint32_t Button_GetGeneratedEvents(const ButtonContext *ctx)
{
    if (!Button_IsReady(ctx)) {
        return BUTTON_EVENT_NONE;
    }

    return ctx->generated_events;
}

uint32_t Button_TakeEvents(ButtonContext *ctx, uint32_t event_mask)
{
    if (!Button_IsReady(ctx)) {
        return BUTTON_EVENT_NONE;
    }

    const uint32_t events = ctx->pending_events & event_mask;
    ctx->pending_events &= ~events;
    return events;
}

uint32_t Button_GetPressDurationMs(const ButtonContext *ctx)
{
    if (!Button_IsReady(ctx)) {
        return 0U;
    }

    return ctx->press_duration_ms;
}

bool Button_WasPressed(ButtonContext *ctx)
{
    return ((Button_TakeEvents(ctx, BUTTON_EVENT_PRESSED) &
             BUTTON_EVENT_PRESSED) != 0U);
}

bool Button_WasReleased(ButtonContext *ctx)
{
    return ((Button_TakeEvents(ctx, BUTTON_EVENT_RELEASED) &
             BUTTON_EVENT_RELEASED) != 0U);
}

bool Button_WasShortPressed(ButtonContext *ctx)
{
    return ((Button_TakeEvents(ctx, BUTTON_EVENT_SHORT_PRESSED) &
             BUTTON_EVENT_SHORT_PRESSED) != 0U);
}

bool Button_WasLongPressed(ButtonContext *ctx)
{
    return ((Button_TakeEvents(ctx, BUTTON_EVENT_LONG_PRESSED) &
             BUTTON_EVENT_LONG_PRESSED) != 0U);
}

} /* namespace drivers */
