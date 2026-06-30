#include "drivers/button/button.h"

namespace drivers {

namespace {

bool Button_ReadRawPressed(const ButtonConfig *config)
{
    const bool pin_high = ((DL_GPIO_readPins(config->port, config->pin) &
                            config->pin) != 0U);

    return config->active_low ? !pin_high : pin_high;
}

} /* namespace */

DriverStatus Button_Init(ButtonContext *ctx, const ButtonConfig *config)
{
    if ((ctx == 0) || (config == 0) || (config->port == 0) ||
        (config->pin == 0U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const bool raw_pressed = Button_ReadRawPressed(config);

    ctx->config = config;
    ctx->initialized = true;
    ctx->raw_pressed = raw_pressed;
    ctx->debounced_pressed = raw_pressed;
    ctx->last_raw_pressed = raw_pressed;
    ctx->pressed_event = false;
    ctx->released_event = false;
    ctx->last_change_ms = 0U;

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
    ctx->pressed_event = false;
    ctx->released_event = false;
    ctx->last_change_ms = 0U;

    return DRIVER_OK;
}

DriverStatus Button_Update(ButtonContext *ctx, uint32_t now_ms)
{
    if ((ctx == 0) || (!ctx->initialized) || (ctx->config == 0)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    const bool raw_pressed = Button_ReadRawPressed(ctx->config);

    ctx->raw_pressed = raw_pressed;

    if (raw_pressed != ctx->last_raw_pressed) {
        ctx->last_raw_pressed = raw_pressed;
        ctx->last_change_ms = now_ms;
    }

    if ((raw_pressed != ctx->debounced_pressed) &&
        ((uint32_t) (now_ms - ctx->last_change_ms) >=
         ctx->config->debounce_ms)) {
        ctx->debounced_pressed = raw_pressed;

        if (raw_pressed) {
            ctx->pressed_event = true;
        } else {
            ctx->released_event = true;
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

bool Button_WasPressed(ButtonContext *ctx)
{
    if (!Button_IsReady(ctx) || !ctx->pressed_event) {
        return false;
    }

    ctx->pressed_event = false;
    return true;
}

bool Button_WasReleased(ButtonContext *ctx)
{
    if (!Button_IsReady(ctx) || !ctx->released_event) {
        return false;
    }

    ctx->released_event = false;
    return true;
}

} /* namespace drivers */
