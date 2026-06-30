#include "drivers/led/led.h"

namespace drivers {

static void Led_DrivePin(const LedConfig *config, bool on)
{
    const bool drive_high = config->active_low ? !on : on;

    if (drive_high) {
        DL_GPIO_setPins(config->port, config->pin);
    } else {
        DL_GPIO_clearPins(config->port, config->pin);
    }
}

DriverStatus Led_Init(LedContext *ctx, const LedConfig *config)
{
    if ((ctx == 0) || (config == 0) || (config->port == 0) ||
        (config->pin == 0U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->config = config;
    ctx->initialized = true;
    ctx->is_on = config->initial_on;
    Led_DrivePin(config, config->initial_on);

    return DRIVER_OK;
}

DriverStatus Led_Deinit(LedContext *ctx)
{
    if (ctx == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    if (ctx->initialized && (ctx->config != 0)) {
        Led_DrivePin(ctx->config, false);
    }

    ctx->config = 0;
    ctx->initialized = false;
    ctx->is_on = false;

    return DRIVER_OK;
}

DriverStatus Led_Set(LedContext *ctx, bool on)
{
    if ((ctx == 0) || (!ctx->initialized) || (ctx->config == 0)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    Led_DrivePin(ctx->config, on);
    ctx->is_on = on;

    return DRIVER_OK;
}

DriverStatus Led_On(LedContext *ctx)
{
    return Led_Set(ctx, true);
}

DriverStatus Led_Off(LedContext *ctx)
{
    return Led_Set(ctx, false);
}

DriverStatus Led_Toggle(LedContext *ctx)
{
    if ((ctx == 0) || (!ctx->initialized) || (ctx->config == 0)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    return Led_Set(ctx, !ctx->is_on);
}

bool Led_IsReady(const LedContext *ctx)
{
    return ((ctx != 0) && ctx->initialized && (ctx->config != 0));
}

bool Led_IsOn(const LedContext *ctx)
{
    if (!Led_IsReady(ctx)) {
        return false;
    }

    return ctx->is_on;
}

} /* namespace drivers */