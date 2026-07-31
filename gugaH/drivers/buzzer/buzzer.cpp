#include "drivers/buzzer/buzzer.h"

namespace drivers {

static void Buzzer_DrivePin(const BuzzerConfig *config, bool on)
{
    const bool drive_high = config->active_high ? on : !on;

    if (drive_high) {
        DL_GPIO_setPins(config->port, config->pin);
    } else {
        DL_GPIO_clearPins(config->port, config->pin);
    }
}

DriverStatus Buzzer_Init(BuzzerContext *ctx, const BuzzerConfig *config)
{
    if ((ctx == 0) || (config == 0) || (config->port == 0) ||
        (config->pin == 0U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->config = config;
    ctx->initialized = true;
    ctx->is_on = config->initial_on;
    Buzzer_DrivePin(config, config->initial_on);

    return DRIVER_OK;
}

DriverStatus Buzzer_Deinit(BuzzerContext *ctx)
{
    if (ctx == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    if (ctx->initialized && (ctx->config != 0)) {
        Buzzer_DrivePin(ctx->config, false);
    }

    ctx->config = 0;
    ctx->initialized = false;
    ctx->is_on = false;

    return DRIVER_OK;
}

DriverStatus Buzzer_Set(BuzzerContext *ctx, bool on)
{
    if ((ctx == 0) || (!ctx->initialized) || (ctx->config == 0)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    Buzzer_DrivePin(ctx->config, on);
    ctx->is_on = on;

    return DRIVER_OK;
}

DriverStatus Buzzer_On(BuzzerContext *ctx)
{
    return Buzzer_Set(ctx, true);
}

DriverStatus Buzzer_Off(BuzzerContext *ctx)
{
    return Buzzer_Set(ctx, false);
}

DriverStatus Buzzer_Toggle(BuzzerContext *ctx)
{
    if ((ctx == 0) || (!ctx->initialized) || (ctx->config == 0)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    return Buzzer_Set(ctx, !ctx->is_on);
}

bool Buzzer_IsReady(const BuzzerContext *ctx)
{
    return ((ctx != 0) && ctx->initialized && (ctx->config != 0));
}

bool Buzzer_IsOn(const BuzzerContext *ctx)
{
    if (!Buzzer_IsReady(ctx)) {
        return false;
    }

    return ctx->is_on;
}

} /* namespace drivers */