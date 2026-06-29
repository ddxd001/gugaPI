#include "drivers/template/template_driver.h"

namespace drivers {

DriverStatus TemplateDriver_Init(TemplateDriverContext *ctx,
                                 const TemplateDriverConfig *config)
{
    if ((ctx == 0) || (config == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->config = config;
    ctx->initialized = true;
    return DRIVER_OK;
}

DriverStatus TemplateDriver_Deinit(TemplateDriverContext *ctx)
{
    if (ctx == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->config = 0;
    ctx->initialized = false;
    return DRIVER_OK;
}

DriverStatus TemplateDriver_Update(TemplateDriverContext *ctx)
{
    if ((ctx == 0) || (!ctx->initialized)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    return DRIVER_OK;
}

bool TemplateDriver_IsReady(const TemplateDriverContext *ctx)
{
    return ((ctx != 0) && ctx->initialized);
}

} /* namespace drivers */