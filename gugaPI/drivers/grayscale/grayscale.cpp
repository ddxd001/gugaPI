#include "drivers/grayscale/grayscale.h"

namespace drivers {
namespace {

void DelayCycles(uint32_t cycles)
{
    for (volatile uint32_t i = 0U; i < cycles; i++) {
    }
}

void SelectChannel(const GrayscaleConfig *cfg, uint8_t channel)
{
    for (uint8_t bit = 0U; bit < 3U; bit++) {
        const uint32_t mask = cfg->sel[bit].pin;
        if (((channel >> bit) & 0x01U) != 0U) {
            DL_GPIO_setPins(cfg->sel[bit].port, mask);
        } else {
            DL_GPIO_clearPins(cfg->sel[bit].port, mask);
        }
    }
}

} /* namespace */

DriverStatus Grayscale_Init(GrayscaleContext *ctx, const GrayscaleConfig *config)
{
    if ((ctx == 0) || (config == 0) || (config->adc == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }
    for (uint8_t i = 0U; i < 3U; i++) {
        if ((config->sel[i].port == 0) || (config->sel[i].pin == 0U)) {
            return DRIVER_ERROR_INVALID_ARG;
        }
    }

    ctx->config = config;

    /* powerDownMode is MANUAL: enable the ADC and allow conversions. */
    DL_ADC12_enablePower(config->adc);
    DelayCycles(1000U);
    DL_ADC12_enableConversions(config->adc);

    /* Start from channel 0 so the mux output is defined. */
    SelectChannel(config, 0U);

    ctx->initialized = true;
    return DRIVER_OK;
}

bool Grayscale_IsReady(const GrayscaleContext *ctx)
{
    return (ctx != 0) && ctx->initialized;
}

DriverStatus Grayscale_ReadChannel(GrayscaleContext *ctx,
                                   uint8_t channel,
                                   uint16_t *raw)
{
    if ((ctx == 0) || (!ctx->initialized) || (ctx->config == 0)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if ((raw == 0) || (channel >= GRAYSCALE_CHANNEL_COUNT)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const GrayscaleConfig *cfg = ctx->config;

    SelectChannel(cfg, channel);
    DelayCycles(cfg->settle_cycles);

    DL_ADC12_clearInterruptStatus(cfg->adc, cfg->result_loaded_mask);
    DL_ADC12_enableConversions(cfg->adc);
    DL_ADC12_startConversion(cfg->adc);

    /* One-shot mode: poll the ADC interrupt index until MEM0 result is loaded.
     * Reading IIDX also clears the flag. IE for MEM0_RESULT_LOADED is enabled
     * in SysConfig; NVIC is left disabled so no CPU interrupt fires. */
    uint32_t timeout = cfg->timeout_iterations;
    while (DL_ADC12_getPendingInterrupt(cfg->adc) !=
           DL_ADC12_IIDX_MEM0_RESULT_LOADED) {
        if (timeout == 0U) {
            DL_ADC12_stopConversion(cfg->adc);
            return DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    *raw = DL_ADC12_getMemResult(cfg->adc, cfg->mem_idx);
    return DRIVER_OK;
}

DriverStatus Grayscale_ReadAll(GrayscaleContext *ctx, uint16_t out[8])
{
    if (out == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    for (uint8_t ch = 0U; ch < GRAYSCALE_CHANNEL_COUNT; ch++) {
        const DriverStatus status = Grayscale_ReadChannel(ctx, ch, &out[ch]);
        if (status != DRIVER_OK) {
            return status;
        }
    }
    return DRIVER_OK;
}

} /* namespace drivers */
