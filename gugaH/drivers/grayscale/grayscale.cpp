#include "drivers/grayscale/grayscale.h"

#include "services/time.h"

namespace drivers {
namespace {

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

DriverStatus ValidateRead(const GrayscaleContext *ctx,
                          uint8_t channel,
                          const uint16_t *raw)
{
    if ((ctx == 0) || (!ctx->initialized) || (ctx->config == 0)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if ((raw == 0) || (channel >= GRAYSCALE_CHANNEL_COUNT)) {
        return DRIVER_ERROR_INVALID_ARG;
    }
    return DRIVER_OK;
}

void WaitForSelectedChannel(const GrayscaleContext *ctx)
{
    const uint32_t elapsed_us =
        services::Time_Micros() - ctx->selected_at_us;
    if (elapsed_us < ctx->config->settle_us) {
        services::Time_DelayUs(ctx->config->settle_us - elapsed_us);
    }
}

DriverStatus ReadAdcResultPolling(GrayscaleContext *ctx, uint16_t *raw)
{
    const GrayscaleConfig *cfg = ctx->config;

    DL_ADC12_clearInterruptStatus(cfg->adc, cfg->result_loaded_mask);
    DL_ADC12_enableConversions(cfg->adc);
    DL_ADC12_startConversion(cfg->adc);

    /* Used only by explicit Shell/debug reads while the ADC NVIC line is
     * temporarily disabled. Reading IIDX also clears the interrupt flag. */
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

void RestoreAdcIrq(const GrayscaleConfig *cfg, bool was_enabled)
{
    if (was_enabled) {
        NVIC_EnableIRQ(cfg->irq);
    }
}

DriverStatus WaitForAsyncIdle(GrayscaleContext *ctx)
{
    const uint32_t start_us = services::Time_Micros();
    while (ctx->conversion_active) {
        if ((services::Time_Micros() - start_us) >=
            ctx->config->conversion_timeout_us) {
            const bool irq_was_enabled =
                (NVIC_GetEnableIRQ(ctx->config->irq) != 0U);
            NVIC_DisableIRQ(ctx->config->irq);

            /* Recheck after masking the IRQ; completion may have raced with
             * the timeout check. */
            if (!ctx->conversion_active) {
                RestoreAdcIrq(ctx->config, irq_was_enabled);
                return DRIVER_OK;
            }

            DL_ADC12_stopConversion(ctx->config->adc);
            DL_ADC12_clearInterruptStatus(ctx->config->adc,
                                          ctx->config->result_loaded_mask);
            ctx->conversion_active = false;
            ctx->result_ready = false;
            NVIC_ClearPendingIRQ(ctx->config->irq);
            RestoreAdcIrq(ctx->config, irq_was_enabled);
            return DRIVER_ERROR_TIMEOUT;
        }
    }
    return DRIVER_OK;
}

DriverStatus ReadChannelBlocking(GrayscaleContext *ctx,
                                 uint8_t channel,
                                 uint16_t *raw,
                                 bool restore_selection)
{
    DriverStatus status = ValidateRead(ctx, channel, raw);
    if (status != DRIVER_OK) {
        return status;
    }

    status = WaitForAsyncIdle(ctx);
    if (status != DRIVER_OK) {
        return status;
    }

    const bool restore_valid = ctx->selected_valid;
    const uint8_t restore_channel = ctx->selected_channel;
    const bool irq_was_enabled =
        (NVIC_GetEnableIRQ(ctx->config->irq) != 0U);
    NVIC_DisableIRQ(ctx->config->irq);

    status = Grayscale_PrepareChannel(ctx, channel);
    if (status == DRIVER_OK) {
        WaitForSelectedChannel(ctx);
        status = ReadAdcResultPolling(ctx, raw);
    }

    if (restore_selection && restore_valid &&
        (restore_channel != channel)) {
        const DriverStatus restore_status =
            Grayscale_PrepareChannel(ctx, restore_channel);
        if ((status == DRIVER_OK) && (restore_status != DRIVER_OK)) {
            status = restore_status;
        }
    }

    NVIC_ClearPendingIRQ(ctx->config->irq);
    RestoreAdcIrq(ctx->config, irq_was_enabled);
    return status;
}

} /* namespace */

DriverStatus Grayscale_Init(GrayscaleContext *ctx, const GrayscaleConfig *config)
{
    if ((ctx == 0) || (config == 0) || (config->adc == 0) ||
        (config->conversion_timeout_us == 0U) ||
        (config->timeout_iterations == 0U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }
    for (uint8_t i = 0U; i < 3U; i++) {
        if ((config->sel[i].port == 0) || (config->sel[i].pin == 0U)) {
            return DRIVER_ERROR_INVALID_ARG;
        }
    }

    ctx->config = config;
    ctx->initialized = false;
    ctx->selected_valid = false;
    ctx->conversion_active = false;
    ctx->conversion_channel = 0U;
    ctx->conversion_next_channel = 0U;
    ctx->conversion_started_us = 0U;
    ctx->result_ready = false;
    ctx->result_channel = 0U;
    ctx->result_raw = 0U;

    /* powerDownMode is MANUAL: enable the ADC and allow conversions. */
    DL_ADC12_enablePower(config->adc);
    services::Time_DelayUs(25U);
    DL_ADC12_enableConversions(config->adc);

    /* Preselect channel 0. Its settling interval overlaps later startup work. */
    SelectChannel(config, 0U);
    ctx->selected_channel = 0U;
    ctx->selected_at_us = services::Time_Micros();
    ctx->selected_valid = true;
    ctx->initialized = true;
    return DRIVER_OK;
}

bool Grayscale_IsReady(const GrayscaleContext *ctx)
{
    return (ctx != 0) && ctx->initialized;
}

DriverStatus Grayscale_PrepareChannel(GrayscaleContext *ctx, uint8_t channel)
{
    if ((ctx == 0) || (!ctx->initialized) || (ctx->config == 0)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (channel >= GRAYSCALE_CHANNEL_COUNT) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    if ((!ctx->selected_valid) || (ctx->selected_channel != channel)) {
        SelectChannel(ctx->config, channel);
        ctx->selected_channel = channel;
        ctx->selected_at_us = services::Time_Micros();
        ctx->selected_valid = true;
    }
    return DRIVER_OK;
}

DriverStatus Grayscale_ReadPreparedChannel(GrayscaleContext *ctx,
                                           uint8_t channel,
                                           uint16_t *raw)
{
    return ReadChannelBlocking(ctx, channel, raw, false);
}

DriverStatus Grayscale_StartPreparedConversion(GrayscaleContext *ctx,
                                               uint8_t channel)
{
    uint8_t next_channel = static_cast<uint8_t>(channel + 1U);
    if (next_channel >= GRAYSCALE_CHANNEL_COUNT) {
        next_channel = 0U;
    }
    return Grayscale_StartPreparedConversion(ctx, channel, next_channel);
}

DriverStatus Grayscale_StartPreparedConversion(GrayscaleContext *ctx,
                                               uint8_t channel,
                                               uint8_t next_channel)
{
    if ((ctx == 0) || (!ctx->initialized) || (ctx->config == 0)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if ((channel >= GRAYSCALE_CHANNEL_COUNT) ||
        (next_channel >= GRAYSCALE_CHANNEL_COUNT)) {
        return DRIVER_ERROR_INVALID_ARG;
    }
    if (ctx->conversion_active || ctx->result_ready) {
        return DRIVER_ERROR_BUSY;
    }

    DriverStatus status = Grayscale_PrepareChannel(ctx, channel);
    if (status != DRIVER_OK) {
        return status;
    }

    const uint32_t now_us = services::Time_Micros();
    if ((now_us - ctx->selected_at_us) < ctx->config->settle_us) {
        return DRIVER_ERROR_BUSY;
    }

    DL_ADC12_clearInterruptStatus(ctx->config->adc,
                                  ctx->config->result_loaded_mask);
    NVIC_ClearPendingIRQ(ctx->config->irq);
    ctx->conversion_channel = channel;
    ctx->conversion_next_channel = next_channel;
    ctx->conversion_started_us = now_us;
    ctx->conversion_active = true;
    DL_ADC12_enableConversions(ctx->config->adc);
    DL_ADC12_startConversion(ctx->config->adc);
    return DRIVER_OK;
}

DriverStatus Grayscale_TakeCompletedConversion(GrayscaleContext *ctx,
                                               uint8_t *channel,
                                               uint16_t *raw)
{
    if ((ctx == 0) || (!ctx->initialized) || (ctx->config == 0)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if ((channel == 0) || (raw == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const bool irq_was_enabled =
        (NVIC_GetEnableIRQ(ctx->config->irq) != 0U);
    NVIC_DisableIRQ(ctx->config->irq);

    DriverStatus status = DRIVER_ERROR_BUSY;
    if (ctx->result_ready) {
        *channel = ctx->result_channel;
        *raw = ctx->result_raw;
        ctx->result_ready = false;
        status = DRIVER_OK;
    } else if (ctx->conversion_active &&
               ((services::Time_Micros() - ctx->conversion_started_us) >=
                ctx->config->conversion_timeout_us)) {
        DL_ADC12_stopConversion(ctx->config->adc);
        DL_ADC12_clearInterruptStatus(ctx->config->adc,
                                      ctx->config->result_loaded_mask);
        ctx->conversion_active = false;
        NVIC_ClearPendingIRQ(ctx->config->irq);
        status = DRIVER_ERROR_TIMEOUT;
    }

    RestoreAdcIrq(ctx->config, irq_was_enabled);
    return status;
}

void Grayscale_HandleInterrupt(GrayscaleContext *ctx)
{
    if ((ctx == 0) || (!ctx->initialized) || (ctx->config == 0)) {
        return;
    }

    if (DL_ADC12_getPendingInterrupt(ctx->config->adc) !=
        DL_ADC12_IIDX_MEM0_RESULT_LOADED) {
        return;
    }
    if (!ctx->conversion_active) {
        return;
    }

    ctx->result_raw = DL_ADC12_getMemResult(ctx->config->adc,
                                           ctx->config->mem_idx);
    ctx->result_channel = ctx->conversion_channel;

    SelectChannel(ctx->config, ctx->conversion_next_channel);
    ctx->selected_channel = ctx->conversion_next_channel;
    ctx->selected_at_us = services::Time_Micros();
    ctx->selected_valid = true;
    ctx->conversion_active = false;
    ctx->result_ready = true;
}

DriverStatus Grayscale_ReadChannel(GrayscaleContext *ctx,
                                   uint8_t channel,
                                   uint16_t *raw)
{
    /* Shell/debug reads wait only for an in-flight asynchronous sample, then
     * poll their own conversion with the ADC IRQ masked. Preserve both the
     * queued periodic result and the channel prepared by its completion ISR. */
    return ReadChannelBlocking(ctx, channel, raw, true);
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
