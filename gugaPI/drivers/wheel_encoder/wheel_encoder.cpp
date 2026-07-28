#include "drivers/wheel_encoder/wheel_encoder.h"

#include <limits.h>

namespace drivers {
namespace {

static const uint32_t kCounterMask = 0xFFFFU;
static const uint8_t kWindowSamples = 3U;

int32_t SaturateInt32(int64_t value)
{
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return static_cast<int32_t>(value);
}

uint16_t ReadCounter(const WheelEncoderContext *ctx)
{
    return static_cast<uint16_t>(
        DL_TimerG_getTimerCount(ctx->config->timer) & kCounterMask);
}

uint8_t ReadState(const WheelEncoderContext *ctx)
{
    uint8_t state = 0U;
    if ((DL_GPIO_readPins(ctx->config->phase_a_port,
                          ctx->config->phase_a_pin) &
         ctx->config->phase_a_pin) != 0U) {
        state = static_cast<uint8_t>(state | 0x01U);
    }
    if ((DL_GPIO_readPins(ctx->config->phase_b_port,
                          ctx->config->phase_b_pin) &
         ctx->config->phase_b_pin) != 0U) {
        state = static_cast<uint8_t>(state | 0x02U);
    }
    return state;
}

void ClearWindow(WheelEncoderContext *ctx)
{
    ctx->counts_per_second = 0;
    ctx->window_delta_count = 0;
    ctx->window_elapsed_ms = 0U;
    ctx->window_index = 0U;
    ctx->window_count = 0U;
    for (uint8_t i = 0U; i < kWindowSamples; i++) {
        ctx->sample_delta[i] = 0;
        ctx->sample_elapsed_ms[i] = 0U;
    }
}

void SyncCounter(WheelEncoderContext *ctx)
{
    const uint16_t current = ReadCounter(ctx);
    const int16_t raw_delta =
        static_cast<int16_t>(current - ctx->last_hw_count);
    ctx->last_hw_count = current;
    ctx->count += static_cast<int32_t>(raw_delta) *
                  static_cast<int32_t>(ctx->config->count_sign);
}

void UpdateSpeed(WheelEncoderContext *ctx, uint32_t now_ms)
{
    if (!ctx->sample_initialized) {
        ctx->last_sample_count = ctx->count;
        ctx->last_sample_ms = now_ms;
        ctx->sample_initialized = true;
        return;
    }

    const uint32_t elapsed_ms = now_ms - ctx->last_sample_ms;
    if (elapsed_ms < ctx->config->sample_period_ms) {
        return;
    }

    const int64_t delta = ctx->count - ctx->last_sample_count;
    const uint8_t slot = ctx->window_index;
    if (ctx->window_count == kWindowSamples) {
        ctx->window_delta_count -= ctx->sample_delta[slot];
        ctx->window_elapsed_ms -= ctx->sample_elapsed_ms[slot];
    } else {
        ctx->window_count++;
    }

    ctx->sample_delta[slot] = delta;
    ctx->sample_elapsed_ms[slot] = elapsed_ms;
    ctx->window_delta_count += delta;
    ctx->window_elapsed_ms += elapsed_ms;
    ctx->window_index = static_cast<uint8_t>((slot + 1U) % kWindowSamples);

    int64_t scaled = ctx->window_delta_count * 1000LL;
    const int64_t divisor = ctx->window_elapsed_ms;
    if (scaled >= 0) {
        scaled += divisor / 2;
    } else {
        scaled -= divisor / 2;
    }
    ctx->counts_per_second = SaturateInt32(scaled / divisor);
    ctx->last_sample_count = ctx->count;
    ctx->last_sample_ms = now_ms;
}

} /* namespace */

DriverStatus WheelEncoder_Init(WheelEncoderContext *ctx,
                               const WheelEncoderConfig *config,
                               uint32_t now_ms)
{
    if ((ctx == 0) || (config == 0) || (config->timer == 0) ||
        (config->phase_a_port == 0) || (config->phase_a_pin == 0U) ||
        (config->phase_b_port == 0) || (config->phase_b_pin == 0U) ||
        ((config->count_sign != 1) && (config->count_sign != -1)) ||
        (config->sample_period_ms == 0U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->config = config;
    ctx->count = 0;
    ctx->last_sample_count = 0;
    ctx->last_sample_ms = now_ms;
    ctx->last_hw_count = 0U;
    ctx->state = 0U;
    ctx->sample_initialized = false;
    ctx->initialized = true;
    ClearWindow(ctx);

    DL_TimerG_stopCounter(config->timer);
    DL_TimerG_setTimerCount(config->timer, 0U);
    ctx->last_hw_count = ReadCounter(ctx);
    ctx->state = ReadState(ctx);
    DL_TimerG_startCounter(config->timer);
    return DRIVER_OK;
}

DriverStatus WheelEncoder_Process(WheelEncoderContext *ctx,
                                  uint32_t now_ms)
{
    if (!WheelEncoder_IsReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    SyncCounter(ctx);
    UpdateSpeed(ctx, now_ms);
    ctx->state = ReadState(ctx);
    return DRIVER_OK;
}

DriverStatus WheelEncoder_Reset(WheelEncoderContext *ctx, uint32_t now_ms)
{
    if (!WheelEncoder_IsReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    SyncCounter(ctx);
    ctx->count = 0;
    ctx->last_sample_count = 0;
    ctx->last_sample_ms = now_ms;
    ctx->sample_initialized = false;
    ClearWindow(ctx);
    return DRIVER_OK;
}

DriverStatus WheelEncoder_GetSnapshot(WheelEncoderContext *ctx,
                                      WheelEncoderSnapshot *snapshot)
{
    if (!WheelEncoder_IsReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (snapshot == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    SyncCounter(ctx);
    /* The public chassis API is intentionally kept at int32. Internal QEI
     * accumulation remains int64 so a long-running robot never invokes signed
     * overflow; only the externally reported lifetime count saturates. */
    snapshot->count = SaturateInt32(ctx->count);
    snapshot->counts_per_second = ctx->counts_per_second;
    snapshot->state = ctx->state;
    return DRIVER_OK;
}

bool WheelEncoder_IsReady(const WheelEncoderContext *ctx)
{
    return (ctx != 0) && ctx->initialized && (ctx->config != 0);
}

} /* namespace drivers */
