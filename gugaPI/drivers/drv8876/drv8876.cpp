#include "drivers/drv8876/drv8876.h"

namespace drivers {
namespace {

static const uint32_t kDirectionDeadtimeMs = 1U;

uint32_t DutyToCompare(const Drv8876Config *config, uint16_t duty_q8)
{
    const uint32_t maximum = 100U * DRV8876_DUTY_Q8_SCALE;
    if (duty_q8 > maximum) {
        duty_q8 = static_cast<uint16_t>(maximum);
    }

    const uint64_t scaled =
        static_cast<uint64_t>(config->pwm_period_counts) * duty_q8 +
        (maximum / 2U);
    const uint32_t active_counts = static_cast<uint32_t>(scaled / maximum);
    return static_cast<uint32_t>(config->pwm_period_counts) - active_counts;
}

void SetDuty(Drv8876Context *ctx, uint16_t duty_q8)
{
    DL_TimerG_setCaptureCompareValue(ctx->config->pwm_timer,
                                     DutyToCompare(ctx->config, duty_q8),
                                     ctx->config->pwm_index);
    ctx->duty_q8 = duty_q8;
}

void SetPhaseLevel(Drv8876Context *ctx, Drv8876PhaseLevel phase_level)
{
    if (phase_level == DRV8876_PHASE_HIGH) {
        DL_GPIO_setPins(ctx->config->phase_port, ctx->config->phase_pin);
    } else {
        DL_GPIO_clearPins(ctx->config->phase_port, ctx->config->phase_pin);
    }
    ctx->phase_level = phase_level;
    ctx->direction_valid = true;
}

} /* namespace */

DriverStatus Drv8876_Init(Drv8876Context *ctx,
                          const Drv8876Config *config)
{
    if ((ctx == 0) || (config == 0) || (config->pwm_timer == 0) ||
        (config->phase_port == 0) || (config->phase_pin == 0U) ||
        (config->sleep_port == 0) || (config->sleep_pin == 0U) ||
        (config->pwm_period_counts == 0U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->config = config;
    ctx->wake_start_ms = 0U;
    ctx->direction_change_start_ms = 0U;
    ctx->duty_q8 = 0U;
    ctx->phase_level = DRV8876_PHASE_LOW;
    ctx->pending_phase_level = DRV8876_PHASE_LOW;
    ctx->initialized = true;
    ctx->awake = false;
    ctx->direction_valid = false;
    ctx->direction_change_pending = false;
    return Drv8876_Sleep(ctx);
}

DriverStatus Drv8876_Sleep(Drv8876Context *ctx)
{
    if (!Drv8876_IsReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    SetDuty(ctx, 0U);
    DL_GPIO_clearPins(ctx->config->sleep_port, ctx->config->sleep_pin);
    ctx->awake = false;
    ctx->wake_start_ms = 0U;
    ctx->direction_change_pending = false;
    return DRIVER_OK;
}

DriverStatus Drv8876_Brake(Drv8876Context *ctx, uint32_t now_ms)
{
    if (!Drv8876_IsReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    SetDuty(ctx, 0U);
    ctx->direction_change_pending = false;
    if (!ctx->awake) {
        DL_GPIO_setPins(ctx->config->sleep_port, ctx->config->sleep_pin);
        ctx->awake = true;
        ctx->wake_start_ms = now_ms;
    }
    return DRIVER_OK;
}

DriverStatus Drv8876_Run(Drv8876Context *ctx,
                         Drv8876PhaseLevel phase_level,
                         uint16_t duty_q8,
                         uint32_t now_ms)
{
    if (!Drv8876_IsReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if ((phase_level != DRV8876_PHASE_LOW) &&
        (phase_level != DRV8876_PHASE_HIGH)) {
        return DRIVER_ERROR_INVALID_ARG;
    }
    const uint32_t maximum = 100U * DRV8876_DUTY_Q8_SCALE;
    if (duty_q8 > maximum) {
        return DRIVER_ERROR_INVALID_ARG;
    }
    if (duty_q8 == 0U) {
        return Drv8876_Brake(ctx, now_ms);
    }

    if ((!ctx->direction_valid) || (ctx->phase_level != phase_level)) {
        if (!ctx->awake) {
            /* The bridge is high impedance, so PH can be selected before the
             * non-blocking wake sequence begins. */
            SetPhaseLevel(ctx, phase_level);
            ctx->direction_change_pending = false;
        } else {
            /* A compare-register write does not prove that the physical PWM
             * pin is already low. Hold zero duty across a full millisecond
             * before changing PH. */
            if ((!ctx->direction_change_pending) ||
                (ctx->pending_phase_level != phase_level)) {
                SetDuty(ctx, 0U);
                ctx->pending_phase_level = phase_level;
                ctx->direction_change_start_ms = now_ms;
                ctx->direction_change_pending = true;
                return DRIVER_ERROR_BUSY;
            }
            SetDuty(ctx, 0U);
            if ((now_ms - ctx->direction_change_start_ms) <
                kDirectionDeadtimeMs) {
                return DRIVER_ERROR_BUSY;
            }
            SetPhaseLevel(ctx, ctx->pending_phase_level);
            ctx->direction_change_pending = false;
        }
    } else {
        ctx->direction_change_pending = false;
    }

    if (!ctx->awake) {
        SetDuty(ctx, 0U);
        DL_GPIO_setPins(ctx->config->sleep_port, ctx->config->sleep_pin);
        ctx->awake = true;
        ctx->wake_start_ms = now_ms;
        return DRIVER_ERROR_BUSY;
    }
    if ((now_ms - ctx->wake_start_ms) < ctx->config->wake_delay_ms) {
        SetDuty(ctx, 0U);
        return DRIVER_ERROR_BUSY;
    }

    SetDuty(ctx, duty_q8);
    return DRIVER_OK;
}

bool Drv8876_IsReady(const Drv8876Context *ctx)
{
    return (ctx != 0) && ctx->initialized && (ctx->config != 0);
}

bool Drv8876_IsAwake(const Drv8876Context *ctx)
{
    return Drv8876_IsReady(ctx) && ctx->awake;
}

uint16_t Drv8876_GetDutyQ8(const Drv8876Context *ctx)
{
    return Drv8876_IsReady(ctx) ? ctx->duty_q8 : 0U;
}

} /* namespace drivers */
