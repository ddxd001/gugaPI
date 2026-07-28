#include "drivers/jyme02_can/jyme02_can.h"

#include <limits.h>

namespace drivers {
namespace {

const uint8_t kMeasurementHeader = 0x55U;
const uint8_t kTemperatureHeader = 0x56U;
const uint8_t kRegisterHeader = 0x5FU;

uint16_t ReadUint16Le(const uint8_t *data)
{
    return static_cast<uint16_t>(
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8U));
}

int16_t ReadInt16Le(const uint8_t *data)
{
    return static_cast<int16_t>(ReadUint16Le(data));
}

int32_t ClampInt64ToInt32(int64_t value)
{
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return static_cast<int32_t>(value);
}

void ResetData(JYME02CanContext *context)
{
    context->data = {};
    context->data.initialized = true;
    context->data.address = context->config.address;
    context->data.sample_time_100us = context->config.sample_time_100us;
    context->pending_register = 0U;
    context->register_pending = false;
}

bool IsContextReady(const JYME02CanContext *context)
{
    return (context != 0) && context->data.initialized;
}

} /* namespace */

DriverStatus JYME02Can_Init(JYME02CanContext *context,
                            const JYME02CanConfig *config)
{
    if ((context == 0) || (config == 0) ||
        (config->address > 0x7FFU) ||
        (config->sample_time_100us == 0U) ||
        (config->stale_timeout_ms == 0U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    context->config = *config;
    ResetData(context);
    return DRIVER_OK;
}

DriverStatus JYME02Can_SetAddress(JYME02CanContext *context,
                                  uint16_t address)
{
    if (!IsContextReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (address > 0x7FFU) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    context->config.address = address;
    context->data.address = address;
    context->register_pending = false;
    return DRIVER_OK;
}

DriverStatus JYME02Can_SetSampleTime(JYME02CanContext *context,
                                     uint16_t sample_time_100us)
{
    if (!IsContextReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (sample_time_100us == 0U) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    context->config.sample_time_100us = sample_time_100us;
    context->data.sample_time_100us = sample_time_100us;
    return DRIVER_OK;
}

JYME02CanFrameType JYME02Can_ProcessFrame(JYME02CanContext *context,
                                          const CanFrame *frame,
                                          uint32_t now_ms)
{
    if ((!IsContextReady(context)) || (frame == 0)) {
        return JYME02_CAN_FRAME_INVALID;
    }
    if (frame->extended || (frame->id != context->config.address)) {
        return JYME02_CAN_FRAME_NOT_FOR_DEVICE;
    }
    if ((frame->length != 8U) || (frame->data[0] != 0x55U)) {
        context->data.invalid_count++;
        return JYME02_CAN_FRAME_INVALID;
    }

    if (frame->data[1] == kMeasurementHeader) {
        const uint16_t angle_raw = ReadUint16Le(&frame->data[2]);
        const int16_t velocity_raw = ReadInt16Le(&frame->data[4]);
        const int64_t velocity_numerator =
            static_cast<int64_t>(velocity_raw) * 360000LL * 10000LL;
        const int64_t velocity_denominator =
            32768LL * context->config.sample_time_100us;

        context->data.angle_raw = angle_raw;
        context->data.angular_velocity_raw = velocity_raw;
        context->data.revolutions = ReadInt16Le(&frame->data[6]);
        context->data.angle_mdeg = static_cast<uint32_t>(
            (static_cast<uint64_t>(angle_raw) * 360000ULL) / 32768ULL);
        context->data.angular_velocity_mdeg_s =
            ClampInt64ToInt32(velocity_numerator / velocity_denominator);
        context->data.last_measurement_ms = now_ms;
        context->data.measurement_count++;
        context->data.measurement_valid = true;
        return JYME02_CAN_FRAME_MEASUREMENT;
    }

    if (frame->data[1] == kTemperatureHeader) {
        context->data.temperature_raw = ReadInt16Le(&frame->data[2]);
        context->data.temperature_mdeg_c =
            static_cast<int32_t>(context->data.temperature_raw) * 10;
        context->data.last_temperature_ms = now_ms;
        context->data.temperature_count++;
        context->data.temperature_valid = true;
        return JYME02_CAN_FRAME_TEMPERATURE;
    }

    if (frame->data[1] == kRegisterHeader) {
        context->data.register_start = context->register_pending ?
            context->pending_register : 0xFFU;
        context->data.register_values[0] = ReadUint16Le(&frame->data[2]);
        context->data.register_values[1] = ReadUint16Le(&frame->data[4]);
        context->data.register_values[2] = ReadUint16Le(&frame->data[6]);
        context->data.register_count++;
        context->data.register_valid = true;
        context->register_pending = false;
        return JYME02_CAN_FRAME_REGISTER;
    }

    context->data.invalid_count++;
    return JYME02_CAN_FRAME_INVALID;
}

DriverStatus JYME02Can_PrepareReadRegister(JYME02CanContext *context,
                                           uint8_t register_address,
                                           CanFrame *frame)
{
    if (!IsContextReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (frame == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    *frame = {};
    frame->id = context->config.address;
    frame->length = 5U;
    frame->extended = false;
    frame->data[0] = 0xFFU;
    frame->data[1] = 0xAAU;
    frame->data[2] = 0x27U;
    frame->data[3] = register_address;
    frame->data[4] = 0x00U;
    context->pending_register = register_address;
    context->register_pending = true;
    return DRIVER_OK;
}

bool JYME02Can_IsMeasurementFresh(const JYME02CanContext *context,
                                  uint32_t now_ms)
{
    return IsContextReady(context) && context->data.measurement_valid &&
           ((uint32_t) (now_ms - context->data.last_measurement_ms) <=
            context->config.stale_timeout_ms);
}

bool JYME02Can_IsTemperatureFresh(const JYME02CanContext *context,
                                  uint32_t now_ms)
{
    return IsContextReady(context) && context->data.temperature_valid &&
           ((uint32_t) (now_ms - context->data.last_temperature_ms) <=
            context->config.stale_timeout_ms);
}

const JYME02CanData *JYME02Can_GetData(const JYME02CanContext *context)
{
    return IsContextReady(context) ? &context->data : 0;
}

void JYME02Can_ClearStatistics(JYME02CanContext *context)
{
    if (!IsContextReady(context)) {
        return;
    }

    const JYME02CanConfig config = context->config;
    context->config = config;
    ResetData(context);
}

} /* namespace drivers */
