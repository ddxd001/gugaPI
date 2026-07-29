#include "drivers/dm_g6220/dm_g6220.h"

namespace drivers {
namespace {

bool IsReady(const DmG6220Context *context)
{
    return (context != 0) && context->initialized;
}

int32_t Clamp(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

uint32_t ToUnsigned(int32_t value,
                    int32_t minimum,
                    int32_t maximum,
                    uint8_t bits)
{
    value = Clamp(value, minimum, maximum);
    const uint32_t full_scale = (1UL << bits) - 1UL;
    const int64_t numerator =
        static_cast<int64_t>(value - minimum) * full_scale;
    const int32_t range = maximum - minimum;
    return static_cast<uint32_t>(
        (numerator + static_cast<int64_t>(range / 2)) / range);
}

int32_t FromUnsigned(uint32_t value,
                     int32_t minimum,
                     int32_t maximum,
                     uint8_t bits)
{
    const uint32_t full_scale = (1UL << bits) - 1UL;
    const int64_t numerator =
        static_cast<int64_t>(value) * (maximum - minimum);
    return minimum + static_cast<int32_t>(
        (numerator + static_cast<int64_t>(full_scale / 2U)) /
        full_scale);
}

} /* namespace */

DriverStatus DmG6220_Init(DmG6220Context *context,
                          const DmG6220Config *config)
{
    if ((context == 0) || (config == 0) ||
        (config->can_id > 0x7FFU) || (config->master_id > 0x7FFU) ||
        (config->motor_id == 0U) || (config->motor_id > 0x0FU)) {
        return DRIVER_ERROR_INVALID_ARG;
    }
    *context = {};
    context->config = *config;
    context->initialized = true;
    return DRIVER_OK;
}

DriverStatus DmG6220_PrepareSpecial(const DmG6220Context *context,
                                    DmG6220Command command,
                                    CanFrame *frame)
{
    if (!IsReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (frame == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }
    *frame = {};
    frame->id = context->config.can_id;
    frame->length = 8U;
    frame->extended = false;
    for (uint8_t i = 0U; i < 7U; i++) {
        frame->data[i] = 0xFFU;
    }
    frame->data[7] = static_cast<uint8_t>(command);
    return DRIVER_OK;
}

DriverStatus DmG6220_PrepareMit(const DmG6220Context *context,
                                int32_t position_mrad,
                                int32_t velocity_mrad_s,
                                int32_t kp_milli,
                                int32_t kd_milli,
                                int32_t torque_mnm,
                                CanFrame *frame)
{
    if (!IsReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (frame == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const uint32_t position = ToUnsigned(
        position_mrad, -DM_G6220_POSITION_LIMIT_MRAD,
        DM_G6220_POSITION_LIMIT_MRAD, 16U);
    const uint32_t velocity = ToUnsigned(
        velocity_mrad_s, -DM_G6220_VELOCITY_LIMIT_MRAD_S,
        DM_G6220_VELOCITY_LIMIT_MRAD_S, 12U);
    const uint32_t kp = ToUnsigned(
        kp_milli, 0, DM_G6220_KP_LIMIT_MILLI, 12U);
    const uint32_t kd = ToUnsigned(
        kd_milli, 0, DM_G6220_KD_LIMIT_MILLI, 12U);
    const uint32_t torque = ToUnsigned(
        torque_mnm, -DM_G6220_TORQUE_LIMIT_MNM,
        DM_G6220_TORQUE_LIMIT_MNM, 12U);

    *frame = {};
    frame->id = context->config.can_id;
    frame->length = 8U;
    frame->extended = false;
    frame->data[0] = static_cast<uint8_t>(position >> 8U);
    frame->data[1] = static_cast<uint8_t>(position);
    frame->data[2] = static_cast<uint8_t>(velocity >> 4U);
    frame->data[3] = static_cast<uint8_t>(
        ((velocity & 0x0FU) << 4U) | (kp >> 8U));
    frame->data[4] = static_cast<uint8_t>(kp);
    frame->data[5] = static_cast<uint8_t>(kd >> 4U);
    frame->data[6] = static_cast<uint8_t>(
        ((kd & 0x0FU) << 4U) | (torque >> 8U));
    frame->data[7] = static_cast<uint8_t>(torque);
    return DRIVER_OK;
}

DmG6220FrameType DmG6220_ProcessFrame(DmG6220Context *context,
                                      const CanFrame *frame,
                                      uint32_t now_ms)
{
    if ((!IsReady(context)) || (frame == 0)) {
        return DM_G6220_FRAME_INVALID;
    }
    if (frame->extended || (frame->id != context->config.master_id)) {
        return DM_G6220_FRAME_NOT_FOR_DEVICE;
    }
    if ((frame->length != 8U) ||
        ((frame->data[0] & 0x0FU) != context->config.motor_id)) {
        context->feedback.invalid_count++;
        return DM_G6220_FRAME_INVALID;
    }

    const uint32_t position =
        (static_cast<uint32_t>(frame->data[1]) << 8U) |
        frame->data[2];
    const uint32_t velocity =
        (static_cast<uint32_t>(frame->data[3]) << 4U) |
        (frame->data[4] >> 4U);
    const uint32_t torque =
        (static_cast<uint32_t>(frame->data[4] & 0x0FU) << 8U) |
        frame->data[5];

    DmG6220Feedback &feedback = context->feedback;
    feedback.valid = true;
    feedback.motor_id = frame->data[0] & 0x0FU;
    feedback.state = frame->data[0] >> 4U;
    feedback.position_mrad = FromUnsigned(
        position, -DM_G6220_POSITION_LIMIT_MRAD,
        DM_G6220_POSITION_LIMIT_MRAD, 16U);
    feedback.velocity_mrad_s = FromUnsigned(
        velocity, -DM_G6220_VELOCITY_LIMIT_MRAD_S,
        DM_G6220_VELOCITY_LIMIT_MRAD_S, 12U);
    feedback.torque_mnm = FromUnsigned(
        torque, -DM_G6220_TORQUE_LIMIT_MNM,
        DM_G6220_TORQUE_LIMIT_MNM, 12U);
    feedback.mos_temperature_c = frame->data[6];
    feedback.coil_temperature_c = frame->data[7];
    feedback.last_update_ms = now_ms;
    feedback.count++;
    return DM_G6220_FRAME_FEEDBACK;
}

const DmG6220Feedback *DmG6220_GetFeedback(
    const DmG6220Context *context)
{
    return IsReady(context) ? &context->feedback : 0;
}

} /* namespace drivers */
