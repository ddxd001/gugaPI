#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "drivers/dm_g6220/dm_g6220.h"

namespace {

int32_t Abs(int32_t value)
{
    return value < 0 ? -value : value;
}

} /* namespace */

int main()
{
    drivers::DmG6220Context context = {};
    const drivers::DmG6220Config config = { 0x001U, 0x000U, 0x01U };
    assert(drivers::DmG6220_Init(&context, &config) == drivers::DRIVER_OK);

    drivers::CanFrame frame = {};
    assert(drivers::DmG6220_PrepareSpecial(
               &context, drivers::DM_G6220_COMMAND_ENABLE, &frame) ==
           drivers::DRIVER_OK);
    assert(frame.id == 0x001U);
    assert(!frame.extended);
    assert(frame.length == 8U);
    for (uint8_t index = 0U; index < 7U; index++) {
        assert(frame.data[index] == 0xFFU);
    }
    assert(frame.data[7] == 0xFCU);

    assert(drivers::DmG6220_PrepareMit(
               &context, 0, 0, 0, 0, 0, &frame) == drivers::DRIVER_OK);
    const uint8_t expected_middle[8] =
        { 0x80U, 0x00U, 0x80U, 0x00U, 0x00U, 0x00U, 0x08U, 0x00U };
    for (uint8_t index = 0U; index < 8U; index++) {
        assert(frame.data[index] == expected_middle[index]);
    }

    assert(drivers::DmG6220_PrepareMit(
               &context, 999999, -999999, 999999, 999999, -999999,
               &frame) == drivers::DRIVER_OK);
    assert(frame.data[0] == 0xFFU);
    assert(frame.data[1] == 0xFFU);
    assert(frame.data[2] == 0x00U);
    assert((frame.data[3] & 0xF0U) == 0x00U);
    assert((frame.data[3] & 0x0FU) == 0x0FU);
    assert(frame.data[4] == 0xFFU);
    assert(frame.data[5] == 0xFFU);
    assert((frame.data[6] & 0xF0U) == 0xF0U);
    assert((frame.data[6] & 0x0FU) == 0x00U);
    assert(frame.data[7] == 0x00U);

    drivers::CanFrame feedback = {};
    feedback.id = 0x000U;
    feedback.length = 8U;
    feedback.data[0] = 0x31U;
    feedback.data[1] = 0x80U;
    feedback.data[2] = 0x00U;
    feedback.data[3] = 0x80U;
    feedback.data[4] = 0x08U;
    feedback.data[5] = 0x00U;
    feedback.data[6] = 42U;
    feedback.data[7] = 39U;
    assert(drivers::DmG6220_ProcessFrame(&context, &feedback, 1234U) ==
           drivers::DM_G6220_FRAME_FEEDBACK);
    const drivers::DmG6220Feedback *decoded =
        drivers::DmG6220_GetFeedback(&context);
    assert(decoded != 0);
    assert(decoded->valid);
    assert(decoded->motor_id == 1U);
    assert(decoded->state == 3U);
    assert(Abs(decoded->position_mrad) <= 1);
    assert(Abs(decoded->velocity_mrad_s) <= 12);
    assert(Abs(decoded->torque_mnm) <= 3);
    assert(decoded->mos_temperature_c == 42U);
    assert(decoded->coil_temperature_c == 39U);
    assert(decoded->last_update_ms == 1234U);

    feedback.id = 0x001U;
    assert(drivers::DmG6220_ProcessFrame(&context, &feedback, 1235U) ==
           drivers::DM_G6220_FRAME_NOT_FOR_DEVICE);
    feedback.id = 0x000U;
    feedback.length = 7U;
    assert(drivers::DmG6220_ProcessFrame(&context, &feedback, 1235U) ==
           drivers::DM_G6220_FRAME_INVALID);
    feedback.length = 8U;
    feedback.data[0] = 0x32U;
    assert(drivers::DmG6220_ProcessFrame(&context, &feedback, 1235U) ==
           drivers::DM_G6220_FRAME_INVALID);

    puts("dm g6220 protocol ok");
    return 0;
}
