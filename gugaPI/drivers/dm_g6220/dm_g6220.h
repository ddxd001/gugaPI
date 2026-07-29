#ifndef DRIVERS_DM_G6220_DM_G6220_H_
#define DRIVERS_DM_G6220_DM_G6220_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/can/can.h"
#include "drivers/common/driver_status.h"

namespace drivers {

static const int32_t DM_G6220_POSITION_LIMIT_MRAD = 12500;
static const int32_t DM_G6220_VELOCITY_LIMIT_MRAD_S = 45000;
static const int32_t DM_G6220_TORQUE_LIMIT_MNM = 10000;
static const int32_t DM_G6220_KP_LIMIT_MILLI = 500000;
static const int32_t DM_G6220_KD_LIMIT_MILLI = 5000;

enum DmG6220Command : uint8_t {
    DM_G6220_COMMAND_CLEAR_ERROR = 0xFBU,
    DM_G6220_COMMAND_ENABLE = 0xFCU,
    DM_G6220_COMMAND_DISABLE = 0xFDU,
    DM_G6220_COMMAND_SET_ZERO = 0xFEU
};

enum DmG6220FrameType : uint8_t {
    DM_G6220_FRAME_NOT_FOR_DEVICE = 0U,
    DM_G6220_FRAME_FEEDBACK,
    DM_G6220_FRAME_INVALID
};

struct DmG6220Config {
    uint16_t can_id;
    uint16_t master_id;
    uint8_t motor_id;
};

struct DmG6220Feedback {
    bool valid;
    uint8_t motor_id;
    uint8_t state;
    int32_t position_mrad;
    int32_t velocity_mrad_s;
    int32_t torque_mnm;
    uint8_t mos_temperature_c;
    uint8_t coil_temperature_c;
    uint32_t last_update_ms;
    uint32_t count;
    uint32_t invalid_count;
};

struct DmG6220Context {
    DmG6220Config config;
    DmG6220Feedback feedback;
    bool initialized;
};

DriverStatus DmG6220_Init(DmG6220Context *context,
                          const DmG6220Config *config);
DriverStatus DmG6220_PrepareSpecial(const DmG6220Context *context,
                                    DmG6220Command command,
                                    CanFrame *frame);
DriverStatus DmG6220_PrepareMit(const DmG6220Context *context,
                                int32_t position_mrad,
                                int32_t velocity_mrad_s,
                                int32_t kp_milli,
                                int32_t kd_milli,
                                int32_t torque_mnm,
                                CanFrame *frame);
DmG6220FrameType DmG6220_ProcessFrame(DmG6220Context *context,
                                      const CanFrame *frame,
                                      uint32_t now_ms);
const DmG6220Feedback *DmG6220_GetFeedback(
    const DmG6220Context *context);

} /* namespace drivers */

#endif /* DRIVERS_DM_G6220_DM_G6220_H_ */
