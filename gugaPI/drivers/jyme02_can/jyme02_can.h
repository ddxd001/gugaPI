#ifndef DRIVERS_JYME02_CAN_JYME02_CAN_H_
#define DRIVERS_JYME02_CAN_JYME02_CAN_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/can/can.h"
#include "drivers/common/driver_status.h"

namespace drivers {

enum JYME02CanFrameType : uint8_t {
    JYME02_CAN_FRAME_NOT_FOR_DEVICE = 0U,
    JYME02_CAN_FRAME_MEASUREMENT,
    JYME02_CAN_FRAME_TEMPERATURE,
    JYME02_CAN_FRAME_REGISTER,
    JYME02_CAN_FRAME_INVALID
};

struct JYME02CanConfig {
    uint16_t address;
    uint16_t sample_time_100us;
    uint32_t stale_timeout_ms;
};

struct JYME02CanData {
    bool initialized;
    bool measurement_valid;
    bool temperature_valid;
    bool register_valid;
    uint16_t address;
    uint16_t sample_time_100us;
    uint16_t angle_raw;
    int16_t angular_velocity_raw;
    int16_t revolutions;
    int16_t temperature_raw;
    uint32_t angle_mdeg;
    int32_t angular_velocity_mdeg_s;
    int32_t temperature_mdeg_c;
    uint8_t register_start;
    uint16_t register_values[3];
    uint32_t last_measurement_ms;
    uint32_t last_temperature_ms;
    uint32_t measurement_count;
    uint32_t temperature_count;
    uint32_t register_count;
    uint32_t invalid_count;
};

struct JYME02CanContext {
    JYME02CanConfig config;
    JYME02CanData data;
    uint8_t pending_register;
    bool register_pending;
};

DriverStatus JYME02Can_Init(JYME02CanContext *context,
                            const JYME02CanConfig *config);
DriverStatus JYME02Can_SetAddress(JYME02CanContext *context,
                                  uint16_t address);
DriverStatus JYME02Can_SetSampleTime(JYME02CanContext *context,
                                     uint16_t sample_time_100us);
JYME02CanFrameType JYME02Can_ProcessFrame(JYME02CanContext *context,
                                          const CanFrame *frame,
                                          uint32_t now_ms);
DriverStatus JYME02Can_PrepareReadRegister(JYME02CanContext *context,
                                           uint8_t register_address,
                                           CanFrame *frame);
bool JYME02Can_IsMeasurementFresh(const JYME02CanContext *context,
                                  uint32_t now_ms);
bool JYME02Can_IsTemperatureFresh(const JYME02CanContext *context,
                                  uint32_t now_ms);
const JYME02CanData *JYME02Can_GetData(const JYME02CanContext *context);
void JYME02Can_ClearStatistics(JYME02CanContext *context);

} /* namespace drivers */

#endif /* DRIVERS_JYME02_CAN_JYME02_CAN_H_ */
