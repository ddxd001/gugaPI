#ifndef APP_CHASSIS_H_
#define APP_CHASSIS_H_

#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace app {

struct ChassisConfig {
    uint32_t wheel_radius_mm;
    uint32_t wheel_track_mm;
    uint32_t left_counts_per_rev;
    uint32_t right_counts_per_rev;
    uint16_t max_wheel_rpm;
};

struct ChassisWheelState {
    int32_t target_rpm;
    int32_t actual_rpm;
    int32_t encoder_count;
    int32_t encoder_counts_per_second;
    uint8_t encoder_state;
};

struct ChassisState {
    bool initialized;
    int32_t target_linear_mm_s;
    int32_t target_angular_mdeg_s;
    ChassisWheelState left;
    ChassisWheelState right;
    ChassisConfig config;
    drivers::DriverStatus last_status;
};

drivers::DriverStatus Chassis_Init(void);
drivers::DriverStatus Chassis_Stop(void);
drivers::DriverStatus Chassis_SetWheelRpm(int32_t left_rpm,
                                          int32_t right_rpm);
drivers::DriverStatus Chassis_SetVelocity(int32_t linear_mm_s,
                                          int32_t angular_mdeg_s);
drivers::DriverStatus Chassis_Service(void);
drivers::DriverStatus Chassis_Update(void);
const ChassisState *Chassis_GetState(void);

} /* namespace app */

#endif /* APP_CHASSIS_H_ */
