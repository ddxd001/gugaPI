#ifndef APP_LOCAL_MOTOR_CONTROLLER_H_
#define APP_LOCAL_MOTOR_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace app {

struct LocalMotorConfig {
    uint32_t left_counts_per_rev;
    uint32_t right_counts_per_rev;
    uint16_t max_wheel_rpm;
    uint16_t accel_rpm_per_s;
    uint16_t decel_rpm_per_s;
    uint8_t kp_q4_4;
    uint8_t ki_q4_4;
    uint8_t kd_q4_4;
    uint8_t max_duty_percent;
    uint8_t min_duty_percent;
    uint8_t output_invert_flags;
    uint8_t encoder_invert_flags;
};

struct LocalMotorWheelState {
    int32_t target_rpm;
    int32_t control_target_rpm;
    int32_t actual_rpm;
    int32_t encoder_count;
    int32_t encoder_counts_per_second;
    uint16_t duty_q8;
    uint8_t encoder_state;
    bool awake;
};

struct LocalMotorState {
    bool initialized;
    bool active;
    bool watchdog_latched;
    LocalMotorConfig config;
    LocalMotorWheelState left;
    LocalMotorWheelState right;
    drivers::DriverStatus last_status;
    uint32_t last_control_ms;
    uint32_t last_lease_ms;
    uint32_t update_sequence;
};

drivers::DriverStatus LocalMotorController_Init(const LocalMotorConfig *config,
                                                uint32_t now_ms);
drivers::DriverStatus LocalMotorController_Configure(
    const LocalMotorConfig *config);
/* In FEATURE_LOCAL_MOTOR_RIGHT_ONLY builds, left_rpm must be exactly zero;
 * nonzero left commands are rejected instead of being silently ignored. */
drivers::DriverStatus LocalMotorController_SetTargets(int32_t left_rpm,
                                                      int32_t right_rpm,
                                                      uint32_t now_ms);
drivers::DriverStatus LocalMotorController_RefreshLease(uint32_t now_ms);
drivers::DriverStatus LocalMotorController_Stop(void);
drivers::DriverStatus LocalMotorController_Update(uint32_t now_ms);
const LocalMotorState *LocalMotorController_GetState(void);

} /* namespace app */

#endif /* APP_LOCAL_MOTOR_CONTROLLER_H_ */
