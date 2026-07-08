#ifndef APP_CONFIG_STORE_H_
#define APP_CONFIG_STORE_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace app {

struct ConfigStoreParams {
    uint32_t left_counts_per_rev;
    uint32_t right_counts_per_rev;
    uint32_t wheel_radius_mm;
    uint32_t wheel_track_mm;
    uint16_t max_wheel_rpm;

    uint8_t speed_kp_q4_4;
    uint8_t speed_ki_q4_4;
    uint8_t speed_kd_q4_4;
    uint8_t speed_max_duty;
    uint8_t speed_min_duty;

    uint8_t position_kp_q4_4;
    uint8_t position_ki_q4_4;
    uint8_t position_kd_q4_4;
    uint16_t position_max_rpm;
    uint16_t position_tolerance_counts;

    int32_t gy931_roll_zero_mdeg;
    int32_t gy931_pitch_zero_mdeg;
    int32_t gy931_yaw_zero_mdeg;

    int32_t imu_accel_bias_x_mg;
    int32_t imu_accel_bias_y_mg;
    int32_t imu_accel_bias_z_mg;
    int32_t imu_gyro_bias_x_mdps;
    int32_t imu_gyro_bias_y_mdps;
    int32_t imu_gyro_bias_z_mdps;
};

struct ConfigStoreStatus {
    bool loaded_from_fram;
    bool dirty;
    uint16_t stored_length;
    uint32_t stored_crc;
    drivers::DriverStatus last_load_status;
    drivers::DriverStatus last_save_status;
};

drivers::DriverStatus ConfigStore_Load(void);
drivers::DriverStatus ConfigStore_Save(void);
void ConfigStore_ResetDefaults(void);
const ConfigStoreParams *ConfigStore_Get(void);
drivers::DriverStatus ConfigStore_Set(const char *name, int32_t value);
bool ConfigStore_GetValue(const char *name,
                          int32_t *value,
                          int32_t *min_value,
                          int32_t *max_value);
uint8_t ConfigStore_ParamCount(void);
const char *ConfigStore_ParamName(uint8_t index);
const ConfigStoreStatus *ConfigStore_GetStatus(void);

} /* namespace app */

#endif /* APP_CONFIG_STORE_H_ */
