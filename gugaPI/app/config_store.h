#ifndef APP_CONFIG_STORE_H_
#define APP_CONFIG_STORE_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace app {

static const uint8_t CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT = 8U;
static const uint8_t DISTANCE_SPEED_MODE_LEGACY = 0U;
static const uint8_t DISTANCE_SPEED_MODE_TRAPEZOID = 1U;

struct ConfigStoreParams {
    uint32_t left_counts_per_rev;
    uint32_t right_counts_per_rev;
    /* Calibrated rolling radius in micrometers (33.05 mm = 33050 um). */
    uint32_t wheel_radius_um;
    /* Legacy whole-millimeter view retained for parameter compatibility. */
    uint32_t wheel_radius_mm;
    uint32_t wheel_track_mm;
    uint16_t max_wheel_rpm;
    uint8_t motor_output_invert_flags;
    uint8_t motor_encoder_invert_flags;

    uint8_t speed_kp_q4_4;
    uint8_t speed_ki_q4_4;
    uint8_t speed_kd_q4_4;
    uint8_t speed_max_duty;
    uint8_t speed_min_duty;
    /* MotorDriver target-speed ramp restored by gugaPI at chassis init. */
    uint16_t speed_accel_rpm_s;
    uint16_t speed_decel_rpm_s;

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

    /* IMU heading closed-loop (v3). Sourced from ICM-45686 gyro yaw. */
    int32_t heading_kp;                  /* gain: correction_rpm = error_mdeg * heading_kp / 1e6 */
    int32_t heading_max_correction_rpm;  /* HOLD differential clamp */
    int32_t heading_turn_max_rpm;        /* TURN max wheel speed */
    int32_t heading_turn_min_rpm;         /* TURN min wheel speed (friction) */
    int32_t heading_tolerance_mdeg;      /* TURN target tolerance */
    uint16_t heading_settle_ms;          /* TURN settle time at target */

    /* Predictive TURN braking and completion gates (v12). */
    uint16_t heading_turn_brake_ms;          /* gyro-rate prediction horizon */
    uint16_t heading_turn_brake_margin_mdeg; /* fixed early-stop margin */
    uint16_t heading_turn_settle_rate_mdps;  /* maximum stopped yaw rate */
    uint16_t heading_turn_settle_rpm;        /* maximum stopped wheel RPM */

    /* Distance speed-profile tuning (v8). The MotorDriver speed-loop period
     * is unchanged; these values shape the RPM requests sent by gugaPI. */
    uint8_t distance_speed_mode;
    uint16_t distance_accel_rpm_s;
    uint16_t distance_decel_rpm_s;
    uint16_t distance_creep_rpm;
    uint16_t distance_stop_latency_ms;
    uint16_t distance_brake_margin_mm;
    uint16_t distance_settle_rpm;
    uint16_t distance_tolerance_mm;

    uint16_t ina219_undervoltage_trip_mv;
    uint16_t ina219_undervoltage_release_mv;
    uint16_t ina219_overcurrent_trip_ma;
    uint16_t ina219_overcurrent_release_ma;
    uint8_t ina219_trip_samples;
    uint8_t ina219_release_samples;
    uint8_t ina219_comm_fail_samples;
    uint8_t ina219_latch_faults;
    uint8_t ina219_motion_inhibit_enable;

    uint16_t grayscale_white[CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT];
    uint16_t grayscale_black[CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT];
    uint16_t grayscale_threshold;
    uint16_t grayscale_hysteresis;
    uint16_t grayscale_position_floor;
    uint16_t grayscale_min_line_strength;
    uint8_t grayscale_track_mask;

    int32_t linefollow_kp;
    int32_t linefollow_kd;
    uint16_t linefollow_max_correction_rpm;
    uint16_t linefollow_lost_hold_ms;
    uint16_t linefollow_lost_stop_ms;
    /* Maximum differential-correction change, relative to base RPM. */
    uint16_t linefollow_correction_slew_permille_per_second;

    /* Stationary disturbance-recovery heading lock (v13). Appended to the
     * persisted payload so every v1-v12 field keeps its original offset. */
    int32_t heading_lock_kp;
    int32_t heading_lock_kd;
    uint16_t heading_lock_wake_mdeg;
    uint16_t heading_lock_settle_mdeg;
    uint16_t heading_lock_min_rpm;
    uint16_t heading_lock_max_rpm;
    uint16_t heading_lock_settle_rate_mdps;
    uint16_t heading_lock_settle_rpm;
    uint16_t heading_lock_settle_ms;
    uint16_t heading_lock_timeout_ms;

    /* Encoder-distance corner alignment before the relative heading turn
     * (v14). RPM is the closed-loop alignment speed ceiling. */
    uint16_t road_align_distance_mm;
    uint16_t road_align_rpm;

    /* Automatic road-corner asymmetric wheel endpoints (v15). The outer
     * wheel retains the existing base+correction law while the inner wheel
     * may reverse independently. */
    uint16_t road_turn_outer_max_rpm;
    uint16_t road_turn_inner_reverse_max_rpm;

    /* DM-G6220 MIT controller (v16). Integer units avoid floating point in
     * the 100 Hz control path. */
    uint16_t dm_position_kp_milli;
    uint16_t dm_position_kd_milli;
    uint16_t dm_speed_kd_milli;
    uint16_t dm_max_velocity_mrad_s;
    uint16_t dm_max_tracking_error_mrad;
    uint16_t dm_speed_slew_mrad_s2;
    uint16_t dm_position_tolerance_mrad;
    uint16_t dm_velocity_tolerance_mrad_s;
    uint16_t dm_settle_ms;
    uint16_t dm_feedback_timeout_ms;
};

enum ConfigStoreLoadOutcome : uint8_t {
    CONFIG_LOAD_NOT_ATTEMPTED = 0U,
    CONFIG_LOAD_FROM_FRAM,
    CONFIG_LOAD_DEFAULTS_INVALID,
    CONFIG_LOAD_DEFAULTS_IO_ERROR,
    CONFIG_LOAD_DEFAULTS_UNSUPPORTED,
    CONFIG_LOAD_DEFAULTS_EXPLICIT
};

struct ConfigStoreStatus {
    bool loaded_from_fram;
    bool dirty;
    uint16_t stored_length;
    uint32_t stored_crc;
    drivers::DriverStatus last_load_status;
    drivers::DriverStatus last_save_status;
    ConfigStoreLoadOutcome load_outcome;
};

drivers::DriverStatus ConfigStore_Load(void);
drivers::DriverStatus ConfigStore_Save(void);
void ConfigStore_ResetDefaults(void);
const ConfigStoreParams *ConfigStore_Get(void);
drivers::DriverStatus ConfigStore_Set(const char *name, int32_t value);
drivers::DriverStatus ConfigStore_SetGrayscaleCalibration(
    const uint16_t white[CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT],
    const uint16_t black[CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT],
    uint16_t threshold);
drivers::DriverStatus ConfigStore_SetGrayscaleCalibration(
    const uint16_t white[CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT],
    const uint16_t black[CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT],
    uint16_t threshold,
    uint16_t hysteresis,
    uint16_t position_floor,
    uint16_t min_line_strength,
    uint8_t track_mask);
bool ConfigStore_GetValue(const char *name,
                          int32_t *value,
                          int32_t *min_value,
                          int32_t *max_value);
uint8_t ConfigStore_ParamCount(void);
const char *ConfigStore_ParamName(uint8_t index);
const ConfigStoreStatus *ConfigStore_GetStatus(void);

} /* namespace app */

#endif /* APP_CONFIG_STORE_H_ */
