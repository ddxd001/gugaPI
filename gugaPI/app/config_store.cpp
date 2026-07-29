#include "app/config_store.h"

#include <stddef.h>
#include <string.h>

#include "board/board_fram.h"
#include "config/feature_config.h"

namespace app {
namespace {

static const uint16_t kFramAddress = 0x0000U;
static const uint32_t kMagic = 0x47504643U; /* "CFPG" little-endian */
static const uint16_t kVersion = 17U;
static const uint16_t kV16Version = 16U;
static const uint16_t kV16PayloadLength = 243U;
static const uint16_t kV15Version = 15U;
static const uint16_t kV15PayloadLength = 223U;
static const uint16_t kV14Version = 14U;
static const uint16_t kV14PayloadLength = 219U;
static const uint16_t kV13Version = 13U;
static const uint16_t kV13PayloadLength = 215U;
static const uint16_t kV12Version = 12U;
static const uint16_t kV12PayloadLength = 191U;
static const uint16_t kV11Version = 11U;
static const uint16_t kV11PayloadLength = 183U;
static const uint16_t kV10Version = 10U;
static const uint16_t kV10PayloadLength = 183U;
static const uint16_t kV9Version = 9U;
static const uint16_t kV9PayloadLength = 181U;
static const uint16_t kV8Version = 8U;
static const uint16_t kV8PayloadLength = 177U;
static const uint16_t kV7Version = 7U;
static const uint16_t kV7PayloadLength = 162U;
static const uint16_t kV6Version = 6U;
static const uint16_t kV6PayloadLength = 158U;
static const uint16_t kV5Version = 5U;
static const uint16_t kV5PayloadLength = 137U;
static const uint16_t kV4Version = 4U;
static const uint16_t kV4PayloadLength = 103U;
static const uint16_t kV3Version = 3U;
static const uint16_t kV3PayloadLength = 90U;
static const uint16_t kLegacyVersion = 1U;
static const uint16_t kLegacyPayloadLength = 66U;
static const uint16_t kV2PayloadLength = 68U; /* v2 layout length (motor_invert guard) */
/* v11 keeps the v10 binary layout and migrates the former default four-channel
 * grayscale tracking mask. v12 appends predictive TURN fields, v13 appends
 * stationary heading-lock fields, v14 appends corner alignment distance and
 * speed, v15 appends asymmetric road-turn wheel limits, and v16 appends
 * the bounded DM-G6220 MIT control parameters. v17 retains the same main
 * payload and adds a separate infrared extension record in the free FRAM
 * tail so neither parameter family nor the sequence region is overwritten;
 * every older field keeps its binary offset. */
static const uint16_t kPayloadLength = 243U;
static const uint16_t kHeaderLength = 8U;
static const uint16_t kCrcLength = 4U;
static const uint16_t kImageLength =
    kHeaderLength + kPayloadLength + kCrcLength;
static const uint16_t kInfraredExtensionAddress = 0x1FD0U;
static const uint32_t kInfraredExtensionMagic = 0x43335249U; /* "IR3C" */
static const uint16_t kInfraredExtensionVersion = 1U;
static const uint16_t kInfraredExtensionPayloadLength = 20U;
static const uint16_t kInfraredExtensionImageLength =
    kHeaderLength + kInfraredExtensionPayloadLength + kCrcLength;
static_assert(kInfraredExtensionAddress + kInfraredExtensionImageLength <=
              0x1FF0U,
              "infrared config must not overlap the FRAM self-test area");
static const uint32_t kCrc32Init = 0xFFFFFFFFU;
static const uint8_t kLegacyDefaultGrayscaleTrackMask = 0x3CU;
static const uint8_t kDefaultGrayscaleTrackMask = 0x7EU;

ConfigStoreParams g_params;
ConfigStoreStatus g_status = {
    false,
    false,
    0U,
    0U,
    drivers::DRIVER_ERROR_NOT_INITIALIZED,
    drivers::DRIVER_ERROR_NOT_INITIALIZED,
    CONFIG_LOAD_NOT_ATTEMPTED
};

enum ParamType : uint8_t {
    PARAM_U8 = 0U,
    PARAM_U16,
    PARAM_U32,
    PARAM_I32
};

struct ParamDescriptor {
    const char *name;
    ParamType type;
    uint16_t offset;
    int32_t min_value;
    int32_t max_value;
};

#define PARAM_OFFSET(field) \
    static_cast<uint16_t>(offsetof(ConfigStoreParams, field))
#define PARAM_ARRAY_OFFSET(field, index) \
    static_cast<uint16_t>(offsetof(ConfigStoreParams, field) + \
                          sizeof(g_params.field[0]) * (index))

static const ParamDescriptor kParamDescriptors[] = {
    { "left_counts_per_rev", PARAM_U32,
      PARAM_OFFSET(left_counts_per_rev), 1, 100000000 },
    { "right_counts_per_rev", PARAM_U32,
      PARAM_OFFSET(right_counts_per_rev), 1, 100000000 },
    { "wheel_radius_um", PARAM_U32,
      PARAM_OFFSET(wheel_radius_um), 1000, 1000000 },
    { "wheel_radius_mm", PARAM_U32,
      PARAM_OFFSET(wheel_radius_mm), 1, 1000 },
    { "wheel_track_mm", PARAM_U32,
      PARAM_OFFSET(wheel_track_mm), 1, 2000 },
    { "max_wheel_rpm", PARAM_U16,
      PARAM_OFFSET(max_wheel_rpm), 1, 1000 },
    { "motor_output_invert_flags", PARAM_U8,
      PARAM_OFFSET(motor_output_invert_flags), 0, 3 },
    { "motor_encoder_invert_flags", PARAM_U8,
      PARAM_OFFSET(motor_encoder_invert_flags), 0, 3 },

    { "speed_kp", PARAM_U8, PARAM_OFFSET(speed_kp_q4_4), 0, 255 },
    { "speed_ki", PARAM_U8, PARAM_OFFSET(speed_ki_q4_4), 0, 255 },
    { "speed_kd", PARAM_U8, PARAM_OFFSET(speed_kd_q4_4), 0, 255 },
    { "speed_max_duty", PARAM_U8, PARAM_OFFSET(speed_max_duty), 0, 100 },
    { "speed_min_duty", PARAM_U8, PARAM_OFFSET(speed_min_duty), 0, 100 },
    { "speed_accel_rpm_s", PARAM_U16,
      PARAM_OFFSET(speed_accel_rpm_s), 0, 65535 },
    { "speed_decel_rpm_s", PARAM_U16,
      PARAM_OFFSET(speed_decel_rpm_s), 0, 65535 },

    { "position_kp", PARAM_U8, PARAM_OFFSET(position_kp_q4_4), 0, 255 },
    { "position_ki", PARAM_U8, PARAM_OFFSET(position_ki_q4_4), 0, 255 },
    { "position_kd", PARAM_U8, PARAM_OFFSET(position_kd_q4_4), 0, 255 },
    { "position_max_rpm", PARAM_U16,
      PARAM_OFFSET(position_max_rpm), 0, 1000 },
    { "position_tolerance_counts", PARAM_U16,
      PARAM_OFFSET(position_tolerance_counts), 0, 65535 },

    { "gy931_roll_zero_mdeg", PARAM_I32,
      PARAM_OFFSET(gy931_roll_zero_mdeg), -180000000, 180000000 },
    { "gy931_pitch_zero_mdeg", PARAM_I32,
      PARAM_OFFSET(gy931_pitch_zero_mdeg), -180000000, 180000000 },
    { "gy931_yaw_zero_mdeg", PARAM_I32,
      PARAM_OFFSET(gy931_yaw_zero_mdeg), -180000000, 180000000 },

    { "imu_accel_bias_x_mg", PARAM_I32,
      PARAM_OFFSET(imu_accel_bias_x_mg), -200000, 200000 },
    { "imu_accel_bias_y_mg", PARAM_I32,
      PARAM_OFFSET(imu_accel_bias_y_mg), -200000, 200000 },
    { "imu_accel_bias_z_mg", PARAM_I32,
      PARAM_OFFSET(imu_accel_bias_z_mg), -200000, 200000 },
    { "imu_gyro_bias_x_mdps", PARAM_I32,
      PARAM_OFFSET(imu_gyro_bias_x_mdps), -2000000, 2000000 },
    { "imu_gyro_bias_y_mdps", PARAM_I32,
      PARAM_OFFSET(imu_gyro_bias_y_mdps), -2000000, 2000000 },
    { "imu_gyro_bias_z_mdps", PARAM_I32,
      PARAM_OFFSET(imu_gyro_bias_z_mdps), -2000000, 2000000 },

    { "heading_kp", PARAM_I32, PARAM_OFFSET(heading_kp), 0, 100000 },
    { "heading_max_correction_rpm", PARAM_I32,
      PARAM_OFFSET(heading_max_correction_rpm), 0, 500 },
    { "heading_turn_max_rpm", PARAM_I32,
      PARAM_OFFSET(heading_turn_max_rpm), 0, 1000 },
    { "heading_turn_min_rpm", PARAM_I32,
      PARAM_OFFSET(heading_turn_min_rpm), 0, 500 },
    { "heading_tolerance_mdeg", PARAM_I32,
      PARAM_OFFSET(heading_tolerance_mdeg), 0, 90000 },
    { "heading_settle_ms", PARAM_U16,
      PARAM_OFFSET(heading_settle_ms), 0, 5000 },
    { "heading_turn_brake_ms", PARAM_U16,
      PARAM_OFFSET(heading_turn_brake_ms), 0, 500 },
    { "heading_turn_brake_margin_mdeg", PARAM_U16,
      PARAM_OFFSET(heading_turn_brake_margin_mdeg), 0, 30000 },
    { "heading_turn_settle_rate_mdps", PARAM_U16,
      PARAM_OFFSET(heading_turn_settle_rate_mdps), 0, 60000 },
    { "heading_turn_settle_rpm", PARAM_U16,
      PARAM_OFFSET(heading_turn_settle_rpm), 0, 100 },
    { "heading_lock_kp", PARAM_I32,
      PARAM_OFFSET(heading_lock_kp), 0, 100000 },
    { "heading_lock_kd", PARAM_I32,
      PARAM_OFFSET(heading_lock_kd), 0, 100000 },
    { "heading_lock_wake_mdeg", PARAM_U16,
      PARAM_OFFSET(heading_lock_wake_mdeg), 100, 30000 },
    { "heading_lock_settle_mdeg", PARAM_U16,
      PARAM_OFFSET(heading_lock_settle_mdeg), 50, 29999 },
    { "heading_lock_min_rpm", PARAM_U16,
      PARAM_OFFSET(heading_lock_min_rpm), 0, 100 },
    { "heading_lock_max_rpm", PARAM_U16,
      PARAM_OFFSET(heading_lock_max_rpm), 1, 200 },
    { "heading_lock_settle_rate_mdps", PARAM_U16,
      PARAM_OFFSET(heading_lock_settle_rate_mdps), 0, 60000 },
    { "heading_lock_settle_rpm", PARAM_U16,
      PARAM_OFFSET(heading_lock_settle_rpm), 0, 100 },
    { "heading_lock_settle_ms", PARAM_U16,
      PARAM_OFFSET(heading_lock_settle_ms), 50, 5000 },
    { "heading_lock_timeout_ms", PARAM_U16,
      PARAM_OFFSET(heading_lock_timeout_ms), 500, 10000 },
    { "road_align_distance_mm", PARAM_U16,
      PARAM_OFFSET(road_align_distance_mm), 0, 300 },
    { "road_align_rpm", PARAM_U16,
      PARAM_OFFSET(road_align_rpm), 1, 300 },
    { "road_turn_outer_max_rpm", PARAM_U16,
      PARAM_OFFSET(road_turn_outer_max_rpm), 1, 1000 },
    { "road_turn_inner_reverse_max_rpm", PARAM_U16,
      PARAM_OFFSET(road_turn_inner_reverse_max_rpm), 0, 1000 },
    { "dm_position_kp_milli", PARAM_U16,
      PARAM_OFFSET(dm_position_kp_milli), 0, 10000 },
    { "dm_position_kd_milli", PARAM_U16,
      PARAM_OFFSET(dm_position_kd_milli), 0, 2000 },
    { "dm_speed_kd_milli", PARAM_U16,
      PARAM_OFFSET(dm_speed_kd_milli), 0, 2000 },
    { "dm_max_velocity_mrad_s", PARAM_U16,
      PARAM_OFFSET(dm_max_velocity_mrad_s), 0, 20000 },
    { "dm_max_tracking_error_mrad", PARAM_U16,
      PARAM_OFFSET(dm_max_tracking_error_mrad), 1, 250 },
    { "dm_speed_slew_mrad_s2", PARAM_U16,
      PARAM_OFFSET(dm_speed_slew_mrad_s2), 1, 10000 },
    { "dm_position_tolerance_mrad", PARAM_U16,
      PARAM_OFFSET(dm_position_tolerance_mrad), 1, 100 },
    { "dm_velocity_tolerance_mrad_s", PARAM_U16,
      PARAM_OFFSET(dm_velocity_tolerance_mrad_s), 1, 500 },
    { "dm_settle_ms", PARAM_U16,
      PARAM_OFFSET(dm_settle_ms), 50, 1000 },
    { "dm_feedback_timeout_ms", PARAM_U16,
      PARAM_OFFSET(dm_feedback_timeout_ms), 50, 500 },
    { "distance_speed_mode", PARAM_U8,
      PARAM_OFFSET(distance_speed_mode),
      DISTANCE_SPEED_MODE_LEGACY, DISTANCE_SPEED_MODE_TRAPEZOID },
    { "distance_accel_rpm_s", PARAM_U16,
      PARAM_OFFSET(distance_accel_rpm_s), 1, 5000 },
    { "distance_decel_rpm_s", PARAM_U16,
      PARAM_OFFSET(distance_decel_rpm_s), 1, 5000 },
    { "distance_creep_rpm", PARAM_U16,
      PARAM_OFFSET(distance_creep_rpm), 1, 500 },
    { "distance_stop_latency_ms", PARAM_U16,
      PARAM_OFFSET(distance_stop_latency_ms), 0, 2000 },
    { "distance_brake_margin_mm", PARAM_U16,
      PARAM_OFFSET(distance_brake_margin_mm), 0, 1000 },
    { "distance_settle_rpm", PARAM_U16,
      PARAM_OFFSET(distance_settle_rpm), 0, 100 },
    { "distance_tolerance_mm", PARAM_U16,
      PARAM_OFFSET(distance_tolerance_mm), 1, 100 },

    { "ina_uv_trip_mv", PARAM_U16,
      PARAM_OFFSET(ina219_undervoltage_trip_mv), 1, 25999 },
    { "ina_uv_release_mv", PARAM_U16,
      PARAM_OFFSET(ina219_undervoltage_release_mv), 2, 26000 },
    { "ina_oc_trip_ma", PARAM_U16,
      PARAM_OFFSET(ina219_overcurrent_trip_ma), 1, 6500 },
    { "ina_oc_release_ma", PARAM_U16,
      PARAM_OFFSET(ina219_overcurrent_release_ma), 0, 6499 },
    { "ina_trip_samples", PARAM_U8,
      PARAM_OFFSET(ina219_trip_samples), 1, 100 },
    { "ina_release_samples", PARAM_U8,
      PARAM_OFFSET(ina219_release_samples), 1, 100 },
    { "ina_comm_fail_samples", PARAM_U8,
      PARAM_OFFSET(ina219_comm_fail_samples), 1, 100 },
    { "ina_latch_faults", PARAM_U8,
      PARAM_OFFSET(ina219_latch_faults), 0, 1 },
    { "ina_motion_inhibit", PARAM_U8,
      PARAM_OFFSET(ina219_motion_inhibit_enable), 0, 1 },

    { "gray_white_0", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_white, 0U), 0, 4095 },
    { "gray_white_1", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_white, 1U), 0, 4095 },
    { "gray_white_2", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_white, 2U), 0, 4095 },
    { "gray_white_3", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_white, 3U), 0, 4095 },
    { "gray_white_4", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_white, 4U), 0, 4095 },
    { "gray_white_5", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_white, 5U), 0, 4095 },
    { "gray_white_6", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_white, 6U), 0, 4095 },
    { "gray_white_7", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_white, 7U), 0, 4095 },
    { "gray_black_0", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_black, 0U), 0, 4095 },
    { "gray_black_1", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_black, 1U), 0, 4095 },
    { "gray_black_2", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_black, 2U), 0, 4095 },
    { "gray_black_3", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_black, 3U), 0, 4095 },
    { "gray_black_4", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_black, 4U), 0, 4095 },
    { "gray_black_5", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_black, 5U), 0, 4095 },
    { "gray_black_6", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_black, 6U), 0, 4095 },
    { "gray_black_7", PARAM_U16,
      PARAM_ARRAY_OFFSET(grayscale_black, 7U), 0, 4095 },
    { "gray_threshold", PARAM_U16,
      PARAM_OFFSET(grayscale_threshold), 1, 999 },
    { "gray_hysteresis", PARAM_U16,
      PARAM_OFFSET(grayscale_hysteresis), 0, 998 },
    { "gray_position_floor", PARAM_U16,
      PARAM_OFFSET(grayscale_position_floor), 0, 999 },
    { "gray_min_strength", PARAM_U16,
      PARAM_OFFSET(grayscale_min_line_strength), 1, 8000 },
    { "gray_track_mask", PARAM_U8,
      PARAM_OFFSET(grayscale_track_mask), 1, 255 },
    { "lf_kp", PARAM_I32,
      PARAM_OFFSET(linefollow_kp), 0, 1000000 },
    { "lf_kd", PARAM_I32,
      PARAM_OFFSET(linefollow_kd), 0, 1000000 },
    { "lf_maxcorr", PARAM_U16,
      PARAM_OFFSET(linefollow_max_correction_rpm), 0, 500 },
    { "lf_lost_hold_ms", PARAM_U16,
      PARAM_OFFSET(linefollow_lost_hold_ms), 0, 10000 },
    { "lf_lost_stop_ms", PARAM_U16,
      PARAM_OFFSET(linefollow_lost_stop_ms), 1, 10000 },
    { "lf_slew_permille_s", PARAM_U16,
      PARAM_OFFSET(linefollow_correction_slew_permille_per_second),
      1, 65535 },
    { "line_sensor_source", PARAM_U8,
      PARAM_OFFSET(line_sensor_source),
      LINE_SENSOR_SOURCE_ADC8, LINE_SENSOR_SOURCE_IR3 },
    { "ir_position_invert", PARAM_U8,
      PARAM_OFFSET(infrared_position_invert), 0, 1 },
    { "ir_position_span_raw", PARAM_U16,
      PARAM_OFFSET(infrared_position_span_raw), 0, 32767 },
    { "ir_adc_threshold", PARAM_U16,
      PARAM_OFFSET(infrared_adc_threshold), 0, 4095 },
    { "ir_adc_hysteresis", PARAM_U16,
      PARAM_OFFSET(infrared_adc_hysteresis), 0, 1000 },
    { "ir_lf_kp", PARAM_I32,
      PARAM_OFFSET(infrared_linefollow_kp), 0, 1000000 },
    { "ir_lf_kd", PARAM_I32,
      PARAM_OFFSET(infrared_linefollow_kd), 0, 1000000 },
    { "ir_lf_maxcorr", PARAM_U16,
      PARAM_OFFSET(infrared_linefollow_max_correction_rpm), 0, 500 },
    { "ir_lf_slew_permille_s", PARAM_U16,
      PARAM_OFFSET(
          infrared_linefollow_correction_slew_permille_per_second),
      1, 65535 }
};

#undef PARAM_OFFSET
#undef PARAM_ARRAY_OFFSET

bool TextEqual(const char *left, const char *right)
{
    if ((left == 0) || (right == 0)) {
        return false;
    }

    while ((*left != '\0') && (*right != '\0')) {
        if (*left != *right) {
            return false;
        }
        left++;
        right++;
    }

    return (*left == '\0') && (*right == '\0');
}

const ParamDescriptor *FindParam(const char *name)
{
    for (uint8_t i = 0U; i < ConfigStore_ParamCount(); i++) {
        if (TextEqual(name, kParamDescriptors[i].name)) {
            return &kParamDescriptors[i];
        }
    }
    return 0;
}

void WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

void WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    data[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
    data[3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

uint16_t ReadU16(const uint8_t *data)
{
    return static_cast<uint16_t>(
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8U));
}

uint32_t ReadU32(const uint8_t *data)
{
    return static_cast<uint32_t>(
        static_cast<uint32_t>(data[0]) |
        (static_cast<uint32_t>(data[1]) << 8U) |
        (static_cast<uint32_t>(data[2]) << 16U) |
        (static_cast<uint32_t>(data[3]) << 24U));
}

uint32_t Crc32Update(uint32_t crc, uint8_t value)
{
    crc ^= static_cast<uint32_t>(value) << 24U;
    for (uint8_t bit = 0U; bit < 8U; bit++) {
        if ((crc & 0x80000000U) != 0U) {
            crc = (crc << 1U) ^ 0x04C11DB7U;
        } else {
            crc <<= 1U;
        }
    }
    return crc;
}

uint32_t Crc32(const uint8_t *data, uint16_t length)
{
    uint32_t crc = kCrc32Init;
    for (uint16_t i = 0U; i < length; i++) {
        crc = Crc32Update(crc, data[i]);
    }
    return crc ^ 0xFFFFFFFFU;
}

void SetDefaults(ConfigStoreParams *params)
{
    (void) memset(params, 0, sizeof(*params));

    params->left_counts_per_rev = 1456U;
    params->right_counts_per_rev = 1456U;
    params->wheel_radius_um = 33050U;
    params->wheel_radius_mm = 33U;
    params->wheel_track_mm = 160U;
    params->max_wheel_rpm = 1000U;
    params->motor_output_invert_flags = 0x03U;
    params->motor_encoder_invert_flags = 0x01U;

    params->speed_kp_q4_4 = 2U;
    params->speed_ki_q4_4 = 2U;
    params->speed_kd_q4_4 = 0U;
    params->speed_max_duty = 60U;
    params->speed_min_duty = 4U;
    params->speed_accel_rpm_s = 1500U;
    params->speed_decel_rpm_s = 2000U;

    params->position_kp_q4_4 = 15U;
    params->position_ki_q4_4 = 0U;
    params->position_kd_q4_4 = 0U;
    params->position_max_rpm = 40U;
    params->position_tolerance_counts = 3U;

    /* IMU heading closed-loop (conservative bring-up values).
     * correction_rpm = error_mdeg * heading_kp / 1e6, so heading_kp=1000
     * means 1 RPM per degree of error. */
    params->heading_kp = 1000;               /* 1 RPM/deg */
    params->heading_max_correction_rpm = 30;
    params->heading_turn_max_rpm = 60;
    params->heading_turn_min_rpm = 20;
    params->heading_tolerance_mdeg = 3000;  /* 3 deg */
    params->heading_settle_ms = 300U;
    params->heading_turn_brake_ms = 60U;
    params->heading_turn_brake_margin_mdeg = 500U;
    params->heading_turn_settle_rate_mdps = 1500U;
    params->heading_turn_settle_rpm = 3U;
    params->heading_lock_kp = 1500;
    params->heading_lock_kd = 250;
    params->heading_lock_wake_mdeg = 2000U;
    params->heading_lock_settle_mdeg = 800U;
    params->heading_lock_min_rpm = 10U;
    params->heading_lock_max_rpm = 30U;
    params->heading_lock_settle_rate_mdps = 1500U;
    params->heading_lock_settle_rpm = 3U;
    params->heading_lock_settle_ms = 250U;
    params->heading_lock_timeout_ms = 3000U;
    params->road_align_distance_mm = 0U;
    params->road_align_rpm = 30U;
    params->road_turn_outer_max_rpm = 220U;
    params->road_turn_inner_reverse_max_rpm = 120U;
    params->dm_position_kp_milli = 4000U;
    params->dm_position_kd_milli = 400U;
    params->dm_speed_kd_milli = 500U;
    params->dm_max_velocity_mrad_s = 2000U;
    params->dm_max_tracking_error_mrad = 250U;
    params->dm_speed_slew_mrad_s2 = 2000U;
    params->dm_position_tolerance_mrad = 10U;
    params->dm_velocity_tolerance_mrad_s = 80U;
    params->dm_settle_ms = 200U;
    params->dm_feedback_timeout_ms = 100U;

    /* MotorDriver closes its speed loop every 10 ms. These conservative
     * endpoint values model end-to-end command/feedback delay and mechanical
     * coast; gugaPI's 100 ms chassis service only refreshes the motion lease. */
    params->distance_speed_mode = DISTANCE_SPEED_MODE_TRAPEZOID;
    params->distance_accel_rpm_s = 600U;
    params->distance_decel_rpm_s = 900U;
    params->distance_creep_rpm = 15U;
    params->distance_stop_latency_ms = 360U;
    params->distance_brake_margin_mm = 5U;
    params->distance_settle_rpm = 3U;
    params->distance_tolerance_mm = 3U;

    /* Placeholder thresholds for a nominal low-voltage robot supply. They
     * are monitored immediately, but automatic motion inhibition remains off
     * until commissioned for the installed battery, wiring and motor load. */
    params->ina219_undervoltage_trip_mv = 6000U;
    params->ina219_undervoltage_release_mv = 6500U;
    params->ina219_overcurrent_trip_ma = 5000U;
    params->ina219_overcurrent_release_ma = 4500U;
    params->ina219_trip_samples = 3U;
    params->ina219_release_samples = 5U;
    params->ina219_comm_fail_samples = 3U;
    params->ina219_latch_faults = 0U;
    params->ina219_motion_inhibit_enable = 0U;

    /* Commissioned on the current gugaPI car. These defaults restore its
     * parameter values after a ConfigStore reset; the normal commissioned
     * calibration safety gate still applies. A replacement sensor or changed
     * mounting height must be calibrated again. */
    static const uint16_t kCommissionedGrayscaleWhite[
        CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT] = {
        3253U, 3217U, 3189U, 3316U, 3151U, 3011U, 2802U, 3188U
    };
    static const uint16_t kCommissionedGrayscaleBlack[
        CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT] = {
        1010U, 934U, 737U, 2010U, 1548U, 1347U, 753U, 1362U
    };
    for (uint8_t i = 0U; i < CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT; i++) {
        params->grayscale_white[i] = kCommissionedGrayscaleWhite[i];
        params->grayscale_black[i] = kCommissionedGrayscaleBlack[i];
    }
    params->grayscale_threshold = 500U;
    params->grayscale_hysteresis = 300U;
    params->grayscale_position_floor = 100U;
    params->grayscale_min_line_strength = 600U;
    params->grayscale_track_mask = kDefaultGrayscaleTrackMask;

    /* Tuned on the commissioned car at a 100 RPM follow speed. Keep the
     * controller's 40 RPM normalization reference for gain compatibility. */
    params->linefollow_kp = 3800;
    params->linefollow_kd = 600;
    params->linefollow_max_correction_rpm = 30U;
    params->linefollow_lost_hold_ms = 150U;
    params->linefollow_lost_stop_ms = 500U;
    params->linefollow_correction_slew_permille_per_second = 25000U;

    /* The new real sensor is the requested default, but zero calibration
     * values deliberately inhibit motion until the five-step wizard commits
     * measured thresholds and position span. */
    params->line_sensor_source = LINE_SENSOR_SOURCE_IR3;
    params->infrared_position_invert = 0U;
    params->infrared_position_span_raw = 0U;
    params->infrared_adc_threshold = 0U;
    params->infrared_adc_hysteresis = 0U;
    params->infrared_linefollow_kp = 3800;
    params->infrared_linefollow_kd = 600;
    params->infrared_linefollow_max_correction_rpm = 30U;
    params->infrared_linefollow_correction_slew_permille_per_second = 25000U;
}

uint8_t *AppendU8(uint8_t *cursor, uint8_t value)
{
    *cursor = value;
    return cursor + 1;
}

uint8_t *AppendU16(uint8_t *cursor, uint16_t value)
{
    WriteU16(cursor, value);
    return cursor + 2;
}

uint8_t *AppendU32(uint8_t *cursor, uint32_t value)
{
    WriteU32(cursor, value);
    return cursor + 4;
}

uint8_t *AppendI32(uint8_t *cursor, int32_t value)
{
    WriteU32(cursor, static_cast<uint32_t>(value));
    return cursor + 4;
}

const uint8_t *ReadU8Field(const uint8_t *cursor, uint8_t *value)
{
    *value = *cursor;
    return cursor + 1;
}

const uint8_t *ReadU16Field(const uint8_t *cursor, uint16_t *value)
{
    *value = ReadU16(cursor);
    return cursor + 2;
}

const uint8_t *ReadU32Field(const uint8_t *cursor, uint32_t *value)
{
    *value = ReadU32(cursor);
    return cursor + 4;
}

const uint8_t *ReadI32Field(const uint8_t *cursor, int32_t *value)
{
    *value = static_cast<int32_t>(ReadU32(cursor));
    return cursor + 4;
}

void EncodePayload(const ConfigStoreParams &params, uint8_t *payload)
{
    uint8_t *cursor = payload;

    cursor = AppendU32(cursor, params.left_counts_per_rev);
    cursor = AppendU32(cursor, params.right_counts_per_rev);
    cursor = AppendU32(cursor, params.wheel_radius_mm);
    cursor = AppendU32(cursor, params.wheel_track_mm);
    cursor = AppendU16(cursor, params.max_wheel_rpm);
    cursor = AppendU8(cursor, params.motor_output_invert_flags);
    cursor = AppendU8(cursor, params.motor_encoder_invert_flags);

    cursor = AppendU8(cursor, params.speed_kp_q4_4);
    cursor = AppendU8(cursor, params.speed_ki_q4_4);
    cursor = AppendU8(cursor, params.speed_kd_q4_4);
    cursor = AppendU8(cursor, params.speed_max_duty);
    cursor = AppendU8(cursor, params.speed_min_duty);

    cursor = AppendU8(cursor, params.position_kp_q4_4);
    cursor = AppendU8(cursor, params.position_ki_q4_4);
    cursor = AppendU8(cursor, params.position_kd_q4_4);
    cursor = AppendU16(cursor, params.position_max_rpm);
    cursor = AppendU16(cursor, params.position_tolerance_counts);

    cursor = AppendI32(cursor, params.gy931_roll_zero_mdeg);
    cursor = AppendI32(cursor, params.gy931_pitch_zero_mdeg);
    cursor = AppendI32(cursor, params.gy931_yaw_zero_mdeg);

    cursor = AppendI32(cursor, params.imu_accel_bias_x_mg);
    cursor = AppendI32(cursor, params.imu_accel_bias_y_mg);
    cursor = AppendI32(cursor, params.imu_accel_bias_z_mg);
    cursor = AppendI32(cursor, params.imu_gyro_bias_x_mdps);
    cursor = AppendI32(cursor, params.imu_gyro_bias_y_mdps);
    cursor = AppendI32(cursor, params.imu_gyro_bias_z_mdps);

    cursor = AppendI32(cursor, params.heading_kp);
    cursor = AppendI32(cursor, params.heading_max_correction_rpm);
    cursor = AppendI32(cursor, params.heading_turn_max_rpm);
    cursor = AppendI32(cursor, params.heading_turn_min_rpm);
    cursor = AppendI32(cursor, params.heading_tolerance_mdeg);
    cursor = AppendU16(cursor, params.heading_settle_ms);

    cursor = AppendU16(cursor, params.ina219_undervoltage_trip_mv);
    cursor = AppendU16(cursor, params.ina219_undervoltage_release_mv);
    cursor = AppendU16(cursor, params.ina219_overcurrent_trip_ma);
    cursor = AppendU16(cursor, params.ina219_overcurrent_release_ma);
    cursor = AppendU8(cursor, params.ina219_trip_samples);
    cursor = AppendU8(cursor, params.ina219_release_samples);
    cursor = AppendU8(cursor, params.ina219_comm_fail_samples);
    cursor = AppendU8(cursor, params.ina219_latch_faults);
    cursor = AppendU8(cursor, params.ina219_motion_inhibit_enable);

    for (uint8_t i = 0U; i < CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT; i++) {
        cursor = AppendU16(cursor, params.grayscale_white[i]);
    }
    for (uint8_t i = 0U; i < CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT; i++) {
        cursor = AppendU16(cursor, params.grayscale_black[i]);
    }
    cursor = AppendU16(cursor, params.grayscale_threshold);
    cursor = AppendU16(cursor, params.grayscale_hysteresis);
    cursor = AppendU16(cursor, params.grayscale_position_floor);
    cursor = AppendU16(cursor, params.grayscale_min_line_strength);
    cursor = AppendU8(cursor, params.grayscale_track_mask);
    cursor = AppendI32(cursor, params.linefollow_kp);
    cursor = AppendI32(cursor, params.linefollow_kd);
    cursor = AppendU16(cursor, params.linefollow_max_correction_rpm);
    cursor = AppendU16(cursor, params.linefollow_lost_hold_ms);
    cursor = AppendU16(cursor, params.linefollow_lost_stop_ms);
    cursor = AppendU32(cursor, params.wheel_radius_um);
    cursor = AppendU8(cursor, params.distance_speed_mode);
    cursor = AppendU16(cursor, params.distance_accel_rpm_s);
    cursor = AppendU16(cursor, params.distance_decel_rpm_s);
    cursor = AppendU16(cursor, params.distance_creep_rpm);
    cursor = AppendU16(cursor, params.distance_stop_latency_ms);
    cursor = AppendU16(cursor, params.distance_brake_margin_mm);
    cursor = AppendU16(cursor, params.distance_settle_rpm);
    cursor = AppendU16(cursor, params.distance_tolerance_mm);
    cursor = AppendU16(cursor, params.speed_accel_rpm_s);
    cursor = AppendU16(cursor, params.speed_decel_rpm_s);
    cursor = AppendU16(
        cursor,
        params.linefollow_correction_slew_permille_per_second);
    cursor = AppendU16(cursor, params.heading_turn_brake_ms);
    cursor = AppendU16(cursor, params.heading_turn_brake_margin_mdeg);
    cursor = AppendU16(cursor, params.heading_turn_settle_rate_mdps);
    cursor = AppendU16(cursor, params.heading_turn_settle_rpm);
    cursor = AppendI32(cursor, params.heading_lock_kp);
    cursor = AppendI32(cursor, params.heading_lock_kd);
    cursor = AppendU16(cursor, params.heading_lock_wake_mdeg);
    cursor = AppendU16(cursor, params.heading_lock_settle_mdeg);
    cursor = AppendU16(cursor, params.heading_lock_min_rpm);
    cursor = AppendU16(cursor, params.heading_lock_max_rpm);
    cursor = AppendU16(cursor, params.heading_lock_settle_rate_mdps);
    cursor = AppendU16(cursor, params.heading_lock_settle_rpm);
    cursor = AppendU16(cursor, params.heading_lock_settle_ms);
    cursor = AppendU16(cursor, params.heading_lock_timeout_ms);
    cursor = AppendU16(cursor, params.road_align_distance_mm);
    cursor = AppendU16(cursor, params.road_align_rpm);
    cursor = AppendU16(cursor, params.road_turn_outer_max_rpm);
    cursor = AppendU16(cursor, params.road_turn_inner_reverse_max_rpm);
    cursor = AppendU16(cursor, params.dm_position_kp_milli);
    cursor = AppendU16(cursor, params.dm_position_kd_milli);
    cursor = AppendU16(cursor, params.dm_speed_kd_milli);
    cursor = AppendU16(cursor, params.dm_max_velocity_mrad_s);
    cursor = AppendU16(cursor, params.dm_max_tracking_error_mrad);
    cursor = AppendU16(cursor, params.dm_speed_slew_mrad_s2);
    cursor = AppendU16(cursor, params.dm_position_tolerance_mrad);
    cursor = AppendU16(cursor, params.dm_velocity_tolerance_mrad_s);
    cursor = AppendU16(cursor, params.dm_settle_ms);
    (void) AppendU16(cursor, params.dm_feedback_timeout_ms);
}

void DecodePayload(const uint8_t *payload,
                   uint16_t payload_length,
                   ConfigStoreParams *params,
                   bool infrared_v16_layout)
{
    const uint8_t *cursor = payload;

    SetDefaults(params);

    cursor = ReadU32Field(cursor, &params->left_counts_per_rev);
    cursor = ReadU32Field(cursor, &params->right_counts_per_rev);
    cursor = ReadU32Field(cursor, &params->wheel_radius_mm);
    cursor = ReadU32Field(cursor, &params->wheel_track_mm);
    cursor = ReadU16Field(cursor, &params->max_wheel_rpm);
    if (payload_length >= kV2PayloadLength) {
        cursor = ReadU8Field(cursor, &params->motor_output_invert_flags);
        cursor = ReadU8Field(cursor, &params->motor_encoder_invert_flags);
    }

    cursor = ReadU8Field(cursor, &params->speed_kp_q4_4);
    cursor = ReadU8Field(cursor, &params->speed_ki_q4_4);
    cursor = ReadU8Field(cursor, &params->speed_kd_q4_4);
    cursor = ReadU8Field(cursor, &params->speed_max_duty);
    cursor = ReadU8Field(cursor, &params->speed_min_duty);

    cursor = ReadU8Field(cursor, &params->position_kp_q4_4);
    cursor = ReadU8Field(cursor, &params->position_ki_q4_4);
    cursor = ReadU8Field(cursor, &params->position_kd_q4_4);
    cursor = ReadU16Field(cursor, &params->position_max_rpm);
    cursor = ReadU16Field(cursor, &params->position_tolerance_counts);

    cursor = ReadI32Field(cursor, &params->gy931_roll_zero_mdeg);
    cursor = ReadI32Field(cursor, &params->gy931_pitch_zero_mdeg);
    cursor = ReadI32Field(cursor, &params->gy931_yaw_zero_mdeg);

    cursor = ReadI32Field(cursor, &params->imu_accel_bias_x_mg);
    cursor = ReadI32Field(cursor, &params->imu_accel_bias_y_mg);
    cursor = ReadI32Field(cursor, &params->imu_accel_bias_z_mg);
    cursor = ReadI32Field(cursor, &params->imu_gyro_bias_x_mdps);
    cursor = ReadI32Field(cursor, &params->imu_gyro_bias_y_mdps);
    cursor = ReadI32Field(cursor, &params->imu_gyro_bias_z_mdps);

    /* v3 heading fields - present in v3 and later layouts; v1/v2
     * keep the SetDefaults values. */
    if (payload_length >= kV3PayloadLength) {
        cursor = ReadI32Field(cursor, &params->heading_kp);
        cursor = ReadI32Field(cursor, &params->heading_max_correction_rpm);
        cursor = ReadI32Field(cursor, &params->heading_turn_max_rpm);
        cursor = ReadI32Field(cursor, &params->heading_turn_min_rpm);
        cursor = ReadI32Field(cursor, &params->heading_tolerance_mdeg);
        cursor = ReadU16Field(cursor, &params->heading_settle_ms);
    }
    if (payload_length >= kV4PayloadLength) {
        cursor = ReadU16Field(cursor,
                              &params->ina219_undervoltage_trip_mv);
        cursor = ReadU16Field(cursor,
                              &params->ina219_undervoltage_release_mv);
        cursor = ReadU16Field(cursor, &params->ina219_overcurrent_trip_ma);
        cursor = ReadU16Field(cursor,
                              &params->ina219_overcurrent_release_ma);
        cursor = ReadU8Field(cursor, &params->ina219_trip_samples);
        cursor = ReadU8Field(cursor, &params->ina219_release_samples);
        cursor = ReadU8Field(cursor, &params->ina219_comm_fail_samples);
        cursor = ReadU8Field(cursor, &params->ina219_latch_faults);
        cursor = ReadU8Field(cursor, &params->ina219_motion_inhibit_enable);
    }
    if (payload_length >= kV5PayloadLength) {
        for (uint8_t i = 0U; i < CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT; i++) {
            cursor = ReadU16Field(cursor, &params->grayscale_white[i]);
        }
        for (uint8_t i = 0U; i < CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT; i++) {
            cursor = ReadU16Field(cursor, &params->grayscale_black[i]);
        }
        cursor = ReadU16Field(cursor, &params->grayscale_threshold);
    }
    if ((payload_length >= kV5PayloadLength) &&
        (payload_length < kV6PayloadLength)) {
        /* v5 allowed a single threshold over the full 1..1000 range. Keep
         * that image loadable while deriving the widest valid v6 hysteresis
         * around its existing center. */
        if (params->grayscale_threshold >= 1000U) {
            params->grayscale_threshold = 999U;
        }
        const uint16_t lower_room = static_cast<uint16_t>(
            (params->grayscale_threshold - 1U) * 2U);
        const uint16_t upper_room = static_cast<uint16_t>(
            (999U - params->grayscale_threshold) * 2U);
        const uint16_t available =
            (lower_room < upper_room) ? lower_room : upper_room;
        if (params->grayscale_hysteresis > available) {
            params->grayscale_hysteresis = available;
        }
    }
    if (payload_length >= kV6PayloadLength) {
        cursor = ReadU16Field(cursor, &params->grayscale_hysteresis);
        cursor = ReadU16Field(cursor, &params->grayscale_position_floor);
        cursor = ReadU16Field(cursor, &params->grayscale_min_line_strength);
        cursor = ReadU8Field(cursor, &params->grayscale_track_mask);
        cursor = ReadI32Field(cursor, &params->linefollow_kp);
        cursor = ReadI32Field(cursor, &params->linefollow_kd);
        cursor = ReadU16Field(cursor,
                              &params->linefollow_max_correction_rpm);
        cursor = ReadU16Field(cursor, &params->linefollow_lost_hold_ms);
        cursor = ReadU16Field(cursor, &params->linefollow_lost_stop_ms);
    }
    if (payload_length >= kV7PayloadLength) {
        cursor = ReadU32Field(cursor, &params->wheel_radius_um);
    } else {
        params->wheel_radius_um = params->wheel_radius_mm * 1000U;

        /* V1-V6 shipped with the 13-PPR encoder's x4 hardware-QEI count
         * omitted. Migrate only the exact old default tuple so that an
         * explicitly calibrated legacy configuration remains untouched. */
        if ((params->left_counts_per_rev == 364U) &&
            (params->right_counts_per_rev == 364U) &&
            (params->wheel_radius_mm == 32U)) {
            params->left_counts_per_rev = 1456U;
            params->right_counts_per_rev = 1456U;
            params->wheel_radius_um = 33050U;
        }
    }
    if (payload_length >= kV8PayloadLength) {
        cursor = ReadU8Field(cursor, &params->distance_speed_mode);
        cursor = ReadU16Field(cursor, &params->distance_accel_rpm_s);
        cursor = ReadU16Field(cursor, &params->distance_decel_rpm_s);
        cursor = ReadU16Field(cursor, &params->distance_creep_rpm);
        cursor = ReadU16Field(cursor, &params->distance_stop_latency_ms);
        cursor = ReadU16Field(cursor, &params->distance_brake_margin_mm);
        cursor = ReadU16Field(cursor, &params->distance_settle_rpm);
        cursor = ReadU16Field(cursor, &params->distance_tolerance_mm);
    }
    if (payload_length >= kV9PayloadLength) {
        cursor = ReadU16Field(cursor, &params->speed_accel_rpm_s);
        cursor = ReadU16Field(cursor, &params->speed_decel_rpm_s);
    }
    if (payload_length >= kV11PayloadLength) {
        cursor = ReadU16Field(
            cursor,
            &params->linefollow_correction_slew_permille_per_second);
    }
    if (payload_length >= kV12PayloadLength) {
        cursor = ReadU16Field(cursor, &params->heading_turn_brake_ms);
        cursor = ReadU16Field(
            cursor,
            &params->heading_turn_brake_margin_mdeg);
        cursor = ReadU16Field(
            cursor,
            &params->heading_turn_settle_rate_mdps);
        cursor = ReadU16Field(cursor, &params->heading_turn_settle_rpm);
    }
    if (payload_length >= kV13PayloadLength) {
        cursor = ReadI32Field(cursor, &params->heading_lock_kp);
        cursor = ReadI32Field(cursor, &params->heading_lock_kd);
        cursor = ReadU16Field(cursor, &params->heading_lock_wake_mdeg);
        cursor = ReadU16Field(cursor, &params->heading_lock_settle_mdeg);
        cursor = ReadU16Field(cursor, &params->heading_lock_min_rpm);
        cursor = ReadU16Field(cursor, &params->heading_lock_max_rpm);
        cursor = ReadU16Field(cursor,
                              &params->heading_lock_settle_rate_mdps);
        cursor = ReadU16Field(cursor, &params->heading_lock_settle_rpm);
        cursor = ReadU16Field(cursor, &params->heading_lock_settle_ms);
        cursor = ReadU16Field(cursor, &params->heading_lock_timeout_ms);
    }
    if (payload_length >= kV14PayloadLength) {
        cursor = ReadU16Field(cursor, &params->road_align_distance_mm);
        cursor = ReadU16Field(cursor, &params->road_align_rpm);
    }
    if (payload_length >= kV15PayloadLength) {
        cursor = ReadU16Field(cursor, &params->road_turn_outer_max_rpm);
        cursor = ReadU16Field(
            cursor,
            &params->road_turn_inner_reverse_max_rpm);
    }
    if (payload_length >= kPayloadLength) {
        if (infrared_v16_layout) {
            cursor = ReadU8Field(cursor, &params->line_sensor_source);
            cursor = ReadU8Field(cursor, &params->infrared_position_invert);
            cursor = ReadU16Field(cursor, &params->infrared_position_span_raw);
            cursor = ReadU16Field(cursor, &params->infrared_adc_threshold);
            cursor = ReadU16Field(cursor, &params->infrared_adc_hysteresis);
            cursor = ReadI32Field(cursor, &params->infrared_linefollow_kp);
            cursor = ReadI32Field(cursor, &params->infrared_linefollow_kd);
            cursor = ReadU16Field(
                cursor, &params->infrared_linefollow_max_correction_rpm);
            (void) ReadU16Field(
                cursor,
                &params->
                    infrared_linefollow_correction_slew_permille_per_second);
        } else {
            cursor = ReadU16Field(cursor, &params->dm_position_kp_milli);
            cursor = ReadU16Field(cursor, &params->dm_position_kd_milli);
            cursor = ReadU16Field(cursor, &params->dm_speed_kd_milli);
            cursor = ReadU16Field(cursor, &params->dm_max_velocity_mrad_s);
            cursor = ReadU16Field(
                cursor, &params->dm_max_tracking_error_mrad);
            cursor = ReadU16Field(cursor, &params->dm_speed_slew_mrad_s2);
            cursor = ReadU16Field(
                cursor, &params->dm_position_tolerance_mrad);
            cursor = ReadU16Field(
                cursor, &params->dm_velocity_tolerance_mrad_s);
            cursor = ReadU16Field(cursor, &params->dm_settle_ms);
            (void) ReadU16Field(cursor, &params->dm_feedback_timeout_ms);
        }
    }
    (void) cursor;
}

void EncodeInfraredExtension(const ConfigStoreParams &params, uint8_t *payload)
{
    uint8_t *cursor = payload;
    cursor = AppendU8(cursor, params.line_sensor_source);
    cursor = AppendU8(cursor, params.infrared_position_invert);
    cursor = AppendU16(cursor, params.infrared_position_span_raw);
    cursor = AppendU16(cursor, params.infrared_adc_threshold);
    cursor = AppendU16(cursor, params.infrared_adc_hysteresis);
    cursor = AppendI32(cursor, params.infrared_linefollow_kp);
    cursor = AppendI32(cursor, params.infrared_linefollow_kd);
    cursor = AppendU16(
        cursor, params.infrared_linefollow_max_correction_rpm);
    (void) AppendU16(
        cursor,
        params.infrared_linefollow_correction_slew_permille_per_second);
}

void DecodeInfraredExtension(const uint8_t *payload,
                             ConfigStoreParams *params)
{
    const uint8_t *cursor = payload;
    cursor = ReadU8Field(cursor, &params->line_sensor_source);
    cursor = ReadU8Field(cursor, &params->infrared_position_invert);
    cursor = ReadU16Field(cursor, &params->infrared_position_span_raw);
    cursor = ReadU16Field(cursor, &params->infrared_adc_threshold);
    cursor = ReadU16Field(cursor, &params->infrared_adc_hysteresis);
    cursor = ReadI32Field(cursor, &params->infrared_linefollow_kp);
    cursor = ReadI32Field(cursor, &params->infrared_linefollow_kd);
    cursor = ReadU16Field(
        cursor, &params->infrared_linefollow_max_correction_rpm);
    (void) ReadU16Field(
        cursor,
        &params->infrared_linefollow_correction_slew_permille_per_second);
}

bool LoadInfraredExtension(ConfigStoreParams *params)
{
    uint8_t image[kInfraredExtensionImageLength];
    if (board::Board_FramRead(kInfraredExtensionAddress,
                              image,
                              sizeof(image)) != drivers::DRIVER_OK) {
        return false;
    }
    if ((ReadU32(&image[0]) != kInfraredExtensionMagic) ||
        (ReadU16(&image[4]) != kInfraredExtensionVersion) ||
        (ReadU16(&image[6]) != kInfraredExtensionPayloadLength)) {
        return false;
    }
    const uint32_t stored_crc = ReadU32(
        &image[kHeaderLength + kInfraredExtensionPayloadLength]);
    if (stored_crc !=
        Crc32(image, kHeaderLength + kInfraredExtensionPayloadLength)) {
        return false;
    }
    DecodeInfraredExtension(&image[kHeaderLength], params);
    return true;
}

drivers::DriverStatus SaveInfraredExtension(const ConfigStoreParams &params)
{
    uint8_t image[kInfraredExtensionImageLength];
    WriteU32(&image[0], kInfraredExtensionMagic);
    WriteU16(&image[4], kInfraredExtensionVersion);
    WriteU16(&image[6], kInfraredExtensionPayloadLength);
    EncodeInfraredExtension(params, &image[kHeaderLength]);
    WriteU32(&image[kHeaderLength + kInfraredExtensionPayloadLength],
             Crc32(image,
                   kHeaderLength + kInfraredExtensionPayloadLength));
    return board::Board_FramWrite(kInfraredExtensionAddress,
                                  image,
                                  sizeof(image));
}

bool ValidateParams(const ConfigStoreParams &params)
{
    ConfigStoreParams saved = g_params;
    g_params = params;

    for (uint8_t i = 0U; i < ConfigStore_ParamCount(); i++) {
        int32_t value = 0;
        int32_t min_value = 0;
        int32_t max_value = 0;
        (void) ConfigStore_GetValue(kParamDescriptors[i].name,
                                    &value,
                                    &min_value,
                                    &max_value);
        if ((value < min_value) || (value > max_value)) {
            g_params = saved;
            return false;
        }
    }

    g_params = saved;
    if (params.speed_min_duty > params.speed_max_duty) {
        return false;
    }
    if (params.heading_turn_min_rpm > params.heading_turn_max_rpm) {
        return false;
    }
    if ((params.heading_lock_settle_mdeg >=
         params.heading_lock_wake_mdeg) ||
        (params.heading_lock_min_rpm > params.heading_lock_max_rpm) ||
        (params.heading_lock_max_rpm > params.max_wheel_rpm)) {
        return false;
    }
    if (params.wheel_radius_mm != (params.wheel_radius_um / 1000U)) {
        return false;
    }
    if ((params.distance_creep_rpm > params.max_wheel_rpm) ||
        (params.distance_settle_rpm > params.distance_creep_rpm)) {
        return false;
    }
    if (params.ina219_undervoltage_release_mv <=
        params.ina219_undervoltage_trip_mv) {
        return false;
    }
    if (params.ina219_overcurrent_release_ma >=
        params.ina219_overcurrent_trip_ma) {
        return false;
    }
    for (uint8_t i = 0U; i < CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT; i++) {
        if (params.grayscale_white[i] == params.grayscale_black[i]) {
            return false;
        }
    }
    const uint16_t lower_half = static_cast<uint16_t>(
        params.grayscale_hysteresis / 2U);
    const uint16_t upper_half = static_cast<uint16_t>(
        (params.grayscale_hysteresis + 1U) / 2U);
    if ((params.grayscale_threshold <= lower_half) ||
        ((static_cast<uint32_t>(params.grayscale_threshold) + upper_half) >=
         1000U) ||
        (params.linefollow_lost_stop_ms < params.linefollow_lost_hold_ms)) {
        return false;
    }
    if (params.infrared_adc_threshold != 0U) {
        const uint16_t ir_lower = static_cast<uint16_t>(
            params.infrared_adc_hysteresis / 2U);
        const uint16_t ir_upper = static_cast<uint16_t>(
            (params.infrared_adc_hysteresis + 1U) / 2U);
        if ((params.infrared_adc_threshold <= ir_lower) ||
            ((static_cast<uint32_t>(params.infrared_adc_threshold) +
              ir_upper) > 4095U) ||
            (params.infrared_position_span_raw == 0U)) {
            return false;
        }
    }
    return true;
}

int32_t ReadParamValue(const ParamDescriptor &param)
{
    const uint8_t *base = reinterpret_cast<const uint8_t *>(&g_params);
    const uint8_t *field = &base[param.offset];

    switch (param.type) {
    case PARAM_U8:
        return static_cast<int32_t>(*field);
    case PARAM_U16:
        return static_cast<int32_t>(
            *reinterpret_cast<const uint16_t *>(field));
    case PARAM_U32:
        return static_cast<int32_t>(
            *reinterpret_cast<const uint32_t *>(field));
    case PARAM_I32:
        return *reinterpret_cast<const int32_t *>(field);
    default:
        return 0;
    }
}

void WriteParamValue(const ParamDescriptor &param, int32_t value)
{
    uint8_t *base = reinterpret_cast<uint8_t *>(&g_params);
    uint8_t *field = &base[param.offset];

    switch (param.type) {
    case PARAM_U8:
        *field = static_cast<uint8_t>(value);
        break;
    case PARAM_U16:
        *reinterpret_cast<uint16_t *>(field) = static_cast<uint16_t>(value);
        break;
    case PARAM_U32:
        *reinterpret_cast<uint32_t *>(field) = static_cast<uint32_t>(value);
        break;
    case PARAM_I32:
        *reinterpret_cast<int32_t *>(field) = value;
        break;
    default:
        break;
    }
}

} /* namespace */

drivers::DriverStatus ConfigStore_Load(void)
{
    uint8_t image[kImageLength];
    ConfigStoreParams loaded = {};

    ConfigStore_ResetDefaults();

#if FEATURE_ENABLE_FRAM
    if (!board::Board_FramIsReady()) {
        g_status.load_outcome = CONFIG_LOAD_DEFAULTS_IO_ERROR;
        g_status.last_load_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return g_status.last_load_status;
    }

    drivers::DriverStatus status =
        board::Board_FramRead(kFramAddress, image, kImageLength);
    if (status != drivers::DRIVER_OK) {
        g_status.load_outcome = CONFIG_LOAD_DEFAULTS_IO_ERROR;
        g_status.last_load_status = status;
        return status;
    }

    const uint32_t magic = ReadU32(&image[0]);
    const uint16_t version = ReadU16(&image[4]);
    const uint16_t length = ReadU16(&image[6]);

    g_status.stored_length = length;
    g_status.stored_crc = 0U;

    const bool current_layout =
        (version == kVersion) && (length == kPayloadLength);
    const bool v16_layout =
        (version == kV16Version) && (length == kV16PayloadLength);
    const bool v15_layout =
        (version == kV15Version) && (length == kV15PayloadLength);
    const bool v14_layout =
        (version == kV14Version) && (length == kV14PayloadLength);
    const bool v13_layout =
        (version == kV13Version) && (length == kV13PayloadLength);
    const bool v12_layout =
        (version == kV12Version) && (length == kV12PayloadLength);
    const bool v11_layout =
        (version == kV11Version) && (length == kV11PayloadLength);
    const bool v10_layout =
        (version == kV10Version) && (length == kV10PayloadLength);
    const bool v9_layout =
        (version == kV9Version) && (length == kV9PayloadLength);
    const bool v8_layout =
        (version == kV8Version) && (length == kV8PayloadLength);
    const bool v7_layout =
        (version == kV7Version) && (length == kV7PayloadLength);
    const bool v6_layout =
        (version == kV6Version) && (length == kV6PayloadLength);
    const bool v5_layout =
        (version == kV5Version) && (length == kV5PayloadLength);
    const bool v4_layout =
        (version == kV4Version) && (length == kV4PayloadLength);
    const bool v3_layout =
        (version == kV3Version) && (length == kV3PayloadLength);
    const bool legacy_v1 =
        (version == kLegacyVersion) && (length == kLegacyPayloadLength);
    const bool legacy_v2 = (version == 2U) && (length == kV2PayloadLength);
    const bool legacy_layout =
        v16_layout || v15_layout || v14_layout || v13_layout || v12_layout || v11_layout || v10_layout || v9_layout || v8_layout || v7_layout || v6_layout ||
        v5_layout || v4_layout || v3_layout || legacy_v1 || legacy_v2;

    if ((magic != kMagic) ||
        ((!current_layout) && (!legacy_layout))) {
        g_status.loaded_from_fram = false;
        g_status.load_outcome = CONFIG_LOAD_DEFAULTS_INVALID;
        g_status.last_load_status = drivers::DRIVER_ERROR;
        return g_status.last_load_status;
    }

    const uint32_t stored_crc = ReadU32(&image[kHeaderLength + length]);
    const uint32_t actual_crc = Crc32(image, kHeaderLength + length);

    g_status.stored_crc = stored_crc;

    if (stored_crc != actual_crc) {
        g_status.loaded_from_fram = false;
        g_status.load_outcome = CONFIG_LOAD_DEFAULTS_INVALID;
        g_status.last_load_status = drivers::DRIVER_ERROR;
        return g_status.last_load_status;
    }

    bool legacy_infrared_v16 = false;
    DecodePayload(&image[kHeaderLength], length, &loaded, false);
    if (v16_layout && !ValidateParams(loaded)) {
        /* The infrared feature branch and upstream DM branch both shipped a
         * 243-byte v16 image before they were merged. Prefer the authoritative
         * upstream DM interpretation when valid; otherwise migrate the old
         * infrared layout without discarding any v1-v15 fields. */
        DecodePayload(&image[kHeaderLength], length, &loaded, true);
        legacy_infrared_v16 = true;
    }
    /* Preserve deliberate user masks. Only the exact historical default is
     * upgraded when loading an older image. */
    if ((version <= kV10Version) &&
        (loaded.grayscale_track_mask == kLegacyDefaultGrayscaleTrackMask)) {
        loaded.grayscale_track_mask = kDefaultGrayscaleTrackMask;
    }
    if (!ValidateParams(loaded)) {
        g_status.loaded_from_fram = false;
        g_status.load_outcome = CONFIG_LOAD_DEFAULTS_INVALID;
        g_status.last_load_status = drivers::DRIVER_ERROR_INVALID_ARG;
        return g_status.last_load_status;
    }

    bool infrared_extension_loaded = false;
    if (!legacy_infrared_v16) {
        ConfigStoreParams extended = loaded;
        if (LoadInfraredExtension(&extended) && ValidateParams(extended)) {
            loaded = extended;
            infrared_extension_loaded = true;
        }
    }

    g_params = loaded;
    g_status.loaded_from_fram = true;
    g_status.load_outcome = CONFIG_LOAD_FROM_FRAM;
    g_status.dirty = !current_layout || !infrared_extension_loaded;
    g_status.last_load_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
#else
    g_status.load_outcome = CONFIG_LOAD_DEFAULTS_UNSUPPORTED;
    g_status.last_load_status = drivers::DRIVER_ERROR_UNSUPPORTED;
    return g_status.last_load_status;
#endif
}

drivers::DriverStatus ConfigStore_Save(void)
{
    uint8_t image[kImageLength];

#if FEATURE_ENABLE_FRAM
    if (!board::Board_FramIsReady()) {
        g_status.last_save_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return g_status.last_save_status;
    }

    drivers::DriverStatus status = SaveInfraredExtension(g_params);
    if (status != drivers::DRIVER_OK) {
        g_status.last_save_status = status;
        return status;
    }

    WriteU32(&image[0], kMagic);
    WriteU16(&image[4], kVersion);
    WriteU16(&image[6], kPayloadLength);
    EncodePayload(g_params, &image[kHeaderLength]);

    const uint32_t crc = Crc32(image, kHeaderLength + kPayloadLength);
    WriteU32(&image[kHeaderLength + kPayloadLength], crc);

    status = board::Board_FramWrite(kFramAddress, image, kImageLength);
    g_status.last_save_status = status;
    if (status == drivers::DRIVER_OK) {
        g_status.loaded_from_fram = true;
        g_status.dirty = false;
        g_status.stored_length = kPayloadLength;
        g_status.stored_crc = crc;
    }
    return status;
#else
    g_status.last_save_status = drivers::DRIVER_ERROR_UNSUPPORTED;
    return g_status.last_save_status;
#endif
}

void ConfigStore_ResetDefaults(void)
{
    SetDefaults(&g_params);
    g_status.loaded_from_fram = false;
    g_status.dirty = true;
    g_status.stored_length = kPayloadLength;
    g_status.stored_crc = 0U;
    g_status.load_outcome = CONFIG_LOAD_DEFAULTS_EXPLICIT;
}

const ConfigStoreParams *ConfigStore_Get(void)
{
    return &g_params;
}

drivers::DriverStatus ConfigStore_Set(const char *name, int32_t value)
{
    const ParamDescriptor *param = FindParam(name);
    const ConfigStoreParams saved = g_params;

    if (param == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if ((value < param->min_value) || (value > param->max_value)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    WriteParamValue(*param, value);
    if (TextEqual(name, "wheel_radius_mm")) {
        g_params.wheel_radius_um = static_cast<uint32_t>(value) * 1000U;
    } else if (TextEqual(name, "wheel_radius_um")) {
        g_params.wheel_radius_mm = static_cast<uint32_t>(value) / 1000U;
    }
    if (!ValidateParams(g_params)) {
        g_params = saved;
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    g_status.dirty = true;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus ConfigStore_SetGrayscaleCalibration(
    const uint16_t white[CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT],
    const uint16_t black[CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT],
    uint16_t threshold)
{
    return ConfigStore_SetGrayscaleCalibration(
        white,
        black,
        threshold,
        g_params.grayscale_hysteresis,
        g_params.grayscale_position_floor,
        g_params.grayscale_min_line_strength,
        g_params.grayscale_track_mask);
}

drivers::DriverStatus ConfigStore_SetInfraredCalibration(
    uint8_t invert,
    uint16_t span_raw,
    uint16_t adc_threshold,
    uint16_t adc_hysteresis)
{
    if ((invert > 1U) || (span_raw == 0U) || (span_raw > 32767U) ||
        (adc_threshold == 0U) || (adc_threshold > 4095U) ||
        (adc_hysteresis > 1000U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    ConfigStoreParams updated = g_params;
    updated.infrared_position_invert = invert;
    updated.infrared_position_span_raw = span_raw;
    updated.infrared_adc_threshold = adc_threshold;
    updated.infrared_adc_hysteresis = adc_hysteresis;
    if (!ValidateParams(updated)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    g_params = updated;
    g_status.dirty = true;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus ConfigStore_SetGrayscaleCalibration(
    const uint16_t white[CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT],
    const uint16_t black[CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT],
    uint16_t threshold,
    uint16_t hysteresis,
    uint16_t position_floor,
    uint16_t min_line_strength,
    uint8_t track_mask)
{
    if ((white == 0) || (black == 0) || (threshold == 0U) ||
        (threshold >= 1000U) || (hysteresis > 998U) ||
        (position_floor >= 1000U) || (min_line_strength == 0U) ||
        (min_line_strength > 8000U) || (track_mask == 0U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    ConfigStoreParams updated = g_params;
    for (uint8_t i = 0U; i < CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT; i++) {
        if ((white[i] > 4095U) || (black[i] > 4095U) ||
            (white[i] == black[i])) {
            return drivers::DRIVER_ERROR_INVALID_ARG;
        }
        updated.grayscale_white[i] = white[i];
        updated.grayscale_black[i] = black[i];
    }
    updated.grayscale_threshold = threshold;
    updated.grayscale_hysteresis = hysteresis;
    updated.grayscale_position_floor = position_floor;
    updated.grayscale_min_line_strength = min_line_strength;
    updated.grayscale_track_mask = track_mask;

    if (!ValidateParams(updated)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    g_params = updated;
    g_status.dirty = true;
    return drivers::DRIVER_OK;
}

bool ConfigStore_GetValue(const char *name,
                          int32_t *value,
                          int32_t *min_value,
                          int32_t *max_value)
{
    const ParamDescriptor *param = FindParam(name);

    if (param == 0) {
        return false;
    }
    if (value != 0) {
        *value = ReadParamValue(*param);
    }
    if (min_value != 0) {
        *min_value = param->min_value;
    }
    if (max_value != 0) {
        *max_value = param->max_value;
    }
    return true;
}

uint8_t ConfigStore_ParamCount(void)
{
    return static_cast<uint8_t>(
        sizeof(kParamDescriptors) / sizeof(kParamDescriptors[0]));
}

const char *ConfigStore_ParamName(uint8_t index)
{
    return (index < ConfigStore_ParamCount()) ?
           kParamDescriptors[index].name : 0;
}

const ConfigStoreStatus *ConfigStore_GetStatus(void)
{
    return &g_status;
}

} /* namespace app */
