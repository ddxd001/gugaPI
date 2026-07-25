#include "app/config_store.h"

#include <stddef.h>
#include <string.h>

#include "board/board_fram.h"
#include "config/feature_config.h"

namespace app {
namespace {

static const uint16_t kFramAddress = 0x0000U;
static const uint32_t kMagic = 0x47504643U; /* "CFPG" little-endian */
static const uint16_t kVersion = 6U;
static const uint16_t kV5Version = 5U;
static const uint16_t kV5PayloadLength = 137U;
static const uint16_t kV4Version = 4U;
static const uint16_t kV4PayloadLength = 103U;
static const uint16_t kV3Version = 3U;
static const uint16_t kV3PayloadLength = 90U;
static const uint16_t kLegacyVersion = 1U;
static const uint16_t kLegacyPayloadLength = 66U;
static const uint16_t kV2PayloadLength = 68U; /* v2 layout length (motor_invert guard) */
static const uint16_t kPayloadLength = 158U; /* v6: +21-byte grayscale/LF tuning */
static const uint16_t kHeaderLength = 8U;
static const uint16_t kCrcLength = 4U;
static const uint16_t kImageLength =
    kHeaderLength + kPayloadLength + kCrcLength;
static const uint32_t kCrc32Init = 0xFFFFFFFFU;

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
      PARAM_OFFSET(linefollow_lost_stop_ms), 1, 10000 }
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

    params->left_counts_per_rev = 364U;
    params->right_counts_per_rev = 364U;
    params->wheel_radius_mm = 32U;
    params->wheel_track_mm = 160U;
    params->max_wheel_rpm = 1000U;
    params->motor_output_invert_flags = 0x01U;
    params->motor_encoder_invert_flags = 0x01U;

    params->speed_kp_q4_4 = 1U;
    params->speed_ki_q4_4 = 1U;
    params->speed_kd_q4_4 = 0U;
    params->speed_max_duty = 50U;
    params->speed_min_duty = 6U;

    params->position_kp_q4_4 = 15U;
    params->position_ki_q4_4 = 0U;
    params->position_kd_q4_4 = 0U;
    params->position_max_rpm = 8U;
    params->position_tolerance_counts = 50U;

    /* IMU heading closed-loop (conservative bring-up values).
     * correction_rpm = error_mdeg * heading_kp / 1e6, so heading_kp=1000
     * means 1 RPM per degree of error. */
    params->heading_kp = 1000;               /* 1 RPM/deg */
    params->heading_max_correction_rpm = 30;
    params->heading_turn_max_rpm = 60;
    params->heading_turn_min_rpm = 20;
    params->heading_tolerance_mdeg = 3000;  /* 3 deg */
    params->heading_settle_ms = 300U;

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

    for (uint8_t i = 0U; i < CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT; i++) {
        params->grayscale_white[i] = 4095U;
        params->grayscale_black[i] = 0U;
    }
    params->grayscale_threshold = 500U;
    params->grayscale_hysteresis = 300U;
    params->grayscale_position_floor = 100U;
    params->grayscale_min_line_strength = 600U;
    params->grayscale_track_mask = 0x3CU;

    params->linefollow_kp = 10000;
    params->linefollow_kd = 0;
    params->linefollow_max_correction_rpm = 30U;
    params->linefollow_lost_hold_ms = 150U;
    params->linefollow_lost_stop_ms = 500U;
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
    (void) AppendU16(cursor, params.linefollow_lost_stop_ms);
}

void DecodePayload(const uint8_t *payload,
                   uint16_t payload_length,
                   ConfigStoreParams *params)
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
        (payload_length < kPayloadLength)) {
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
    if (payload_length >= kPayloadLength) {
        cursor = ReadU16Field(cursor, &params->grayscale_hysteresis);
        cursor = ReadU16Field(cursor, &params->grayscale_position_floor);
        cursor = ReadU16Field(cursor, &params->grayscale_min_line_strength);
        cursor = ReadU8Field(cursor, &params->grayscale_track_mask);
        cursor = ReadI32Field(cursor, &params->linefollow_kp);
        cursor = ReadI32Field(cursor, &params->linefollow_kd);
        cursor = ReadU16Field(cursor,
                              &params->linefollow_max_correction_rpm);
        cursor = ReadU16Field(cursor, &params->linefollow_lost_hold_ms);
        (void) ReadU16Field(cursor, &params->linefollow_lost_stop_ms);
    }
    (void) cursor;
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

    DecodePayload(&image[kHeaderLength], length, &loaded);
    if (!ValidateParams(loaded)) {
        g_status.loaded_from_fram = false;
        g_status.load_outcome = CONFIG_LOAD_DEFAULTS_INVALID;
        g_status.last_load_status = drivers::DRIVER_ERROR_INVALID_ARG;
        return g_status.last_load_status;
    }

    g_params = loaded;
    g_status.loaded_from_fram = true;
    g_status.load_outcome = CONFIG_LOAD_FROM_FRAM;
    g_status.dirty = false;
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

    WriteU32(&image[0], kMagic);
    WriteU16(&image[4], kVersion);
    WriteU16(&image[6], kPayloadLength);
    EncodePayload(g_params, &image[kHeaderLength]);

    const uint32_t crc = Crc32(image, kHeaderLength + kPayloadLength);
    WriteU32(&image[kHeaderLength + kPayloadLength], crc);

    const drivers::DriverStatus status =
        board::Board_FramWrite(kFramAddress, image, kImageLength);
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
    if ((white == 0) || (black == 0) || (threshold == 0U) ||
        (threshold >= 1000U)) {
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
