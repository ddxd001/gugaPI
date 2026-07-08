#include "app/config_store.h"

#include <stddef.h>
#include <string.h>

#include "board/board_fram.h"
#include "config/feature_config.h"

namespace app {
namespace {

static const uint16_t kFramAddress = 0x0000U;
static const uint32_t kMagic = 0x47504643U; /* "CFPG" little-endian */
static const uint16_t kVersion = 2U;
static const uint16_t kLegacyVersion = 1U;
static const uint16_t kLegacyPayloadLength = 66U;
static const uint16_t kPayloadLength = 68U;
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
    drivers::DRIVER_ERROR_NOT_INITIALIZED
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
      PARAM_OFFSET(imu_gyro_bias_z_mdps), -2000000, 2000000 }
};

#undef PARAM_OFFSET

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

    params->left_counts_per_rev = 22400U;
    params->right_counts_per_rev = 22400U;
    params->wheel_radius_mm = 32U;
    params->wheel_track_mm = 160U;
    params->max_wheel_rpm = 1000U;
    params->motor_output_invert_flags = 0U;
    params->motor_encoder_invert_flags = 0U;

    params->speed_kp_q4_4 = 1U;
    params->speed_ki_q4_4 = 1U;
    params->speed_kd_q4_4 = 0U;
    params->speed_max_duty = 100U;
    params->speed_min_duty = 0U;

    params->position_kp_q4_4 = 15U;
    params->position_ki_q4_4 = 0U;
    params->position_kd_q4_4 = 0U;
    params->position_max_rpm = 8U;
    params->position_tolerance_counts = 50U;
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
    (void) AppendI32(cursor, params.imu_gyro_bias_z_mdps);
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
    if (payload_length >= kPayloadLength) {
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
    (void) ReadI32Field(cursor, &params->imu_gyro_bias_z_mdps);
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
        g_status.last_load_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return g_status.last_load_status;
    }

    drivers::DriverStatus status =
        board::Board_FramRead(kFramAddress, image, kImageLength);
    if (status != drivers::DRIVER_OK) {
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
    const bool legacy_layout =
        (version == kLegacyVersion) && (length == kLegacyPayloadLength);

    if ((magic != kMagic) || ((!current_layout) && (!legacy_layout))) {
        g_status.loaded_from_fram = false;
        g_status.last_load_status = drivers::DRIVER_ERROR;
        return g_status.last_load_status;
    }

    const uint32_t stored_crc = ReadU32(&image[kHeaderLength + length]);
    const uint32_t actual_crc = Crc32(image, kHeaderLength + length);

    g_status.stored_crc = stored_crc;

    if (stored_crc != actual_crc) {
        g_status.loaded_from_fram = false;
        g_status.last_load_status = drivers::DRIVER_ERROR;
        return g_status.last_load_status;
    }

    DecodePayload(&image[kHeaderLength], length, &loaded);
    if (!ValidateParams(loaded)) {
        g_status.loaded_from_fram = false;
        g_status.last_load_status = drivers::DRIVER_ERROR_INVALID_ARG;
        return g_status.last_load_status;
    }

    g_params = loaded;
    g_status.loaded_from_fram = true;
    g_status.dirty = false;
    g_status.last_load_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
#else
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
