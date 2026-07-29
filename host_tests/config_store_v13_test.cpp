#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/config_store.h"
#include "board/board_fram.h"

namespace {

uint8_t g_fram[8192] = {};

void WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

uint16_t ReadU16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

void WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    data[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
    data[3] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

uint32_t Crc32(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint16_t i = 0U; i < length; i++) {
        crc ^= static_cast<uint32_t>(data[i]) << 24U;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x80000000U)
                ? ((crc << 1U) ^ 0x04C11DB7U)
                : (crc << 1U);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

} /* namespace */

namespace board {

bool Board_FramIsReady(void)
{
    return true;
}

drivers::DriverStatus Board_FramRead(uint16_t address,
                                     uint8_t *data,
                                     uint16_t length)
{
    if ((data == 0) ||
        (static_cast<uint32_t>(address) + length > sizeof(g_fram))) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    (void) memcpy(data, &g_fram[address], length);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_FramWrite(uint16_t address,
                                      const uint8_t *data,
                                      uint16_t length)
{
    if ((data == 0) ||
        (static_cast<uint32_t>(address) + length > sizeof(g_fram))) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    (void) memcpy(&g_fram[address], data, length);
    return drivers::DRIVER_OK;
}

} /* namespace board */

int main(void)
{
    using namespace app;
    static const uint16_t kHeaderLength = 8U;
    static const uint16_t kV16PayloadLength = 243U;
    static const uint16_t kV15PayloadLength = 223U;
    static const uint16_t kV14PayloadLength = 219U;
    static const uint16_t kV13PayloadLength = 215U;
    static const uint16_t kInfraredExtensionAddress = 0x1FD0U;
    static const uint16_t kInfraredExtensionLength = 32U;

    ConfigStore_ResetDefaults();
    assert(ConfigStore_Get()->road_align_distance_mm == 0U);
    assert(ConfigStore_Get()->road_align_rpm == 30U);
    assert(ConfigStore_Get()->road_turn_outer_max_rpm == 220U);
    assert(ConfigStore_Get()->road_turn_inner_reverse_max_rpm == 120U);
    assert(ConfigStore_Get()->dm_position_kp_milli == 4000U);
    assert(ConfigStore_Get()->dm_feedback_timeout_ms == 100U);
    assert(ConfigStore_Set("heading_kp", 4321) == drivers::DRIVER_OK);
    assert(ConfigStore_Set("road_align_distance_mm", 20) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Set("road_align_rpm", 45) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Save() == drivers::DRIVER_OK);
    assert(ReadU16(&g_fram[4]) == 17U);
    assert(ReadU16(&g_fram[6]) == kV16PayloadLength);
    assert(kHeaderLength + kV16PayloadLength + 4U < 0x0100U);
    assert(ConfigStore_Get()->line_sensor_source == LINE_SENSOR_SOURCE_IR3);
    assert(ConfigStore_Get()->infrared_position_span_raw == 0U);

    /* Upstream v16 stores DM parameters in the final 20 bytes. It must load
     * unchanged when no infrared extension exists. */
    WriteU16(&g_fram[4], 16U);
    WriteU32(&g_fram[kHeaderLength + kV16PayloadLength],
             Crc32(g_fram, kHeaderLength + kV16PayloadLength));
    (void) memset(&g_fram[kInfraredExtensionAddress],
                  0,
                  kInfraredExtensionLength);
    ConfigStore_ResetDefaults();
    assert(ConfigStore_Load() == drivers::DRIVER_OK);
    assert(ConfigStore_Get()->heading_kp == 4321);
    assert(ConfigStore_Get()->dm_position_kp_milli == 4000U);
    assert(ConfigStore_Get()->dm_feedback_timeout_ms == 100U);
    assert(ConfigStore_Get()->infrared_position_span_raw == 0U);
    assert(ConfigStore_GetStatus()->dirty);

    /* The pre-merge infrared branch also used version 16 and the same main
     * length. Its distinct tail is detected and migrated into the extension
     * record without losing the common v1-v15 prefix. */
    uint8_t *legacy_ir = &g_fram[kHeaderLength + kV15PayloadLength];
    legacy_ir[0] = LINE_SENSOR_SOURCE_IR3;
    legacy_ir[1] = 1U;
    WriteU16(&legacy_ir[2], 3210U);
    WriteU16(&legacy_ir[4], 2000U);
    WriteU16(&legacy_ir[6], 200U);
    WriteU32(&legacy_ir[8], 4200U);
    WriteU32(&legacy_ir[12], 700U);
    WriteU16(&legacy_ir[16], 31U);
    WriteU16(&legacy_ir[18], 25000U);
    WriteU32(&g_fram[kHeaderLength + kV16PayloadLength],
             Crc32(g_fram, kHeaderLength + kV16PayloadLength));
    ConfigStore_ResetDefaults();
    assert(ConfigStore_Load() == drivers::DRIVER_OK);
    assert(ConfigStore_Get()->heading_kp == 4321);
    assert(ConfigStore_Get()->infrared_position_invert == 1U);
    assert(ConfigStore_Get()->infrared_position_span_raw == 3210U);
    assert(ConfigStore_Get()->infrared_adc_threshold == 2000U);
    assert(ConfigStore_Get()->infrared_linefollow_kp == 4200);
    assert(ConfigStore_Get()->dm_position_kp_milli == 4000U);
    assert(ConfigStore_GetStatus()->dirty);
    assert(ConfigStore_Save() == drivers::DRIVER_OK);
    assert(ReadU16(&g_fram[4]) == 17U);

    /* A v15 image keeps every existing setting and receives the appended
     * DM-G6220 and infrared defaults, then becomes dirty until saved as v17. */
    WriteU16(&g_fram[4], 15U);
    WriteU16(&g_fram[6], kV15PayloadLength);
    WriteU32(&g_fram[kHeaderLength + kV15PayloadLength],
             Crc32(g_fram, kHeaderLength + kV15PayloadLength));
    (void) memset(&g_fram[kInfraredExtensionAddress],
                  0,
                  kInfraredExtensionLength);
    ConfigStore_ResetDefaults();
    assert(ConfigStore_Load() == drivers::DRIVER_OK);
    const ConfigStoreParams *params = ConfigStore_Get();
    assert(params->heading_kp == 4321);
    assert(params->road_align_distance_mm == 20U);
    assert(params->road_align_rpm == 45U);
    assert(params->line_sensor_source == LINE_SENSOR_SOURCE_IR3);
    assert(params->infrared_position_span_raw == 0U);
    assert(params->dm_position_kp_milli == 4000U);
    assert(params->dm_position_kd_milli == 400U);
    assert(params->dm_speed_kd_milli == 500U);
    assert(params->dm_max_velocity_mrad_s == 2000U);
    assert(params->dm_max_tracking_error_mrad == 250U);
    assert(params->dm_speed_slew_mrad_s2 == 2000U);
    assert(params->dm_position_tolerance_mrad == 10U);
    assert(params->dm_velocity_tolerance_mrad_s == 80U);
    assert(params->dm_settle_ms == 200U);
    assert(params->dm_feedback_timeout_ms == 100U);
    assert(ConfigStore_GetStatus()->dirty);
    assert(ConfigStore_Save() == drivers::DRIVER_OK);
    assert(ReadU16(&g_fram[4]) == 17U);
    assert(ReadU16(&g_fram[6]) == kV16PayloadLength);

    /* A v14 image retains alignment settings and receives new asymmetric
     * turn defaults without changing any prior field offset. */
    WriteU16(&g_fram[4], 14U);
    WriteU16(&g_fram[6], kV14PayloadLength);
    WriteU32(&g_fram[kHeaderLength + kV14PayloadLength],
             Crc32(g_fram, kHeaderLength + kV14PayloadLength));

    ConfigStore_ResetDefaults();
    assert(ConfigStore_Load() == drivers::DRIVER_OK);
    params = ConfigStore_Get();
    assert(params->heading_kp == 4321);
    assert(params->road_align_distance_mm == 20U);
    assert(params->road_align_rpm == 45U);
    assert(params->road_turn_outer_max_rpm == 220U);
    assert(params->road_turn_inner_reverse_max_rpm == 120U);
    assert(ConfigStore_GetStatus()->dirty);
    assert(ConfigStore_GetStatus()->stored_length == kV14PayloadLength);

    assert(ConfigStore_Save() == drivers::DRIVER_OK);
    assert(ReadU16(&g_fram[4]) == 17U);
    assert(ReadU16(&g_fram[6]) == kV16PayloadLength);
    assert(!ConfigStore_GetStatus()->dirty);

    /* Convert the freshly encoded prefix into a valid historical v13 image. */
    WriteU16(&g_fram[4], 13U);
    WriteU16(&g_fram[6], kV13PayloadLength);
    WriteU32(&g_fram[kHeaderLength + kV13PayloadLength],
             Crc32(g_fram, kHeaderLength + kV13PayloadLength));

    ConfigStore_ResetDefaults();
    assert(ConfigStore_Load() == drivers::DRIVER_OK);
    params = ConfigStore_Get();
    assert(params->heading_kp == 4321);
    assert(params->heading_lock_kp == 1500);
    assert(params->heading_lock_kd == 250);
    assert(params->heading_lock_wake_mdeg == 2000U);
    assert(params->heading_lock_settle_mdeg == 800U);
    assert(params->heading_lock_timeout_ms == 3000U);
    assert(params->road_align_distance_mm == 0U);
    assert(params->road_align_rpm == 30U);
    assert(params->road_turn_outer_max_rpm == 220U);
    assert(params->road_turn_inner_reverse_max_rpm == 120U);
    assert(ConfigStore_GetStatus()->dirty);
    assert(ConfigStore_GetStatus()->stored_length == kV13PayloadLength);

    assert(ConfigStore_Save() == drivers::DRIVER_OK);
    assert(ReadU16(&g_fram[4]) == 17U);
    assert(ReadU16(&g_fram[6]) == kV16PayloadLength);
    assert(!ConfigStore_GetStatus()->dirty);

    assert(ConfigStore_Set("heading_lock_settle_mdeg", 2500) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ConfigStore_Set("heading_lock_wake_mdeg", 3000) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Set("heading_lock_settle_mdeg", 2500) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Set("heading_lock_min_rpm", 40) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ConfigStore_Set("max_wheel_rpm", 20) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ConfigStore_Set("road_align_distance_mm", 301) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ConfigStore_Set("road_align_rpm", 0) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ConfigStore_Set("road_align_rpm", 301) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ConfigStore_Set("road_turn_outer_max_rpm", 0) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ConfigStore_Set("road_turn_outer_max_rpm", 1001) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ConfigStore_Set("road_turn_inner_reverse_max_rpm", -1) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ConfigStore_Set("road_turn_inner_reverse_max_rpm", 1001) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ConfigStore_Set("line_sensor_source", 2) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ConfigStore_SetInfraredCalibration(0U, 100U, 2000U, 200U) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Set("ir_lf_kp", 4200) == drivers::DRIVER_OK);
    assert(ConfigStore_Set("ir_lf_kd", 700) == drivers::DRIVER_OK);
    assert(ConfigStore_Set("dm_max_velocity_mrad_s", 20001) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ConfigStore_Set("dm_max_velocity_mrad_s", 20000) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Set("dm_max_velocity_mrad_s", 0) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Set("dm_max_tracking_error_mrad", 251) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ConfigStore_Set("dm_feedback_timeout_ms", 49) ==
           drivers::DRIVER_ERROR_INVALID_ARG);

    assert(ConfigStore_Set("heading_lock_kp", 1750) == drivers::DRIVER_OK);
    assert(ConfigStore_Set("heading_lock_kd", 300) == drivers::DRIVER_OK);
    assert(ConfigStore_Set("road_align_distance_mm", 35) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Set("road_align_rpm", 80) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Set("road_turn_outer_max_rpm", 210) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Set("road_turn_inner_reverse_max_rpm", 140) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Set("dm_position_kp_milli", 4500) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Set("dm_feedback_timeout_ms", 120) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Save() == drivers::DRIVER_OK);
    ConfigStore_ResetDefaults();
    assert(ConfigStore_Load() == drivers::DRIVER_OK);
    params = ConfigStore_Get();
    assert(params->heading_lock_kp == 1750);
    assert(params->heading_lock_kd == 300);
    assert(params->heading_lock_wake_mdeg == 3000U);
    assert(params->heading_lock_settle_mdeg == 2500U);
    assert(params->road_align_distance_mm == 35U);
    assert(params->road_align_rpm == 80U);
    assert(params->road_turn_outer_max_rpm == 210U);
    assert(params->road_turn_inner_reverse_max_rpm == 140U);
    assert(params->infrared_position_span_raw == 100U);
    assert(params->infrared_adc_threshold == 2000U);
    assert(params->infrared_adc_hysteresis == 200U);
    assert(params->infrared_linefollow_kp == 4200);
    assert(params->infrared_linefollow_kd == 700);
    assert(params->dm_position_kp_milli == 4500U);
    assert(params->dm_feedback_timeout_ms == 120U);
    assert(!ConfigStore_GetStatus()->dirty);

    puts("config store v17 main+infrared extension migration ok");
    return 0;
}
