#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/config_store.h"
#include "app/fram_layout.h"
#include "board/board_fram.h"

namespace {

uint8_t g_fram[8192];
bool g_read_fail = false;
bool g_write_fail = false;

uint16_t ReadU16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(
               static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t ReadU32(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8U) |
           (static_cast<uint32_t>(data[2]) << 16U) |
           (static_cast<uint32_t>(data[3]) << 24U);
}

void WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8U);
    data[2] = static_cast<uint8_t>(value >> 16U);
    data[3] = static_cast<uint8_t>(value >> 24U);
}

uint32_t ConfigBankCrc(const uint8_t *image, uint16_t payload_length)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint16_t i = 0U; i < 12U; i++) {
        crc ^= static_cast<uint32_t>(image[i]) << 24U;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = ((crc & 0x80000000U) != 0U)
                ? (crc << 1U) ^ 0x04C11DB7U
                : crc << 1U;
        }
    }
    for (uint16_t i = 13U;
         i < static_cast<uint16_t>(16U + payload_length);
         i++) {
        crc ^= static_cast<uint32_t>(image[i]) << 24U;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = ((crc & 0x80000000U) != 0U)
                ? (crc << 1U) ^ 0x04C11DB7U
                : crc << 1U;
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
    if (g_read_fail) {
        return drivers::DRIVER_ERROR;
    }
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
    if (g_write_fail) {
        return drivers::DRIVER_ERROR;
    }
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

    assert(FRAM_CONFIG_BANK_A_ADDRESS == 0x0000U);
    assert(FRAM_CONFIG_BANK_B_ADDRESS == 0x0400U);
    assert(FRAM_SEQ_HEADER_ADDRESS == 0x0800U);
    assert(FRAM_SELF_TEST_ADDRESS == 0x1FF8U);
    assert(FRAM_SELF_TEST_ADDRESS + 8U == sizeof(g_fram));

    (void) memset(g_fram, 0xCC, sizeof(g_fram));
    ConfigStore_ResetDefaults();
    assert(ConfigStore_ParamCount() == 130U);
    assert(ConfigStore_Get()->linefollow_max_steering_permille == 400U);
    assert(ConfigStore_Get()->linefollow_deadband_mpos == 20U);
    assert(ConfigStore_Get()->task0_cruise_rpm == 110U);
    assert(ConfigStore_Get()->task0_approach_rpm == 60U);
    assert(ConfigStore_Get()->task0_lap_mm == 6142U);
    assert(ConfigStore_Get()->ball_kp_mdeg_per_0p1mm == 10);
    assert(ConfigStore_Get()->ball_map_angle_mdeg[0] == -8000);
    assert(ConfigStore_Get()->ball_map_dm_mrad[4] == 1000);

    /* Former layouts are intentionally unsupported and load defaults. */
    assert(ConfigStore_Load() == drivers::DRIVER_ERROR);
    assert(!ConfigStore_GetStatus()->loaded_from_fram);
    assert(ConfigStore_GetStatus()->dirty);
    assert(ConfigStore_GetStatus()->active_bank == 0xFFU);

    assert(ConfigStore_Set("heading_kp", 4321) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Save() == drivers::DRIVER_OK);
    assert(ReadU32(&g_fram[0]) == 0x31464347U);
    assert(ReadU16(&g_fram[4]) == 1U);
    assert(ReadU16(&g_fram[6]) == 315U);
    assert(ReadU32(&g_fram[8]) == 1U);
    assert(g_fram[12] == 0xA5U);
    assert(ConfigStore_GetStatus()->active_bank == 0U);
    assert(ConfigStore_GetStatus()->generation == 1U);
    assert(ConfigStore_GetStatus()->payload_capacity == 1004U);

    /* The retired 20-byte infrared payload remains reserved so every later
     * field keeps its deployed v1 offset. New images always zero the slot. */
    for (uint16_t i = 0U; i < 20U; i++) {
        assert(g_fram[16U + 243U + i] == 0U);
        g_fram[16U + 243U + i] = static_cast<uint8_t>(0x80U + i);
    }
    WriteU32(&g_fram[16U + 315U], ConfigBankCrc(&g_fram[0], 315U));
    ConfigStore_ResetDefaults();
    assert(ConfigStore_Load() == drivers::DRIVER_OK);
    assert(ConfigStore_Get()->heading_kp == 4321);
    assert(ConfigStore_Get()->ball_kp_mdeg_per_0p1mm == 10);

    /* The preceding steering-only 307-byte extension remains readable; new
     * soft-deadband and task 0 fields receive current defaults. */
    g_fram[6] = 307U & 0xFFU;
    g_fram[7] = 307U >> 8U;
    WriteU32(&g_fram[16U + 307U], ConfigBankCrc(&g_fram[0], 307U));
    ConfigStore_ResetDefaults();
    assert(ConfigStore_Load() == drivers::DRIVER_OK);
    assert(ConfigStore_GetStatus()->stored_length == 307U);
    assert(ConfigStore_Get()->heading_kp == 4321);
    assert(ConfigStore_Get()->linefollow_max_steering_permille == 400U);
    assert(ConfigStore_Get()->linefollow_deadband_mpos == 20U);
    assert(ConfigStore_Get()->task0_cruise_rpm == 110U);
    assert(ConfigStore_Get()->task0_approach_rpm == 60U);
    assert(ConfigStore_Get()->task0_lap_mm == 6142U);

    /* The deployed 305-byte layout remains readable. Its missing extension
     * receives the safe historical 400-permille steering limit. */
    g_fram[6] = 305U & 0xFFU;
    g_fram[7] = 305U >> 8U;
    WriteU32(&g_fram[16U + 305U], ConfigBankCrc(&g_fram[0], 305U));
    ConfigStore_ResetDefaults();
    assert(ConfigStore_Load() == drivers::DRIVER_OK);
    assert(ConfigStore_GetStatus()->stored_length == 305U);
    assert(ConfigStore_Get()->heading_kp == 4321);
    assert(ConfigStore_Get()->linefollow_max_steering_permille == 400U);
    assert(ConfigStore_Get()->linefollow_deadband_mpos == 20U);
    assert(ConfigStore_Get()->task0_lap_mm == 6142U);

    /* Atomic mapping accepts a reversed mechanism. */
    const int16_t angles[5] = { -8000, -4000, 0, 4000, 8000 };
    const int16_t reversed[5] = { 1000, 500, 0, -500, -1000 };
    assert(ConfigStore_SetBallMap(angles, reversed) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Set("ball_kp_mdeg_per_0p1mm", 25) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Set("lf_max_ratio_permille", 500) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Save() == drivers::DRIVER_OK);
    assert(ReadU32(&g_fram[0x0408]) == 2U);
    assert(ReadU16(&g_fram[0x0406]) == 315U);
    assert(g_fram[0x040CU] == 0xA5U);
    for (uint16_t i = 0U; i < 20U; i++) {
        assert(g_fram[0x0400U + 16U + 243U + i] == 0U);
    }
    assert(ConfigStore_GetStatus()->active_bank == 1U);

    ConfigStore_ResetDefaults();
    assert(ConfigStore_Load() == drivers::DRIVER_OK);
    assert(ConfigStore_Get()->heading_kp == 4321);
    assert(ConfigStore_Get()->ball_kp_mdeg_per_0p1mm == 25);
    assert(ConfigStore_Get()->ball_map_dm_mrad[0] == 1000);
    assert(ConfigStore_Get()->ball_map_dm_mrad[4] == -1000);
    assert(ConfigStore_Get()->linefollow_max_steering_permille == 500U);
    assert(ConfigStore_Get()->linefollow_deadband_mpos == 20U);
    assert(ConfigStore_Get()->task0_cruise_rpm == 110U);
    assert(ConfigStore_GetStatus()->active_bank == 1U);
    assert(!ConfigStore_GetStatus()->dirty);

    /* Generation zero is newer than UINT32_MAX after wraparound. */
    WriteU32(&g_fram[8], 0xFFFFFFFFU);
    WriteU32(&g_fram[16U + 305U], ConfigBankCrc(&g_fram[0], 305U));
    WriteU32(&g_fram[0x0408U], 0U);
    WriteU32(&g_fram[0x0400U + 16U + 315U],
             ConfigBankCrc(&g_fram[0x0400U], 315U));
    ConfigStore_ResetDefaults();
    assert(ConfigStore_Load() == drivers::DRIVER_OK);
    assert(ConfigStore_GetStatus()->active_bank == 1U);
    assert(ConfigStore_GetStatus()->generation == 0U);
    assert(ConfigStore_Get()->ball_kp_mdeg_per_0p1mm == 25);
    assert(ConfigStore_Get()->linefollow_max_steering_permille == 500U);

    /* Corrupting the newest bank falls back to the prior valid copy. */
    g_fram[0x0410U] ^= 0x01U;
    ConfigStore_ResetDefaults();
    assert(ConfigStore_Load() == drivers::DRIVER_OK);
    assert(ConfigStore_GetStatus()->active_bank == 0U);
    assert(ConfigStore_Get()->heading_kp == 4321);
    assert(ConfigStore_Get()->ball_kp_mdeg_per_0p1mm == 10);
    assert(ConfigStore_Get()->linefollow_max_steering_permille == 400U);

    /* A failed inactive-bank write leaves the active bank loadable. */
    assert(ConfigStore_Set("heading_kp", 9999) ==
           drivers::DRIVER_OK);
    g_write_fail = true;
    assert(ConfigStore_Save() == drivers::DRIVER_ERROR);
    g_write_fail = false;
    ConfigStore_ResetDefaults();
    assert(ConfigStore_Load() == drivers::DRIVER_OK);
    assert(ConfigStore_Get()->heading_kp == 4321);

    const int16_t repeated[5] = { 1000, 500, 0, 0, -1000 };
    assert(ConfigStore_SetBallMap(angles, repeated) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ConfigStore_Set("ball_degraded_angle_mdeg", 9000) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ConfigStore_Set("ball_max_angle_mdeg", 9000) ==
           drivers::DRIVER_OK);
    assert(ConfigStore_Set("ball_degraded_angle_mdeg", 9000) ==
           drivers::DRIVER_OK);

    /* Formatting invalidates both banks but never auto-saves defaults. */
    assert(ConfigStore_Format() == drivers::DRIVER_OK);
    assert(g_fram[12] == 0U);
    assert(g_fram[0x040CU] == 0U);
    assert(ConfigStore_GetStatus()->dirty);
    assert(ConfigStore_Load() == drivers::DRIVER_ERROR);

    g_read_fail = true;
    assert(ConfigStore_Load() == drivers::DRIVER_ERROR);
    assert(ConfigStore_GetStatus()->load_outcome ==
           CONFIG_LOAD_DEFAULTS_IO_ERROR);
    g_read_fail = false;

    puts("config store clean layout and dual-bank tests passed");
    return 0;
}
