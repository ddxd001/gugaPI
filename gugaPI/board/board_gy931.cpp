#include "board/board_gy931.h"

#include "board/board_pins.h"
#include "drivers/soft_i2c/soft_i2c.h"

namespace board {
namespace {

drivers::SoftI2cContext g_gy931I2cContext;
drivers::Gy931Context g_gy931Context;
uint8_t g_gy931Address = BOARD_GY931_I2C_ADDRESS;

const drivers::SoftI2cConfig g_gy931I2cConfig = {
    BOARD_GY931_I2C_SCL_PORT,
    BOARD_GY931_I2C_SCL_PIN,
    BOARD_GY931_I2C_SCL_IOMUX,
    BOARD_GY931_I2C_SDA_PORT,
    BOARD_GY931_I2C_SDA_PIN,
    BOARD_GY931_I2C_SDA_IOMUX,
    BOARD_GY931_I2C_HALF_PERIOD_CYCLES,
    BOARD_GY931_I2C_TIMEOUT_CYCLES
};

drivers::Gy931Config g_gy931Config = {
    &g_gy931I2cContext,
    BOARD_GY931_I2C_ADDRESS
};

drivers::DriverStatus EnsureSoftI2cReady(void)
{
    if (drivers::SoftI2c_IsReady(&g_gy931I2cContext)) {
        return drivers::DRIVER_OK;
    }

    return drivers::SoftI2c_Init(&g_gy931I2cContext, &g_gy931I2cConfig);
}

bool IsAddressValid(uint8_t address)
{
    return (address >= 0x08U) && (address <= 0x77U);
}

} /* namespace */

drivers::DriverStatus Board_Gy931Init(void)
{
    drivers::DriverStatus status = EnsureSoftI2cReady();
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    g_gy931Config.i2c_address = g_gy931Address;
    status = drivers::Gy931_Init(&g_gy931Context, &g_gy931Config);
    if (status == drivers::DRIVER_OK) {
        g_gy931Address = g_gy931Config.i2c_address;
    }
    return status;
}

drivers::DriverStatus Board_Gy931Probe(void)
{
    drivers::DriverStatus status = EnsureSoftI2cReady();
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    if (drivers::Gy931_IsReady(&g_gy931Context)) {
        return drivers::Gy931_Probe(&g_gy931Context);
    }

    return drivers::SoftI2c_ProbeAddress(&g_gy931I2cContext,
                                         g_gy931Address);
}

drivers::DriverStatus Board_Gy931ProbeAddress(uint8_t address)
{
    if (!IsAddressValid(address)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    drivers::DriverStatus status = EnsureSoftI2cReady();
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    return drivers::SoftI2c_ProbeAddress(&g_gy931I2cContext, address);
}

drivers::DriverStatus Board_Gy931SetAddress(uint8_t address)
{
    if (!IsAddressValid(address)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    drivers::DriverStatus status = EnsureSoftI2cReady();
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    g_gy931Address = address;
    g_gy931Config.i2c_address = address;
    return drivers::Gy931_Init(&g_gy931Context, &g_gy931Config);
}

drivers::DriverStatus Board_Gy931RecoverBus(void)
{
    drivers::DriverStatus status = EnsureSoftI2cReady();
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    return drivers::SoftI2c_RecoverBus(&g_gy931I2cContext);
}

drivers::DriverStatus Board_Gy931GetBusStatus(bool *scl_high,
                                              bool *sda_high)
{
    if ((scl_high == 0) || (sda_high == 0)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    drivers::DriverStatus status = EnsureSoftI2cReady();
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    drivers::SoftI2cBusStatus bus_status;
    status = drivers::SoftI2c_GetBusStatus(&g_gy931I2cContext, &bus_status);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    *scl_high = bus_status.scl_high;
    *sda_high = bus_status.sda_high;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_Gy931ReadAngles(drivers::Gy931Angles *angles)
{
    if (!drivers::Gy931_IsReady(&g_gy931Context)) {
        const drivers::DriverStatus status = Board_Gy931Init();
        if (status != drivers::DRIVER_OK) {
            return status;
        }
    }

    return drivers::Gy931_ReadAngles(&g_gy931Context, angles);
}

drivers::DriverStatus Board_Gy931ReadSample(drivers::Gy931Sample *sample)
{
    if (!drivers::Gy931_IsReady(&g_gy931Context)) {
        const drivers::DriverStatus status = Board_Gy931Init();
        if (status != drivers::DRIVER_OK) {
            return status;
        }
    }

    return drivers::Gy931_ReadSample(&g_gy931Context, sample);
}

drivers::DriverStatus Board_Gy931ReadRawRegisters(uint8_t start_reg,
                                                  int16_t *words,
                                                  uint8_t word_count)
{
    if (!drivers::Gy931_IsReady(&g_gy931Context)) {
        const drivers::DriverStatus status = Board_Gy931Init();
        if (status != drivers::DRIVER_OK) {
            return status;
        }
    }

    return drivers::Gy931_ReadRawRegisters(&g_gy931Context,
                                           start_reg,
                                           words,
                                           word_count);
}

drivers::DriverStatus Board_Gy931ReadAlgorithm(
    drivers::Gy931Algorithm *algorithm)
{
    if (!drivers::Gy931_IsReady(&g_gy931Context)) {
        const drivers::DriverStatus status = Board_Gy931Init();
        if (status != drivers::DRIVER_OK) {
            return status;
        }
    }

    return drivers::Gy931_ReadAlgorithm(&g_gy931Context, algorithm);
}

drivers::DriverStatus Board_Gy931SetAlgorithmTemporary(
    drivers::Gy931Algorithm algorithm)
{
    if (!drivers::Gy931_IsReady(&g_gy931Context)) {
        const drivers::DriverStatus status = Board_Gy931Init();
        if (status != drivers::DRIVER_OK) {
            return status;
        }
    }

    return drivers::Gy931_SetAlgorithmTemporary(&g_gy931Context,
                                                 algorithm);
}

bool Board_Gy931IsReady(void)
{
    return drivers::Gy931_IsReady(&g_gy931Context);
}

uint8_t Board_Gy931Address(void)
{
    return g_gy931Address;
}

} /* namespace board */
