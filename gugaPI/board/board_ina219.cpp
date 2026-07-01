#include "board/board_ina219.h"

#include "board/board_pins.h"

namespace board {
namespace {

// SysConfig 默认不启用内部上拉，这里补上上拉，便于没有外部上拉时调试。
void ConfigureI2cPinsWithPullUps(const drivers::Ina219Config *config)
{
    DL_I2C_disableController(config->i2c);
    DL_GPIO_initPeripheralInputFunctionFeatures(config->sda_iomux,
                                                config->sda_iomux_func,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_PULL_UP,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(config->scl_iomux,
                                                config->scl_iomux_func,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_PULL_UP,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(config->sda_iomux);
    DL_GPIO_enableHiZ(config->scl_iomux);
    DL_I2C_enableController(config->i2c);
}

} /* namespace */

// 板级配置集中绑定引脚、采样电阻和电流方向，底层驱动不硬编码具体板子。
static const drivers::Ina219Config g_ina219Config = {
    BOARD_INA219_I2C_INST,
    BOARD_INA219_I2C_ADDRESS,
    BOARD_INA219_TIMEOUT_ITERATIONS,
    BOARD_INA219_SHUNT_MILLIOHMS,
    BOARD_INA219_CURRENT_LSB_UA,
    (BOARD_INA219_INVERT_CURRENT != 0U),
    BOARD_INA219_I2C_SCL_PORT,
    BOARD_INA219_I2C_SCL_PIN,
    BOARD_INA219_I2C_SCL_IOMUX,
    BOARD_INA219_I2C_SCL_IOMUX_FUNC,
    BOARD_INA219_I2C_SDA_PORT,
    BOARD_INA219_I2C_SDA_PIN,
    BOARD_INA219_I2C_SDA_IOMUX,
    BOARD_INA219_I2C_SDA_IOMUX_FUNC
};

static drivers::Ina219Context g_ina219Context = {
    0,
    0U,
    false,
    0U
};

drivers::DriverStatus Board_Ina219Init(void)
{
    ConfigureI2cPinsWithPullUps(&g_ina219Config);
    return drivers::Ina219_Init(&g_ina219Context, &g_ina219Config);
}

drivers::DriverStatus Board_Ina219Reset(void)
{
    return drivers::Ina219_Reset(&g_ina219Context);
}

drivers::DriverStatus Board_Ina219ConfigureDefault(void)
{
    return drivers::Ina219_ConfigureDefault(&g_ina219Context);
}


drivers::DriverStatus Board_Ina219SetAddress(uint8_t address)
{
    return drivers::Ina219_SetAddress(&g_ina219Context, address);
}

drivers::DriverStatus Board_Ina219ProbeAddress(uint8_t address,
                                               uint16_t *config_value)
{
    return drivers::Ina219_ProbeAddress(&g_ina219Context,
                                        address,
                                        config_value);
}

drivers::DriverStatus Board_Ina219ReadMeasurement(
    drivers::Ina219Measurement *measurement)
{
    return drivers::Ina219_ReadMeasurement(&g_ina219Context, measurement);
}

drivers::DriverStatus Board_Ina219ReadRawRegisters(
    drivers::Ina219RawRegisters *raw)
{
    return drivers::Ina219_ReadRawRegisters(&g_ina219Context, raw);
}

drivers::DriverStatus Board_Ina219ReadRegister(uint8_t reg, uint16_t *value)
{
    return drivers::Ina219_ReadRegister(&g_ina219Context, reg, value);
}

drivers::DriverStatus Board_Ina219WriteRegister(uint8_t reg, uint16_t value)
{
    return drivers::Ina219_WriteRegister(&g_ina219Context, reg, value);
}

drivers::DriverStatus Board_Ina219RecoverBus(void)
{
    return drivers::Ina219_RecoverBus(&g_ina219Context);
}

drivers::DriverStatus Board_Ina219GetBusStatus(uint32_t *controller_status,
                                               bool *scl_high,
                                               bool *sda_high)
{
    drivers::Ina219BusStatus status = { 0U, false, false };
    const drivers::DriverStatus driver_status =
        drivers::Ina219_GetBusStatus(&g_ina219Context, &status);

    if (driver_status != drivers::DRIVER_OK) {
        return driver_status;
    }

    if ((controller_status == 0) || (scl_high == 0) || (sda_high == 0)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    *controller_status = status.controller_status;
    *scl_high = status.scl_high;
    *sda_high = status.sda_high;

    return drivers::DRIVER_OK;
}

bool Board_Ina219IsReady(void)
{
    return drivers::Ina219_IsReady(&g_ina219Context);
}

uint16_t Board_Ina219Calibration(void)
{
    return drivers::Ina219_GetCalibration(&g_ina219Context);
}


uint8_t Board_Ina219GetAddress(void)
{
    return drivers::Ina219_GetAddress(&g_ina219Context);
}
} /* namespace board */