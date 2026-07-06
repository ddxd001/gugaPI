#include "board/board_i2c_bus.h"

#include "board/board_pins.h"

namespace board {
namespace {

bool StrEqual(const char *left, const char *right)
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

// 板级 I2C 总线表。后续增加新设备时，在这里追加总线别名和引脚配置。
static const drivers::I2cDiagBusConfig g_i2cBuses[] = {
    {
        "motor",
        BOARD_MOTOR_DRIVER_I2C_INST,
        BOARD_MOTOR_DRIVER_I2C_TIMEOUT_ITERATIONS,
        BOARD_MOTOR_DRIVER_I2C_SCL_PORT,
        BOARD_MOTOR_DRIVER_I2C_SCL_PIN,
        BOARD_MOTOR_DRIVER_I2C_SCL_IOMUX,
        BOARD_MOTOR_DRIVER_I2C_SCL_IOMUX_FUNC,
        BOARD_MOTOR_DRIVER_I2C_SDA_PORT,
        BOARD_MOTOR_DRIVER_I2C_SDA_PIN,
        BOARD_MOTOR_DRIVER_I2C_SDA_IOMUX,
        BOARD_MOTOR_DRIVER_I2C_SDA_IOMUX_FUNC
    },
    {
        "fram",
        BOARD_FRAM_I2C_INST,
        BOARD_FRAM_TIMEOUT_ITERATIONS,
        BOARD_FRAM_I2C_SCL_PORT,
        BOARD_FRAM_I2C_SCL_PIN,
        BOARD_FRAM_I2C_SCL_IOMUX,
        BOARD_FRAM_I2C_SCL_IOMUX_FUNC,
        BOARD_FRAM_I2C_SDA_PORT,
        BOARD_FRAM_I2C_SDA_PIN,
        BOARD_FRAM_I2C_SDA_IOMUX,
        BOARD_FRAM_I2C_SDA_IOMUX_FUNC
    },
    {
        BOARD_OLED_I2C_BUS_NAME,
        BOARD_OLED_I2C_INST,
        BOARD_OLED_TIMEOUT_ITERATIONS,
        BOARD_OLED_I2C_SCL_PORT,
        BOARD_OLED_I2C_SCL_PIN,
        BOARD_OLED_I2C_SCL_IOMUX,
        BOARD_OLED_I2C_SCL_IOMUX_FUNC,
        BOARD_OLED_I2C_SDA_PORT,
        BOARD_OLED_I2C_SDA_PIN,
        BOARD_OLED_I2C_SDA_IOMUX,
        BOARD_OLED_I2C_SDA_IOMUX_FUNC
    },
    {
        "ina219",
        BOARD_INA219_I2C_INST,
        BOARD_INA219_TIMEOUT_ITERATIONS,
        BOARD_INA219_I2C_SCL_PORT,
        BOARD_INA219_I2C_SCL_PIN,
        BOARD_INA219_I2C_SCL_IOMUX,
        BOARD_INA219_I2C_SCL_IOMUX_FUNC,
        BOARD_INA219_I2C_SDA_PORT,
        BOARD_INA219_I2C_SDA_PIN,
        BOARD_INA219_I2C_SDA_IOMUX,
        BOARD_INA219_I2C_SDA_IOMUX_FUNC
    }
};

} /* namespace */

uint32_t Board_I2cBusCount(void)
{
    return (uint32_t) (sizeof(g_i2cBuses) / sizeof(g_i2cBuses[0]));
}

const drivers::I2cDiagBusConfig *Board_I2cBusGet(uint32_t index)
{
    if (index >= Board_I2cBusCount()) {
        return 0;
    }

    return &g_i2cBuses[index];
}

const drivers::I2cDiagBusConfig *Board_I2cBusFind(const char *name)
{
    for (uint32_t i = 0U; i < Board_I2cBusCount(); i++) {
        if (StrEqual(name, g_i2cBuses[i].name)) {
            return &g_i2cBuses[i];
        }
    }

    return 0;
}

} /* namespace board */
