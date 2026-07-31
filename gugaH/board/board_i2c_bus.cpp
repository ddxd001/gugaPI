#include "board/board_i2c_bus.h"

#include "board/board_pins.h"

namespace board {
namespace {

bool Equal(const char *left, const char *right)
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

const drivers::I2cDiagBusConfig kBuses[] = {
    {
        "motor",
        BOARD_MOTOR_I2C_INST,
        BOARD_MOTOR_I2C_TIMEOUT,
        BOARD_MOTOR_I2C_SCL_PORT,
        BOARD_MOTOR_I2C_SCL_PIN,
        BOARD_MOTOR_I2C_SCL_IOMUX,
        BOARD_MOTOR_I2C_SCL_IOMUX_FUNC,
        BOARD_MOTOR_I2C_SDA_PORT,
        BOARD_MOTOR_I2C_SDA_PIN,
        BOARD_MOTOR_I2C_SDA_IOMUX,
        BOARD_MOTOR_I2C_SDA_IOMUX_FUNC
    },
    {
        "fram",
        BOARD_SENSOR_I2C_INST,
        BOARD_SENSOR_I2C_TIMEOUT,
        BOARD_SENSOR_I2C_SCL_PORT,
        BOARD_SENSOR_I2C_SCL_PIN,
        BOARD_SENSOR_I2C_SCL_IOMUX,
        BOARD_SENSOR_I2C_SCL_IOMUX_FUNC,
        BOARD_SENSOR_I2C_SDA_PORT,
        BOARD_SENSOR_I2C_SDA_PIN,
        BOARD_SENSOR_I2C_SDA_IOMUX,
        BOARD_SENSOR_I2C_SDA_IOMUX_FUNC
    },
    {
        "oled",
        BOARD_SENSOR_I2C_INST,
        BOARD_SENSOR_I2C_TIMEOUT,
        BOARD_SENSOR_I2C_SCL_PORT,
        BOARD_SENSOR_I2C_SCL_PIN,
        BOARD_SENSOR_I2C_SCL_IOMUX,
        BOARD_SENSOR_I2C_SCL_IOMUX_FUNC,
        BOARD_SENSOR_I2C_SDA_PORT,
        BOARD_SENSOR_I2C_SDA_PIN,
        BOARD_SENSOR_I2C_SDA_IOMUX,
        BOARD_SENSOR_I2C_SDA_IOMUX_FUNC
    }
};

} /* namespace */

uint32_t Board_I2cBusCount(void)
{
    return static_cast<uint32_t>(
        sizeof(kBuses) / sizeof(kBuses[0]));
}

const drivers::I2cDiagBusConfig *Board_I2cBusGet(uint32_t index)
{
    return (index < Board_I2cBusCount()) ? &kBuses[index] : 0;
}

const drivers::I2cDiagBusConfig *Board_I2cBusFind(const char *name)
{
    for (uint32_t i = 0U; i < Board_I2cBusCount(); i++) {
        if (Equal(name, kBuses[i].name)) {
            return &kBuses[i];
        }
    }
    return 0;
}

} /* namespace board */
