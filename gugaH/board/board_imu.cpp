#include "board/board_imu.h"

#include "board/board_pins.h"

namespace board {
namespace {

const drivers::Icm45686Config g_config = {
    BOARD_IMU_SPI_INST,
    BOARD_IMU_CS_PORT,
    BOARD_IMU_CS_PIN,
    BOARD_IMU_SPI_TIMEOUT_ITERATIONS,
    drivers::ICM45686_ACCEL_FS_4G,
    drivers::ICM45686_GYRO_FS_1000DPS,
    drivers::ICM45686_ODR_200HZ,
    drivers::ICM45686_ODR_200HZ
};

drivers::Icm45686Context g_context = { &g_config, false };

} /* namespace */

drivers::DriverStatus Board_ImuInit(void)
{
    DL_SPI_disable(BOARD_IMU_SPI_INST);
    DL_SPI_setFrameFormat(BOARD_IMU_SPI_INST,
                          DL_SPI_FRAME_FORMAT_MOTO4_POL1_PHA1);
    DL_SPI_enable(BOARD_IMU_SPI_INST);
    DL_GPIO_setPins(BOARD_IMU_CS_PORT, BOARD_IMU_CS_PIN);
    DL_GPIO_enableOutput(BOARD_IMU_CS_PORT, BOARD_IMU_CS_PIN);
    return drivers::Icm45686_Init(&g_context, &g_config);
}

bool Board_ImuIsReady(void)
{
    return drivers::Icm45686_IsReady(&g_context);
}

drivers::DriverStatus Board_ImuRead(drivers::Icm45686SensorData *data)
{
    return drivers::Icm45686_ReadSensors(&g_context, data);
}

} /* namespace board */
