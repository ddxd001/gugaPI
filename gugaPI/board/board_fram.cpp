#include "board/board_fram.h"

#include "board/board_pins.h"
#include "drivers/fm24cl64b/fm24cl64b.h"

namespace board {

static const drivers::Fm24cl64bConfig g_framConfig = {
    BOARD_FRAM_I2C_INST,
    BOARD_FRAM_I2C_ADDRESS,
    BOARD_FRAM_TIMEOUT_ITERATIONS,
    BOARD_FRAM_I2C_SCL_PORT,
    BOARD_FRAM_I2C_SCL_PIN,
    BOARD_FRAM_I2C_SCL_IOMUX,
    BOARD_FRAM_I2C_SCL_IOMUX_FUNC,
    BOARD_FRAM_I2C_SDA_PORT,
    BOARD_FRAM_I2C_SDA_PIN,
    BOARD_FRAM_I2C_SDA_IOMUX,
    BOARD_FRAM_I2C_SDA_IOMUX_FUNC
};

static drivers::Fm24cl64bContext g_framContext = {
    0,
    false
};

drivers::DriverStatus Board_FramInit(void)
{
    return drivers::Fm24cl64b_Init(&g_framContext, &g_framConfig);
}

drivers::DriverStatus Board_FramRead(uint16_t address,
                                     uint8_t *data,
                                     uint16_t length)
{
    return drivers::Fm24cl64b_Read(&g_framContext, address, data, length);
}

drivers::DriverStatus Board_FramWrite(uint16_t address,
                                      const uint8_t *data,
                                      uint16_t length)
{
    return drivers::Fm24cl64b_Write(&g_framContext, address, data, length);
}

drivers::DriverStatus Board_FramReadByte(uint16_t address, uint8_t *data)
{
    return drivers::Fm24cl64b_ReadByte(&g_framContext, address, data);
}

drivers::DriverStatus Board_FramWriteByte(uint16_t address, uint8_t data)
{
    return drivers::Fm24cl64b_WriteByte(&g_framContext, address, data);
}

drivers::DriverStatus Board_FramSelfTest(void)
{
    return drivers::Fm24cl64b_SelfTest(&g_framContext,
                                       BOARD_FRAM_SELF_TEST_ADDRESS);
}

drivers::DriverStatus Board_FramRecoverBus(void)
{
    return drivers::Fm24cl64b_RecoverBus(&g_framContext);
}

drivers::DriverStatus Board_FramGetBusStatus(uint32_t *controller_status,
                                             bool *scl_high,
                                             bool *sda_high)
{
    drivers::Fm24cl64bBusStatus status = { 0U, false, false };
    const drivers::DriverStatus driver_status =
        drivers::Fm24cl64b_GetBusStatus(&g_framContext, &status);

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

bool Board_FramIsReady(void)
{
    return drivers::Fm24cl64b_IsReady(&g_framContext);
}

uint16_t Board_FramCapacityBytes(void)
{
    return drivers::FM24CL64B_SIZE_BYTES;
}

} /* namespace board */