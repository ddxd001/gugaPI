#include "board/board_oled.h"

#include "board/board_i2c_bus.h"
#include "board/board_pins.h"
#include "drivers/i2c_diag/i2c_diag.h"
#include "drivers/oled/oled_ssd1306.h"

namespace board {
namespace {

static drivers::OledSsd1306Config g_oledConfig = {
    0,
    BOARD_OLED_I2C_ADDRESS,
    BOARD_OLED_WIDTH,
    BOARD_OLED_HEIGHT
};

static drivers::OledSsd1306Context g_oledContext = {
    0,
    false
};

const drivers::I2cDiagBusConfig *OledBus(void)
{
    return Board_I2cBusFind(BOARD_OLED_I2C_BUS_NAME);
}

} /* namespace */

drivers::DriverStatus Board_OledInit(void)
{
    g_oledConfig.bus = OledBus();
    return drivers::OledSsd1306_Init(&g_oledContext, &g_oledConfig);
}

drivers::DriverStatus Board_OledProbe(void)
{
    if (g_oledConfig.bus == 0) {
        g_oledConfig.bus = OledBus();
    }

    if (g_oledConfig.bus == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    return drivers::I2cDiag_ProbeAddress(g_oledConfig.bus,
                                         g_oledConfig.i2c_address);
}

drivers::DriverStatus Board_OledClear(void)
{
    return drivers::OledSsd1306_Clear(&g_oledContext);
}

drivers::DriverStatus Board_OledFill(uint8_t pattern)
{
    return drivers::OledSsd1306_Fill(&g_oledContext, pattern);
}

drivers::DriverStatus Board_OledTestPattern(void)
{
    return drivers::OledSsd1306_DrawChecker(&g_oledContext);
}

drivers::DriverStatus Board_OledWriteText(uint8_t row,
                                          uint8_t col,
                                          const char *text)
{
    return drivers::OledSsd1306_DrawString(&g_oledContext, row, col, text);
}

drivers::DriverStatus Board_OledSetDisplayOn(bool on)
{
    return drivers::OledSsd1306_SetDisplayOn(&g_oledContext, on);
}

drivers::DriverStatus Board_OledSetInvert(bool invert)
{
    return drivers::OledSsd1306_SetInvert(&g_oledContext, invert);
}

drivers::DriverStatus Board_OledGetBusStatus(uint32_t *controller_status,
                                             bool *scl_high,
                                             bool *sda_high)
{
    drivers::I2cDiagBusStatus status = { 0U, false, false };

    if (g_oledConfig.bus == 0) {
        g_oledConfig.bus = OledBus();
    }

    const drivers::DriverStatus driver_status =
        drivers::I2cDiag_GetBusStatus(g_oledConfig.bus, &status);
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

bool Board_OledIsReady(void)
{
    return drivers::OledSsd1306_IsReady(&g_oledContext);
}

uint8_t Board_OledAddress(void)
{
    return g_oledConfig.i2c_address;
}

uint8_t Board_OledWidth(void)
{
    return g_oledConfig.width;
}

uint8_t Board_OledHeight(void)
{
    return g_oledConfig.height;
}

} /* namespace board */
