#ifndef DRIVERS_OLED_OLED_SSD1306_H_
#define DRIVERS_OLED_OLED_SSD1306_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "drivers/i2c_diag/i2c_diag.h"

namespace drivers {

struct OledSsd1306Config {
    const I2cDiagBusConfig *bus;
    uint8_t i2c_address;
    uint8_t width;
    uint8_t height;
};

struct OledSsd1306Context {
    const OledSsd1306Config *config;
    bool initialized;
};

DriverStatus OledSsd1306_Init(OledSsd1306Context *ctx,
                              const OledSsd1306Config *config);
DriverStatus OledSsd1306_Probe(OledSsd1306Context *ctx);
DriverStatus OledSsd1306_Clear(OledSsd1306Context *ctx);
DriverStatus OledSsd1306_Fill(OledSsd1306Context *ctx, uint8_t pattern);
DriverStatus OledSsd1306_DrawChecker(OledSsd1306Context *ctx);
DriverStatus OledSsd1306_SetDisplayOn(OledSsd1306Context *ctx, bool on);
DriverStatus OledSsd1306_SetInvert(OledSsd1306Context *ctx, bool invert);
DriverStatus OledSsd1306_DrawString(OledSsd1306Context *ctx,
                                    uint8_t row,
                                    uint8_t col,
                                    const char *text);
DriverStatus OledSsd1306_WriteBuffer(OledSsd1306Context *ctx,
                                     const uint8_t *buffer,
                                     uint16_t length);
bool OledSsd1306_IsReady(const OledSsd1306Context *ctx);

} /* namespace drivers */

#endif /* DRIVERS_OLED_OLED_SSD1306_H_ */
