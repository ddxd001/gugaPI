#ifndef DRIVERS_OLED_OLED_SSD1306_H_
#define DRIVERS_OLED_OLED_SSD1306_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "drivers/i2c_controller/i2c_controller.h"

namespace drivers {

static const uint16_t OLED_SSD1306_FRAME_BYTES = 512U;
static const uint16_t OLED_SSD1306_DMA_BUFFER_BYTES =
    OLED_SSD1306_FRAME_BYTES + 1U;
static const uint8_t OLED_SSD1306_WINDOW_COMMAND_BYTES = 9U;

enum OledSsd1306FlushPhase {
    OLED_FLUSH_IDLE = 0,
    OLED_FLUSH_WINDOW,
    OLED_FLUSH_DATA
};

struct OledSsd1306Config {
    const I2cControllerConfig *bus;
    const I2cControllerDmaTxConfig *dma_tx;
    uint8_t i2c_address;
    uint8_t width;
    uint8_t height;
};

struct OledSsd1306Context {
    const OledSsd1306Config *config;
    bool initialized;
    OledSsd1306FlushPhase flush_phase;
    uint8_t dirty_pages;
    uint8_t dirty_first_column;
    uint8_t dirty_last_column;
    uint8_t active_pages;
    uint8_t active_first_page;
    uint8_t active_last_page;
    uint8_t active_first_column;
    uint8_t active_last_column;
    uint16_t active_data_length;
    DriverStatus last_flush_status;
    uint8_t framebuffer[OLED_SSD1306_FRAME_BYTES];
    uint8_t command_buffer[OLED_SSD1306_WINDOW_COMMAND_BYTES];
    uint8_t dma_buffer[OLED_SSD1306_DMA_BUFFER_BYTES];
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
DriverStatus OledSsd1306_Service(OledSsd1306Context *ctx);
void OledSsd1306_HandleI2cInterrupt(OledSsd1306Context *ctx);
void OledSsd1306_HandleDmaFault(OledSsd1306Context *ctx);
bool OledSsd1306_HasPendingFlush(const OledSsd1306Context *ctx);
bool OledSsd1306_IsReady(const OledSsd1306Context *ctx);

} /* namespace drivers */

#endif /* DRIVERS_OLED_OLED_SSD1306_H_ */
