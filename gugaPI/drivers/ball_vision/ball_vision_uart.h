#ifndef DRIVERS_BALL_VISION_BALL_VISION_UART_H_
#define DRIVERS_BALL_VISION_BALL_VISION_UART_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

struct BallVisionUartConfig {
    UART_Regs *uart;
    IRQn_Type irq;
    uint8_t *rx_buffer;
    uint16_t rx_buffer_size;
};

struct BallVisionUartContext {
    const BallVisionUartConfig *config;
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint32_t dropped_bytes;
    volatile uint32_t uart_errors;
    volatile uint32_t irq_count;
    bool initialized;
};

DriverStatus BallVisionUart_Init(BallVisionUartContext *context,
                                 const BallVisionUartConfig *config);
bool BallVisionUart_ReadByte(BallVisionUartContext *context, uint8_t *data);
void BallVisionUart_Clear(BallVisionUartContext *context);
void BallVisionUart_IrqHandler(BallVisionUartContext *context);

} /* namespace drivers */

#endif /* DRIVERS_BALL_VISION_BALL_VISION_UART_H_ */
