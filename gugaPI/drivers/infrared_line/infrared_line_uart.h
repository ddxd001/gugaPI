#ifndef DRIVERS_INFRARED_LINE_INFRARED_LINE_UART_H_
#define DRIVERS_INFRARED_LINE_INFRARED_LINE_UART_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

struct InfraredLineUartConfig {
    UART_Regs *uart;
    IRQn_Type irq;
    uint8_t *rx_buffer;
    uint16_t rx_buffer_size;
};

struct InfraredLineUartContext {
    const InfraredLineUartConfig *config;
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint32_t rx_dropped_count;
    volatile uint32_t uart_error_count;
    volatile uint32_t irq_count;
    volatile uint32_t fifo_byte_count;
    volatile uint32_t polled_byte_count;
    bool initialized;
};

DriverStatus InfraredLineUart_Init(InfraredLineUartContext *context,
                                   const InfraredLineUartConfig *config);
bool InfraredLineUart_ReadByte(InfraredLineUartContext *context,
                               uint8_t *data);
void InfraredLineUart_ServiceRx(InfraredLineUartContext *context);
void InfraredLineUart_Clear(InfraredLineUartContext *context);
void InfraredLineUart_IrqHandler(InfraredLineUartContext *context);

} /* namespace drivers */

#endif /* DRIVERS_INFRARED_LINE_INFRARED_LINE_UART_H_ */
