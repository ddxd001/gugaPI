#ifndef DRIVERS_INFRARED_LINE_INFRARED_LINE_UART_H_
#define DRIVERS_INFRARED_LINE_INFRARED_LINE_UART_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "drivers/infrared_line/infrared_line_dma_cursor.h"
#include "ti_msp_dl_config.h"

namespace drivers {

struct InfraredLineUartConfig {
    UART_Regs *uart;
    IRQn_Type irq;
    DMA_Regs *dma;
    uint8_t dma_channel;
    uint8_t *dma_buffer;
    uint16_t dma_buffer_size;
};

struct InfraredLineUartContext {
    const InfraredLineUartConfig *config;
    InfraredLineDmaCursor cursor;
    uint64_t cycle_base;
    uint64_t stats_base_producer;
    volatile uint32_t cycle_wrap_generation;
    volatile uint32_t dma_wrap_count;
    volatile uint32_t uart_error_count;
    volatile uint32_t rx_timeout_count;
    volatile uint32_t overrun_error_count;
    volatile uint32_t framing_error_count;
    volatile uint32_t parity_error_count;
    volatile uint32_t noise_error_count;
    volatile uint32_t irq_count;
    volatile uint32_t polled_byte_count;
    volatile uint32_t dma_fault_count;
    volatile bool initialized;
};

DriverStatus InfraredLineUart_Init(InfraredLineUartContext *context,
                                   const InfraredLineUartConfig *config);
bool InfraredLineUart_ReadByte(InfraredLineUartContext *context,
                               uint8_t *data);
void InfraredLineUart_ServiceRx(InfraredLineUartContext *context);
void InfraredLineUart_ClearStats(InfraredLineUartContext *context);
void InfraredLineUart_IrqHandler(InfraredLineUartContext *context);
void InfraredLineUart_HandleDmaFault(InfraredLineUartContext *context);
uint64_t InfraredLineUart_GetProducedCount(
    const InfraredLineUartContext *context);
uint64_t InfraredLineUart_GetConsumedCount(
    const InfraredLineUartContext *context);
uint32_t InfraredLineUart_GetCurrentLag(
    const InfraredLineUartContext *context);
uint16_t InfraredLineUart_GetDmaRemaining(
    const InfraredLineUartContext *context);

} /* namespace drivers */

#endif /* DRIVERS_INFRARED_LINE_INFRARED_LINE_UART_H_ */
