#ifndef DRIVERS_LORA_LORA_H_
#define DRIVERS_LORA_LORA_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

struct LoraUartConfig {
    UART_Regs *uart;
    IRQn_Type irq;
    uint32_t baudrate;
    uint32_t tx_timeout_iterations;
    uint8_t *rx_buffer;
    uint16_t rx_buffer_size;
};

struct LoraUartContext {
    const LoraUartConfig *config;
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint32_t rx_dropped_count;
    bool initialized;
};

DriverStatus LoraUart_Init(LoraUartContext *context,
                           const LoraUartConfig *config);
DriverStatus LoraUart_Deinit(LoraUartContext *context);
bool LoraUart_IsReady(const LoraUartContext *context);
DriverStatus LoraUart_WriteByte(LoraUartContext *context, uint8_t data);
DriverStatus LoraUart_Write(LoraUartContext *context,
                            const uint8_t *data,
                            uint16_t length);
bool LoraUart_ReadByte(LoraUartContext *context, uint8_t *data);
uint16_t LoraUart_GetRxAvailable(const LoraUartContext *context);
uint32_t LoraUart_GetRxDroppedCount(const LoraUartContext *context);
uint32_t LoraUart_GetBaudrate(const LoraUartContext *context);
DriverStatus LoraUart_GetControllerStatus(const LoraUartContext *context,
                                           uint32_t *status);
DriverStatus LoraUart_ClearRxBuffer(LoraUartContext *context);
void LoraUart_IrqHandler(LoraUartContext *context);

} /* namespace drivers */

#endif /* DRIVERS_LORA_LORA_H_ */