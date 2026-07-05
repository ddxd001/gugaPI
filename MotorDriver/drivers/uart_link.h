#ifndef DRIVERS_UART_LINK_H_
#define DRIVERS_UART_LINK_H_

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

namespace drivers {

enum class UartLinkStatus : uint8_t {
    Ok = 0U,
    InvalidArgument,
    NotInitialized,
    Timeout,
};

struct UartLinkConfig {
    UART_Regs *uart;
    IRQn_Type irq;
    uint32_t tx_timeout_iterations;
    uint8_t *rx_buffer;
    uint16_t rx_buffer_size;
};

struct UartLinkContext {
    const UartLinkConfig *config;
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint32_t rx_dropped_count;
    bool initialized;
};

UartLinkStatus UartLink_Init(UartLinkContext *context,
                             const UartLinkConfig *config);
bool UartLink_IsReady(const UartLinkContext *context);
UartLinkStatus UartLink_WriteByte(UartLinkContext *context, uint8_t value);
UartLinkStatus UartLink_Write(UartLinkContext *context,
                              const uint8_t *data,
                              uint8_t length);
bool UartLink_ReadByte(UartLinkContext *context, uint8_t *value);
uint16_t UartLink_RxAvailable(const UartLinkContext *context);
uint32_t UartLink_RxDroppedCount(const UartLinkContext *context);
void UartLink_IrqHandler(UartLinkContext *context);

}  // namespace drivers

#endif  // DRIVERS_UART_LINK_H_
