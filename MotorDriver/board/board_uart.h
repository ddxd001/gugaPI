#ifndef BOARD_BOARD_UART_H_
#define BOARD_BOARD_UART_H_

#include <stdint.h>

#include "drivers/uart_link.h"

namespace board {

drivers::UartLinkStatus BoardUart_Init(void);
drivers::UartLinkStatus BoardUart_Write(const uint8_t *data, uint8_t length);
bool BoardUart_ReadByte(uint8_t *value);
uint16_t BoardUart_RxAvailable(void);
uint32_t BoardUart_RxDroppedCount(void);
void BoardUart_IrqHandler(void);

}  // namespace board

#endif  // BOARD_BOARD_UART_H_
