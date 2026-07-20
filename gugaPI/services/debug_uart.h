#ifndef SERVICES_DEBUG_UART_H_
#define SERVICES_DEBUG_UART_H_

#include <stdbool.h>
#include <stdint.h>

namespace services {

void DebugUart_Init(void);
bool DebugUart_IsReady(void);
void DebugUart_WriteChar(char ch);
void DebugUart_WriteData(const uint8_t *data, uint32_t length);
void DebugUart_WriteString(const char *text);
void DebugUart_WriteUInt32(uint32_t value);
void DebugUart_WriteLineUInt32(uint32_t value);
bool DebugUart_ReadByte(uint8_t *out_byte);
uint16_t DebugUart_GetRxAvailable(void);
uint32_t DebugUart_GetRxDroppedCount(void);
void DebugUart_ClearRxBuffer(void);
void DebugUart_TxPump(void);
uint16_t DebugUart_GetTxPending(void);
uint32_t DebugUart_GetTxDroppedCount(void);
void DebugUart_IrqHandler(void);

} /* namespace services */

#endif /* SERVICES_DEBUG_UART_H_ */