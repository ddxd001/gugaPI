#ifndef GUGAH_SERVICES_DEBUG_UART_H_
#define GUGAH_SERVICES_DEBUG_UART_H_

#include <stdbool.h>
#include <stdint.h>

namespace services {

void DebugUart_Init(void);
bool DebugUart_ReadByte(uint8_t *value);
void DebugUart_WriteChar(char value);
void DebugUart_WriteData(const uint8_t *data, uint32_t length);
bool DebugUart_TryWriteData(const uint8_t *data, uint32_t length);
void DebugUart_WriteString(const char *text);
void DebugUart_Pump(void);
uint32_t DebugUart_GetRxDropped(void);
uint32_t DebugUart_GetTxDropped(void);
void DebugUart_IrqHandler(void);
void DebugUart_HandleDmaFault(void);

} /* namespace services */

#endif /* GUGAH_SERVICES_DEBUG_UART_H_ */
