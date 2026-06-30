#ifndef SERVICES_DEBUG_UART_H_
#define SERVICES_DEBUG_UART_H_

#include <stdbool.h>
#include <stdint.h>

namespace services {

void DebugUart_Init(void);
bool DebugUart_IsReady(void);
void DebugUart_WriteChar(char ch);
void DebugUart_WriteString(const char *text);
void DebugUart_WriteUInt32(uint32_t value);
void DebugUart_WriteLineUInt32(uint32_t value);

} /* namespace services */

#endif /* SERVICES_DEBUG_UART_H_ */