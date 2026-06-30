#include "services/debug_uart.h"

#include "config/feature_config.h"
#include "ti_msp_dl_config.h"

namespace services {

static bool g_debugUartReady = false;

void DebugUart_Init(void)
{
#if FEATURE_ENABLE_DEBUG_UART
    g_debugUartReady = true;
#else
    g_debugUartReady = false;
#endif
}

bool DebugUart_IsReady(void)
{
    return g_debugUartReady;
}

void DebugUart_WriteChar(char ch)
{
#if FEATURE_ENABLE_DEBUG_UART
    if (!g_debugUartReady) {
        return;
    }

    DL_UART_Main_transmitDataBlocking(DEBUG_UART_INST, (uint8_t) ch);
#else
    (void) ch;
#endif
}

void DebugUart_WriteString(const char *text)
{
    if (text == 0) {
        return;
    }

    while (*text != '\0') {
        DebugUart_WriteChar(*text);
        text++;
    }
}

void DebugUart_WriteUInt32(uint32_t value)
{
    char buffer[11];
    uint32_t index = 0U;

    if (value == 0U) {
        DebugUart_WriteChar('0');
        return;
    }

    while ((value > 0U) && (index < sizeof(buffer))) {
        buffer[index] = (char) ('0' + (value % 10U));
        value /= 10U;
        index++;
    }

    while (index > 0U) {
        index--;
        DebugUart_WriteChar(buffer[index]);
    }
}

void DebugUart_WriteLineUInt32(uint32_t value)
{
    DebugUart_WriteUInt32(value);
    DebugUart_WriteString("\r\n");
}

} /* namespace services */