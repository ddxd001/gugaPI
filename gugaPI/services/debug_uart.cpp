#include "services/debug_uart.h"

#include "config/debug_config.h"
#include "config/feature_config.h"
#include "ti_msp_dl_config.h"

namespace services {
namespace {

static bool g_debugUartReady = false;
static volatile uint16_t g_rxHead = 0U;
static volatile uint16_t g_rxTail = 0U;
static volatile uint32_t g_rxDroppedCount = 0U;
static uint8_t g_rxBuffer[DEBUG_UART_RX_BUFFER_SIZE];

uint16_t NextRxIndex(uint16_t index)
{
    index++;
    if (index >= DEBUG_UART_RX_BUFFER_SIZE) {
        index = 0U;
    }
    return index;
}

void PushRxByteFromIsr(uint8_t data)
{
    const uint16_t nextHead = NextRxIndex(g_rxHead);

    if (nextHead == g_rxTail) {
        g_rxDroppedCount++;
        return;
    }

    g_rxBuffer[g_rxHead] = data;
    g_rxHead = nextHead;
}

} /* namespace */

void DebugUart_Init(void)
{
#if FEATURE_ENABLE_DEBUG_UART
    NVIC_DisableIRQ(DEBUG_UART_INST_INT_IRQN);

    g_rxHead = 0U;
    g_rxTail = 0U;
    g_rxDroppedCount = 0U;

    DL_UART_Main_clearInterruptStatus(DEBUG_UART_INST,
                                      DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(DEBUG_UART_INST_INT_IRQN);

    g_debugUartReady = true;
    NVIC_EnableIRQ(DEBUG_UART_INST_INT_IRQN);
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

void DebugUart_WriteData(const uint8_t *data, uint32_t length)
{
    if (data == 0) {
        return;
    }

    for (uint32_t i = 0U; i < length; i++) {
        DebugUart_WriteChar((char) data[i]);
    }
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

bool DebugUart_ReadByte(uint8_t *out_byte)
{
#if FEATURE_ENABLE_DEBUG_UART
    if ((out_byte == 0) || (!g_debugUartReady) || (g_rxHead == g_rxTail)) {
        return false;
    }

    *out_byte = g_rxBuffer[g_rxTail];
    g_rxTail = NextRxIndex(g_rxTail);
    return true;
#else
    (void) out_byte;
    return false;
#endif
}

uint16_t DebugUart_GetRxAvailable(void)
{
#if FEATURE_ENABLE_DEBUG_UART
    const uint16_t head = g_rxHead;
    const uint16_t tail = g_rxTail;

    if (head >= tail) {
        return (uint16_t) (head - tail);
    }

    return (uint16_t) (DEBUG_UART_RX_BUFFER_SIZE - tail + head);
#else
    return 0U;
#endif
}

uint32_t DebugUart_GetRxDroppedCount(void)
{
    return g_rxDroppedCount;
}

void DebugUart_ClearRxBuffer(void)
{
#if FEATURE_ENABLE_DEBUG_UART
    const bool wasReady = g_debugUartReady;

    NVIC_DisableIRQ(DEBUG_UART_INST_INT_IRQN);
    g_rxHead = 0U;
    g_rxTail = 0U;
    NVIC_ClearPendingIRQ(DEBUG_UART_INST_INT_IRQN);

    if (wasReady) {
        NVIC_EnableIRQ(DEBUG_UART_INST_INT_IRQN);
    }
#endif
}

void DebugUart_IrqHandler(void)
{
#if FEATURE_ENABLE_DEBUG_UART
    switch (DL_UART_Main_getPendingInterrupt(DEBUG_UART_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(DEBUG_UART_INST)) {
            PushRxByteFromIsr(DL_UART_Main_receiveData(DEBUG_UART_INST));
        }
        DL_UART_Main_clearInterruptStatus(DEBUG_UART_INST,
                                          DL_UART_MAIN_INTERRUPT_RX);
        break;

    default:
        break;
    }
#endif
}

} /* namespace services */

extern "C" void DEBUG_UART_INST_IRQHandler(void)
{
    services::DebugUart_IrqHandler();
}