#include "services/debug_uart.h"

#include <string.h>

#include "config/debug_config.h"
#include "config/feature_config.h"
#include "services/dma_fault.h"
#include "ti_msp_dl_config.h"

namespace services {
namespace {

static bool g_debugUartReady = false;
static volatile uint16_t g_rxHead = 0U;
static volatile uint16_t g_rxTail = 0U;
static volatile uint32_t g_rxDroppedCount = 0U;
static uint8_t g_rxBuffer[DEBUG_UART_RX_BUFFER_SIZE];

/*
 * TX path: the main-loop producer publishes complete spans into a ring. DMA
 * consumes one contiguous span at a time; the UART DMA-done interrupt advances
 * the tail and immediately starts the next span. This preserves the existing
 * non-blocking Shell API without polling or per-byte TX interrupts.
 *
 * Only g_txHead is written by main-loop code and only g_txTail is advanced by
 * the UART ISR. A producer copies bytes before publishing the new head, so DMA
 * can never see a partially copied span. Write calls from interrupt context are
 * intentionally unsupported, matching the service layering rule that ISRs do
 * not format or log text.
 */
static volatile uint16_t g_txHead = 0U;
static volatile uint16_t g_txTail = 0U;
static volatile uint32_t g_txDroppedCount = 0U;
static uint8_t g_txBuffer[DEBUG_UART_TX_BUFFER_SIZE];
static volatile bool g_txDmaActive = false;
static volatile uint16_t g_txDmaLength = 0U;
static volatile uint32_t g_txDmaBlockCount = 0U;
static volatile uint32_t g_txDmaErrorCount = 0U;
static const uint8_t kMaximumDmaFaultHandlers = 4U;
static DmaFaultHandler g_dmaFaultHandlers[kMaximumDmaFaultHandlers] = {};
static uint8_t g_dmaFaultHandlerCount = 0U;
static volatile uint32_t g_dmaFaultCount = 0U;
static bool g_dmaFaultIrqInitialized = false;

static_assert(DEBUG_UART_TX_BUFFER_SIZE > 1U,
              "Debug UART TX ring must hold at least one byte");
static_assert(DEBUG_UART_TX_BUFFER_SIZE <= UINT16_MAX,
              "Debug UART TX ring indices are uint16_t");
static_assert((DEBUG_UART_TX_DMA_BLOCK_SIZE > 0U) &&
                  (DEBUG_UART_TX_DMA_BLOCK_SIZE <
                   DEBUG_UART_TX_BUFFER_SIZE),
              "Debug UART DMA block must fit inside the TX ring");

uint16_t NextRxIndex(uint16_t index)
{
    index++;
    if (index >= DEBUG_UART_RX_BUFFER_SIZE) {
        index = 0U;
    }
    return index;
}

uint16_t AdvanceTxIndex(uint16_t index, uint16_t amount)
{
    uint32_t advanced = static_cast<uint32_t>(index) + amount;
    if (advanced >= DEBUG_UART_TX_BUFFER_SIZE) {
        advanced -= DEBUG_UART_TX_BUFFER_SIZE;
    }
    return static_cast<uint16_t>(advanced);
}

uint16_t TxFreeSpace(uint16_t head, uint16_t tail)
{
    if (head >= tail) {
        return static_cast<uint16_t>(
            DEBUG_UART_TX_BUFFER_SIZE - (head - tail) - 1U);
    }
    return static_cast<uint16_t>(tail - head - 1U);
}

void StartNextTxDma(void)
{
#if FEATURE_ENABLE_DEBUG_UART
    if ((!g_debugUartReady) || g_txDmaActive ||
        (g_txTail == g_txHead)) {
        return;
    }

    uint16_t length = (g_txHead > g_txTail) ?
        static_cast<uint16_t>(g_txHead - g_txTail) :
        static_cast<uint16_t>(DEBUG_UART_TX_BUFFER_SIZE - g_txTail);
    if (length > DEBUG_UART_TX_DMA_BLOCK_SIZE) {
        length = DEBUG_UART_TX_DMA_BLOCK_SIZE;
    }

    g_txDmaLength = length;
    g_txDmaActive = true;

    DL_DMA_disableChannel(DMA, DEBUG_UART_DMA_TX_CHAN_ID);
    DL_DMA_setSrcAddr(
        DMA,
        DEBUG_UART_DMA_TX_CHAN_ID,
        reinterpret_cast<uint32_t>(&g_txBuffer[g_txTail]));
    DL_DMA_setDestAddr(
        DMA,
        DEBUG_UART_DMA_TX_CHAN_ID,
        reinterpret_cast<uint32_t>(&DEBUG_UART_INST->TXDATA));
    DL_DMA_setTransferSize(DMA, DEBUG_UART_DMA_TX_CHAN_ID, length);
    DL_DMA_enableChannel(DMA, DEBUG_UART_DMA_TX_CHAN_ID);
#endif
}

void CompleteTxDma(void)
{
#if FEATURE_ENABLE_DEBUG_UART
    DL_DMA_disableChannel(DMA, DEBUG_UART_DMA_TX_CHAN_ID);

    if ((!g_txDmaActive) || (g_txDmaLength == 0U)) {
        g_txDmaErrorCount++;
        return;
    }

    g_txTail = AdvanceTxIndex(g_txTail, g_txDmaLength);
    g_txDmaLength = 0U;
    g_txDmaActive = false;
    g_txDmaBlockCount++;
    StartNextTxDma();
#endif
}

void RecoverTxDmaError(void)
{
#if FEATURE_ENABLE_DEBUG_UART
    DL_DMA_disableChannel(DMA, DEBUG_UART_DMA_TX_CHAN_ID);
    g_txDmaErrorCount++;

    /* A DMA address/data fault does not report how many UART bytes made it to
     * the FIFO. Drop the active span rather than risking duplicated protocol
     * text, then continue with the next queued span. */
    if (g_txDmaActive && (g_txDmaLength > 0U)) {
        g_txTail = AdvanceTxIndex(g_txTail, g_txDmaLength);
        g_txDroppedCount += g_txDmaLength;
    }
    g_txDmaLength = 0U;
    g_txDmaActive = false;
    StartNextTxDma();
#endif
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

bool DmaFault_RegisterHandler(DmaFaultHandler handler)
{
    if (handler == 0) {
        return false;
    }
    for (uint8_t i = 0U; i < g_dmaFaultHandlerCount; i++) {
        if (g_dmaFaultHandlers[i] == handler) {
            return true;
        }
    }
    if (g_dmaFaultHandlerCount >= kMaximumDmaFaultHandlers) {
        return false;
    }
    g_dmaFaultHandlers[g_dmaFaultHandlerCount++] = handler;
    if (!g_dmaFaultIrqInitialized) {
        NVIC_DisableIRQ(DMA_INT_IRQn);
        DL_DMA_clearInterruptStatus(
            DMA, DL_DMA_INTERRUPT_ADDR_ERROR | DL_DMA_INTERRUPT_DATA_ERROR);
        DL_DMA_enableInterrupt(
            DMA, DL_DMA_INTERRUPT_ADDR_ERROR | DL_DMA_INTERRUPT_DATA_ERROR);
        NVIC_SetPriority(DMA_INT_IRQn, 0U);
        NVIC_ClearPendingIRQ(DMA_INT_IRQn);
        g_dmaFaultIrqInitialized = true;
        NVIC_EnableIRQ(DMA_INT_IRQn);
    }
    return true;
}

uint32_t DmaFault_GetCount(void)
{
    return g_dmaFaultCount;
}

void DmaFault_IrqHandler(void)
{
    bool fault = false;
    while (true) {
        const DL_DMA_EVENT_IIDX pending = DL_DMA_getPendingInterrupt(DMA);
        if (pending == DL_DMA_EVENT_IIDX_ADDR_ERROR) {
            DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_ADDR_ERROR);
            fault = true;
        } else if (pending == DL_DMA_EVENT_IIDX_DATA_ERROR) {
            DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_DATA_ERROR);
            fault = true;
        } else {
            break;
        }
    }
    if (!fault) {
        return;
    }
    if (g_dmaFaultCount != UINT32_MAX) {
        g_dmaFaultCount++;
    }
    for (uint8_t i = 0U; i < g_dmaFaultHandlerCount; i++) {
        g_dmaFaultHandlers[i]();
    }
}

void DebugUart_Init(void)
{
#if FEATURE_ENABLE_DEBUG_UART
    NVIC_DisableIRQ(DEBUG_UART_INST_INT_IRQN);
    g_rxHead = 0U;
    g_rxTail = 0U;
    g_rxDroppedCount = 0U;
    g_txHead = 0U;
    g_txTail = 0U;
    g_txDroppedCount = 0U;
    g_txDmaActive = false;
    g_txDmaLength = 0U;
    g_txDmaBlockCount = 0U;
    g_txDmaErrorCount = 0U;

    DL_DMA_disableChannel(DMA, DEBUG_UART_DMA_TX_CHAN_ID);

    DL_UART_Main_clearInterruptStatus(DEBUG_UART_INST,
                                      DL_UART_MAIN_INTERRUPT_RX |
                                      DL_UART_MAIN_INTERRUPT_DMA_DONE_TX);
    NVIC_ClearPendingIRQ(DEBUG_UART_INST_INT_IRQN);

    g_debugUartReady = true;
    (void) DmaFault_RegisterHandler(DebugUart_HandleDmaFault);
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
    const uint8_t data = static_cast<uint8_t>(ch);
    DebugUart_WriteData(&data, 1U);
}

void DebugUart_TxPump(void)
{
#if FEATURE_ENABLE_DEBUG_UART
    /* Compatibility entry point retained for the existing main loop. DMA
     * completion normally chains blocks from the ISR; this call only kicks an
     * idle queue after newly published data or an unusual missed kick. */
    StartNextTxDma();
#endif
}

uint16_t DebugUart_GetTxPending(void)
{
#if FEATURE_ENABLE_DEBUG_UART
    const uint16_t head = g_txHead;
    const uint16_t tail = g_txTail;

    if (head >= tail) {
        return (uint16_t) (head - tail);
    }

    return (uint16_t) (DEBUG_UART_TX_BUFFER_SIZE - tail + head);
#else
    return 0U;
#endif
}

uint32_t DebugUart_GetTxDroppedCount(void)
{
    return g_txDroppedCount;
}

bool DebugUart_IsTxDmaActive(void)
{
    return g_txDmaActive;
}

uint32_t DebugUart_GetTxDmaBlockCount(void)
{
    return g_txDmaBlockCount;
}

uint32_t DebugUart_GetTxDmaErrorCount(void)
{
    return g_txDmaErrorCount;
}

void DebugUart_WriteData(const uint8_t *data, uint32_t length)
{
    if ((data == 0) || (length == 0U)) {
        return;
    }

#if FEATURE_ENABLE_DEBUG_UART
    if (!g_debugUartReady) {
        /* Boot path only: preserve early diagnostics before the DMA queue is
         * initialized. Runtime writes never enter this blocking branch. */
        for (uint32_t i = 0U; i < length; i++) {
            DL_UART_Main_transmitDataBlocking(DEBUG_UART_INST, data[i]);
        }
        return;
    }

    const uint16_t head = g_txHead;
    const uint16_t tail = g_txTail;
    const uint16_t freeSpace = TxFreeSpace(head, tail);
    uint32_t accepted = length;
    if (accepted > freeSpace) {
        accepted = freeSpace;
    }

    uint32_t firstLength = DEBUG_UART_TX_BUFFER_SIZE - head;
    if (firstLength > accepted) {
        firstLength = accepted;
    }
    if (firstLength > 0U) {
        (void) memcpy(&g_txBuffer[head], data, firstLength);
    }

    const uint32_t secondLength = accepted - firstLength;
    if (secondLength > 0U) {
        (void) memcpy(&g_txBuffer[0], &data[firstLength], secondLength);
    }

    uint32_t publishedHead = static_cast<uint32_t>(head) + accepted;
    if (publishedHead >= DEBUG_UART_TX_BUFFER_SIZE) {
        publishedHead -= DEBUG_UART_TX_BUFFER_SIZE;
    }
    g_txHead = static_cast<uint16_t>(publishedHead);

    if (accepted < length) {
        g_txDroppedCount += length - accepted;
    }

    StartNextTxDma();
#else
    (void) length;
#endif
}

void DebugUart_WriteString(const char *text)
{
    if (text == 0) {
        return;
    }

    DebugUart_WriteData(reinterpret_cast<const uint8_t *>(text),
                        static_cast<uint32_t>(strlen(text)));
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

    for (uint32_t left = 0U, right = index - 1U;
         left < right;
         left++, right--) {
        const char temp = buffer[left];
        buffer[left] = buffer[right];
        buffer[right] = temp;
    }
    DebugUart_WriteData(reinterpret_cast<const uint8_t *>(buffer), index);
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

    case DL_UART_MAIN_IIDX_DMA_DONE_TX:
        CompleteTxDma();
        break;

    default:
        break;
    }
#endif
}

void DebugUart_HandleDmaFault(void)
{
#if FEATURE_ENABLE_DEBUG_UART
    RecoverTxDmaError();
#endif
}

} /* namespace services */

extern "C" void DEBUG_UART_INST_IRQHandler(void)
{
    services::DebugUart_IrqHandler();
}

extern "C" void DMA_IRQHandler(void)
{
    services::DmaFault_IrqHandler();
}
