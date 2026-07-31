#include "services/debug_uart.h"

#include <string.h>

#include "services/dma_fault.h"
#include "ti_msp_dl_config.h"

namespace services {
namespace {

static const uint16_t kRxSize = 256U;
static const uint16_t kTxSize = 1024U;
static const uint16_t kDmaBlockSize = 128U;
uint8_t g_rx[kRxSize] = {};
uint8_t g_tx[kTxSize] = {};
volatile uint16_t g_rx_head = 0U;
volatile uint16_t g_rx_tail = 0U;
volatile uint16_t g_tx_head = 0U;
volatile uint16_t g_tx_tail = 0U;
volatile uint16_t g_dma_length = 0U;
volatile uint32_t g_rx_dropped = 0U;
volatile uint32_t g_tx_dropped = 0U;
volatile bool g_dma_active = false;
bool g_ready = false;

uint16_t Next(uint16_t value, uint16_t size)
{
    value++;
    return (value >= size) ? 0U : value;
}

uint16_t TxFree(void)
{
    const uint16_t head = g_tx_head;
    const uint16_t tail = g_tx_tail;
    return (head >= tail)
        ? static_cast<uint16_t>(kTxSize - (head - tail) - 1U)
        : static_cast<uint16_t>(tail - head - 1U);
}

void StartDma(void)
{
    if (!g_ready || g_dma_active || (g_tx_head == g_tx_tail)) {
        return;
    }
    uint16_t length = (g_tx_head > g_tx_tail)
        ? static_cast<uint16_t>(g_tx_head - g_tx_tail)
        : static_cast<uint16_t>(kTxSize - g_tx_tail);
    if (length > kDmaBlockSize) {
        length = kDmaBlockSize;
    }
    g_dma_length = length;
    g_dma_active = true;
    DL_DMA_disableChannel(DMA, DEBUG_UART_DMA_TX_CHAN_ID);
    DL_DMA_setSrcAddr(
        DMA, DEBUG_UART_DMA_TX_CHAN_ID,
        reinterpret_cast<uint32_t>(&g_tx[g_tx_tail]));
    DL_DMA_setDestAddr(
        DMA, DEBUG_UART_DMA_TX_CHAN_ID,
        reinterpret_cast<uint32_t>(&DEBUG_UART_INST->TXDATA));
    DL_DMA_setTransferSize(DMA, DEBUG_UART_DMA_TX_CHAN_ID, length);
    DL_DMA_enableChannel(DMA, DEBUG_UART_DMA_TX_CHAN_ID);
}

void CompleteDma(void)
{
    DL_DMA_disableChannel(DMA, DEBUG_UART_DMA_TX_CHAN_ID);
    g_tx_tail = static_cast<uint16_t>(
        (static_cast<uint32_t>(g_tx_tail) + g_dma_length) % kTxSize);
    g_dma_length = 0U;
    g_dma_active = false;
    StartDma();
}

} /* namespace */

void DebugUart_Init(void)
{
    g_rx_head = g_rx_tail = 0U;
    g_tx_head = g_tx_tail = 0U;
    g_dma_active = false;
    g_ready = true;
    (void)DmaFault_RegisterHandler(DebugUart_HandleDmaFault);
    DL_UART_Main_clearInterruptStatus(
        DEBUG_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_DMA_DONE_TX);
    NVIC_ClearPendingIRQ(DEBUG_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(DEBUG_UART_INST_INT_IRQN);
}

bool DebugUart_ReadByte(uint8_t *value)
{
    if ((value == 0) || (g_rx_head == g_rx_tail)) {
        return false;
    }
    *value = g_rx[g_rx_tail];
    g_rx_tail = Next(g_rx_tail, kRxSize);
    return true;
}

void DebugUart_WriteChar(char value)
{
    const uint8_t byte = static_cast<uint8_t>(value);
    DebugUart_WriteData(&byte, 1U);
}

void DebugUart_WriteData(const uint8_t *data, uint32_t length)
{
    if ((data == 0) || (length == 0U)) {
        return;
    }
    uint32_t accepted = length;
    const uint16_t free = TxFree();
    if (accepted > free) {
        accepted = free;
        g_tx_dropped += length - accepted;
    }
    for (uint32_t i = 0U; i < accepted; i++) {
        g_tx[g_tx_head] = data[i];
        g_tx_head = Next(g_tx_head, kTxSize);
    }
    StartDma();
}

bool DebugUart_TryWriteData(const uint8_t *data, uint32_t length)
{
    if ((data == 0) || (length == 0U)) {
        return false;
    }
    if (length > TxFree()) {
        g_tx_dropped += length;
        return false;
    }
    for (uint32_t i = 0U; i < length; i++) {
        g_tx[g_tx_head] = data[i];
        g_tx_head = Next(g_tx_head, kTxSize);
    }
    StartDma();
    return true;
}

void DebugUart_WriteString(const char *text)
{
    if (text != 0) {
        DebugUart_WriteData(
            reinterpret_cast<const uint8_t *>(text),
            static_cast<uint32_t>(strlen(text)));
    }
}

void DebugUart_Pump(void)
{
    StartDma();
}

uint32_t DebugUart_GetRxDropped(void)
{
    return g_rx_dropped;
}

uint32_t DebugUart_GetTxDropped(void)
{
    return g_tx_dropped;
}

void DebugUart_IrqHandler(void)
{
    const DL_UART_IIDX pending =
        DL_UART_Main_getPendingInterrupt(DEBUG_UART_INST);
    if (pending == DL_UART_MAIN_IIDX_RX) {
        while (!DL_UART_Main_isRXFIFOEmpty(DEBUG_UART_INST)) {
            const uint16_t next = Next(g_rx_head, kRxSize);
            const uint8_t value =
                DL_UART_Main_receiveData(DEBUG_UART_INST);
            if (next == g_rx_tail) {
                g_rx_dropped++;
            } else {
                g_rx[g_rx_head] = value;
                g_rx_head = next;
            }
        }
        DL_UART_Main_clearInterruptStatus(
            DEBUG_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    } else if (pending == DL_UART_MAIN_IIDX_DMA_DONE_TX) {
        CompleteDma();
    }
}

void DebugUart_HandleDmaFault(void)
{
    DL_DMA_disableChannel(DMA, DEBUG_UART_DMA_TX_CHAN_ID);
    if (g_dma_active) {
        g_tx_tail = static_cast<uint16_t>(
            (static_cast<uint32_t>(g_tx_tail) + g_dma_length) % kTxSize);
        g_tx_dropped += g_dma_length;
    }
    g_dma_length = 0U;
    g_dma_active = false;
    StartDma();
}

} /* namespace services */

extern "C" void DEBUG_UART_INST_IRQHandler(void)
{
    services::DebugUart_IrqHandler();
}
