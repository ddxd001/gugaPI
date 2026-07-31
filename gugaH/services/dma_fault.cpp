#include "services/dma_fault.h"

#include "ti_msp_dl_config.h"

namespace services {
namespace {

static const uint8_t kMaximumHandlers = 4U;
static DmaFaultHandler g_handlers[kMaximumHandlers] = {};
static uint8_t g_handler_count = 0U;
static volatile uint32_t g_fault_count = 0U;

} /* namespace */

bool DmaFault_RegisterHandler(DmaFaultHandler handler)
{
    if (handler == 0) {
        return false;
    }
    for (uint8_t i = 0U; i < g_handler_count; i++) {
        if (g_handlers[i] == handler) {
            return true;
        }
    }
    if (g_handler_count >= kMaximumHandlers) {
        return false;
    }
    g_handlers[g_handler_count++] = handler;
    DL_DMA_clearInterruptStatus(
        DMA, DL_DMA_INTERRUPT_ADDR_ERROR | DL_DMA_INTERRUPT_DATA_ERROR);
    DL_DMA_enableInterrupt(
        DMA, DL_DMA_INTERRUPT_ADDR_ERROR | DL_DMA_INTERRUPT_DATA_ERROR);
    NVIC_ClearPendingIRQ(DMA_INT_IRQn);
    NVIC_EnableIRQ(DMA_INT_IRQn);
    return true;
}

uint32_t DmaFault_GetCount(void)
{
    return g_fault_count;
}

void DmaFault_IrqHandler(void)
{
    bool fault = false;
    while (true) {
        const DL_DMA_EVENT_IIDX pending =
            DL_DMA_getPendingInterrupt(DMA);
        if (pending == DL_DMA_EVENT_IIDX_ADDR_ERROR) {
            DL_DMA_clearInterruptStatus(
                DMA, DL_DMA_INTERRUPT_ADDR_ERROR);
            fault = true;
        } else if (pending == DL_DMA_EVENT_IIDX_DATA_ERROR) {
            DL_DMA_clearInterruptStatus(
                DMA, DL_DMA_INTERRUPT_DATA_ERROR);
            fault = true;
        } else {
            break;
        }
    }
    if (!fault) {
        return;
    }
    g_fault_count++;
    for (uint8_t i = 0U; i < g_handler_count; i++) {
        g_handlers[i]();
    }
}

} /* namespace services */

extern "C" void DMA_IRQHandler(void)
{
    services::DmaFault_IrqHandler();
}
