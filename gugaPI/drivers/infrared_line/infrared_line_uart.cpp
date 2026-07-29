#include "drivers/infrared_line/infrared_line_uart.h"

namespace drivers {
namespace {

uint32_t RxClearMask(void)
{
    return DL_UART_MAIN_INTERRUPT_RX |
           DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
           DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
           DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
           DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
           DL_UART_MAIN_INTERRUPT_NOISE_ERROR;
}

uint32_t RxStatusMask(void)
{
    return DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
           DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
           DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
           DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
           DL_UART_MAIN_INTERRUPT_NOISE_ERROR;
}

void IncrementSaturated(volatile uint32_t *value)
{
    if (*value != UINT32_MAX) {
        (*value)++;
    }
}

void RecordRxStatus(InfraredLineUartContext *context, uint32_t status)
{
    if ((status & DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR) != 0U) {
        /* RX timeout reports an idle gap after received data. It is useful
         * for diagnostics but is not a malformed UART character. */
        IncrementSaturated(&context->rx_timeout_count);
    }
    if ((status & DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR) != 0U) {
        IncrementSaturated(&context->overrun_error_count);
        IncrementSaturated(&context->uart_error_count);
    }
    if ((status & DL_UART_MAIN_INTERRUPT_FRAMING_ERROR) != 0U) {
        IncrementSaturated(&context->framing_error_count);
        IncrementSaturated(&context->uart_error_count);
    }
    if ((status & DL_UART_MAIN_INTERRUPT_PARITY_ERROR) != 0U) {
        IncrementSaturated(&context->parity_error_count);
        IncrementSaturated(&context->uart_error_count);
    }
    if ((status & DL_UART_MAIN_INTERRUPT_NOISE_ERROR) != 0U) {
        IncrementSaturated(&context->noise_error_count);
        IncrementSaturated(&context->uart_error_count);
    }
}

uint16_t NextIndex(const InfraredLineUartConfig *config, uint16_t index)
{
    index++;
    return (index >= config->rx_buffer_size) ? 0U : index;
}

void DrainRxFifo(InfraredLineUartContext *context, bool polled)
{
    while (!DL_UART_Main_isRXFIFOEmpty(context->config->uart)) {
        const uint8_t data = DL_UART_Main_receiveData(context->config->uart);
        context->fifo_byte_count++;
        if (polled) {
            context->polled_byte_count++;
        }
        const uint16_t next = NextIndex(context->config, context->rx_head);
        if (next == context->rx_tail) {
            context->rx_dropped_count++;
        } else {
            context->config->rx_buffer[context->rx_head] = data;
            context->rx_head = next;
        }
    }
}

} /* namespace */

DriverStatus InfraredLineUart_Init(InfraredLineUartContext *context,
                                   const InfraredLineUartConfig *config)
{
    if ((context == 0) || (config == 0) || (config->uart == 0) ||
        (config->rx_buffer == 0) || (config->rx_buffer_size < 16U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }
    /* The project owns SYSCFG_DL_init() and calls generated peripheral init
     * functions conditionally. Refuse to report a ready transport if the
     * generated UART init was accidentally omitted from that chain. */
    if ((!DL_UART_Main_isPowerEnabled(config->uart)) ||
        (!DL_UART_Main_isEnabled(config->uart))) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    NVIC_DisableIRQ(config->irq);
    context->config = config;
    context->rx_head = 0U;
    context->rx_tail = 0U;
    context->rx_dropped_count = 0U;
    context->uart_error_count = 0U;
    context->rx_timeout_count = 0U;
    context->overrun_error_count = 0U;
    context->framing_error_count = 0U;
    context->parity_error_count = 0U;
    context->noise_error_count = 0U;
    context->irq_count = 0U;
    context->fifo_byte_count = 0U;
    context->polled_byte_count = 0U;
    DL_UART_Main_clearInterruptStatus(config->uart, RxClearMask());
    NVIC_ClearPendingIRQ(config->irq);
    context->initialized = true;
    NVIC_EnableIRQ(config->irq);
    return DRIVER_OK;
}

bool InfraredLineUart_ReadByte(InfraredLineUartContext *context,
                               uint8_t *data)
{
    if ((context == 0) || (!context->initialized) || (data == 0) ||
        (context->rx_head == context->rx_tail)) {
        return false;
    }
    *data = context->config->rx_buffer[context->rx_tail];
    context->rx_tail = NextIndex(context->config, context->rx_tail);
    return true;
}

void InfraredLineUart_ServiceRx(InfraredLineUartContext *context)
{
    if ((context == 0) || (!context->initialized)) {
        return;
    }
    /* The interrupt remains the normal low-latency path. This bounded 1 ms
     * fallback also drains a pending FIFO if the NVIC/vector path is broken,
     * and exposes that fact through polled_byte_count for bench diagnosis. */
    NVIC_DisableIRQ(context->config->irq);
    const uint32_t raw_status = DL_UART_Main_getRawInterruptStatus(
        context->config->uart, RxStatusMask());
    RecordRxStatus(context, raw_status);
    DrainRxFifo(context, true);
    DL_UART_Main_clearInterruptStatus(context->config->uart, RxClearMask());
    NVIC_ClearPendingIRQ(context->config->irq);
    NVIC_EnableIRQ(context->config->irq);
}

void InfraredLineUart_Clear(InfraredLineUartContext *context)
{
    if ((context == 0) || (!context->initialized)) {
        return;
    }
    NVIC_DisableIRQ(context->config->irq);
    while (!DL_UART_Main_isRXFIFOEmpty(context->config->uart)) {
        (void) DL_UART_Main_receiveData(context->config->uart);
    }
    context->rx_head = 0U;
    context->rx_tail = 0U;
    context->rx_dropped_count = 0U;
    context->uart_error_count = 0U;
    context->rx_timeout_count = 0U;
    context->overrun_error_count = 0U;
    context->framing_error_count = 0U;
    context->parity_error_count = 0U;
    context->noise_error_count = 0U;
    context->irq_count = 0U;
    context->fifo_byte_count = 0U;
    context->polled_byte_count = 0U;
    DL_UART_Main_clearInterruptStatus(context->config->uart, RxClearMask());
    NVIC_ClearPendingIRQ(context->config->irq);
    NVIC_EnableIRQ(context->config->irq);
}

void InfraredLineUart_IrqHandler(InfraredLineUartContext *context)
{
    if ((context == 0) || (!context->initialized)) {
        return;
    }
    context->irq_count++;
    const uint32_t raw_status = DL_UART_Main_getRawInterruptStatus(
        context->config->uart, RxStatusMask());
    RecordRxStatus(context, raw_status);
    DrainRxFifo(context, false);
    DL_UART_Main_clearInterruptStatus(context->config->uart, RxClearMask());
}

} /* namespace drivers */
