#include "drivers/ball_vision/ball_vision_uart.h"

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

uint16_t NextIndex(const BallVisionUartConfig *config, uint16_t index)
{
    index++;
    return (index >= config->rx_buffer_size) ? 0U : index;
}

void PushFromIsr(BallVisionUartContext *context, uint8_t data)
{
    const uint16_t next = NextIndex(context->config, context->rx_head);
    if (next == context->rx_tail) {
        context->dropped_bytes++;
        return;
    }
    context->config->rx_buffer[context->rx_head] = data;
    context->rx_head = next;
}

void DiscardHardwareRx(UART_Regs *uart)
{
    while (!DL_UART_Main_isRXFIFOEmpty(uart)) {
        (void)DL_UART_Main_receiveData(uart);
    }
}

} /* namespace */

DriverStatus BallVisionUart_Init(BallVisionUartContext *context,
                                 const BallVisionUartConfig *config)
{
    if ((context == 0) || (config == 0) || (config->uart == 0) ||
        (config->rx_buffer == 0) || (config->rx_buffer_size < 2U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    NVIC_DisableIRQ(config->irq);
    context->config = config;
    context->rx_head = 0U;
    context->rx_tail = 0U;
    context->dropped_bytes = 0U;
    context->uart_errors = 0U;
    context->irq_count = 0U;
    /* SysConfig enables the UART before this driver runs.  If MaixCAM is
     * already streaming, the hardware FIFO can contain the tail of a frame
     * from before software initialization.  Drop it and lock to the next
     * A5 5A header instead of publishing startup residue. */
    DiscardHardwareRx(config->uart);
    DL_UART_Main_clearInterruptStatus(config->uart, RxClearMask());
    NVIC_ClearPendingIRQ(config->irq);
    context->initialized = true;
    NVIC_EnableIRQ(config->irq);
    return DRIVER_OK;
}

bool BallVisionUart_ReadByte(BallVisionUartContext *context, uint8_t *data)
{
    if ((context == 0) || (!context->initialized) ||
        (context->config == 0) || (data == 0) ||
        (context->rx_head == context->rx_tail)) {
        return false;
    }
    *data = context->config->rx_buffer[context->rx_tail];
    context->rx_tail = NextIndex(context->config, context->rx_tail);
    return true;
}

void BallVisionUart_Clear(BallVisionUartContext *context)
{
    if ((context == 0) || (!context->initialized) ||
        (context->config == 0)) {
        return;
    }
    NVIC_DisableIRQ(context->config->irq);
    context->rx_head = 0U;
    context->rx_tail = 0U;
    context->dropped_bytes = 0U;
    context->uart_errors = 0U;
    context->irq_count = 0U;
    DiscardHardwareRx(context->config->uart);
    DL_UART_Main_clearInterruptStatus(context->config->uart, RxClearMask());
    NVIC_ClearPendingIRQ(context->config->irq);
    NVIC_EnableIRQ(context->config->irq);
}

void BallVisionUart_IrqHandler(BallVisionUartContext *context)
{
    if ((context == 0) || (!context->initialized) ||
        (context->config == 0)) {
        return;
    }
    context->irq_count++;
    const uint32_t pending =
        DL_UART_Main_getPendingInterrupt(context->config->uart);
    if ((pending != DL_UART_MAIN_IIDX_RX) &&
        (pending != DL_UART_MAIN_IIDX_NO_INTERRUPT)) {
        context->uart_errors++;
    }
    while (!DL_UART_Main_isRXFIFOEmpty(context->config->uart)) {
        PushFromIsr(
            context,
            DL_UART_Main_receiveData(context->config->uart));
    }
    DL_UART_Main_clearInterruptStatus(context->config->uart, RxClearMask());
}

} /* namespace drivers */
