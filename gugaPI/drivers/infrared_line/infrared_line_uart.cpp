#include "drivers/infrared_line/infrared_line_uart.h"

namespace drivers {
namespace {

uint32_t ErrorStatusMask(void)
{
    return DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
           DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
           DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
           DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
           DL_UART_MAIN_INTERRUPT_NOISE_ERROR;
}

uint32_t RxStatusMask(void)
{
    return DL_UART_MAIN_INTERRUPT_DMA_DONE_RX | ErrorStatusMask();
}

uint32_t EnabledInterruptMask(void)
{
    return DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
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

void AddSaturated(volatile uint32_t *value, uint32_t amount)
{
    const uint32_t result = *value + amount;
    *value = (result < *value) ? UINT32_MAX : result;
}

void RecordRxStatus(InfraredLineUartContext *context, uint32_t status)
{
    if ((status & DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR) != 0U) {
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

void AccountWrap(InfraredLineUartContext *context, bool polled)
{
    context->cycle_wrap_generation++;
    IncrementSaturated(&context->dma_wrap_count);
    if (polled) {
        AddSaturated(&context->polled_byte_count,
                     context->config->dma_buffer_size);
    }
    __DMB();
}

void ArmRepeatDma(InfraredLineUartContext *context)
{
    const InfraredLineUartConfig *config = context->config;
    DL_DMA_disableChannel(config->dma, config->dma_channel);
    DL_DMA_setSrcAddr(
        config->dma,
        config->dma_channel,
        static_cast<uint32_t>(
            reinterpret_cast<uintptr_t>(&config->uart->RXDATA)));
    DL_DMA_setDestAddr(
        config->dma,
        config->dma_channel,
        static_cast<uint32_t>(
            reinterpret_cast<uintptr_t>(config->dma_buffer)));
    DL_DMA_setTransferSize(
        config->dma, config->dma_channel, config->dma_buffer_size);
    DL_DMA_enableChannel(config->dma, config->dma_channel);
}

void RestartReceive(InfraredLineUartContext *context)
{
    const InfraredLineUartConfig *config = context->config;
    DL_DMA_disableChannel(config->dma, config->dma_channel);
    while (!DL_UART_Main_isRXFIFOEmpty(config->uart)) {
        (void) DL_UART_Main_receiveData(config->uart);
    }
    context->cursor.consumer = context->cursor.producer;
    context->cycle_base = context->cursor.producer;
    context->cycle_wrap_generation = 0U;
    DL_UART_Main_clearInterruptStatus(config->uart, RxStatusMask());
    NVIC_ClearPendingIRQ(config->irq);
    ArmRepeatDma(context);
}

uint64_t SnapshotProducer(InfraredLineUartContext *context,
                          uint32_t *observed_errors)
{
    const InfraredLineUartConfig *config = context->config;
    uint32_t wraps = context->cycle_wrap_generation;
    uint16_t remaining = DL_DMA_getTransferSize(
        config->dma, config->dma_channel);

    /* A wrap may occur between the generation and DMASZ reads. Consume only
     * the completion event that was explicitly observed, then retry. Events
     * arriving after the final check remain pending for the ISR. */
    for (uint8_t attempt = 0U; attempt < 3U; attempt++) {
        const uint32_t status = DL_UART_Main_getRawInterruptStatus(
            config->uart, RxStatusMask());
        *observed_errors |= status & ErrorStatusMask();
        if ((status & DL_UART_MAIN_INTERRUPT_DMA_DONE_RX) != 0U) {
            AccountWrap(context, true);
            DL_UART_Main_clearInterruptStatus(
                config->uart, DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
            wraps = context->cycle_wrap_generation;
            remaining = DL_DMA_getTransferSize(
                config->dma, config->dma_channel);
            continue;
        }
        const uint32_t wraps_after = context->cycle_wrap_generation;
        const uint16_t remaining_after = DL_DMA_getTransferSize(
            config->dma, config->dma_channel);
        if (wraps_after == wraps) {
            remaining = remaining_after;
            break;
        }
        wraps = wraps_after;
        remaining = remaining_after;
    }
    if (remaining > config->dma_buffer_size) {
        remaining = config->dma_buffer_size;
    }
    return context->cycle_base +
        static_cast<uint64_t>(wraps) * config->dma_buffer_size +
        static_cast<uint64_t>(config->dma_buffer_size - remaining);
}

} /* namespace */

DriverStatus InfraredLineUart_Init(InfraredLineUartContext *context,
                                   const InfraredLineUartConfig *config)
{
    if ((context == 0) || (config == 0) || (config->uart == 0) ||
        (config->dma == 0) || (config->dma_buffer == 0) ||
        (config->dma_buffer_size < 16U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }
    if ((!DL_UART_Main_isPowerEnabled(config->uart)) ||
        (!DL_UART_Main_isEnabled(config->uart))) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (DL_DMA_getTransferMode(config->dma, config->dma_channel) !=
        DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    NVIC_DisableIRQ(config->irq);
    *context = {};
    context->config = config;
    while (!DL_UART_Main_isRXFIFOEmpty(config->uart)) {
        (void) DL_UART_Main_receiveData(config->uart);
    }
    DL_UART_Main_clearInterruptStatus(config->uart, RxStatusMask());
    DL_UART_Main_enableInterrupt(config->uart, EnabledInterruptMask());
    NVIC_SetPriority(config->irq, 0U);
    NVIC_ClearPendingIRQ(config->irq);
    context->initialized = true;
    ArmRepeatDma(context);
    NVIC_EnableIRQ(config->irq);
    return DRIVER_OK;
}

bool InfraredLineUart_ReadByte(InfraredLineUartContext *context,
                               uint8_t *data)
{
    if ((context == 0) || (!context->initialized) || (data == 0)) {
        return false;
    }
    uint16_t index = 0U;
    if (!InfraredLineDmaCursor_ReadIndex(&context->cursor,
                                         context->cycle_base,
                                         context->config->dma_buffer_size,
                                         &index)) {
        return false;
    }
    *data = context->config->dma_buffer[index];
    return true;
}

void InfraredLineUart_ServiceRx(InfraredLineUartContext *context)
{
    if ((context == 0) || (!context->initialized)) {
        return;
    }
    NVIC_DisableIRQ(context->config->irq);
    uint32_t observed_errors = 0U;
    const uint64_t producer = SnapshotProducer(context, &observed_errors);
    if (observed_errors != 0U) {
        RecordRxStatus(context, observed_errors);
        DL_UART_Main_clearInterruptStatus(context->config->uart,
                                           observed_errors);
    }
    __DMB();
    (void) InfraredLineDmaCursor_Publish(
        &context->cursor, producer, context->config->dma_buffer_size);
    NVIC_EnableIRQ(context->config->irq);
}

void InfraredLineUart_ClearStats(InfraredLineUartContext *context)
{
    if ((context == 0) || (!context->initialized)) {
        return;
    }
    NVIC_DisableIRQ(context->config->irq);
    context->stats_base_producer = context->cursor.producer;
    context->cursor.maximum_lag = 0U;
    context->cursor.overwrite_count = 0U;
    context->cursor.dropped_bytes = 0U;
    context->dma_wrap_count = 0U;
    context->uart_error_count = 0U;
    context->rx_timeout_count = 0U;
    context->overrun_error_count = 0U;
    context->framing_error_count = 0U;
    context->parity_error_count = 0U;
    context->noise_error_count = 0U;
    context->irq_count = 0U;
    context->polled_byte_count = 0U;
    context->dma_fault_count = 0U;
    NVIC_EnableIRQ(context->config->irq);
}

void InfraredLineUart_IrqHandler(InfraredLineUartContext *context)
{
    if ((context == 0) || (!context->initialized)) {
        return;
    }
    IncrementSaturated(&context->irq_count);
    const uint32_t status = DL_UART_Main_getRawInterruptStatus(
        context->config->uart, RxStatusMask());
    if ((status & DL_UART_MAIN_INTERRUPT_DMA_DONE_RX) != 0U) {
        AccountWrap(context, false);
    }
    RecordRxStatus(context, status & ErrorStatusMask());
    if (status != 0U) {
        DL_UART_Main_clearInterruptStatus(context->config->uart, status);
    }
}

void InfraredLineUart_HandleDmaFault(InfraredLineUartContext *context)
{
    if ((context == 0) || (!context->initialized)) {
        return;
    }
    NVIC_DisableIRQ(context->config->irq);
    IncrementSaturated(&context->dma_fault_count);
    RestartReceive(context);
    NVIC_EnableIRQ(context->config->irq);
}

uint64_t InfraredLineUart_GetProducedCount(
    const InfraredLineUartContext *context)
{
    return (context == 0) ? 0U : context->cursor.producer;
}

uint64_t InfraredLineUart_GetConsumedCount(
    const InfraredLineUartContext *context)
{
    return (context == 0) ? 0U : context->cursor.consumer;
}

uint32_t InfraredLineUart_GetCurrentLag(
    const InfraredLineUartContext *context)
{
    return (context == 0) ? 0U :
        InfraredLineDmaCursor_Lag(&context->cursor);
}

uint16_t InfraredLineUart_GetDmaRemaining(
    const InfraredLineUartContext *context)
{
    if ((context == 0) || (!context->initialized)) {
        return 0U;
    }
    return DL_DMA_getTransferSize(context->config->dma,
                                  context->config->dma_channel);
}

} /* namespace drivers */
