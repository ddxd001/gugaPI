#include "drivers/uart_link.h"

namespace drivers {
namespace {

bool IsConfigValid(const UartLinkConfig *config)
{
    return (config != 0) && (config->uart != 0) && (config->rx_buffer != 0) &&
           (config->rx_buffer_size > 1U) &&
           (config->tx_timeout_iterations > 0U);
}

uint16_t NextRxIndex(const UartLinkConfig *config, uint16_t index)
{
    index++;
    if (index >= config->rx_buffer_size) {
        index = 0U;
    }

    return index;
}

void PushRxFromIsr(UartLinkContext *context, uint8_t value)
{
    const uint16_t next_head = NextRxIndex(context->config, context->rx_head);

    if (next_head == context->rx_tail) {
        context->rx_dropped_count++;
        return;
    }

    context->config->rx_buffer[context->rx_head] = value;
    context->rx_head = next_head;
}

bool WaitTxSpace(const UartLinkConfig *config)
{
    uint32_t timeout = config->tx_timeout_iterations;

    while (DL_UART_Main_isTXFIFOFull(config->uart)) {
        if (timeout == 0U) {
            return false;
        }
        timeout--;
    }

    return true;
}

}  // namespace

UartLinkStatus UartLink_Init(UartLinkContext *context,
                             const UartLinkConfig *config)
{
    if ((context == 0) || (!IsConfigValid(config))) {
        return UartLinkStatus::InvalidArgument;
    }

    NVIC_DisableIRQ(config->irq);

    context->config = config;
    context->rx_head = 0U;
    context->rx_tail = 0U;
    context->rx_dropped_count = 0U;
    context->initialized = true;

    DL_UART_Main_clearInterruptStatus(config->uart,
                                      DL_UART_MAIN_INTERRUPT_RX |
                                          DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
                                          DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                          DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                          DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                          DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    NVIC_ClearPendingIRQ(config->irq);
    NVIC_EnableIRQ(config->irq);

    return UartLinkStatus::Ok;
}

bool UartLink_IsReady(const UartLinkContext *context)
{
    return (context != 0) && context->initialized && (context->config != 0);
}

UartLinkStatus UartLink_WriteByte(UartLinkContext *context, uint8_t value)
{
    if (!UartLink_IsReady(context)) {
        return UartLinkStatus::NotInitialized;
    }

    if (!WaitTxSpace(context->config)) {
        return UartLinkStatus::Timeout;
    }

    DL_UART_Main_transmitData(context->config->uart, value);
    return UartLinkStatus::Ok;
}

UartLinkStatus UartLink_Write(UartLinkContext *context,
                              const uint8_t *data,
                              uint8_t length)
{
    if (!UartLink_IsReady(context)) {
        return UartLinkStatus::NotInitialized;
    }
    if ((data == 0) && (length > 0U)) {
        return UartLinkStatus::InvalidArgument;
    }

    for (uint8_t index = 0U; index < length; index++) {
        const UartLinkStatus status = UartLink_WriteByte(context, data[index]);
        if (status != UartLinkStatus::Ok) {
            return status;
        }
    }

    return UartLinkStatus::Ok;
}

bool UartLink_ReadByte(UartLinkContext *context, uint8_t *value)
{
    if ((!UartLink_IsReady(context)) || (value == 0) ||
        (context->rx_head == context->rx_tail)) {
        return false;
    }

    *value = context->config->rx_buffer[context->rx_tail];
    context->rx_tail = NextRxIndex(context->config, context->rx_tail);
    return true;
}

uint16_t UartLink_RxAvailable(const UartLinkContext *context)
{
    if (!UartLink_IsReady(context)) {
        return 0U;
    }

    const uint16_t head = context->rx_head;
    const uint16_t tail = context->rx_tail;

    if (head >= tail) {
        return static_cast<uint16_t>(head - tail);
    }

    return static_cast<uint16_t>(context->config->rx_buffer_size - tail + head);
}

uint32_t UartLink_RxDroppedCount(const UartLinkContext *context)
{
    if (context == 0) {
        return 0U;
    }

    return context->rx_dropped_count;
}

void UartLink_IrqHandler(UartLinkContext *context)
{
    if (!UartLink_IsReady(context)) {
        return;
    }

    (void) DL_UART_Main_getPendingInterrupt(context->config->uart);

    while (!DL_UART_Main_isRXFIFOEmpty(context->config->uart)) {
        PushRxFromIsr(context,
                      DL_UART_Main_receiveData(context->config->uart));
    }

    DL_UART_Main_clearInterruptStatus(context->config->uart,
                                      DL_UART_MAIN_INTERRUPT_RX |
                                          DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
                                          DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                          DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                          DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                          DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
}

}  // namespace drivers
