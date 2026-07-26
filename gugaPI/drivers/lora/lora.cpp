#include "drivers/lora/lora.h"

namespace drivers {
namespace {

bool IsConfigValid(const LoraUartConfig *config)
{
    return (config != 0) && (config->uart != 0) && (config->rx_buffer != 0) &&
           (config->rx_buffer_size > 1U) &&
           (config->tx_timeout_iterations > 0U);
}

uint16_t NextRxIndex(const LoraUartConfig *config, uint16_t index)
{
    index++;
    if (index >= config->rx_buffer_size) {
        index = 0U;
    }
    return index;
}

void PushRxByteFromIsr(LoraUartContext *context, uint8_t data)
{
    const uint16_t next_head = NextRxIndex(context->config, context->rx_head);

    if (next_head == context->rx_tail) {
        context->rx_dropped_count++;
        return;
    }

    context->config->rx_buffer[context->rx_head] = data;
    context->rx_head = next_head;
}

bool WaitTxFifoSpace(const LoraUartConfig *config)
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

/* Clear RX plus every error flag so an overrun/framing/parity/noise event
 * does not leave the flag set and re-fire the ISR forever. Mirrors the
 * motor_driver_uart handler. */
uint32_t RxClearMask(void)
{
    return DL_UART_MAIN_INTERRUPT_RX |
           DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
           DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
           DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
           DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
           DL_UART_MAIN_INTERRUPT_NOISE_ERROR;
}

} /* namespace */

DriverStatus LoraUart_Init(LoraUartContext *context,
                           const LoraUartConfig *config)
{
    if ((context == 0) || (!IsConfigValid(config))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    NVIC_DisableIRQ(config->irq);

    context->config = config;
    context->rx_head = 0U;
    context->rx_tail = 0U;
    context->rx_dropped_count = 0U;

    /* LoRa 模块为串口透传模式，UART 时钟和引脚复用由 SysConfig 完成。 */
    DL_UART_Main_clearInterruptStatus(config->uart, RxClearMask());
    NVIC_ClearPendingIRQ(config->irq);

    context->initialized = true;
    NVIC_EnableIRQ(config->irq);

    return DRIVER_OK;
}

DriverStatus LoraUart_Deinit(LoraUartContext *context)
{
    if ((context == 0) || (context->config == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    NVIC_DisableIRQ(context->config->irq);
    context->initialized = false;
    context->rx_head = 0U;
    context->rx_tail = 0U;

    return DRIVER_OK;
}

bool LoraUart_IsReady(const LoraUartContext *context)
{
    return (context != 0) && context->initialized && (context->config != 0);
}

DriverStatus LoraUart_WriteByte(LoraUartContext *context, uint8_t data)
{
    if (!LoraUart_IsReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    if (!WaitTxFifoSpace(context->config)) {
        return DRIVER_ERROR_TIMEOUT;
    }

    /* 不使用 transmitDataBlocking，避免 UART busy 位异常时锁死 shell。 */
    DL_UART_Main_transmitData(context->config->uart, data);
    return DRIVER_OK;
}

DriverStatus LoraUart_Write(LoraUartContext *context,
                            const uint8_t *data,
                            uint16_t length)
{
    if (!LoraUart_IsReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    if ((data == 0) && (length != 0U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    for (uint16_t i = 0U; i < length; i++) {
        const DriverStatus status = LoraUart_WriteByte(context, data[i]);
        if (status != DRIVER_OK) {
            return status;
        }
    }

    return DRIVER_OK;
}

bool LoraUart_ReadByte(LoraUartContext *context, uint8_t *data)
{
    if ((!LoraUart_IsReady(context)) || (data == 0) ||
        (context->rx_head == context->rx_tail)) {
        return false;
    }

    *data = context->config->rx_buffer[context->rx_tail];
    context->rx_tail = NextRxIndex(context->config, context->rx_tail);

    return true;
}

uint16_t LoraUart_GetRxAvailable(const LoraUartContext *context)
{
    if (!LoraUart_IsReady(context)) {
        return 0U;
    }

    const uint16_t head = context->rx_head;
    const uint16_t tail = context->rx_tail;

    if (head >= tail) {
        return (uint16_t) (head - tail);
    }

    return (uint16_t) (context->config->rx_buffer_size - tail + head);
}

uint32_t LoraUart_GetRxDroppedCount(const LoraUartContext *context)
{
    if (context == 0) {
        return 0U;
    }

    return context->rx_dropped_count;
}

uint32_t LoraUart_GetBaudrate(const LoraUartContext *context)
{
    if (!LoraUart_IsReady(context)) {
        return 0U;
    }

    return context->config->baudrate;
}

DriverStatus LoraUart_GetControllerStatus(const LoraUartContext *context,
                                           uint32_t *status)
{
    if (!LoraUart_IsReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    if (status == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    *status = context->config->uart->STAT;
    return DRIVER_OK;
}

DriverStatus LoraUart_ClearRxBuffer(LoraUartContext *context)
{
    if (!LoraUart_IsReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    NVIC_DisableIRQ(context->config->irq);
    context->rx_head = 0U;
    context->rx_tail = 0U;
    context->rx_dropped_count = 0U;
    NVIC_ClearPendingIRQ(context->config->irq);
    NVIC_EnableIRQ(context->config->irq);

    return DRIVER_OK;
}

void LoraUart_IrqHandler(LoraUartContext *context)
{
    if (!LoraUart_IsReady(context)) {
        return;
    }

    (void) DL_UART_Main_getPendingInterrupt(context->config->uart);

    while (!DL_UART_Main_isRXFIFOEmpty(context->config->uart)) {
        PushRxByteFromIsr(
            context,
            DL_UART_Main_receiveData(context->config->uart));
    }

    DL_UART_Main_clearInterruptStatus(context->config->uart, RxClearMask());
}

} /* namespace drivers */