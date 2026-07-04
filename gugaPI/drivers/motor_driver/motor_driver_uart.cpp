#include "drivers/motor_driver/motor_driver_uart.h"

namespace drivers {
namespace {

bool IsConfigValid(const MotorDriverUartConfig *config)
{
    return (config != 0) && (config->uart != 0) && (config->rx_buffer != 0) &&
           (config->rx_buffer_size > 1U) &&
           (config->tx_timeout_iterations > 0U);
}

uint16_t NextRxIndex(const MotorDriverUartConfig *config, uint16_t index)
{
    index++;
    if (index >= config->rx_buffer_size) {
        index = 0U;
    }
    return index;
}

void PushRxByteFromIsr(MotorDriverUartContext *context, uint8_t data)
{
    const uint16_t next_head = NextRxIndex(context->config, context->rx_head);

    if (next_head == context->rx_tail) {
        context->rx_dropped_count++;
        return;
    }

    context->config->rx_buffer[context->rx_head] = data;
    context->rx_head = next_head;
}

bool WaitTxFifoSpace(const MotorDriverUartConfig *config)
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

} /* namespace */

DriverStatus MotorDriverUart_Init(MotorDriverUartContext *context,
                                  const MotorDriverUartConfig *config)
{
    if ((context == 0) || (!IsConfigValid(config))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    NVIC_DisableIRQ(config->irq);

    context->config = config;
    context->rx_head = 0U;
    context->rx_tail = 0U;
    context->rx_dropped_count = 0U;

    /* MotorDriver 串口只做链路收发，具体电机协议后续再叠加。 */
    DL_UART_Main_clearInterruptStatus(config->uart, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(config->irq);

    context->initialized = true;
    NVIC_EnableIRQ(config->irq);

    return DRIVER_OK;
}

bool MotorDriverUart_IsReady(const MotorDriverUartContext *context)
{
    return (context != 0) && context->initialized && (context->config != 0);
}

DriverStatus MotorDriverUart_WriteByte(MotorDriverUartContext *context,
                                       uint8_t data)
{
    if (!MotorDriverUart_IsReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    if (!WaitTxFifoSpace(context->config)) {
        return DRIVER_ERROR_TIMEOUT;
    }

    DL_UART_Main_transmitData(context->config->uart, data);
    return DRIVER_OK;
}

DriverStatus MotorDriverUart_Write(MotorDriverUartContext *context,
                                   const uint8_t *data,
                                   uint16_t length)
{
    if (!MotorDriverUart_IsReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    if ((data == 0) && (length != 0U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    for (uint16_t i = 0U; i < length; i++) {
        const DriverStatus status = MotorDriverUart_WriteByte(context, data[i]);
        if (status != DRIVER_OK) {
            return status;
        }
    }

    return DRIVER_OK;
}

bool MotorDriverUart_ReadByte(MotorDriverUartContext *context, uint8_t *data)
{
    if ((!MotorDriverUart_IsReady(context)) || (data == 0) ||
        (context->rx_head == context->rx_tail)) {
        return false;
    }

    *data = context->config->rx_buffer[context->rx_tail];
    context->rx_tail = NextRxIndex(context->config, context->rx_tail);

    return true;
}

uint16_t MotorDriverUart_GetRxAvailable(const MotorDriverUartContext *context)
{
    if (!MotorDriverUart_IsReady(context)) {
        return 0U;
    }

    const uint16_t head = context->rx_head;
    const uint16_t tail = context->rx_tail;

    if (head >= tail) {
        return (uint16_t) (head - tail);
    }

    return (uint16_t) (context->config->rx_buffer_size - tail + head);
}

uint32_t MotorDriverUart_GetRxDroppedCount(
    const MotorDriverUartContext *context)
{
    if (context == 0) {
        return 0U;
    }

    return context->rx_dropped_count;
}

uint32_t MotorDriverUart_GetBaudrate(const MotorDriverUartContext *context)
{
    if (!MotorDriverUart_IsReady(context)) {
        return 0U;
    }

    return context->config->baudrate;
}

DriverStatus MotorDriverUart_GetControllerStatus(
    const MotorDriverUartContext *context,
    uint32_t *status)
{
    if (!MotorDriverUart_IsReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    if (status == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    *status = context->config->uart->STAT;
    return DRIVER_OK;
}

DriverStatus MotorDriverUart_ClearRxBuffer(MotorDriverUartContext *context)
{
    if (!MotorDriverUart_IsReady(context)) {
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

void MotorDriverUart_IrqHandler(MotorDriverUartContext *context)
{
    if (!MotorDriverUart_IsReady(context)) {
        return;
    }

    switch (DL_UART_Main_getPendingInterrupt(context->config->uart)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(context->config->uart)) {
            PushRxByteFromIsr(
                context,
                DL_UART_Main_receiveData(context->config->uart));
        }
        DL_UART_Main_clearInterruptStatus(context->config->uart,
                                          DL_UART_MAIN_INTERRUPT_RX);
        break;

    default:
        break;
    }
}

} /* namespace drivers */
