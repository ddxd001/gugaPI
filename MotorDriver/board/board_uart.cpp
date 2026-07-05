#include "board/board_uart.h"

#include "board/board_motor_pins.h"

namespace board {
namespace {

uint8_t g_rx_buffer[kUartRxBufferSize];

const drivers::UartLinkConfig g_uart_config = {
    BOARD_UART_INST,
    BOARD_UART_IRQN,
    kUartTxTimeoutIterations,
    g_rx_buffer,
    kUartRxBufferSize,
};

drivers::UartLinkContext g_uart_context = {
    0,
    0U,
    0U,
    0U,
    false,
};

void ConfigureRxPullUp(void)
{
    DL_GPIO_initPeripheralInputFunctionFeatures(BOARD_UART_RX_IOMUX,
                                                BOARD_UART_RX_IOMUX_FUNC,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_PULL_UP,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
}

}  // namespace

drivers::UartLinkStatus BoardUart_Init(void)
{
    ConfigureRxPullUp();
    return drivers::UartLink_Init(&g_uart_context, &g_uart_config);
}

drivers::UartLinkStatus BoardUart_Write(const uint8_t *data, uint8_t length)
{
    return drivers::UartLink_Write(&g_uart_context, data, length);
}

bool BoardUart_ReadByte(uint8_t *value)
{
    return drivers::UartLink_ReadByte(&g_uart_context, value);
}

uint16_t BoardUart_RxAvailable(void)
{
    return drivers::UartLink_RxAvailable(&g_uart_context);
}

uint32_t BoardUart_RxDroppedCount(void)
{
    return drivers::UartLink_RxDroppedCount(&g_uart_context);
}

void BoardUart_IrqHandler(void)
{
    drivers::UartLink_IrqHandler(&g_uart_context);
}

}  // namespace board

extern "C" void UART_CONTROL_INST_IRQHandler(void)
{
    board::BoardUart_IrqHandler();
}
