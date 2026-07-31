#include "board/board_ball_vision.h"

#include "board/board_pins.h"
#include "drivers/ball_vision/ball_vision_uart.h"

namespace board {
namespace {

uint8_t g_rxBuffer[BOARD_BALL_VISION_RX_BUFFER_SIZE];

const drivers::BallVisionUartConfig g_config = {
    BOARD_BALL_VISION_UART_INST,
    BOARD_BALL_VISION_UART_IRQN,
    BOARD_BALL_VISION_TX_TIMEOUT_ITERATIONS,
    g_rxBuffer,
    BOARD_BALL_VISION_RX_BUFFER_SIZE
};

drivers::BallVisionUartContext g_context = {};

void ConfigureRxPullUp(void)
{
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_BALL_VISION_UART_IOMUX_RX,
        GPIO_BALL_VISION_UART_IOMUX_RX_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

} /* namespace */

drivers::DriverStatus Board_BallVisionInit(void)
{
    ConfigureRxPullUp();
    return drivers::BallVisionUart_Init(&g_context, &g_config);
}

drivers::DriverStatus Board_BallVisionWrite(const uint8_t *data,
                                            uint16_t length)
{
    return drivers::BallVisionUart_Write(&g_context, data, length);
}

bool Board_BallVisionReadByte(uint8_t *data)
{
    return drivers::BallVisionUart_ReadByte(&g_context, data);
}

void Board_BallVisionClear(void)
{
    drivers::BallVisionUart_Clear(&g_context);
}

bool Board_BallVisionIsReady(void)
{
    return g_context.initialized;
}

uint32_t Board_BallVisionGetDroppedBytes(void)
{
    return g_context.dropped_bytes;
}

uint32_t Board_BallVisionGetUartErrors(void)
{
    return g_context.uart_errors;
}

uint32_t Board_BallVisionGetIrqCount(void)
{
    return g_context.irq_count;
}

void Board_BallVisionIrqHandler(void)
{
    drivers::BallVisionUart_IrqHandler(&g_context);
}

} /* namespace board */

extern "C" void BALL_VISION_UART_INST_IRQHandler(void)
{
    board::Board_BallVisionIrqHandler();
}
