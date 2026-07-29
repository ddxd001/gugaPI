#include "board/board_infrared_sensor.h"

#include "board/board_pins.h"
#include "drivers/infrared_line/infrared_line_uart.h"

namespace board {
namespace {

uint8_t g_rxBuffer[BOARD_INFRARED_SENSOR_RX_BUFFER_SIZE];

const drivers::InfraredLineUartConfig g_config = {
    BOARD_INFRARED_SENSOR_UART_INST,
    BOARD_INFRARED_SENSOR_UART_IRQN,
    g_rxBuffer,
    BOARD_INFRARED_SENSOR_RX_BUFFER_SIZE
};

drivers::InfraredLineUartContext g_context = {
    0,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    false
};

void ConfigureRxPullUp(void)
{
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_INFRARED_UART_IOMUX_RX,
        GPIO_INFRARED_UART_IOMUX_RX_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

} /* namespace */

drivers::DriverStatus Board_InfraredSensorInit(void)
{
    ConfigureRxPullUp();
    return drivers::InfraredLineUart_Init(&g_context, &g_config);
}

bool Board_InfraredSensorReadByte(uint8_t *data)
{
    return drivers::InfraredLineUart_ReadByte(&g_context, data);
}

void Board_InfraredSensorServiceRx(void)
{
    drivers::InfraredLineUart_ServiceRx(&g_context);
}

bool Board_InfraredSensorIsReady(void)
{
    return g_context.initialized;
}

uint32_t Board_InfraredSensorGetDroppedCount(void)
{
    return g_context.rx_dropped_count;
}

uint32_t Board_InfraredSensorGetUartErrorCount(void)
{
    return g_context.uart_error_count;
}

uint32_t Board_InfraredSensorGetIrqCount(void)
{
    return g_context.irq_count;
}

uint32_t Board_InfraredSensorGetFifoByteCount(void)
{
    return g_context.fifo_byte_count;
}

uint32_t Board_InfraredSensorGetPolledByteCount(void)
{
    return g_context.polled_byte_count;
}

bool Board_InfraredSensorIsPowered(void)
{
    return DL_UART_Main_isPowerEnabled(BOARD_INFRARED_SENSOR_UART_INST);
}

bool Board_InfraredSensorIsUartEnabled(void)
{
    return DL_UART_Main_isEnabled(BOARD_INFRARED_SENSOR_UART_INST);
}

bool Board_InfraredSensorIsRxPinHigh(void)
{
    return (DL_GPIO_readPins(GPIO_INFRARED_UART_RX_PORT,
                             GPIO_INFRARED_UART_RX_PIN) != 0U);
}

bool Board_InfraredSensorIsRxFifoEmpty(void)
{
    return DL_UART_Main_isRXFIFOEmpty(BOARD_INFRARED_SENSOR_UART_INST);
}

void Board_InfraredSensorClear(void)
{
    drivers::InfraredLineUart_Clear(&g_context);
}

void Board_InfraredSensorIrqHandler(void)
{
    drivers::InfraredLineUart_IrqHandler(&g_context);
}

} /* namespace board */

extern "C" void INFRARED_UART_INST_IRQHandler(void)
{
    board::Board_InfraredSensorIrqHandler();
}
