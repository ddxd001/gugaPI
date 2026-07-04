#include "board/board_motor_driver.h"

#include "board/board_pins.h"
#include "drivers/motor_driver/motor_driver_uart.h"

namespace board {
namespace {

void ConfigureMotorDriverRxPullUp(void)
{
    /* MotorDriver TX 未连接或高阻时，保持 MCU RX 为空闲高电平。 */
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_MOTOR_UART_IOMUX_RX,
        GPIO_MOTOR_UART_IOMUX_RX_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

} /* namespace */

static uint8_t g_motorDriverRxBuffer[BOARD_MOTOR_DRIVER_RX_BUFFER_SIZE];

/* gugaPI PA8/UART1_TX 接 MotorDriver RX，PA9/UART1_RX 接 MotorDriver TX。 */
static const drivers::MotorDriverUartConfig g_motorDriverConfig = {
    BOARD_MOTOR_DRIVER_UART_INST,
    BOARD_MOTOR_DRIVER_UART_IRQN,
    BOARD_MOTOR_DRIVER_BAUDRATE,
    BOARD_MOTOR_DRIVER_TX_TIMEOUT_ITERATIONS,
    g_motorDriverRxBuffer,
    BOARD_MOTOR_DRIVER_RX_BUFFER_SIZE
};

static drivers::MotorDriverUartContext g_motorDriverContext = {
    0,
    0U,
    0U,
    0U,
    false
};

drivers::DriverStatus Board_MotorDriverInit(void)
{
    ConfigureMotorDriverRxPullUp();
    return drivers::MotorDriverUart_Init(&g_motorDriverContext,
                                         &g_motorDriverConfig);
}

drivers::DriverStatus Board_MotorDriverWriteByte(uint8_t data)
{
    return drivers::MotorDriverUart_WriteByte(&g_motorDriverContext, data);
}

drivers::DriverStatus Board_MotorDriverWrite(const uint8_t *data,
                                             uint16_t length)
{
    return drivers::MotorDriverUart_Write(&g_motorDriverContext, data, length);
}

bool Board_MotorDriverReadByte(uint8_t *data)
{
    return drivers::MotorDriverUart_ReadByte(&g_motorDriverContext, data);
}

uint16_t Board_MotorDriverGetRxAvailable(void)
{
    return drivers::MotorDriverUart_GetRxAvailable(&g_motorDriverContext);
}

uint32_t Board_MotorDriverGetRxDroppedCount(void)
{
    return drivers::MotorDriverUart_GetRxDroppedCount(&g_motorDriverContext);
}

uint32_t Board_MotorDriverGetBaudrate(void)
{
    return drivers::MotorDriverUart_GetBaudrate(&g_motorDriverContext);
}

drivers::DriverStatus Board_MotorDriverGetControllerStatus(uint32_t *status)
{
    return drivers::MotorDriverUart_GetControllerStatus(&g_motorDriverContext,
                                                       status);
}

drivers::DriverStatus Board_MotorDriverClearRxBuffer(void)
{
    return drivers::MotorDriverUart_ClearRxBuffer(&g_motorDriverContext);
}

bool Board_MotorDriverIsReady(void)
{
    return drivers::MotorDriverUart_IsReady(&g_motorDriverContext);
}

void Board_MotorDriverIrqHandler(void)
{
    drivers::MotorDriverUart_IrqHandler(&g_motorDriverContext);
}

} /* namespace board */

extern "C" void MOTOR_UART_INST_IRQHandler(void)
{
    board::Board_MotorDriverIrqHandler();
}
