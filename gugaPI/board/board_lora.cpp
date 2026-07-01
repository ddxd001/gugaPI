#include "board/board_lora.h"

#include "board/board_pins.h"
#include "drivers/lora/lora.h"

namespace board {
namespace {

void ConfigureLoraRxPullUp(void)
{
    /* LoRa TX 断开或模块高阻时，保持 MCU UART RX 为空闲高电平。 */
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_LORA_UART_IOMUX_RX,
                                                GPIO_LORA_UART_IOMUX_RX_FUNC,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_PULL_UP,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
}

} /* namespace */

static uint8_t g_loraRxBuffer[BOARD_LORA_RX_BUFFER_SIZE];

/* LoRa 接线：20 脚 PB0/UART0_TX 接模块 RX，21 脚 PB1/UART0_RX 接模块 TX。 */
static const drivers::LoraUartConfig g_loraConfig = {
    BOARD_LORA_UART_INST,
    BOARD_LORA_UART_IRQN,
    BOARD_LORA_BAUDRATE,
    BOARD_LORA_TX_TIMEOUT_ITERATIONS,
    g_loraRxBuffer,
    BOARD_LORA_RX_BUFFER_SIZE
};

static drivers::LoraUartContext g_loraContext = {
    0,
    0U,
    0U,
    0U,
    false
};

drivers::DriverStatus Board_LoraInit(void)
{
    ConfigureLoraRxPullUp();
    return drivers::LoraUart_Init(&g_loraContext, &g_loraConfig);
}

drivers::DriverStatus Board_LoraWriteByte(uint8_t data)
{
    return drivers::LoraUart_WriteByte(&g_loraContext, data);
}

drivers::DriverStatus Board_LoraWrite(const uint8_t *data, uint16_t length)
{
    return drivers::LoraUart_Write(&g_loraContext, data, length);
}

bool Board_LoraReadByte(uint8_t *data)
{
    return drivers::LoraUart_ReadByte(&g_loraContext, data);
}

uint16_t Board_LoraGetRxAvailable(void)
{
    return drivers::LoraUart_GetRxAvailable(&g_loraContext);
}

uint32_t Board_LoraGetRxDroppedCount(void)
{
    return drivers::LoraUart_GetRxDroppedCount(&g_loraContext);
}

uint32_t Board_LoraGetBaudrate(void)
{
    return drivers::LoraUart_GetBaudrate(&g_loraContext);
}

drivers::DriverStatus Board_LoraGetControllerStatus(uint32_t *status)
{
    return drivers::LoraUart_GetControllerStatus(&g_loraContext, status);
}

drivers::DriverStatus Board_LoraClearRxBuffer(void)
{
    return drivers::LoraUart_ClearRxBuffer(&g_loraContext);
}

drivers::DriverStatus Board_LoraGetLineStatus(bool *tx_high, bool *rx_high)
{
    if ((tx_high == 0) || (rx_high == 0)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    *tx_high = (DL_GPIO_readPins(BOARD_LORA_TX_PORT,
                                 BOARD_LORA_TX_PIN) != 0U);
    *rx_high = (DL_GPIO_readPins(BOARD_LORA_RX_PORT,
                                 BOARD_LORA_RX_PIN) != 0U);

    return drivers::DRIVER_OK;
}

bool Board_LoraIsReady(void)
{
    return drivers::LoraUart_IsReady(&g_loraContext);
}

void Board_LoraIrqHandler(void)
{
    drivers::LoraUart_IrqHandler(&g_loraContext);
}

} /* namespace board */

extern "C" void LORA_UART_INST_IRQHandler(void)
{
    board::Board_LoraIrqHandler();
}