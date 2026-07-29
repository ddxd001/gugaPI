#include "board/board_infrared_sensor.h"

#include "board/board_pins.h"
#include "drivers/infrared_line/infrared_line_uart.h"
#include "services/dma_fault.h"

namespace board {
namespace {

alignas(4) uint8_t g_dmaRxBuffer[BOARD_INFRARED_SENSOR_DMA_BUFFER_SIZE];

const drivers::InfraredLineUartConfig g_config = {
    BOARD_INFRARED_SENSOR_UART_INST,
    BOARD_INFRARED_SENSOR_UART_IRQN,
    BOARD_INFRARED_SENSOR_DMA_INST,
    BOARD_INFRARED_SENSOR_DMA_CHANNEL,
    g_dmaRxBuffer,
    BOARD_INFRARED_SENSOR_DMA_BUFFER_SIZE
};

drivers::InfraredLineUartContext g_context = {};

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
    const drivers::DriverStatus status =
        drivers::InfraredLineUart_Init(&g_context, &g_config);
    if ((status == drivers::DRIVER_OK) &&
        (!services::DmaFault_RegisterHandler(
            Board_InfraredSensorHandleDmaFault))) {
        return drivers::DRIVER_ERROR;
    }
    return status;
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
    return g_context.cursor.dropped_bytes;
}

uint32_t Board_InfraredSensorGetUartErrorCount(void)
{
    return g_context.uart_error_count;
}

uint32_t Board_InfraredSensorGetRxTimeoutCount(void)
{
    return g_context.rx_timeout_count;
}

uint32_t Board_InfraredSensorGetOverrunErrorCount(void)
{
    return g_context.overrun_error_count;
}

uint32_t Board_InfraredSensorGetFramingErrorCount(void)
{
    return g_context.framing_error_count;
}

uint32_t Board_InfraredSensorGetParityErrorCount(void)
{
    return g_context.parity_error_count;
}

uint32_t Board_InfraredSensorGetNoiseErrorCount(void)
{
    return g_context.noise_error_count;
}

uint32_t Board_InfraredSensorGetIrqCount(void)
{
    return g_context.irq_count;
}

uint32_t Board_InfraredSensorGetFifoByteCount(void)
{
    return Board_InfraredSensorGetDmaByteCount();
}

uint32_t Board_InfraredSensorGetPolledByteCount(void)
{
    return g_context.polled_byte_count;
}

uint32_t Board_InfraredSensorGetDmaBlockCount(void)
{
    return g_context.dma_wrap_count;
}

uint32_t Board_InfraredSensorGetDmaByteCount(void)
{
    const uint64_t produced = g_context.cursor.producer -
        g_context.stats_base_producer;
    return (produced > UINT32_MAX) ? UINT32_MAX :
        static_cast<uint32_t>(produced);
}

uint32_t Board_InfraredSensorGetDmaOverwriteCount(void)
{
    return g_context.cursor.overwrite_count;
}

uint32_t Board_InfraredSensorGetDmaFaultCount(void)
{
    return g_context.dma_fault_count;
}

uint32_t Board_InfraredSensorGetDmaProducedCount(void)
{
    return static_cast<uint32_t>(
        drivers::InfraredLineUart_GetProducedCount(&g_context));
}

uint32_t Board_InfraredSensorGetDmaConsumedCount(void)
{
    return static_cast<uint32_t>(
        drivers::InfraredLineUart_GetConsumedCount(&g_context));
}

uint32_t Board_InfraredSensorGetDmaCurrentLag(void)
{
    return drivers::InfraredLineUart_GetCurrentLag(&g_context);
}

uint32_t Board_InfraredSensorGetDmaMaximumLag(void)
{
    return g_context.cursor.maximum_lag;
}

uint16_t Board_InfraredSensorGetDmaRemaining(void)
{
    return drivers::InfraredLineUart_GetDmaRemaining(&g_context);
}

uint16_t Board_InfraredSensorGetDmaBufferSize(void)
{
    return BOARD_INFRARED_SENSOR_DMA_BUFFER_SIZE;
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

bool Board_InfraredSensorIsDmaEnabled(void)
{
    return DL_DMA_isChannelEnabled(BOARD_INFRARED_SENSOR_DMA_INST,
                                   BOARD_INFRARED_SENSOR_DMA_CHANNEL);
}

void Board_InfraredSensorClear(void)
{
    drivers::InfraredLineUart_ClearStats(&g_context);
}

void Board_InfraredSensorIrqHandler(void)
{
    drivers::InfraredLineUart_IrqHandler(&g_context);
}

void Board_InfraredSensorHandleDmaFault(void)
{
    drivers::InfraredLineUart_HandleDmaFault(&g_context);
}

} /* namespace board */

extern "C" void INFRARED_UART_INST_IRQHandler(void)
{
    board::Board_InfraredSensorIrqHandler();
}
