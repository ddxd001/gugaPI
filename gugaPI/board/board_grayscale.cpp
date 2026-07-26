#include "board/board_grayscale.h"

#include "board/board_pins.h"
#include "drivers/grayscale/grayscale.h"

namespace board {
namespace {

static const drivers::GrayscaleConfig kGrayscaleConfig = {
    BOARD_GRAYSCALE_ADC_INST,
    BOARD_GRAYSCALE_ADC_IRQN,
    BOARD_GRAYSCALE_ADC_MEM_IDX,
    BOARD_GRAYSCALE_ADC_RESULT_MASK,
    {
        { BOARD_GRAYSCALE_SEL0_PORT, BOARD_GRAYSCALE_SEL0_PIN },
        { BOARD_GRAYSCALE_SEL1_PORT, BOARD_GRAYSCALE_SEL1_PIN },
        { BOARD_GRAYSCALE_SEL2_PORT, BOARD_GRAYSCALE_SEL2_PIN },
    },
    BOARD_GRAYSCALE_SETTLE_US,
    BOARD_GRAYSCALE_ADC_TIMEOUT_US,
    BOARD_GRAYSCALE_ADC_TIMEOUT,
};

static drivers::GrayscaleContext g_grayscaleCtx = {};

} /* namespace */

drivers::DriverStatus Board_GrayscaleInit(void)
{
    const drivers::DriverStatus status =
        drivers::Grayscale_Init(&g_grayscaleCtx, &kGrayscaleConfig);
    if (status == drivers::DRIVER_OK) {
        NVIC_ClearPendingIRQ(BOARD_GRAYSCALE_ADC_IRQN);
        NVIC_EnableIRQ(BOARD_GRAYSCALE_ADC_IRQN);
    }
    return status;
}

bool Board_GrayscaleIsReady(void)
{
    return drivers::Grayscale_IsReady(&g_grayscaleCtx);
}

drivers::DriverStatus Board_GrayscalePrepareChannel(uint8_t channel)
{
    return drivers::Grayscale_PrepareChannel(&g_grayscaleCtx, channel);
}

drivers::DriverStatus Board_GrayscaleReadPreparedChannel(uint8_t channel,
                                                         uint16_t *raw)
{
    return drivers::Grayscale_ReadPreparedChannel(&g_grayscaleCtx,
                                                  channel,
                                                  raw);
}

drivers::DriverStatus Board_GrayscaleStartPreparedConversion(uint8_t channel)
{
    return drivers::Grayscale_StartPreparedConversion(&g_grayscaleCtx,
                                                      channel);
}

drivers::DriverStatus Board_GrayscaleStartPreparedConversion(
    uint8_t channel,
    uint8_t next_channel)
{
    return drivers::Grayscale_StartPreparedConversion(&g_grayscaleCtx,
                                                      channel,
                                                      next_channel);
}

drivers::DriverStatus Board_GrayscaleTakeCompletedConversion(uint8_t *channel,
                                                             uint16_t *raw)
{
    return drivers::Grayscale_TakeCompletedConversion(&g_grayscaleCtx,
                                                      channel,
                                                      raw);
}

drivers::DriverStatus Board_GrayscaleReadChannel(uint8_t channel, uint16_t *raw)
{
    return drivers::Grayscale_ReadChannel(&g_grayscaleCtx, channel, raw);
}

drivers::DriverStatus Board_GrayscaleReadAll(uint16_t out[8])
{
    return drivers::Grayscale_ReadAll(&g_grayscaleCtx, out);
}

void Board_GrayscaleHandleInterrupt(void)
{
    drivers::Grayscale_HandleInterrupt(&g_grayscaleCtx);
}

} /* namespace board */

extern "C" void GRAYSCALE_ADC_INST_IRQHandler(void)
{
    board::Board_GrayscaleHandleInterrupt();
}
