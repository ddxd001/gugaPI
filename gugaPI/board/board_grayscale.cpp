#include "board/board_grayscale.h"

#include "board/board_pins.h"
#include "drivers/grayscale/grayscale.h"

namespace board {
namespace {

static const drivers::GrayscaleConfig kGrayscaleConfig = {
    BOARD_GRAYSCALE_ADC_INST,
    BOARD_GRAYSCALE_ADC_MEM_IDX,
    BOARD_GRAYSCALE_ADC_RESULT_MASK,
    {
        { BOARD_GRAYSCALE_SEL0_PORT, BOARD_GRAYSCALE_SEL0_PIN },
        { BOARD_GRAYSCALE_SEL1_PORT, BOARD_GRAYSCALE_SEL1_PIN },
        { BOARD_GRAYSCALE_SEL2_PORT, BOARD_GRAYSCALE_SEL2_PIN },
    },
    BOARD_GRAYSCALE_SETTLE_CYCLES,
    BOARD_GRAYSCALE_ADC_TIMEOUT,
};

static drivers::GrayscaleContext g_grayscaleCtx = { &kGrayscaleConfig, false };

} /* namespace */

drivers::DriverStatus Board_GrayscaleInit(void)
{
    return drivers::Grayscale_Init(&g_grayscaleCtx, &kGrayscaleConfig);
}

bool Board_GrayscaleIsReady(void)
{
    return drivers::Grayscale_IsReady(&g_grayscaleCtx);
}

drivers::DriverStatus Board_GrayscaleReadChannel(uint8_t channel, uint16_t *raw)
{
    return drivers::Grayscale_ReadChannel(&g_grayscaleCtx, channel, raw);
}

drivers::DriverStatus Board_GrayscaleReadAll(uint16_t out[8])
{
    return drivers::Grayscale_ReadAll(&g_grayscaleCtx, out);
}

} /* namespace board */
