#ifndef DRIVERS_GRAYSCALE_GRAYSCALE_H_
#define DRIVERS_GRAYSCALE_GRAYSCALE_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

static const uint8_t GRAYSCALE_CHANNEL_COUNT = 8U;

struct GrayscaleSelPin {
    GPIO_Regs *port;
    uint32_t pin;
};

struct GrayscaleConfig {
    ADC12_Regs *adc;
    DL_ADC12_MEM_IDX mem_idx;        /* memory result slot (e.g. MEM_IDX_0) */
    uint32_t result_loaded_mask;     /* DL_ADC12_INTERRUPT_MEMx_RESULT_LOADED */
    GrayscaleSelPin sel[3];          /* sel[0]=bit0, sel[1]=bit1, sel[2]=bit2 */
    uint32_t settle_cycles;          /* busy-wait after switching select */
    uint32_t timeout_iterations;     /* per-conversion poll budget */
};

struct GrayscaleContext {
    const GrayscaleConfig *config;
    bool initialized;
};

DriverStatus Grayscale_Init(GrayscaleContext *ctx,
                            const GrayscaleConfig *config);
bool Grayscale_IsReady(const GrayscaleContext *ctx);
DriverStatus Grayscale_ReadChannel(GrayscaleContext *ctx,
                                   uint8_t channel,
                                   uint16_t *raw);
DriverStatus Grayscale_ReadAll(GrayscaleContext *ctx, uint16_t out[8]);

} /* namespace drivers */

#endif /* DRIVERS_GRAYSCALE_GRAYSCALE_H_ */
