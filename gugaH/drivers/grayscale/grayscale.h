#ifndef DRIVERS_GRAYSCALE_GRAYSCALE_H_
#define DRIVERS_GRAYSCALE_GRAYSCALE_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/grayscale/grayscale_processing.h"
#include "ti_msp_dl_config.h"

namespace drivers {

struct GrayscaleSelPin {
    GPIO_Regs *port;
    uint32_t pin;
};

struct GrayscaleConfig {
    ADC12_Regs *adc;
    IRQn_Type irq;
    DL_ADC12_MEM_IDX mem_idx;        /* memory result slot (e.g. MEM_IDX_0) */
    uint32_t result_loaded_mask;     /* DL_ADC12_INTERRUPT_MEMx_RESULT_LOADED */
    GrayscaleSelPin sel[3];          /* sel[0]=bit0, sel[1]=bit1, sel[2]=bit2 */
    uint32_t settle_us;              /* delay after mux select, in microseconds */
    uint32_t conversion_timeout_us;  /* asynchronous completion deadline */
    uint32_t timeout_iterations;     /* per-conversion poll budget */
};

struct GrayscaleContext {
    const GrayscaleConfig *config;
    bool initialized;
    volatile uint8_t selected_channel;
    volatile uint32_t selected_at_us;
    volatile bool selected_valid;
    volatile bool conversion_active;
    volatile uint8_t conversion_channel;
    volatile uint8_t conversion_next_channel;
    volatile uint32_t conversion_started_us;
    volatile bool result_ready;
    volatile uint8_t result_channel;
    volatile uint16_t result_raw;
};

DriverStatus Grayscale_Init(GrayscaleContext *ctx,
                            const GrayscaleConfig *config);
bool Grayscale_IsReady(const GrayscaleContext *ctx);
DriverStatus Grayscale_PrepareChannel(GrayscaleContext *ctx, uint8_t channel);
DriverStatus Grayscale_ReadPreparedChannel(GrayscaleContext *ctx,
                                           uint8_t channel,
                                           uint16_t *raw);
DriverStatus Grayscale_StartPreparedConversion(GrayscaleContext *ctx,
                                               uint8_t channel);
DriverStatus Grayscale_StartPreparedConversion(GrayscaleContext *ctx,
                                               uint8_t channel,
                                               uint8_t next_channel);
DriverStatus Grayscale_TakeCompletedConversion(GrayscaleContext *ctx,
                                               uint8_t *channel,
                                               uint16_t *raw);
void Grayscale_HandleInterrupt(GrayscaleContext *ctx);
DriverStatus Grayscale_ReadChannel(GrayscaleContext *ctx,
                                   uint8_t channel,
                                   uint16_t *raw);
DriverStatus Grayscale_ReadAll(GrayscaleContext *ctx, uint16_t out[8]);

} /* namespace drivers */

#endif /* DRIVERS_GRAYSCALE_GRAYSCALE_H_ */
