#ifndef DRIVERS_WHEEL_ENCODER_WHEEL_ENCODER_H_
#define DRIVERS_WHEEL_ENCODER_WHEEL_ENCODER_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

struct WheelEncoderConfig {
    GPTIMER_Regs *timer;
    GPIO_Regs *phase_a_port;
    uint32_t phase_a_pin;
    GPIO_Regs *phase_b_port;
    uint32_t phase_b_pin;
    int8_t count_sign;
    uint16_t sample_period_ms;
};

struct WheelEncoderSnapshot {
    int32_t count;
    int32_t counts_per_second;
    uint8_t state;
};

struct WheelEncoderContext {
    const WheelEncoderConfig *config;
    int64_t count;
    int32_t counts_per_second;
    int64_t last_sample_count;
    int64_t sample_delta[3];
    int64_t window_delta_count;
    uint32_t last_sample_ms;
    uint32_t sample_elapsed_ms[3];
    uint32_t window_elapsed_ms;
    uint16_t last_hw_count;
    uint8_t window_index;
    uint8_t window_count;
    uint8_t state;
    bool sample_initialized;
    bool initialized;
};

DriverStatus WheelEncoder_Init(WheelEncoderContext *ctx,
                               const WheelEncoderConfig *config,
                               uint32_t now_ms);
DriverStatus WheelEncoder_Process(WheelEncoderContext *ctx,
                                  uint32_t now_ms);
DriverStatus WheelEncoder_Reset(WheelEncoderContext *ctx, uint32_t now_ms);
DriverStatus WheelEncoder_GetSnapshot(WheelEncoderContext *ctx,
                                      WheelEncoderSnapshot *snapshot);
bool WheelEncoder_IsReady(const WheelEncoderContext *ctx);

} /* namespace drivers */

#endif /* DRIVERS_WHEEL_ENCODER_WHEEL_ENCODER_H_ */
