#ifndef DRIVERS_QUADRATURE_ENCODER_H_
#define DRIVERS_QUADRATURE_ENCODER_H_

#include <stdint.h>

namespace drivers {

enum class EncoderId : uint8_t {
    Encoder1 = 0U,
    Encoder2 = 1U,
};

struct EncoderSnapshot {
    int32_t count;
    int32_t counts_per_second;
    uint8_t state;
};

void QuadratureEncoder_Init(void);
void QuadratureEncoder_Process(uint32_t now_ms);
void QuadratureEncoder_SetEnabled(EncoderId encoder, bool enabled);
void QuadratureEncoder_HandleGpioInterrupt(void);
void QuadratureEncoder_Reset(EncoderId encoder);
EncoderSnapshot QuadratureEncoder_GetSnapshot(EncoderId encoder);

}  // namespace drivers

#endif  // DRIVERS_QUADRATURE_ENCODER_H_
