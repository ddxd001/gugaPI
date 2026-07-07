#ifndef BOARD_BOARD_ENCODERS_H_
#define BOARD_BOARD_ENCODERS_H_

#include <stdint.h>

#include "drivers/quadrature_encoder.h"

namespace board {

void BoardEncoders_Init(void);
void BoardEncoders_Process(uint32_t now_ms);
void BoardEncoders_SetEnabled(drivers::EncoderId encoder, bool enabled);
void BoardEncoders_Reset(drivers::EncoderId encoder);
drivers::EncoderSnapshot BoardEncoders_GetSnapshot(drivers::EncoderId encoder);
void BoardEncoders_GpioIrqHandler(void);

}  // namespace board

#endif  // BOARD_BOARD_ENCODERS_H_
