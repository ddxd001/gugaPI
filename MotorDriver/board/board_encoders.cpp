#include "board/board_encoders.h"

namespace board {

void BoardEncoders_Init(void)
{
    drivers::QuadratureEncoder_Init();
}

void BoardEncoders_Process(uint32_t now_ms)
{
    drivers::QuadratureEncoder_Process(now_ms);
}

void BoardEncoders_SetEnabled(drivers::EncoderId encoder, bool enabled)
{
    drivers::QuadratureEncoder_SetEnabled(encoder, enabled);
}

void BoardEncoders_Reset(drivers::EncoderId encoder)
{
    drivers::QuadratureEncoder_Reset(encoder);
}

drivers::EncoderSnapshot BoardEncoders_GetSnapshot(drivers::EncoderId encoder)
{
    return drivers::QuadratureEncoder_GetSnapshot(encoder);
}

void BoardEncoders_GpioIrqHandler(void)
{
    drivers::QuadratureEncoder_HandleGpioInterrupt();
}

}  // namespace board

extern "C" void GROUP1_IRQHandler(void)
{
    board::BoardEncoders_GpioIrqHandler();
}
