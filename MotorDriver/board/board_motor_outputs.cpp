#include "board/board_motor_outputs.h"

namespace board {

void BoardMotorOutputs_Init(void)
{
    drivers::Drv8701_Init();
}

void BoardMotorOutputs_Coast(drivers::MotorId motor)
{
    drivers::Drv8701_SetCoast(motor);
}

void BoardMotorOutputs_Brake(drivers::MotorId motor)
{
    drivers::Drv8701_SetBrake(motor);
}

void BoardMotorOutputs_Run(drivers::MotorId motor,
                           drivers::MotorDirection direction,
                           uint8_t duty_percent)
{
    drivers::Drv8701_SetRun(motor, direction, duty_percent);
}

}  // namespace board
