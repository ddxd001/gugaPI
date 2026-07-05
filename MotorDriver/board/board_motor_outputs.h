#ifndef BOARD_BOARD_MOTOR_OUTPUTS_H_
#define BOARD_BOARD_MOTOR_OUTPUTS_H_

#include <stdint.h>

#include "drivers/drv8701.h"

namespace board {

void BoardMotorOutputs_Init(void);
void BoardMotorOutputs_Coast(drivers::MotorId motor);
void BoardMotorOutputs_Brake(drivers::MotorId motor);
void BoardMotorOutputs_Run(drivers::MotorId motor,
                           drivers::MotorDirection direction,
                           uint8_t duty_percent);

}  // namespace board

#endif  // BOARD_BOARD_MOTOR_OUTPUTS_H_
