#ifndef CONTROL_MOTOR_CONTROL_H_
#define CONTROL_MOTOR_CONTROL_H_

#include "protocol/motor_protocol.h"

void MotorControl_init(void);
void MotorControl_apply(const MotorRegisterMap& registers);
void MotorControl_stopAll(void);

#endif  // CONTROL_MOTOR_CONTROL_H_
