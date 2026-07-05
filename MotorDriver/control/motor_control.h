#ifndef CONTROL_MOTOR_CONTROL_H_
#define CONTROL_MOTOR_CONTROL_H_

#include "protocol/register_map.h"

namespace control {

void MotorControl_Init(void);
bool MotorControl_UpdateSpeed(protocol::RegisterMap &registers,
                              int16_t m1_rpm,
                              int16_t m2_rpm,
                              uint32_t now_ms);
void MotorControl_Apply(const protocol::RegisterMap &registers,
                        bool output_inhibited);
void MotorControl_StopAll(void);

}  // namespace control

#endif  // CONTROL_MOTOR_CONTROL_H_
