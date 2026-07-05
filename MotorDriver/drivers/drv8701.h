#ifndef DRIVERS_DRV8701_H_
#define DRIVERS_DRV8701_H_

#include <stdint.h>

namespace drivers {

enum class MotorId : uint8_t {
    Motor1 = 0U,
    Motor2 = 1U,
};

enum class MotorDirection : uint8_t {
    Forward = 0U,
    Reverse = 1U,
};

void Drv8701_Init(void);
void Drv8701_SetCoast(MotorId motor);
void Drv8701_SetBrake(MotorId motor);
void Drv8701_SetRun(MotorId motor, MotorDirection direction, uint8_t duty_percent);

}  // namespace drivers

#endif  // DRIVERS_DRV8701_H_
