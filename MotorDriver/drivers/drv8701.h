#ifndef DRIVERS_DRV8701_H_
#define DRIVERS_DRV8701_H_

#include <stdint.h>

enum class MotorId : uint8_t {
    Motor1 = 0,
    Motor2 = 1,
};

enum class MotorDirection : uint8_t {
    Forward = 0,
    Reverse = 1,
};

void Drv8701_init(void);
void Drv8701_setCoast(MotorId motor);
void Drv8701_setBrake(MotorId motor);
void Drv8701_setRun(MotorId motor, MotorDirection direction, uint8_t dutyPercent);

#endif  // DRIVERS_DRV8701_H_
