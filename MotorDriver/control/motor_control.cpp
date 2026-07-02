#include "control/motor_control.h"

#include "drivers/drv8701.h"
#include "protocol/motor_registers.h"

namespace {

MotorDirection directionFromRegister(uint8_t value)
{
    return value == MotorProtocol::MOTOR_DIRECTION_REVERSE ? MotorDirection::Reverse
                                                           : MotorDirection::Forward;
}

void applyMotor(MotorId motor, uint8_t mode, uint8_t duty, uint8_t direction)
{
    switch (mode) {
        case MotorProtocol::MOTOR_MODE_RUN:
            Drv8701_setRun(motor, directionFromRegister(direction), duty);
            break;
        case MotorProtocol::MOTOR_MODE_BRAKE:
            Drv8701_setBrake(motor);
            break;
        case MotorProtocol::MOTOR_MODE_COAST:
        default:
            Drv8701_setCoast(motor);
            break;
    }
}

}  // namespace

void MotorControl_init(void)
{
    Drv8701_init();
}

void MotorControl_apply(const MotorRegisterMap& registers)
{
    if ((registers.read(MotorProtocol::REG_CONTROL_FLAGS) & MotorProtocol::CONTROL_ENABLE) == 0U) {
        MotorControl_stopAll();
        return;
    }

    applyMotor(MotorId::Motor1,
        registers.read(MotorProtocol::REG_M1_MODE),
        registers.read(MotorProtocol::REG_M1_DUTY),
        registers.read(MotorProtocol::REG_M1_DIRECTION));

    applyMotor(MotorId::Motor2,
        registers.read(MotorProtocol::REG_M2_MODE),
        registers.read(MotorProtocol::REG_M2_DUTY),
        registers.read(MotorProtocol::REG_M2_DIRECTION));
}

void MotorControl_stopAll(void)
{
    Drv8701_setCoast(MotorId::Motor1);
    Drv8701_setCoast(MotorId::Motor2);
}
