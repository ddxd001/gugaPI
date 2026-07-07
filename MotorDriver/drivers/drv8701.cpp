#include "drivers/drv8701.h"

#include "board/board_motor_pins.h"

namespace drivers {
namespace {

uint32_t DutyToCompare(uint8_t duty_percent)
{
    if (duty_percent > 100U) {
        duty_percent = 100U;
    }

    return board::kMotorPwmPeriodCounts -
           ((board::kMotorPwmPeriodCounts * duty_percent) / 100U);
}

void ForceEnableLow(MotorId motor)
{
    if (motor == MotorId::Motor1) {
        DL_GPIO_initDigitalOutput(BOARD_M1_PWM_IOMUX);
        DL_GPIO_clearPins(BOARD_M1_PWM_PORT, BOARD_M1_PWM_PIN);
        DL_GPIO_enableOutput(BOARD_M1_PWM_PORT, BOARD_M1_PWM_PIN);
    } else {
        DL_GPIO_initDigitalOutput(BOARD_M2_PWM_IOMUX);
        DL_GPIO_clearPins(BOARD_M2_PWM_PORT, BOARD_M2_PWM_PIN);
        DL_GPIO_enableOutput(BOARD_M2_PWM_PORT, BOARD_M2_PWM_PIN);
    }
}

void ForceEnableHigh(MotorId motor)
{
    if (motor == MotorId::Motor1) {
        DL_GPIO_initDigitalOutput(BOARD_M1_PWM_IOMUX);
        DL_GPIO_setPins(BOARD_M1_PWM_PORT, BOARD_M1_PWM_PIN);
        DL_GPIO_enableOutput(BOARD_M1_PWM_PORT, BOARD_M1_PWM_PIN);
    } else {
        DL_GPIO_initDigitalOutput(BOARD_M2_PWM_IOMUX);
        DL_GPIO_setPins(BOARD_M2_PWM_PORT, BOARD_M2_PWM_PIN);
        DL_GPIO_enableOutput(BOARD_M2_PWM_PORT, BOARD_M2_PWM_PIN);
    }
}

void ConfigureEnablePwm(MotorId motor)
{
    if (motor == MotorId::Motor1) {
        DL_GPIO_initPeripheralOutputFunction(BOARD_M1_PWM_IOMUX,
                                             BOARD_M1_PWM_IOMUX_FUNC);
        DL_GPIO_enableOutput(BOARD_M1_PWM_PORT, BOARD_M1_PWM_PIN);
    } else {
        DL_GPIO_initPeripheralOutputFunction(BOARD_M2_PWM_IOMUX,
                                             BOARD_M2_PWM_IOMUX_FUNC);
        DL_GPIO_enableOutput(BOARD_M2_PWM_PORT, BOARD_M2_PWM_PIN);
    }
}

void SetPhase(MotorId motor, MotorDirection direction)
{
    const bool reverse = (direction == MotorDirection::Reverse);

    if (motor == MotorId::Motor1) {
        if (reverse) {
            DL_GPIO_setPins(BOARD_M1_PH_PORT, BOARD_M1_PH_PIN);
        } else {
            DL_GPIO_clearPins(BOARD_M1_PH_PORT, BOARD_M1_PH_PIN);
        }
    } else {
        if (reverse) {
            DL_GPIO_setPins(BOARD_M2_PH_PORT, BOARD_M2_PH_PIN);
        } else {
            DL_GPIO_clearPins(BOARD_M2_PH_PORT, BOARD_M2_PH_PIN);
        }
    }
}

void SetEnableDuty(MotorId motor, uint8_t duty_percent)
{
    if (duty_percent == 0U) {
        ForceEnableLow(motor);
        return;
    }

    if (duty_percent >= 100U) {
        ForceEnableHigh(motor);
        return;
    }

    const uint32_t compare = DutyToCompare(duty_percent);

    if (motor == MotorId::Motor1) {
        DL_TimerG_setCaptureCompareValue(BOARD_M1_PWM_INST,
                                         compare,
                                         BOARD_M1_PWM_INDEX);
    } else {
        DL_TimerG_setCaptureCompareValue(BOARD_M2_PWM_INST,
                                         compare,
                                         BOARD_M2_PWM_INDEX);
    }

    ConfigureEnablePwm(motor);
}

}  // namespace

void Drv8701_Init(void)
{
    DL_TimerG_setCaptureCompareValue(BOARD_M1_PWM_INST,
                                     board::kMotorPwmPeriodCounts,
                                     BOARD_M1_PWM_INDEX);
    DL_TimerG_setCaptureCompareValue(BOARD_M2_PWM_INST,
                                     board::kMotorPwmPeriodCounts,
                                     BOARD_M2_PWM_INDEX);

    DL_TimerG_startCounter(BOARD_M1_PWM_INST);
    DL_TimerG_startCounter(BOARD_M2_PWM_INST);

    Drv8701_SetCoast(MotorId::Motor1);
    Drv8701_SetCoast(MotorId::Motor2);
}

void Drv8701_SetCoast(MotorId motor)
{
    SetEnableDuty(motor, 0U);
}

void Drv8701_SetBrake(MotorId motor)
{
    SetEnableDuty(motor, 0U);
}

void Drv8701_SetRun(MotorId motor, MotorDirection direction, uint8_t duty_percent)
{
    ForceEnableLow(motor);
    SetPhase(motor, direction);
    SetEnableDuty(motor, duty_percent);
}

}  // namespace drivers
