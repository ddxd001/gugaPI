#include "drivers/drv8701.h"

#include "ti_msp_dl_config.h"

namespace {

#if defined(PWM_M1_EN_INST)
constexpr uint32_t kPwmPeriodCounts = 1600U;
uint32_t dutyToCompareValue(uint8_t dutyPercent)
{
    if (dutyPercent > 100U) {
        dutyPercent = 100U;
    }

    return kPwmPeriodCounts - ((kPwmPeriodCounts * dutyPercent) / 100U);
}
#endif

void forceEnableLow(MotorId motor)
{
#if defined(PWM_M1_EN_INST)
    if (motor == MotorId::Motor1) {
        DL_GPIO_initDigitalOutput(GPIO_PWM_M1_EN_C1_IOMUX);
        DL_GPIO_clearPins(GPIO_PWM_M1_EN_C1_PORT, GPIO_PWM_M1_EN_C1_PIN);
        DL_GPIO_enableOutput(GPIO_PWM_M1_EN_C1_PORT, GPIO_PWM_M1_EN_C1_PIN);
    } else {
        DL_GPIO_initDigitalOutput(GPIO_PWM_M2_EN_C1_IOMUX);
        DL_GPIO_clearPins(GPIO_PWM_M2_EN_C1_PORT, GPIO_PWM_M2_EN_C1_PIN);
        DL_GPIO_enableOutput(GPIO_PWM_M2_EN_C1_PORT, GPIO_PWM_M2_EN_C1_PIN);
    }
#else
    if (motor == MotorId::Motor1) {
        DL_GPIO_clearPins(GPIO_M1_EN_PORT, GPIO_M1_EN_M1_EN_PIN_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_M2_EN_PORT, GPIO_M2_EN_M2_EN_PIN_PIN);
    }
#endif
}

void configureEnablePwm(MotorId motor)
{
#if defined(PWM_M1_EN_INST)
    if (motor == MotorId::Motor1) {
        DL_GPIO_initPeripheralOutputFunction(
            GPIO_PWM_M1_EN_C1_IOMUX, GPIO_PWM_M1_EN_C1_IOMUX_FUNC);
        DL_GPIO_enableOutput(GPIO_PWM_M1_EN_C1_PORT, GPIO_PWM_M1_EN_C1_PIN);
    } else {
        DL_GPIO_initPeripheralOutputFunction(
            GPIO_PWM_M2_EN_C1_IOMUX, GPIO_PWM_M2_EN_C1_IOMUX_FUNC);
        DL_GPIO_enableOutput(GPIO_PWM_M2_EN_C1_PORT, GPIO_PWM_M2_EN_C1_PIN);
    }
#else
    (void) motor;
#endif
}

void setPhase(MotorId motor, MotorDirection direction)
{
    const bool reverse = (direction == MotorDirection::Reverse);

    if (motor == MotorId::Motor1) {
        if (reverse) {
            DL_GPIO_setPins(GPIO_M1_PH_PORT, GPIO_M1_PH_M1_PH_PIN_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_M1_PH_PORT, GPIO_M1_PH_M1_PH_PIN_PIN);
        }
    } else {
        if (reverse) {
            DL_GPIO_setPins(GPIO_M2_PH_PORT, GPIO_M2_PH_M2_PH_PIN_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_M2_PH_PORT, GPIO_M2_PH_M2_PH_PIN_PIN);
        }
    }
}

void setEnableDuty(MotorId motor, uint8_t dutyPercent)
{
#if defined(PWM_M1_EN_INST)
    if (dutyPercent == 0U) {
        forceEnableLow(motor);
        return;
    }

    const uint32_t compareValue = dutyToCompareValue(dutyPercent);

    if (motor == MotorId::Motor1) {
        DL_Timer_setCaptureCompareValue(PWM_M1_EN_INST, compareValue, GPIO_PWM_M1_EN_C1_IDX);
    } else {
        DL_Timer_setCaptureCompareValue(PWM_M2_EN_INST, compareValue, GPIO_PWM_M2_EN_C1_IDX);
    }
    configureEnablePwm(motor);
#else
    const bool enable = (dutyPercent > 0U);
    if (motor == MotorId::Motor1) {
        if (enable) {
            DL_GPIO_setPins(GPIO_M1_EN_PORT, GPIO_M1_EN_M1_EN_PIN_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_M1_EN_PORT, GPIO_M1_EN_M1_EN_PIN_PIN);
        }
    } else {
        if (enable) {
            DL_GPIO_setPins(GPIO_M2_EN_PORT, GPIO_M2_EN_M2_EN_PIN_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_M2_EN_PORT, GPIO_M2_EN_M2_EN_PIN_PIN);
        }
    }
#endif
}

}  // namespace

void Drv8701_init(void)
{
    Drv8701_setCoast(MotorId::Motor1);
    Drv8701_setCoast(MotorId::Motor2);
#if defined(PWM_M1_EN_INST)
    DL_Timer_startCounter(PWM_M1_EN_INST);
    DL_Timer_startCounter(PWM_M2_EN_INST);
#endif
}

void Drv8701_setCoast(MotorId motor)
{
    setEnableDuty(motor, 0);
}

void Drv8701_setBrake(MotorId motor)
{
    setEnableDuty(motor, 0);
}

void Drv8701_setRun(MotorId motor, MotorDirection direction, uint8_t dutyPercent)
{
    setEnableDuty(motor, 0);
    setPhase(motor, direction);
    setEnableDuty(motor, dutyPercent);
}
