#include "drivers/drv8701.h"

#include "board/board_motor_pins.h"

namespace drivers {
namespace {

enum class EnableOutputMode : uint8_t {
    Unknown = 0U,
    Low,
    Pwm,
    High,
};

struct OutputState {
    MotorDirection direction;
    EnableOutputMode mode;
    bool direction_valid;
};

OutputState g_m1_output = {};
OutputState g_m2_output = {};

OutputState *StateForMotor(MotorId motor)
{
    return (motor == MotorId::Motor1) ? &g_m1_output : &g_m2_output;
}

uint32_t DutyToCompare(uint16_t duty_percent_q8)
{
    const uint32_t maximum = 100U * kDutyPercentQ8Scale;
    if (duty_percent_q8 > maximum) {
        duty_percent_q8 = static_cast<uint16_t>(maximum);
    }

    const uint64_t scaled =
        (static_cast<uint64_t>(board::kMotorPwmPeriodCounts) *
         duty_percent_q8) + (maximum / 2U);
    const uint32_t active_counts =
        static_cast<uint32_t>(scaled / maximum);
    return board::kMotorPwmPeriodCounts - active_counts;
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

void SetEnableDuty(MotorId motor, uint16_t duty_percent_q8)
{
    OutputState *state = StateForMotor(motor);
    const uint32_t full_scale = 100U * kDutyPercentQ8Scale;

    if (duty_percent_q8 == 0U) {
        if (state->mode != EnableOutputMode::Low) {
            ForceEnableLow(motor);
            state->mode = EnableOutputMode::Low;
        }
        return;
    }

    if (duty_percent_q8 >= full_scale) {
        if (state->mode != EnableOutputMode::High) {
            ForceEnableHigh(motor);
            state->mode = EnableOutputMode::High;
        }
        return;
    }

    const uint32_t compare = DutyToCompare(duty_percent_q8);

    if (motor == MotorId::Motor1) {
        DL_TimerG_setCaptureCompareValue(BOARD_M1_PWM_INST,
                                         compare,
                                         BOARD_M1_PWM_INDEX);
    } else {
        DL_TimerG_setCaptureCompareValue(BOARD_M2_PWM_INST,
                                         compare,
                                         BOARD_M2_PWM_INDEX);
    }

    if (state->mode != EnableOutputMode::Pwm) {
        ConfigureEnablePwm(motor);
        state->mode = EnableOutputMode::Pwm;
    }
}

}  // namespace

void Drv8701_Init(void)
{
    g_m1_output = {};
    g_m2_output = {};
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
    Drv8701_SetRunFine(
        motor,
        direction,
        static_cast<uint16_t>(duty_percent) * kDutyPercentQ8Scale);
}

void Drv8701_SetRunFine(MotorId motor,
                        MotorDirection direction,
                        uint16_t duty_percent_q8)
{
    OutputState *state = StateForMotor(motor);
    if ((!state->direction_valid) || (state->direction != direction)) {
        if (state->mode != EnableOutputMode::Low) {
            ForceEnableLow(motor);
            state->mode = EnableOutputMode::Low;
        }
        SetPhase(motor, direction);
        state->direction = direction;
        state->direction_valid = true;
    }
    SetEnableDuty(motor, duty_percent_q8);
}

}  // namespace drivers
