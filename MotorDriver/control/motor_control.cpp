#include "control/motor_control.h"

#include "board/board_motor_outputs.h"
#include "protocol/motor_registers.h"

namespace control {
namespace {

constexpr uint32_t kSpeedControlPeriodMs = 100U;
constexpr int32_t kPidScale = 16;
constexpr int32_t kHoldPositionGain = 10;
constexpr uint16_t kHoldMaxRpm = 100U;

struct SpeedPidState {
    int32_t integral_q4;
    int32_t previous_error_rpm;
    int32_t hold_count;
    uint32_t last_update_ms;
    uint16_t previous_target_rpm;
    uint8_t previous_mode;
    uint8_t previous_direction;
    bool hold_mode;
    bool initialized;
};

SpeedPidState g_m1_speed = {};
SpeedPidState g_m2_speed = {};

drivers::MotorDirection DirectionFromRegister(uint8_t value)
{
    return (value == protocol::MOTOR_DIRECTION_REVERSE)
               ? drivers::MotorDirection::Reverse
               : drivers::MotorDirection::Forward;
}

void ResetPid(SpeedPidState *state)
{
    state->integral_q4 = 0;
    state->previous_error_rpm = 0;
    state->hold_count = 0;
    state->previous_target_rpm = 0U;
    state->previous_mode = protocol::MOTOR_MODE_COAST;
    state->previous_direction = protocol::MOTOR_DIRECTION_FORWARD;
    state->hold_mode = false;
    state->initialized = false;
}

uint8_t ClampDuty(int32_t duty_q4, uint8_t min_duty, uint8_t max_duty)
{
    if (max_duty > 100U) {
        max_duty = 100U;
    }
    if (min_duty > max_duty) {
        min_duty = max_duty;
    }

    if (duty_q4 <= 0) {
        return 0U;
    }

    const int32_t min_q4 = static_cast<int32_t>(min_duty) * kPidScale;
    const int32_t max_q4 = static_cast<int32_t>(max_duty) * kPidScale;

    if (duty_q4 < min_q4) {
        duty_q4 = min_q4;
    }
    if (duty_q4 > max_q4) {
        duty_q4 = max_q4;
    }

    return static_cast<uint8_t>((duty_q4 + (kPidScale / 2)) / kPidScale);
}

bool UpdateSpeedMotor(protocol::RegisterMap &registers,
                      SpeedPidState *state,
                      bool motor1,
                      int32_t count,
                      int16_t measured_rpm,
                      uint32_t now_ms)
{
    const uint8_t mode =
        registers.Read(motor1 ? protocol::REG_M1_MODE : protocol::REG_M2_MODE);
    const uint8_t direction =
        registers.Read(motor1 ? protocol::REG_M1_DIRECTION
                              : protocol::REG_M2_DIRECTION);
    const uint16_t target_rpm =
        motor1 ? registers.M1TargetRpm() : registers.M2TargetRpm();
    const uint32_t counts_per_rev =
        motor1 ? registers.M1CountsPerRev() : registers.M2CountsPerRev();
    const bool hold_mode =
        (mode == protocol::MOTOR_MODE_SPEED) && (target_rpm == 0U);

    if ((mode != protocol::MOTOR_MODE_SPEED) ||
        (counts_per_rev == 0U)) {
        ResetPid(state);
        if (mode == protocol::MOTOR_MODE_SPEED) {
            const uint8_t duty =
                registers.Read(motor1 ? protocol::REG_M1_DUTY
                                      : protocol::REG_M2_DUTY);
            if (duty != 0U) {
                registers.SetMotorDutyFromControl(motor1, 0U);
                return true;
            }
        }
        return false;
    }

    if ((!state->initialized) ||
        (state->previous_mode != mode) ||
        (state->previous_direction != direction) ||
        (state->previous_target_rpm != target_rpm) ||
        (state->hold_mode != hold_mode)) {
        ResetPid(state);
        state->hold_count = count;
        state->last_update_ms = now_ms;
        state->previous_mode = mode;
        state->previous_direction = direction;
        state->previous_target_rpm = target_rpm;
        state->hold_mode = hold_mode;
        state->initialized = true;
        if (hold_mode) {
            registers.UpdateHoldTarget(motor1, state->hold_count);
            registers.SetMotorDutyFromControl(motor1, 0U);
            return true;
        }
    }

    if ((now_ms - state->last_update_ms) < kSpeedControlPeriodMs) {
        return false;
    }
    state->last_update_ms = now_ms;

    uint16_t command_rpm = target_rpm;
    uint8_t output_direction = direction;

    if (hold_mode) {
        const int64_t position_error =
            static_cast<int64_t>(state->hold_count) -
            static_cast<int64_t>(count);
        const uint64_t abs_error =
            (position_error < 0) ? static_cast<uint64_t>(-position_error) :
                                   static_cast<uint64_t>(position_error);

        if (position_error < 0) {
            output_direction = protocol::MOTOR_DIRECTION_REVERSE;
        } else if (position_error > 0) {
            output_direction = protocol::MOTOR_DIRECTION_FORWARD;
        } else if (measured_rpm > 0) {
            output_direction = protocol::MOTOR_DIRECTION_REVERSE;
        } else if (measured_rpm < 0) {
            output_direction = protocol::MOTOR_DIRECTION_FORWARD;
        }

        uint64_t hold_rpm =
            (abs_error * 60ULL * static_cast<uint64_t>(kHoldPositionGain)) /
            static_cast<uint64_t>(counts_per_rev);
        if ((abs_error > 0U) && (hold_rpm == 0U)) {
            hold_rpm = 1;
        }
        if (hold_rpm > kHoldMaxRpm) {
            hold_rpm = kHoldMaxRpm;
        }
        command_rpm = static_cast<uint16_t>(hold_rpm);
    }

    const int32_t measured_aligned =
        (output_direction == protocol::MOTOR_DIRECTION_REVERSE)
            ? -static_cast<int32_t>(measured_rpm)
            : static_cast<int32_t>(measured_rpm);
    const int32_t error_rpm =
        static_cast<int32_t>(command_rpm) - measured_aligned;
    const int32_t derivative_rpm = error_rpm - state->previous_error_rpm;
    state->previous_error_rpm = error_rpm;

    if ((hold_mode) && (command_rpm == 0U)) {
        state->integral_q4 = 0;
    }

    const int32_t max_q4 =
        static_cast<int32_t>(registers.SpeedMaxDuty()) * kPidScale;
    state->integral_q4 +=
        static_cast<int32_t>(registers.SpeedKiQ4_4()) * error_rpm;
    if (state->integral_q4 > max_q4) {
        state->integral_q4 = max_q4;
    } else if (state->integral_q4 < -max_q4) {
        state->integral_q4 = -max_q4;
    }

    const int32_t output_q4 =
        (static_cast<int32_t>(registers.SpeedKpQ4_4()) * error_rpm) +
        state->integral_q4 +
        (static_cast<int32_t>(registers.SpeedKdQ4_4()) * derivative_rpm);
    const uint8_t duty = ClampDuty(output_q4,
                                   registers.SpeedMinDuty(),
                                   registers.SpeedMaxDuty());

    registers.SetMotorOutputFromControl(motor1, duty, output_direction);
    return true;
}

void ApplyMotor(drivers::MotorId motor,
                uint8_t mode,
                uint8_t duty,
                uint8_t direction)
{
    switch (mode) {
    case protocol::MOTOR_MODE_RUN:
    case protocol::MOTOR_MODE_SPEED:
        if (duty == 0U) {
            board::BoardMotorOutputs_Coast(motor);
        } else {
            board::BoardMotorOutputs_Run(motor,
                                         DirectionFromRegister(direction),
                                         duty);
        }
        break;

    case protocol::MOTOR_MODE_BRAKE:
        board::BoardMotorOutputs_Brake(motor);
        break;

    case protocol::MOTOR_MODE_COAST:
    default:
        board::BoardMotorOutputs_Coast(motor);
        break;
    }
}

}  // namespace

void MotorControl_Init(void)
{
    ResetPid(&g_m1_speed);
    ResetPid(&g_m2_speed);
    board::BoardMotorOutputs_Init();
}

bool MotorControl_UpdateSpeed(protocol::RegisterMap &registers,
                              int32_t m1_count,
                              int16_t m1_rpm,
                              int32_t m2_count,
                              int16_t m2_rpm,
                              uint32_t now_ms)
{
    const bool m1_changed =
        UpdateSpeedMotor(registers,
                         &g_m1_speed,
                         true,
                         m1_count,
                         m1_rpm,
                         now_ms);
    const bool m2_changed =
        UpdateSpeedMotor(registers,
                         &g_m2_speed,
                         false,
                         m2_count,
                         m2_rpm,
                         now_ms);

    return m1_changed || m2_changed;
}

void MotorControl_Apply(const protocol::RegisterMap &registers,
                        bool output_inhibited)
{
    if (output_inhibited || (!registers.IsEnabled())) {
        MotorControl_StopAll();
        return;
    }

    ApplyMotor(drivers::MotorId::Motor1,
               registers.Read(protocol::REG_M1_MODE),
               registers.Read(protocol::REG_M1_DUTY),
               registers.Read(protocol::REG_M1_DIRECTION));
    ApplyMotor(drivers::MotorId::Motor2,
               registers.Read(protocol::REG_M2_MODE),
               registers.Read(protocol::REG_M2_DUTY),
               registers.Read(protocol::REG_M2_DIRECTION));
}

void MotorControl_StopAll(void)
{
    board::BoardMotorOutputs_Coast(drivers::MotorId::Motor1);
    board::BoardMotorOutputs_Coast(drivers::MotorId::Motor2);
}

}  // namespace control
