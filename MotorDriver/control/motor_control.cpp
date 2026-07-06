#include "control/motor_control.h"

#include "board/board_motor_outputs.h"
#include "protocol/motor_registers.h"

namespace control {
namespace {

constexpr uint32_t kSpeedControlPeriodMs = 100U;
constexpr int32_t kPidScale = 16;

struct SpeedPidState {
    int32_t integral_q4;
    int32_t previous_error_rpm;
    int32_t hold_count;
    int32_t previous_position_error;
    int32_t previous_target_position;
    int64_t position_integral_q4;
    uint32_t last_update_ms;
    uint32_t position_settle_until_ms;
    uint16_t previous_target_rpm;
    uint8_t previous_mode;
    uint8_t previous_direction;
    bool hold_mode;
    bool position_at_target;
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
    state->previous_position_error = 0;
    state->previous_target_position = 0;
    state->position_integral_q4 = 0;
    state->previous_target_rpm = 0U;
    state->previous_mode = protocol::MOTOR_MODE_COAST;
    state->previous_direction = protocol::MOTOR_DIRECTION_FORWARD;
    state->position_settle_until_ms = 0U;
    state->hold_mode = false;
    state->position_at_target = false;
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

int64_t ClampQ4(int64_t value, uint16_t max_rpm)
{
    const int64_t max_q4 = static_cast<int64_t>(max_rpm) * kPidScale;

    if (value > max_q4) {
        return max_q4;
    }
    if (value < -max_q4) {
        return -max_q4;
    }
    return value;
}

uint32_t AbsCount(int32_t value)
{
    return (value < 0) ? static_cast<uint32_t>(-static_cast<int64_t>(value)) :
                         static_cast<uint32_t>(value);
}

int8_t SignOfInt32(int32_t value)
{
    if (value > 0) {
        return 1;
    }
    if (value < 0) {
        return -1;
    }
    return 0;
}

bool TimeBefore(uint32_t now_ms, uint32_t deadline_ms)
{
    return static_cast<int32_t>(now_ms - deadline_ms) < 0;
}

int64_t PositionTermQ4(int32_t error_count,
                       uint8_t gain_q4_4,
                       uint32_t counts_per_rev)
{
    if ((gain_q4_4 == 0U) || (counts_per_rev == 0U)) {
        return 0;
    }

    return (static_cast<int64_t>(error_count) *
            static_cast<int64_t>(gain_q4_4) * 60) /
           static_cast<int64_t>(counts_per_rev);
}

int32_t SaturateInt32(int64_t value)
{
    if (value > 2147483647LL) {
        return 2147483647;
    }
    if (value < -2147483647LL - 1LL) {
        return static_cast<int32_t>(-2147483647LL - 1LL);
    }
    return static_cast<int32_t>(value);
}

uint16_t CommandRpmFromQ4(int64_t command_q4, uint16_t max_rpm)
{
    uint64_t magnitude =
        (command_q4 < 0) ? static_cast<uint64_t>(-command_q4) :
                           static_cast<uint64_t>(command_q4);

    if (magnitude > (static_cast<uint64_t>(max_rpm) * kPidScale)) {
        magnitude = static_cast<uint64_t>(max_rpm) * kPidScale;
    }

    return static_cast<uint16_t>((magnitude + (kPidScale / 2)) / kPidScale);
}

uint8_t PositionDutyFromCommand(uint16_t command_rpm,
                                uint16_t max_rpm,
                                uint8_t min_duty,
                                uint8_t max_duty)
{
    if (max_duty > 100U) {
        max_duty = 100U;
    }
    if (min_duty > 100U) {
        min_duty = 100U;
    }
    if (min_duty > max_duty) {
        min_duty = max_duty;
    }
    if ((command_rpm == 0U) || (max_rpm == 0U) || (max_duty == 0U)) {
        return 0U;
    }
    if (command_rpm >= max_rpm) {
        return max_duty;
    }

    const uint16_t span =
        static_cast<uint16_t>(max_duty - min_duty);
    const uint32_t scaled =
        (static_cast<uint32_t>(command_rpm) * span) +
        (static_cast<uint32_t>(max_rpm) / 2U);
    return static_cast<uint8_t>(
        static_cast<uint16_t>(min_duty) +
        static_cast<uint16_t>(scaled / max_rpm));
}

bool UpdatePositionMotor(protocol::RegisterMap &registers,
                         SpeedPidState *state,
                         bool motor1,
                         bool hold_mode,
                         int32_t target_position,
                         int32_t count,
                         uint32_t counts_per_rev,
                         uint32_t now_ms)
{
    const int32_t position_target =
        hold_mode ? state->hold_count : target_position;
    const int32_t position_error =
        SaturateInt32(static_cast<int64_t>(position_target) -
                      static_cast<int64_t>(count));
    const uint32_t abs_error = AbsCount(position_error);
    const uint16_t enter_tolerance = registers.PositionTolerance();
    uint16_t exit_tolerance = registers.PositionExitTolerance();
    if (exit_tolerance < enter_tolerance) {
        exit_tolerance = enter_tolerance;
    }

    const int32_t previous_error = state->previous_position_error;
    const int8_t previous_sign = SignOfInt32(previous_error);
    const int8_t current_sign = SignOfInt32(position_error);
    const bool sign_changed =
        (previous_sign != 0) && (current_sign != 0) &&
        (previous_sign != current_sign);

    registers.UpdatePositionError(motor1, position_error);

    if (sign_changed && (!state->position_at_target)) {
        state->position_at_target = true;
        state->position_integral_q4 = 0;
        state->position_settle_until_ms =
            now_ms + registers.PositionSettleMs();
    }

    uint16_t command_rpm = 0U;
    uint8_t output_duty = 0U;
    uint8_t output_direction = protocol::MOTOR_DIRECTION_FORWARD;

    if (state->position_at_target) {
        if ((!TimeBefore(now_ms, state->position_settle_until_ms)) &&
            (abs_error > exit_tolerance)) {
            state->position_at_target = false;
        }
    } else if (abs_error <= enter_tolerance) {
        state->position_at_target = true;
        state->position_integral_q4 = 0;
        state->position_settle_until_ms =
            now_ms + registers.PositionSettleMs();
    }

    if (!state->position_at_target) {
        const uint16_t max_rpm = registers.PositionMaxRpm();
        const int32_t derivative_count =
            SaturateInt32(static_cast<int64_t>(position_error) -
                          static_cast<int64_t>(previous_error));

        state->position_integral_q4 +=
            PositionTermQ4(position_error,
                           registers.PositionKiQ4_4(),
                           counts_per_rev);
        state->position_integral_q4 =
            ClampQ4(state->position_integral_q4, max_rpm);

        int64_t command_q4 =
            PositionTermQ4(position_error,
                           registers.PositionKpQ4_4(),
                           counts_per_rev) +
            state->position_integral_q4 +
            PositionTermQ4(derivative_count,
                           registers.PositionKdQ4_4(),
                           counts_per_rev);
        command_q4 = ClampQ4(command_q4, max_rpm);

        output_direction = (command_q4 < 0) ?
            protocol::MOTOR_DIRECTION_REVERSE :
            protocol::MOTOR_DIRECTION_FORWARD;
        command_rpm = CommandRpmFromQ4(command_q4, max_rpm);
        if ((command_rpm == 0U) && (abs_error > enter_tolerance) &&
            (max_rpm > 0U) &&
            ((registers.PositionKpQ4_4() != 0U) ||
             (registers.PositionKiQ4_4() != 0U) ||
             (registers.PositionKdQ4_4() != 0U))) {
            command_rpm = 1U;
        }

        output_duty = PositionDutyFromCommand(command_rpm,
                                              max_rpm,
                                              registers.PositionMinDuty(),
                                              registers.PositionMaxDuty());
    }

    if (!hold_mode) {
        registers.UpdateTargetRpmFromControl(motor1, command_rpm);
    }
    registers.SetMotorOutputFromControl(motor1, output_duty, output_direction);
    state->previous_position_error = position_error;
    return true;
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
    const int32_t target_position =
        motor1 ? registers.M1TargetPosition() : registers.M2TargetPosition();
    const uint32_t counts_per_rev =
        motor1 ? registers.M1CountsPerRev() : registers.M2CountsPerRev();
    const bool speed_mode = mode == protocol::MOTOR_MODE_SPEED;
    const bool position_mode = mode == protocol::MOTOR_MODE_POSITION;
    const bool hold_mode =
        speed_mode && (target_rpm == 0U);
    const uint16_t target_rpm_key = speed_mode ? target_rpm : 0U;
    const uint8_t direction_key =
        speed_mode ? direction : protocol::MOTOR_DIRECTION_FORWARD;
    const int32_t target_position_key =
        position_mode ? target_position : 0;

    if (((!speed_mode) && (!position_mode)) || (counts_per_rev == 0U)) {
        ResetPid(state);
        if (speed_mode || position_mode) {
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
        (state->previous_direction != direction_key) ||
        (state->previous_target_rpm != target_rpm_key) ||
        (state->previous_target_position != target_position_key) ||
        (state->hold_mode != hold_mode)) {
        ResetPid(state);
        state->hold_count = count;
        state->last_update_ms = now_ms;
        state->previous_mode = mode;
        state->previous_direction = direction_key;
        state->previous_target_rpm = target_rpm_key;
        state->previous_target_position = target_position_key;
        state->hold_mode = hold_mode;
        state->initialized = true;
        if (hold_mode) {
            const int32_t position_error = 0;
            registers.UpdateHoldTarget(motor1, state->hold_count);
            registers.UpdatePositionError(motor1, position_error);
            state->previous_position_error = position_error;
            state->position_at_target = true;
            registers.SetMotorDutyFromControl(motor1, 0U);
            return true;
        }
        if (position_mode) {
            const int32_t position_error =
                SaturateInt32(static_cast<int64_t>(target_position) -
                              static_cast<int64_t>(count));
            registers.UpdatePositionError(motor1, position_error);
            state->previous_position_error = position_error;
            state->position_at_target =
                AbsCount(position_error) <= registers.PositionTolerance();
            registers.UpdateTargetRpmFromControl(motor1, 0U);
            registers.SetMotorDutyFromControl(motor1, 0U);
            return true;
        }
    }

    if ((now_ms - state->last_update_ms) < kSpeedControlPeriodMs) {
        return false;
    }
    state->last_update_ms = now_ms;

    if (hold_mode || position_mode) {
        (void) measured_rpm;
        return UpdatePositionMotor(registers,
                                   state,
                                   motor1,
                                   hold_mode,
                                   target_position,
                                   count,
                                   counts_per_rev,
                                   now_ms);
    }

    const uint16_t command_rpm = target_rpm;
    const uint8_t output_direction = direction;
    const int32_t measured_aligned =
        (output_direction == protocol::MOTOR_DIRECTION_REVERSE)
            ? -static_cast<int32_t>(measured_rpm)
            : static_cast<int32_t>(measured_rpm);
    const int32_t error_rpm =
        static_cast<int32_t>(command_rpm) - measured_aligned;
    const int32_t derivative_rpm = error_rpm - state->previous_error_rpm;
    state->previous_error_rpm = error_rpm;

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
    case protocol::MOTOR_MODE_POSITION:
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
