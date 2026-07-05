#include "protocol/register_map.h"

namespace protocol {

void RegisterMap::Init(void)
{
    for (uint8_t index = 0U; index < kRegisterCount; index++) {
        registers_[index] = 0U;
    }

    timeout_active_ = false;

    registers_[REG_DEVICE_ID] = kDeviceId;
    registers_[REG_FW_VERSION] = kFirmwareVersion;
    registers_[REG_STATUS] = STATUS_READY | STATUS_ENABLED;
    registers_[REG_FAULT_FLAGS] = 0U;
    registers_[REG_CONTROL_FLAGS] = CONTROL_ENABLE;
    registers_[REG_I2C_ADDRESS] = 0U;
    registers_[REG_M1_MODE] = MOTOR_MODE_COAST;
    registers_[REG_M1_DUTY] = 0U;
    registers_[REG_M1_DIRECTION] = MOTOR_DIRECTION_FORWARD;
    registers_[REG_M1_STATUS] = 0U;
    registers_[REG_M1_ENCODER_STATE] = 0U;
    registers_[REG_M2_MODE] = MOTOR_MODE_COAST;
    registers_[REG_M2_DUTY] = 0U;
    registers_[REG_M2_DIRECTION] = MOTOR_DIRECTION_FORWARD;
    registers_[REG_M2_STATUS] = 0U;
    registers_[REG_M2_ENCODER_STATE] = 0U;
    registers_[REG_WATCHDOG_TIMEOUT] = 100U;
    registers_[REG_ENCODER_CONTROL] = 0U;
    registers_[REG_SPEED_KP_Q4_4] = 1U;
    registers_[REG_SPEED_KI_Q4_4] = 1U;
    registers_[REG_SPEED_KD_Q4_4] = 1U;
    registers_[REG_SPEED_MAX_DUTY] = 30U;
    registers_[REG_SPEED_MIN_DUTY] = 0U;
    registers_[REG_POSITION_KP_Q4_4] = 160U;
    registers_[REG_POSITION_KI_Q4_4] = 0U;
    registers_[REG_POSITION_KD_Q4_4] = 0U;
    StoreUint16(REG_POSITION_MAX_RPM_0, 100U);
    StoreUint16(REG_POSITION_TOLERANCE_0, 20U);
    StoreInt32(REG_M1_COUNTS_PER_REV_0, 22400);
    StoreInt32(REG_M2_COUNTS_PER_REV_0, 22400);

    RefreshStatus();
}

uint8_t RegisterMap::Read(uint8_t reg) const
{
    if (reg >= kRegisterCount) {
        return 0U;
    }

    return registers_[reg];
}

bool RegisterMap::ReadBlock(uint8_t start_reg, uint8_t length, uint8_t *out) const
{
    if ((out == 0) || (!IsReadableRange(start_reg, length))) {
        return false;
    }

    for (uint8_t index = 0U; index < length; index++) {
        out[index] = Read(static_cast<uint8_t>(start_reg + index));
    }

    return true;
}

bool RegisterMap::CommitWrite(uint8_t start_reg,
                              const uint8_t *data,
                              uint8_t length,
                              bool *control_changed)
{
    if ((data == 0) || (length == 0U) ||
        ((uint16_t) start_reg + (uint16_t) length > kRegisterCount)) {
        return false;
    }

    uint8_t next[kRegisterCount];
    bool changed = false;

    for (uint8_t index = 0U; index < kRegisterCount; index++) {
        next[index] = registers_[index];
    }

    for (uint8_t index = 0U; index < length; index++) {
        const uint8_t reg = static_cast<uint8_t>(start_reg + index);
        if (!ApplyWriteTo(next, reg, data[index])) {
            return false;
        }
        if (IsControlRegister(reg)) {
            changed = true;
        }
    }

    NormalizeControlState(next, start_reg, length);

    for (uint8_t index = 0U; index < kRegisterCount; index++) {
        registers_[index] = next[index];
    }

    RefreshStatus();

    if (control_changed != 0) {
        *control_changed = changed;
    }

    return true;
}

void RegisterMap::SetTimeoutActive(bool active)
{
    timeout_active_ = active;
    RefreshStatus();
}

void RegisterMap::ForceCoastOnTimeout(void)
{
    ClearMotorMotion(registers_, true);
    ClearMotorMotion(registers_, false);
    registers_[REG_FAULT_FLAGS] =
        static_cast<uint8_t>(registers_[REG_FAULT_FLAGS] | FAULT_WATCHDOG_TIMEOUT);
    timeout_active_ = true;
    RefreshStatus();
}

void RegisterMap::RefreshStatus(void)
{
    uint8_t status = STATUS_READY;
    const bool enabled = IsEnabled();

    if (enabled) {
        status = static_cast<uint8_t>(status | STATUS_ENABLED);
    }

    const uint8_t m1_status = MotorStatus(true,
                                          registers_[REG_M1_MODE],
                                          registers_[REG_M1_DUTY],
                                          registers_[REG_M1_DIRECTION]);
    const uint8_t m2_status = MotorStatus(false,
                                          registers_[REG_M2_MODE],
                                          registers_[REG_M2_DUTY],
                                          registers_[REG_M2_DIRECTION]);

    registers_[REG_M1_STATUS] = m1_status;
    registers_[REG_M2_STATUS] = m2_status;
    registers_[REG_POSITION_STATUS] =
        static_cast<uint8_t>((PositionAtTarget(true) ?
                              POSITION_STATUS_M1_AT_TARGET : 0U) |
                             (PositionAtTarget(false) ?
                              POSITION_STATUS_M2_AT_TARGET : 0U));

    if (enabled && (!timeout_active_) &&
        (((m1_status | m2_status) & MOTOR_STATUS_ACTIVE) != 0U)) {
        status = static_cast<uint8_t>(status | STATUS_ACTIVE);
    }

    if (timeout_active_) {
        status = static_cast<uint8_t>(status | STATUS_TIMEOUT);
    }

    registers_[REG_STATUS] = status;
}

void RegisterMap::UpdateEncoderSnapshot(int32_t m1_count,
                                        int32_t m1_counts_per_second,
                                        uint8_t m1_state,
                                        int32_t m2_count,
                                        int32_t m2_counts_per_second,
                                        uint8_t m2_state)
{
    StoreInt32(REG_M1_ENCODER_COUNT_0, m1_count);
    StoreInt32(REG_M1_ENCODER_CPS_0, m1_counts_per_second);
    registers_[REG_M1_ENCODER_STATE] = static_cast<uint8_t>(m1_state & 0x03U);

    StoreInt32(REG_M2_ENCODER_COUNT_0, m2_count);
    StoreInt32(REG_M2_ENCODER_CPS_0, m2_counts_per_second);
    registers_[REG_M2_ENCODER_STATE] = static_cast<uint8_t>(m2_state & 0x03U);
}

void RegisterMap::UpdateMeasuredRpm(int16_t m1_rpm, int16_t m2_rpm)
{
    StoreInt16(REG_M1_MEASURED_RPM_0, m1_rpm);
    StoreInt16(REG_M2_MEASURED_RPM_0, m2_rpm);
}

void RegisterMap::SetMotorDutyFromControl(bool motor1, uint8_t duty)
{
    SetMotorOutputFromControl(
        motor1,
        duty,
        registers_[motor1 ? REG_M1_DIRECTION : REG_M2_DIRECTION]);
}

void RegisterMap::SetMotorOutputFromControl(bool motor1,
                                            uint8_t duty,
                                            uint8_t direction)
{
    if (duty > 100U) {
        duty = 100U;
    }
    if (direction > MOTOR_DIRECTION_REVERSE) {
        direction = MOTOR_DIRECTION_FORWARD;
    }

    registers_[motor1 ? REG_M1_DUTY : REG_M2_DUTY] = duty;
    registers_[motor1 ? REG_M1_DIRECTION : REG_M2_DIRECTION] = direction;
    RefreshStatus();
}

void RegisterMap::UpdateTargetRpmFromControl(bool motor1, uint16_t rpm)
{
    StoreUint16(motor1 ? REG_M1_TARGET_RPM_0 : REG_M2_TARGET_RPM_0, rpm);
    RefreshStatus();
}

void RegisterMap::UpdateHoldTarget(bool motor1, int32_t count)
{
    StoreInt32(motor1 ? REG_M1_HOLD_COUNT_0 : REG_M2_HOLD_COUNT_0, count);
}

void RegisterMap::UpdatePositionError(bool motor1, int32_t error)
{
    StoreInt32(motor1 ? REG_M1_POSITION_ERROR_0 : REG_M2_POSITION_ERROR_0,
               error);
    RefreshStatus();
}

uint8_t RegisterMap::EncoderControl(void) const
{
    return registers_[REG_ENCODER_CONTROL];
}

void RegisterMap::ClearEncoderControl(void)
{
    registers_[REG_ENCODER_CONTROL] = 0U;
}

bool RegisterMap::IsEnabled(void) const
{
    return (registers_[REG_CONTROL_FLAGS] & CONTROL_ENABLE) != 0U;
}

bool RegisterMap::HasActiveCommand(void) const
{
    if ((!IsEnabled()) || timeout_active_) {
        return false;
    }

    if ((registers_[REG_M1_MODE] == MOTOR_MODE_RUN) &&
        (registers_[REG_M1_DUTY] > 0U)) {
        return true;
    }
    if ((registers_[REG_M2_MODE] == MOTOR_MODE_RUN) &&
        (registers_[REG_M2_DUTY] > 0U)) {
        return true;
    }
    if (registers_[REG_M1_MODE] == MOTOR_MODE_SPEED) {
        return true;
    }
    if (registers_[REG_M2_MODE] == MOTOR_MODE_SPEED) {
        return true;
    }
    if (registers_[REG_M1_MODE] == MOTOR_MODE_POSITION) {
        return true;
    }
    if (registers_[REG_M2_MODE] == MOTOR_MODE_POSITION) {
        return true;
    }

    return false;
}

uint8_t RegisterMap::WatchdogTimeout10ms(void) const
{
    return registers_[REG_WATCHDOG_TIMEOUT];
}

uint16_t RegisterMap::M1TargetRpm(void) const
{
    return LoadUint16(REG_M1_TARGET_RPM_0);
}

uint16_t RegisterMap::M2TargetRpm(void) const
{
    return LoadUint16(REG_M2_TARGET_RPM_0);
}

uint8_t RegisterMap::SpeedKpQ4_4(void) const
{
    return registers_[REG_SPEED_KP_Q4_4];
}

uint8_t RegisterMap::SpeedKiQ4_4(void) const
{
    return registers_[REG_SPEED_KI_Q4_4];
}

uint8_t RegisterMap::SpeedKdQ4_4(void) const
{
    return registers_[REG_SPEED_KD_Q4_4];
}

uint8_t RegisterMap::SpeedMaxDuty(void) const
{
    return registers_[REG_SPEED_MAX_DUTY];
}

uint8_t RegisterMap::SpeedMinDuty(void) const
{
    return registers_[REG_SPEED_MIN_DUTY];
}

uint32_t RegisterMap::M1CountsPerRev(void) const
{
    return LoadUint32(REG_M1_COUNTS_PER_REV_0);
}

uint32_t RegisterMap::M2CountsPerRev(void) const
{
    return LoadUint32(REG_M2_COUNTS_PER_REV_0);
}

int32_t RegisterMap::M1TargetPosition(void) const
{
    return LoadInt32(REG_M1_TARGET_POSITION_0);
}

int32_t RegisterMap::M2TargetPosition(void) const
{
    return LoadInt32(REG_M2_TARGET_POSITION_0);
}

uint8_t RegisterMap::PositionKpQ4_4(void) const
{
    return registers_[REG_POSITION_KP_Q4_4];
}

uint8_t RegisterMap::PositionKiQ4_4(void) const
{
    return registers_[REG_POSITION_KI_Q4_4];
}

uint8_t RegisterMap::PositionKdQ4_4(void) const
{
    return registers_[REG_POSITION_KD_Q4_4];
}

uint16_t RegisterMap::PositionMaxRpm(void) const
{
    return LoadUint16(REG_POSITION_MAX_RPM_0);
}

uint16_t RegisterMap::PositionTolerance(void) const
{
    return LoadUint16(REG_POSITION_TOLERANCE_0);
}

uint16_t RegisterMap::LoadUint16(uint8_t reg) const
{
    return static_cast<uint16_t>(
        static_cast<uint16_t>(registers_[reg]) |
        (static_cast<uint16_t>(registers_[static_cast<uint8_t>(reg + 1U)]) << 8U));
}

uint32_t RegisterMap::LoadUint32(uint8_t reg) const
{
    return static_cast<uint32_t>(
        static_cast<uint32_t>(registers_[reg]) |
        (static_cast<uint32_t>(registers_[static_cast<uint8_t>(reg + 1U)]) << 8U) |
        (static_cast<uint32_t>(registers_[static_cast<uint8_t>(reg + 2U)]) << 16U) |
        (static_cast<uint32_t>(registers_[static_cast<uint8_t>(reg + 3U)]) << 24U));
}

int32_t RegisterMap::LoadInt32(uint8_t reg) const
{
    return static_cast<int32_t>(LoadUint32(reg));
}

void RegisterMap::StoreUint16(uint8_t reg, uint16_t value)
{
    registers_[reg] = static_cast<uint8_t>(value & 0xFFU);
    registers_[static_cast<uint8_t>(reg + 1U)] =
        static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

void RegisterMap::StoreInt16(uint8_t reg, int16_t value)
{
    const uint16_t encoded = static_cast<uint16_t>(value);

    registers_[reg] = static_cast<uint8_t>(encoded & 0xFFU);
    registers_[static_cast<uint8_t>(reg + 1U)] =
        static_cast<uint8_t>((encoded >> 8U) & 0xFFU);
}

void RegisterMap::StoreInt32(uint8_t reg, int32_t value)
{
    const uint32_t encoded = static_cast<uint32_t>(value);

    registers_[reg] = static_cast<uint8_t>(encoded & 0xFFU);
    registers_[static_cast<uint8_t>(reg + 1U)] =
        static_cast<uint8_t>((encoded >> 8U) & 0xFFU);
    registers_[static_cast<uint8_t>(reg + 2U)] =
        static_cast<uint8_t>((encoded >> 16U) & 0xFFU);
    registers_[static_cast<uint8_t>(reg + 3U)] =
        static_cast<uint8_t>((encoded >> 24U) & 0xFFU);
}

bool RegisterMap::PositionAtTarget(bool motor1) const
{
    const uint8_t mode = registers_[motor1 ? REG_M1_MODE : REG_M2_MODE];
    if (mode != MOTOR_MODE_POSITION) {
        return false;
    }

    const int32_t error =
        LoadInt32(motor1 ? REG_M1_POSITION_ERROR_0 : REG_M2_POSITION_ERROR_0);
    const uint32_t abs_error =
        (error < 0) ? static_cast<uint32_t>(-static_cast<int64_t>(error)) :
                      static_cast<uint32_t>(error);
    return abs_error <= PositionTolerance();
}

void RegisterMap::ClearMotorMotion(uint8_t *target, bool motor1) const
{
    target[motor1 ? REG_M1_MODE : REG_M2_MODE] = MOTOR_MODE_COAST;
    target[motor1 ? REG_M1_DUTY : REG_M2_DUTY] = 0U;
    target[motor1 ? REG_M1_TARGET_RPM_0 : REG_M2_TARGET_RPM_0] = 0U;
    target[motor1 ? REG_M1_TARGET_RPM_1 : REG_M2_TARGET_RPM_1] = 0U;
}

void RegisterMap::NormalizeControlState(uint8_t *target,
                                        uint8_t start_reg,
                                        uint8_t length) const
{
    const uint16_t start = start_reg;
    const uint16_t end = static_cast<uint16_t>(start_reg) + length;
    const bool control_written =
        (start <= REG_CONTROL_FLAGS) && (REG_CONTROL_FLAGS < end);
    const bool m1_mode_written =
        (start <= REG_M1_MODE) && (REG_M1_MODE < end);
    const bool m2_mode_written =
        (start <= REG_M2_MODE) && (REG_M2_MODE < end);

    if (control_written &&
        ((target[REG_CONTROL_FLAGS] & CONTROL_ENABLE) == 0U)) {
        ClearMotorMotion(target, true);
        ClearMotorMotion(target, false);
        return;
    }

    if (m1_mode_written && (target[REG_M1_MODE] == MOTOR_MODE_COAST)) {
        ClearMotorMotion(target, true);
    } else if (m1_mode_written && (target[REG_M1_MODE] == MOTOR_MODE_BRAKE)) {
        target[REG_M1_DUTY] = 0U;
        target[REG_M1_TARGET_RPM_0] = 0U;
        target[REG_M1_TARGET_RPM_1] = 0U;
    } else if (m1_mode_written && (target[REG_M1_MODE] == MOTOR_MODE_RUN)) {
        target[REG_M1_TARGET_RPM_0] = 0U;
        target[REG_M1_TARGET_RPM_1] = 0U;
    } else if (m1_mode_written && (target[REG_M1_MODE] == MOTOR_MODE_POSITION)) {
        target[REG_M1_DUTY] = 0U;
        target[REG_M1_TARGET_RPM_0] = 0U;
        target[REG_M1_TARGET_RPM_1] = 0U;
    }

    if (m2_mode_written && (target[REG_M2_MODE] == MOTOR_MODE_COAST)) {
        ClearMotorMotion(target, false);
    } else if (m2_mode_written && (target[REG_M2_MODE] == MOTOR_MODE_BRAKE)) {
        target[REG_M2_DUTY] = 0U;
        target[REG_M2_TARGET_RPM_0] = 0U;
        target[REG_M2_TARGET_RPM_1] = 0U;
    } else if (m2_mode_written && (target[REG_M2_MODE] == MOTOR_MODE_RUN)) {
        target[REG_M2_TARGET_RPM_0] = 0U;
        target[REG_M2_TARGET_RPM_1] = 0U;
    } else if (m2_mode_written && (target[REG_M2_MODE] == MOTOR_MODE_POSITION)) {
        target[REG_M2_DUTY] = 0U;
        target[REG_M2_TARGET_RPM_0] = 0U;
        target[REG_M2_TARGET_RPM_1] = 0U;
    }
}

bool RegisterMap::ApplyWriteTo(uint8_t *target, uint8_t reg, uint8_t value) const
{
    switch (reg) {
    case REG_FAULT_FLAGS:
        if ((value & static_cast<uint8_t>(~FAULT_WATCHDOG_TIMEOUT)) != 0U) {
            return false;
        }
        target[reg] = static_cast<uint8_t>(target[reg] & static_cast<uint8_t>(~value));
        return true;

    case REG_CONTROL_FLAGS:
        if ((value & static_cast<uint8_t>(~CONTROL_ENABLE)) != 0U) {
            return false;
        }
        target[reg] = value;
        return true;

    case REG_M1_MODE:
    case REG_M2_MODE:
        if (value > MOTOR_MODE_POSITION) {
            return false;
        }
        target[reg] = value;
        return true;

    case REG_M1_DUTY:
    case REG_M2_DUTY:
        if (value > 100U) {
            return false;
        }
        target[reg] = value;
        return true;

    case REG_M1_DIRECTION:
    case REG_M2_DIRECTION:
        if (value > MOTOR_DIRECTION_REVERSE) {
            return false;
        }
        target[reg] = value;
        return true;

    case REG_WATCHDOG_TIMEOUT:
        target[reg] = value;
        return true;

    case REG_ENCODER_CONTROL:
        if ((value & static_cast<uint8_t>(~(ENCODER_CONTROL_RESET_M1 |
                                            ENCODER_CONTROL_RESET_M2))) != 0U) {
            return false;
        }
        target[reg] = value;
        return true;

    case REG_M1_TARGET_RPM_0:
    case REG_M1_TARGET_RPM_1:
    case REG_M2_TARGET_RPM_0:
    case REG_M2_TARGET_RPM_1:
    case REG_SPEED_KP_Q4_4:
    case REG_SPEED_KI_Q4_4:
    case REG_SPEED_KD_Q4_4:
    case REG_M1_COUNTS_PER_REV_0:
    case REG_M1_COUNTS_PER_REV_1:
    case REG_M1_COUNTS_PER_REV_2:
    case REG_M1_COUNTS_PER_REV_3:
    case REG_M2_COUNTS_PER_REV_0:
    case REG_M2_COUNTS_PER_REV_1:
    case REG_M2_COUNTS_PER_REV_2:
    case REG_M2_COUNTS_PER_REV_3:
    case REG_M1_TARGET_POSITION_0:
    case REG_M1_TARGET_POSITION_1:
    case REG_M1_TARGET_POSITION_2:
    case REG_M1_TARGET_POSITION_3:
    case REG_M2_TARGET_POSITION_0:
    case REG_M2_TARGET_POSITION_1:
    case REG_M2_TARGET_POSITION_2:
    case REG_M2_TARGET_POSITION_3:
    case REG_POSITION_KP_Q4_4:
    case REG_POSITION_KI_Q4_4:
    case REG_POSITION_KD_Q4_4:
    case REG_POSITION_MAX_RPM_0:
    case REG_POSITION_MAX_RPM_1:
    case REG_POSITION_TOLERANCE_0:
    case REG_POSITION_TOLERANCE_1:
        target[reg] = value;
        return true;

    case REG_SPEED_MAX_DUTY:
    case REG_SPEED_MIN_DUTY:
        if (value > 100U) {
            return false;
        }
        target[reg] = value;
        return true;

    default:
        return false;
    }
}

bool RegisterMap::IsReadableRange(uint8_t start_reg, uint8_t length) const
{
    if ((length == 0U) ||
        ((uint16_t) start_reg + (uint16_t) length > kRegisterCount)) {
        return false;
    }

    return true;
}

bool RegisterMap::IsControlRegister(uint8_t reg) const
{
    switch (reg) {
    case REG_CONTROL_FLAGS:
    case REG_M1_MODE:
    case REG_M1_DUTY:
    case REG_M1_DIRECTION:
    case REG_M1_TARGET_RPM_0:
    case REG_M1_TARGET_RPM_1:
    case REG_M2_MODE:
    case REG_M2_DUTY:
    case REG_M2_DIRECTION:
    case REG_M2_TARGET_RPM_0:
    case REG_M2_TARGET_RPM_1:
    case REG_SPEED_KP_Q4_4:
    case REG_SPEED_KI_Q4_4:
    case REG_SPEED_KD_Q4_4:
    case REG_SPEED_MAX_DUTY:
    case REG_SPEED_MIN_DUTY:
    case REG_M1_COUNTS_PER_REV_0:
    case REG_M1_COUNTS_PER_REV_1:
    case REG_M1_COUNTS_PER_REV_2:
    case REG_M1_COUNTS_PER_REV_3:
    case REG_M2_COUNTS_PER_REV_0:
    case REG_M2_COUNTS_PER_REV_1:
    case REG_M2_COUNTS_PER_REV_2:
    case REG_M2_COUNTS_PER_REV_3:
    case REG_M1_TARGET_POSITION_0:
    case REG_M1_TARGET_POSITION_1:
    case REG_M1_TARGET_POSITION_2:
    case REG_M1_TARGET_POSITION_3:
    case REG_M2_TARGET_POSITION_0:
    case REG_M2_TARGET_POSITION_1:
    case REG_M2_TARGET_POSITION_2:
    case REG_M2_TARGET_POSITION_3:
    case REG_POSITION_KP_Q4_4:
    case REG_POSITION_KI_Q4_4:
    case REG_POSITION_KD_Q4_4:
    case REG_POSITION_MAX_RPM_0:
    case REG_POSITION_MAX_RPM_1:
    case REG_POSITION_TOLERANCE_0:
    case REG_POSITION_TOLERANCE_1:
        return true;
    default:
        return false;
    }
}

uint8_t RegisterMap::MotorStatus(bool motor1,
                                 uint8_t mode,
                                 uint8_t duty,
                                 uint8_t direction) const
{
    uint8_t status = 0U;

    if (IsEnabled() && (!timeout_active_) &&
        (((mode == MOTOR_MODE_RUN) && (duty > 0U)) ||
         (mode == MOTOR_MODE_SPEED) ||
         (mode == MOTOR_MODE_POSITION))) {
        status = static_cast<uint8_t>(status | MOTOR_STATUS_ACTIVE);
    }

    if (direction == MOTOR_DIRECTION_REVERSE) {
        status = static_cast<uint8_t>(status | MOTOR_STATUS_REVERSE);
    }

    if (PositionAtTarget(motor1)) {
        status = static_cast<uint8_t>(status | MOTOR_STATUS_AT_TARGET);
    }

    return status;
}

}  // namespace protocol
