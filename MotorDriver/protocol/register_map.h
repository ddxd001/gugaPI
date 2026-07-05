#ifndef PROTOCOL_REGISTER_MAP_H_
#define PROTOCOL_REGISTER_MAP_H_

#include <stdint.h>

#include "protocol/motor_registers.h"

namespace protocol {

class RegisterMap {
public:
    void Init(void);

    uint8_t Read(uint8_t reg) const;
    bool ReadBlock(uint8_t start_reg, uint8_t length, uint8_t *out) const;
    bool CommitWrite(uint8_t start_reg,
                     const uint8_t *data,
                     uint8_t length,
                     bool *control_changed);

    void SetTimeoutActive(bool active);
    void ForceCoastOnTimeout(void);
    void RefreshStatus(void);
    void UpdateEncoderSnapshot(int32_t m1_count,
                               int32_t m1_counts_per_second,
                               uint8_t m1_state,
                               int32_t m2_count,
                               int32_t m2_counts_per_second,
                               uint8_t m2_state);
    void UpdateMeasuredRpm(int16_t m1_rpm, int16_t m2_rpm);
    void SetMotorDutyFromControl(bool motor1, uint8_t duty);
    uint8_t EncoderControl(void) const;
    void ClearEncoderControl(void);

    bool IsEnabled(void) const;
    uint8_t WatchdogTimeout10ms(void) const;
    uint16_t M1TargetRpm(void) const;
    uint16_t M2TargetRpm(void) const;
    uint8_t SpeedKpQ4_4(void) const;
    uint8_t SpeedKiQ4_4(void) const;
    uint8_t SpeedKdQ4_4(void) const;
    uint8_t SpeedMaxDuty(void) const;
    uint8_t SpeedMinDuty(void) const;

private:
    uint16_t LoadUint16(uint8_t reg) const;
    void StoreInt16(uint8_t reg, int16_t value);
    void StoreInt32(uint8_t reg, int32_t value);
    bool ApplyWriteTo(uint8_t *target, uint8_t reg, uint8_t value) const;
    bool IsReadableRange(uint8_t start_reg, uint8_t length) const;
    bool IsControlRegister(uint8_t reg) const;
    uint8_t MotorStatus(uint8_t mode, uint8_t duty, uint8_t direction) const;

    uint8_t registers_[kRegisterCount];
    bool timeout_active_;
};

}  // namespace protocol

#endif  // PROTOCOL_REGISTER_MAP_H_
