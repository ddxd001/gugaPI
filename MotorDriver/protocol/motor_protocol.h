#ifndef PROTOCOL_MOTOR_PROTOCOL_H_
#define PROTOCOL_MOTOR_PROTOCOL_H_

#include <stdint.h>
#include "protocol/motor_registers.h"

class MotorRegisterMap {
public:
    void init(uint8_t i2cAddress);
    uint8_t read(uint8_t reg) const;
    void write(uint8_t reg, uint8_t value);
    void set(uint8_t reg, uint8_t value);
    const uint8_t* raw() const;

private:
    uint8_t registers_[MotorProtocol::kRegisterCount] = {};
};

#endif  // PROTOCOL_MOTOR_PROTOCOL_H_
