#ifndef DRIVERS_I2C_SLAVE_H_
#define DRIVERS_I2C_SLAVE_H_

#include "protocol/motor_protocol.h"

void I2cSlave_init(MotorRegisterMap* registers, uint8_t address);
bool I2cSlave_consumeWriteActivity(void);

#endif  // DRIVERS_I2C_SLAVE_H_
