#ifndef BOARD_BOARD_I2C_TARGET_H_
#define BOARD_BOARD_I2C_TARGET_H_

#include <stdint.h>

#include "protocol/motor_registers.h"
#include "protocol/register_map.h"

namespace board {

struct BoardI2cTargetWrite {
    uint8_t reg;
    uint8_t length;
    uint8_t data[protocol::kMaxPayloadLength];
};

void BoardI2cTarget_Init(const protocol::RegisterMap *registers);
bool BoardI2cTarget_Process(uint32_t now_ms);
bool BoardI2cTarget_TakeWrite(BoardI2cTargetWrite *write);
uint8_t BoardI2cTarget_Address(void);
void BoardI2cTarget_SetAddress(uint8_t address);
uint32_t BoardI2cTarget_DroppedWrites(void);
uint32_t BoardI2cTarget_ErrorFlags(void);
uint32_t BoardI2cTarget_RecoveryCount(void);
void BoardI2cTarget_IrqHandler(void);

}  // namespace board

#endif  // BOARD_BOARD_I2C_TARGET_H_
