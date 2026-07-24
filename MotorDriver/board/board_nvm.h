#ifndef BOARD_BOARD_NVM_H_
#define BOARD_BOARD_NVM_H_

#include <stdint.h>

namespace board {

uint8_t BoardNvm_LoadI2cAddress(void);
bool BoardNvm_SaveI2cAddress(uint8_t address);

}  // namespace board

#endif  // BOARD_BOARD_NVM_H_
