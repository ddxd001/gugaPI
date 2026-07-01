#ifndef BOARD_BOARD_LORA_H_
#define BOARD_BOARD_LORA_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace board {

drivers::DriverStatus Board_LoraInit(void);
drivers::DriverStatus Board_LoraWriteByte(uint8_t data);
drivers::DriverStatus Board_LoraWrite(const uint8_t *data, uint16_t length);
bool Board_LoraReadByte(uint8_t *data);
uint16_t Board_LoraGetRxAvailable(void);
uint32_t Board_LoraGetRxDroppedCount(void);
uint32_t Board_LoraGetBaudrate(void);
drivers::DriverStatus Board_LoraGetControllerStatus(uint32_t *status);
drivers::DriverStatus Board_LoraClearRxBuffer(void);
drivers::DriverStatus Board_LoraGetLineStatus(bool *tx_high, bool *rx_high);
bool Board_LoraIsReady(void);
void Board_LoraIrqHandler(void);

} /* namespace board */

#endif /* BOARD_BOARD_LORA_H_ */