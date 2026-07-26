#ifndef BOARD_BOARD_GY931_H_
#define BOARD_BOARD_GY931_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "drivers/gy931/gy931.h"

namespace board {

drivers::DriverStatus Board_Gy931Init(void);
drivers::DriverStatus Board_Gy931Probe(void);
drivers::DriverStatus Board_Gy931ProbeAddress(uint8_t address);
drivers::DriverStatus Board_Gy931SetAddress(uint8_t address);
drivers::DriverStatus Board_Gy931RecoverBus(void);
drivers::DriverStatus Board_Gy931GetBusStatus(bool *scl_high,
                                              bool *sda_high);
drivers::DriverStatus Board_Gy931ReadAngles(drivers::Gy931Angles *angles);
drivers::DriverStatus Board_Gy931ReadSample(drivers::Gy931Sample *sample);
drivers::DriverStatus Board_Gy931ReadRawRegisters(uint8_t start_reg,
                                                  int16_t *words,
                                                  uint8_t word_count);
drivers::DriverStatus Board_Gy931ReadAlgorithm(
    drivers::Gy931Algorithm *algorithm);
drivers::DriverStatus Board_Gy931SetAlgorithmTemporary(
    drivers::Gy931Algorithm algorithm);
bool Board_Gy931IsReady(void);
uint8_t Board_Gy931Address(void);

} /* namespace board */

#endif /* BOARD_BOARD_GY931_H_ */
