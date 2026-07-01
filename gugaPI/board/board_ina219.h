#ifndef BOARD_BOARD_INA219_H_
#define BOARD_BOARD_INA219_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "drivers/ina219/ina219.h"

namespace board {

drivers::DriverStatus Board_Ina219Init(void);
drivers::DriverStatus Board_Ina219Reset(void);
drivers::DriverStatus Board_Ina219ConfigureDefault(void);
drivers::DriverStatus Board_Ina219SetAddress(uint8_t address);
drivers::DriverStatus Board_Ina219ProbeAddress(uint8_t address,
                                               uint16_t *config_value);
drivers::DriverStatus Board_Ina219ReadMeasurement(
    drivers::Ina219Measurement *measurement);
drivers::DriverStatus Board_Ina219ReadRawRegisters(
    drivers::Ina219RawRegisters *raw);
drivers::DriverStatus Board_Ina219ReadRegister(uint8_t reg, uint16_t *value);
drivers::DriverStatus Board_Ina219WriteRegister(uint8_t reg, uint16_t value);
drivers::DriverStatus Board_Ina219RecoverBus(void);
drivers::DriverStatus Board_Ina219GetBusStatus(uint32_t *controller_status,
                                               bool *scl_high,
                                               bool *sda_high);
bool Board_Ina219IsReady(void);
uint16_t Board_Ina219Calibration(void);
uint8_t Board_Ina219GetAddress(void);

} /* namespace board */

#endif /* BOARD_BOARD_INA219_H_ */