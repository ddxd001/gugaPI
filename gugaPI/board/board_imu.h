#ifndef BOARD_BOARD_IMU_H_
#define BOARD_BOARD_IMU_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "drivers/icm45686/icm45686.h"

namespace board {

struct BoardImuLineStatus {
    bool icm_cs_high;
    bool lis_cs_high;
    bool icm_cs_latch_high;
    bool lis_cs_latch_high;
    bool icm_cs_output_enabled;
    bool lis_cs_output_enabled;
    bool icm_int1_high;
    bool icm_int2_fsync_high;
    bool lis_drdy_high;
};

drivers::DriverStatus Board_ImuInit(void);
bool Board_ImuIsReady(void);
drivers::DriverStatus Board_ImuSetChipSelectDebug(bool icm_output_enable,
                                                  bool icm_high,
                                                  bool lis_output_enable,
                                                  bool lis_high);
drivers::DriverStatus Board_ImuSetSpiMode(uint8_t mode);
drivers::DriverStatus Board_ImuWiggleSpiPins(uint32_t loops);
drivers::DriverStatus Board_ImuSpiBurstIcm(uint32_t bytes, uint8_t value);
drivers::DriverStatus Board_ImuSpiSampleIcm(uint8_t tx,
                                            uint8_t *rx,
                                            uint32_t count);
drivers::DriverStatus Board_ImuGetLineStatus(BoardImuLineStatus *status);
drivers::DriverStatus Board_Lis3mdlReadRegister(uint8_t reg, uint8_t *value);
drivers::DriverStatus Board_Lis3mdlReadWhoAmI(uint8_t *value);

/* ICM-45686 device-level access (delegated to drivers::icm45686). */
drivers::DriverStatus Board_Icm45686Init(void);
bool Board_Icm45686IsReady(void);
drivers::DriverStatus Board_Icm45686ReadRegister(uint8_t reg, uint8_t *value);
drivers::DriverStatus Board_Icm45686WriteRegister(uint8_t reg, uint8_t value);
drivers::DriverStatus Board_Icm45686ReadBurst(uint8_t reg,
                                              uint8_t *buf,
                                              uint16_t len);
drivers::DriverStatus Board_Icm45686ReadWhoAmI(uint8_t *value);
drivers::DriverStatus Board_Icm45686ReadSensors(drivers::Icm45686SensorData *data);
const drivers::Icm45686Config *Board_Icm45686GetConfig(void);

} /* namespace board */

#endif /* BOARD_BOARD_IMU_H_ */
