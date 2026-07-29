#ifndef BOARD_BOARD_INFRARED_SENSOR_H_
#define BOARD_BOARD_INFRARED_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace board {

drivers::DriverStatus Board_InfraredSensorInit(void);
bool Board_InfraredSensorReadByte(uint8_t *data);
void Board_InfraredSensorServiceRx(void);
bool Board_InfraredSensorIsReady(void);
uint32_t Board_InfraredSensorGetDroppedCount(void);
uint32_t Board_InfraredSensorGetUartErrorCount(void);
uint32_t Board_InfraredSensorGetRxTimeoutCount(void);
uint32_t Board_InfraredSensorGetOverrunErrorCount(void);
uint32_t Board_InfraredSensorGetFramingErrorCount(void);
uint32_t Board_InfraredSensorGetParityErrorCount(void);
uint32_t Board_InfraredSensorGetNoiseErrorCount(void);
uint32_t Board_InfraredSensorGetIrqCount(void);
uint32_t Board_InfraredSensorGetFifoByteCount(void);
uint32_t Board_InfraredSensorGetPolledByteCount(void);
bool Board_InfraredSensorIsPowered(void);
bool Board_InfraredSensorIsUartEnabled(void);
bool Board_InfraredSensorIsRxPinHigh(void);
bool Board_InfraredSensorIsRxFifoEmpty(void);
void Board_InfraredSensorClear(void);
void Board_InfraredSensorIrqHandler(void);

} /* namespace board */

#endif /* BOARD_BOARD_INFRARED_SENSOR_H_ */
