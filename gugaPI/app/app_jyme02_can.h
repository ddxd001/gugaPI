#ifndef APP_APP_JYME02_CAN_H_
#define APP_APP_JYME02_CAN_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/can/can.h"
#include "drivers/common/driver_status.h"
#include "drivers/jyme02_can/jyme02_can.h"

namespace app {

void AppJyme02Can_Init(void);
void AppJyme02Can_Update(void);
const drivers::JYME02CanData *AppJyme02Can_GetData(void);
bool AppJyme02Can_IsMeasurementFresh(uint32_t now_ms);
bool AppJyme02Can_IsTemperatureFresh(uint32_t now_ms);
drivers::DriverStatus AppJyme02Can_SetAddress(uint16_t address);
drivers::DriverStatus AppJyme02Can_SetSampleTime(uint16_t sample_time_100us);
drivers::DriverStatus AppJyme02Can_ReadRegister(uint8_t register_address);
bool AppJyme02Can_ReadRaw(drivers::CanFrame *frame);
uint16_t AppJyme02Can_GetRawAvailable(void);
uint32_t AppJyme02Can_GetRawDropped(void);
void AppJyme02Can_Clear(void);

} /* namespace app */

#endif /* APP_APP_JYME02_CAN_H_ */
