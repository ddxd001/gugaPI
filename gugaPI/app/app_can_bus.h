#ifndef APP_APP_CAN_BUS_H_
#define APP_APP_CAN_BUS_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/can/can.h"

namespace app {

void AppCanBus_Init(void);
void AppCanBus_Update(void);
bool AppCanBus_ReadRaw(drivers::CanFrame *frame);
uint16_t AppCanBus_GetRawAvailable(void);
uint32_t AppCanBus_GetRawDropped(void);
void AppCanBus_Clear(void);

} /* namespace app */

#endif /* APP_APP_CAN_BUS_H_ */
