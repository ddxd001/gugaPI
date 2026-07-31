#ifndef GUGAH_CONTROL_DM_ACTUATOR_H_
#define GUGAH_CONTROL_DM_ACTUATOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "control/control_types.h"
#include "drivers/common/driver_status.h"

namespace gugah {

drivers::DriverStatus DmActuator_Init(void);
drivers::DriverStatus DmActuator_Enable(void);
drivers::DriverStatus DmActuator_Disable(void);
drivers::DriverStatus DmActuator_ClearError(void);
drivers::DriverStatus DmActuator_HoldCurrent(void);
drivers::DriverStatus DmActuator_SetPosition(int32_t position_mrad);
void DmActuator_Update(uint32_t now_ms);
DmFeedback DmActuator_GetFeedback(void);
bool DmActuator_IsReady(void);
uint32_t DmActuator_GetErrorCount(void);
uint32_t DmActuator_GetBusyCount(void);
int32_t DmActuator_GetTargetPosition(void);
int32_t DmActuator_GetReferencePosition(void);
bool DmActuator_IsEnabling(void);

} /* namespace gugah */

#endif /* GUGAH_CONTROL_DM_ACTUATOR_H_ */
