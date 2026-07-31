#ifndef GUGAH_CONTROL_CHASSIS_H_
#define GUGAH_CONTROL_CHASSIS_H_

#include <stdbool.h>
#include <stdint.h>

#include "control/control_types.h"
#include "drivers/common/driver_status.h"

namespace gugah {

struct HConfig;

drivers::DriverStatus Chassis_Init(const HConfig *config, uint32_t now_ms);
drivers::DriverStatus Chassis_SetWheelRpm(int16_t left_rpm,
                                          int16_t right_rpm,
                                          uint32_t now_ms);
drivers::DriverStatus Chassis_SetWheelPosition(int32_t left_count,
                                               int32_t right_count,
                                               uint32_t now_ms);
drivers::DriverStatus Chassis_Stop(uint32_t now_ms);
drivers::DriverStatus Chassis_Update(uint32_t now_ms);
ChassisFeedback Chassis_GetFeedback(void);
bool Chassis_IsReady(void);
uint32_t Chassis_GetErrorCount(void);

} /* namespace gugah */

#endif /* GUGAH_CONTROL_CHASSIS_H_ */
