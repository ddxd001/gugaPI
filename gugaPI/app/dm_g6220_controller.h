#ifndef APP_DM_G6220_CONTROLLER_H_
#define APP_DM_G6220_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/can/can.h"
#include "drivers/common/driver_status.h"
#include "drivers/dm_g6220/dm_g6220.h"

namespace app {

enum DmG6220ControlMode : uint8_t {
    DM_CONTROL_BOOT_WAIT = 0U,
    DM_CONTROL_BOOT_CLEAR,
    DM_CONTROL_BOOT_DISABLE,
    DM_CONTROL_PROBING,
    DM_CONTROL_OFFLINE,
    DM_CONTROL_READY,
    DM_CONTROL_ENABLING,
    DM_CONTROL_HOLD,
    DM_CONTROL_POSITION,
    DM_CONTROL_EXTERNAL_POSITION,
    DM_CONTROL_SPEED,
    DM_CONTROL_SPEED_STOPPING,
    DM_CONTROL_DISABLING,
    DM_CONTROL_ZEROING,
    DM_CONTROL_FAULT_LOCKED
};

enum DmG6220OperationResult : uint8_t {
    DM_OPERATION_IDLE = 0U,
    DM_OPERATION_RUNNING,
    DM_OPERATION_SUCCESS,
    DM_OPERATION_TARGET_TIMEOUT,
    DM_OPERATION_STOP_TIMEOUT,
    DM_OPERATION_FAULT
};

enum DmG6220PositionFrame : uint8_t {
    DM_POSITION_ABSOLUTE = 0U,
    DM_POSITION_RELATIVE
};

enum DmG6220ExternalOwner : uint8_t {
    DM_EXTERNAL_OWNER_NONE = 0U,
    DM_EXTERNAL_OWNER_BALL_BALANCE
};

struct DmG6220ControlState {
    bool initialized;
    DmG6220ControlMode mode;
    DmG6220OperationResult operation_result;
    drivers::DriverStatus last_status;
    int32_t reference_position_mrad;
    int32_t target_position_mrad;
    int32_t reference_velocity_mrad_s;
    int32_t target_velocity_mrad_s;
    uint32_t operation_start_ms;
    uint32_t operation_timeout_ms;
    uint32_t settle_start_ms;
    uint32_t tx_count;
    uint32_t tx_busy_count;
    uint32_t tx_error_count;
    uint32_t probe_count;
    uint8_t special_remaining;
    bool enabled;
    DmG6220ExternalOwner external_owner;
};

void DmG6220Controller_Init(void);
void DmG6220Controller_Update(void);
drivers::DmG6220FrameType DmG6220Controller_ProcessFrame(
    const drivers::CanFrame *frame,
    uint32_t now_ms);
drivers::DriverStatus DmG6220Controller_Probe(void);
drivers::DriverStatus DmG6220Controller_EnableHold(void);
drivers::DriverStatus DmG6220Controller_StartPosition(
    DmG6220PositionFrame frame,
    int32_t target_mrad,
    int32_t max_velocity_mrad_s,
    uint32_t timeout_ms);
drivers::DriverStatus DmG6220Controller_StartSpeed(
    int32_t velocity_mrad_s);
drivers::DriverStatus DmG6220Controller_StopSpeedAndHold(void);
drivers::DriverStatus DmG6220Controller_HoldCurrent(void);
drivers::DriverStatus DmG6220Controller_Disable(void);
drivers::DriverStatus DmG6220Controller_ClearError(void);
drivers::DriverStatus DmG6220Controller_SetZero(void);
drivers::DriverStatus DmG6220Controller_ExternalAcquire(
    DmG6220ExternalOwner owner);
drivers::DriverStatus DmG6220Controller_ExternalSetPosition(
    DmG6220ExternalOwner owner,
    int32_t target_mrad);
drivers::DriverStatus DmG6220Controller_ExternalRelease(
    DmG6220ExternalOwner owner,
    bool disable);
void DmG6220Controller_EmergencyDisable(void);
bool DmG6220Controller_IsFeedbackFresh(uint32_t now_ms);
bool DmG6220Controller_IsTxReserved(void);
const DmG6220ControlState *DmG6220Controller_GetState(void);
const drivers::DmG6220Feedback *DmG6220Controller_GetFeedback(void);
const char *DmG6220Controller_ModeText(DmG6220ControlMode mode);

} /* namespace app */

#endif /* APP_DM_G6220_CONTROLLER_H_ */
