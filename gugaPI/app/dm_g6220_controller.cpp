#include "app/dm_g6220_controller.h"

#include "app/config_store.h"
#include "board/board_can.h"
#include "config/feature_config.h"
#include "services/fault.h"
#include "services/time.h"

namespace app {
namespace {

static const uint16_t kMotorCanId = 0x001U;
static const uint16_t kMasterCanId = 0x000U;
static const uint8_t kMotorId = 0x01U;
static const uint32_t kBootDelayMs = 1000U;
static const uint32_t kProbeIntervalMs = 50U;
static const uint32_t kProbeTimeoutMs = 500U;
static const uint32_t kSpecialIntervalMs = 20U;
static const uint8_t kSpecialRepeatCount = 3U;
static const uint32_t kEnableTimeoutMs = 1000U;
static const uint32_t kSpeedStopTimeoutMs = 1000U;
static const uint32_t kControlPeriodMs = 10U;

drivers::DmG6220Context g_protocol = {};
DmG6220ControlState g_state = {};
DmG6220ControlMode g_afterBurstMode = DM_CONTROL_READY;
DmG6220ControlMode g_enableTargetMode = DM_CONTROL_HOLD;
drivers::DmG6220Command g_specialCommand =
    drivers::DM_G6220_COMMAND_DISABLE;
uint32_t g_bootStartMs = 0U;
uint32_t g_lastTxMs = 0U;
uint32_t g_lastControlMs = 0U;
uint32_t g_probeStartMs = 0U;
uint32_t g_speedStopStartMs = 0U;
uint8_t g_enableSendCount = 0U;
int32_t g_positionMaxVelocityMradS = 0;
bool g_settlePending = false;
bool g_emergencyBurst = false;

enum PendingMotion : uint8_t {
    PENDING_MOTION_NONE = 0U,
    PENDING_MOTION_HOLD,
    PENDING_MOTION_POSITION,
    PENDING_MOTION_SPEED
};

struct PendingMotionRequest {
    PendingMotion kind;
    DmG6220PositionFrame position_frame;
    int32_t target;
    int32_t max_velocity_mrad_s;
    uint32_t timeout_ms;
};

PendingMotionRequest g_pendingMotion = {};
uint32_t g_enableStartMs = 0U;

int32_t AbsInt32(int32_t value)
{
    return (value < 0) ? -value : value;
}

int32_t Clamp(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

int32_t StepToward(int32_t current, int32_t target, int32_t step)
{
    if (current < target) {
        return (target - current <= step) ? target : current + step;
    }
    if (current > target) {
        return (current - target <= step) ? target : current - step;
    }
    return current;
}

const ConfigStoreParams *Params(void)
{
    return ConfigStore_Get();
}

bool IsMotionMode(DmG6220ControlMode mode)
{
    return (mode == DM_CONTROL_ENABLING) ||
           (mode == DM_CONTROL_HOLD) ||
           (mode == DM_CONTROL_POSITION) ||
           (mode == DM_CONTROL_SPEED) ||
           (mode == DM_CONTROL_SPEED_STOPPING);
}

bool CanAcceptMotionCommand(void)
{
    return (g_state.mode == DM_CONTROL_READY) ||
           (g_state.mode == DM_CONTROL_OFFLINE) ||
           (g_state.mode == DM_CONTROL_HOLD);
}

bool FeedbackFresh(uint32_t now_ms)
{
    const drivers::DmG6220Feedback *feedback =
        drivers::DmG6220_GetFeedback(&g_protocol);
    const ConfigStoreParams *params = Params();
    return (feedback != 0) && feedback->valid && (params != 0) &&
           ((now_ms - feedback->last_update_ms) <=
            params->dm_feedback_timeout_ms);
}

drivers::DriverStatus Send(const drivers::CanFrame &frame)
{
    const drivers::DriverStatus status = board::Board_CanSend(&frame);
    g_state.last_status = status;
    if (status == drivers::DRIVER_OK) {
        g_state.tx_count++;
        g_lastTxMs = services::Time_Millis();
    } else if (status == drivers::DRIVER_ERROR_BUSY) {
        g_state.tx_busy_count++;
    } else {
        g_state.tx_error_count++;
    }
    return status;
}

drivers::DriverStatus SendSpecial(drivers::DmG6220Command command)
{
    drivers::CanFrame frame = {};
    const drivers::DriverStatus prepare =
        drivers::DmG6220_PrepareSpecial(&g_protocol, command, &frame);
    return (prepare == drivers::DRIVER_OK) ? Send(frame) : prepare;
}

drivers::DriverStatus SendMit(int32_t position_mrad,
                              int32_t velocity_mrad_s,
                              int32_t kp_milli,
                              int32_t kd_milli)
{
    drivers::CanFrame frame = {};
    const drivers::DriverStatus prepare = drivers::DmG6220_PrepareMit(
        &g_protocol, position_mrad, velocity_mrad_s,
        kp_milli, kd_milli, 0, &frame);
    return (prepare == drivers::DRIVER_OK) ? Send(frame) : prepare;
}

void BeginBurst(drivers::DmG6220Command command,
                DmG6220ControlMode mode,
                DmG6220ControlMode after_mode,
                bool emergency)
{
    g_specialCommand = command;
    g_state.mode = mode;
    g_afterBurstMode = after_mode;
    g_state.special_remaining = kSpecialRepeatCount;
    g_lastTxMs = 0U;
    g_emergencyBurst = emergency;
}

void StartProbe(bool invalidate_feedback)
{
    if (invalidate_feedback) {
        g_protocol.feedback.valid = false;
    }
    g_state.mode = DM_CONTROL_PROBING;
    g_probeStartMs = services::Time_Millis();
    g_lastTxMs = 0U;
    g_state.probe_count++;
}

void SetGlobalFault(services::FaultCode code)
{
    g_pendingMotion = {};
    g_state.enabled = false;
    g_state.operation_result = DM_OPERATION_FAULT;
    g_state.last_status = drivers::DRIVER_ERROR_TIMEOUT;
    BeginBurst(drivers::DM_G6220_COMMAND_DISABLE,
               DM_CONTROL_DISABLING,
               DM_CONTROL_FAULT_LOCKED,
               true);
    services::Fault_Set(code);
}

bool CheckActiveSafety(uint32_t now_ms)
{
    const drivers::DmG6220Feedback *feedback =
        drivers::DmG6220_GetFeedback(&g_protocol);
    if (((g_state.mode == DM_CONTROL_READY) ||
         (g_state.mode == DM_CONTROL_OFFLINE) ||
         IsMotionMode(g_state.mode)) &&
        FeedbackFresh(now_ms) && (feedback != 0) &&
        (feedback->state >= 8U) && (feedback->state <= 14U)) {
        SetGlobalFault(services::FAULT_DM_MOTOR);
        return false;
    }
    if (!IsMotionMode(g_state.mode)) {
        return true;
    }
    drivers::CanStatus can_status = {};
    if ((board::Board_CanGetStatus(&can_status) == drivers::DRIVER_OK) &&
        can_status.bus_off) {
        SetGlobalFault(services::FAULT_DM_TIMEOUT);
        return false;
    }
    if (!FeedbackFresh(now_ms)) {
        SetGlobalFault(services::FAULT_DM_TIMEOUT);
        return false;
    }
    return true;
}

void ServiceBurst(uint32_t now_ms)
{
    if (g_state.special_remaining == 0U) {
        return;
    }
    if ((g_lastTxMs != 0U) &&
        ((now_ms - g_lastTxMs) < kSpecialIntervalMs)) {
        return;
    }
    const drivers::DriverStatus status = SendSpecial(g_specialCommand);
    if (status == drivers::DRIVER_OK) {
        g_state.special_remaining--;
        if (g_state.special_remaining == 0U) {
            const bool was_disable =
                g_specialCommand == drivers::DM_G6220_COMMAND_DISABLE;
            g_state.mode = g_afterBurstMode;
            if (was_disable && !g_emergencyBurst &&
                (g_state.operation_result == DM_OPERATION_RUNNING)) {
                g_state.operation_result = DM_OPERATION_SUCCESS;
            }
            g_emergencyBurst = false;
        }
    } else if ((status != drivers::DRIVER_ERROR_BUSY) &&
               !g_emergencyBurst) {
        SetGlobalFault(services::FAULT_DM_TIMEOUT);
    }
}

void EnterEnabledMode(DmG6220ControlMode mode)
{
    g_state.enabled = true;
    g_state.mode = mode;
    g_lastControlMs = services::Time_Millis();
    g_settlePending = false;
}

void UpdateEnabling(uint32_t now_ms)
{
    const drivers::DmG6220Feedback *feedback =
        drivers::DmG6220_GetFeedback(&g_protocol);
    if ((feedback != 0) && feedback->valid && (feedback->state == 1U)) {
        EnterEnabledMode(g_enableTargetMode);
        return;
    }
    if ((now_ms - g_enableStartMs) > kEnableTimeoutMs) {
        SetGlobalFault(services::FAULT_DM_TIMEOUT);
        return;
    }
    if ((g_enableSendCount < kSpecialRepeatCount) &&
        ((g_lastTxMs == 0U) ||
         ((now_ms - g_lastTxMs) >= kSpecialIntervalMs))) {
        const drivers::DriverStatus status =
            SendSpecial(drivers::DM_G6220_COMMAND_ENABLE);
        if (status == drivers::DRIVER_OK) {
            g_enableSendCount++;
        } else if (status != drivers::DRIVER_ERROR_BUSY) {
            SetGlobalFault(services::FAULT_DM_TIMEOUT);
        }
        return;
    }
    const ConfigStoreParams *params = Params();
    if (params != 0) {
        const drivers::DriverStatus status = SendMit(
            g_state.reference_position_mrad, 0,
            params->dm_position_kp_milli,
            params->dm_position_kd_milli);
        if ((status != drivers::DRIVER_OK) &&
            (status != drivers::DRIVER_ERROR_BUSY)) {
            SetGlobalFault(services::FAULT_DM_TIMEOUT);
        }
    }
}

void UpdatePosition(uint32_t now_ms, bool hold_only)
{
    const ConfigStoreParams *params = Params();
    const drivers::DmG6220Feedback *feedback =
        drivers::DmG6220_GetFeedback(&g_protocol);
    if ((params == 0) || (feedback == 0)) {
        SetGlobalFault(services::FAULT_DM_TIMEOUT);
        return;
    }
    uint32_t elapsed = now_ms - g_lastControlMs;
    if (elapsed == 0U) {
        elapsed = kControlPeriodMs;
    }
    int32_t step = static_cast<int32_t>(
        (static_cast<int64_t>(g_positionMaxVelocityMradS) * elapsed) /
        1000LL);
    if (step < 1) {
        step = 1;
    }
    int32_t next_reference = StepToward(
        g_state.reference_position_mrad,
        g_state.target_position_mrad,
        step);
    next_reference = Clamp(
        next_reference,
        feedback->position_mrad -
            static_cast<int32_t>(params->dm_max_tracking_error_mrad),
        feedback->position_mrad +
            static_cast<int32_t>(params->dm_max_tracking_error_mrad));
    const drivers::DriverStatus send_status = SendMit(
        next_reference, 0,
        params->dm_position_kp_milli,
        params->dm_position_kd_milli);
    g_lastControlMs = now_ms;
    if (send_status == drivers::DRIVER_ERROR_BUSY) {
        return;
    }
    if (send_status != drivers::DRIVER_OK) {
        SetGlobalFault(services::FAULT_DM_TIMEOUT);
        return;
    }
    g_state.reference_position_mrad = next_reference;

    if (hold_only) {
        return;
    }
    if ((now_ms - g_state.operation_start_ms) >
        g_state.operation_timeout_ms) {
        g_state.reference_position_mrad = feedback->position_mrad;
        g_state.target_position_mrad = feedback->position_mrad;
        g_state.mode = DM_CONTROL_HOLD;
        g_state.operation_result = DM_OPERATION_TARGET_TIMEOUT;
        g_settlePending = false;
        return;
    }
    const bool settled =
        (AbsInt32(g_state.target_position_mrad -
                  feedback->position_mrad) <=
         static_cast<int32_t>(params->dm_position_tolerance_mrad)) &&
        (AbsInt32(feedback->velocity_mrad_s) <=
         static_cast<int32_t>(params->dm_velocity_tolerance_mrad_s));
    if (!settled) {
        g_settlePending = false;
        g_state.settle_start_ms = 0U;
        return;
    }
    if (!g_settlePending) {
        g_settlePending = true;
        g_state.settle_start_ms = now_ms;
        return;
    }
    if ((now_ms - g_state.settle_start_ms) >= params->dm_settle_ms) {
        g_state.reference_position_mrad = g_state.target_position_mrad;
        g_state.mode = DM_CONTROL_HOLD;
        g_state.operation_result = DM_OPERATION_SUCCESS;
        g_settlePending = false;
    }
}

void UpdateSpeed(uint32_t now_ms, bool stopping)
{
    const ConfigStoreParams *params = Params();
    const drivers::DmG6220Feedback *feedback =
        drivers::DmG6220_GetFeedback(&g_protocol);
    if ((params == 0) || (feedback == 0)) {
        SetGlobalFault(services::FAULT_DM_TIMEOUT);
        return;
    }
    uint32_t elapsed = now_ms - g_lastControlMs;
    if (elapsed == 0U) {
        elapsed = kControlPeriodMs;
    }
    int32_t step = static_cast<int32_t>(
        (static_cast<int64_t>(params->dm_speed_slew_mrad_s2) *
         elapsed) / 1000LL);
    if (step < 1) {
        step = 1;
    }
    const int32_t next_velocity = StepToward(
        g_state.reference_velocity_mrad_s,
        g_state.target_velocity_mrad_s,
        step);
    const drivers::DriverStatus send_status = SendMit(
        0, next_velocity, 0, params->dm_speed_kd_milli);
    g_lastControlMs = now_ms;
    if (send_status == drivers::DRIVER_ERROR_BUSY) {
        return;
    }
    if (send_status != drivers::DRIVER_OK) {
        SetGlobalFault(services::FAULT_DM_TIMEOUT);
        return;
    }
    g_state.reference_velocity_mrad_s = next_velocity;
    if (!stopping) {
        return;
    }

    const bool stopped =
        (g_state.reference_velocity_mrad_s == 0) &&
        (AbsInt32(feedback->velocity_mrad_s) <=
         static_cast<int32_t>(params->dm_velocity_tolerance_mrad_s));
    if (stopped) {
        if (!g_settlePending) {
            g_settlePending = true;
            g_state.settle_start_ms = now_ms;
        } else if ((now_ms - g_state.settle_start_ms) >=
                   params->dm_settle_ms) {
            g_state.reference_position_mrad = feedback->position_mrad;
            g_state.target_position_mrad = feedback->position_mrad;
            g_positionMaxVelocityMradS =
                params->dm_max_velocity_mrad_s;
            g_state.mode = DM_CONTROL_HOLD;
            g_state.operation_result = DM_OPERATION_SUCCESS;
            g_settlePending = false;
        }
        return;
    }
    g_settlePending = false;
    if ((now_ms - g_speedStopStartMs) > kSpeedStopTimeoutMs) {
        g_state.reference_position_mrad = feedback->position_mrad;
        g_state.target_position_mrad = feedback->position_mrad;
        g_positionMaxVelocityMradS = params->dm_max_velocity_mrad_s;
        g_state.mode = DM_CONTROL_HOLD;
        g_state.operation_result = DM_OPERATION_STOP_TIMEOUT;
    }
}

void BeginEnable(DmG6220ControlMode target_mode)
{
    g_enableStartMs = services::Time_Millis();
    const drivers::DmG6220Feedback *feedback =
        drivers::DmG6220_GetFeedback(&g_protocol);
    g_enableTargetMode = target_mode;
    g_enableSendCount = 0U;
    g_lastTxMs = 0U;
    g_state.enabled = false;
    g_state.mode = DM_CONTROL_ENABLING;
    if (feedback != 0) {
        g_state.reference_position_mrad = feedback->position_mrad;
    }
}

void StartPendingProbe(const PendingMotionRequest &request)
{
    g_pendingMotion = request;
    g_state.operation_result = DM_OPERATION_RUNNING;
    StartProbe(true);
}

void StartHoldWithFeedback(uint32_t now_ms)
{
    const ConfigStoreParams *params = Params();
    const drivers::DmG6220Feedback *feedback =
        drivers::DmG6220_GetFeedback(&g_protocol);
    g_state.reference_position_mrad = feedback->position_mrad;
    g_state.target_position_mrad = feedback->position_mrad;
    g_positionMaxVelocityMradS = params->dm_max_velocity_mrad_s;
    g_state.operation_start_ms = now_ms;
    g_state.operation_result = DM_OPERATION_RUNNING;
    if (feedback->state == 1U) {
        g_state.operation_result = DM_OPERATION_SUCCESS;
        EnterEnabledMode(DM_CONTROL_HOLD);
    } else {
        BeginEnable(DM_CONTROL_HOLD);
    }
}

bool StartPositionWithFeedback(const PendingMotionRequest &request,
                               uint32_t now_ms)
{
    const drivers::DmG6220Feedback *feedback =
        drivers::DmG6220_GetFeedback(&g_protocol);
    const int32_t final_target =
        (request.position_frame == DM_POSITION_RELATIVE) ?
            feedback->position_mrad + request.target : request.target;
    if ((final_target < -drivers::DM_G6220_POSITION_LIMIT_MRAD) ||
        (final_target > drivers::DM_G6220_POSITION_LIMIT_MRAD)) {
        g_state.mode = DM_CONTROL_READY;
        g_state.operation_result = DM_OPERATION_TARGET_TIMEOUT;
        return false;
    }
    g_state.reference_position_mrad = feedback->position_mrad;
    g_state.target_position_mrad = final_target;
    g_positionMaxVelocityMradS = request.max_velocity_mrad_s;
    g_state.operation_start_ms = now_ms;
    g_state.operation_timeout_ms = request.timeout_ms;
    g_state.operation_result = DM_OPERATION_RUNNING;
    if (feedback->state == 1U) {
        EnterEnabledMode(DM_CONTROL_POSITION);
    } else {
        BeginEnable(DM_CONTROL_POSITION);
    }
    return true;
}

void StartSpeedWithFeedback(int32_t velocity_mrad_s, uint32_t now_ms)
{
    const drivers::DmG6220Feedback *feedback =
        drivers::DmG6220_GetFeedback(&g_protocol);
    g_state.reference_position_mrad = feedback->position_mrad;
    g_state.reference_velocity_mrad_s = 0;
    g_state.target_velocity_mrad_s = velocity_mrad_s;
    g_state.operation_start_ms = now_ms;
    g_state.operation_result = DM_OPERATION_RUNNING;
    if (feedback->state == 1U) {
        EnterEnabledMode(DM_CONTROL_SPEED);
    } else {
        BeginEnable(DM_CONTROL_SPEED);
    }
}

void ResumePendingMotion(uint32_t now_ms)
{
    const PendingMotionRequest request = g_pendingMotion;
    g_pendingMotion = {};
    switch (request.kind) {
    case PENDING_MOTION_HOLD:
        StartHoldWithFeedback(now_ms);
        break;
    case PENDING_MOTION_POSITION:
        (void) StartPositionWithFeedback(request, now_ms);
        break;
    case PENDING_MOTION_SPEED:
        StartSpeedWithFeedback(request.target, now_ms);
        break;
    case PENDING_MOTION_NONE:
    default:
        g_state.mode = DM_CONTROL_READY;
        if (g_state.operation_result == DM_OPERATION_RUNNING) {
            g_state.operation_result = DM_OPERATION_SUCCESS;
        }
        break;
    }
}

void UpdateIdleFeedback(uint32_t now_ms)
{
    if (FeedbackFresh(now_ms)) {
        g_state.mode = DM_CONTROL_READY;
    }
    if ((g_lastTxMs == 0U) ||
        ((now_ms - g_lastTxMs) >= kProbeIntervalMs)) {
        (void) SendMit(0, 0, 0, 0);
    }
}

} /* namespace */

void DmG6220Controller_Init(void)
{
    g_state = {};
    const drivers::DmG6220Config config = {
        kMotorCanId, kMasterCanId, kMotorId
    };
    g_state.last_status = drivers::DmG6220_Init(&g_protocol, &config);
    g_state.initialized = g_state.last_status == drivers::DRIVER_OK;
    g_state.mode = g_state.initialized ?
        DM_CONTROL_BOOT_WAIT : DM_CONTROL_FAULT_LOCKED;
    g_state.operation_result = DM_OPERATION_IDLE;
    g_bootStartMs = services::Time_Millis();
    g_lastTxMs = 0U;
    g_lastControlMs = g_bootStartMs;
    g_positionMaxVelocityMradS = 200;
    g_pendingMotion = {};
    g_enableStartMs = 0U;
}

void DmG6220Controller_Update(void)
{
#if FEATURE_ENABLE_CAN && FEATURE_ENABLE_DM_G6220_CAN
    if (!g_state.initialized) {
        return;
    }
    const uint32_t now_ms = services::Time_Millis();
    if (services::Fault_HasFault() && IsMotionMode(g_state.mode)) {
        DmG6220Controller_EmergencyDisable();
    }
    if (!CheckActiveSafety(now_ms)) {
        return;
    }
    if (g_state.special_remaining != 0U) {
        ServiceBurst(now_ms);
        return;
    }

    switch (g_state.mode) {
    case DM_CONTROL_BOOT_WAIT:
        if ((now_ms - g_bootStartMs) >= kBootDelayMs) {
            BeginBurst(drivers::DM_G6220_COMMAND_CLEAR_ERROR,
                       DM_CONTROL_BOOT_CLEAR,
                       DM_CONTROL_BOOT_DISABLE,
                       false);
        }
        break;
    case DM_CONTROL_BOOT_DISABLE:
        BeginBurst(drivers::DM_G6220_COMMAND_DISABLE,
                   DM_CONTROL_BOOT_DISABLE,
                   DM_CONTROL_PROBING,
                   false);
        g_probeStartMs = now_ms;
        break;
    case DM_CONTROL_PROBING:
        if (FeedbackFresh(now_ms)) {
            const drivers::DmG6220Feedback *feedback =
                drivers::DmG6220_GetFeedback(&g_protocol);
            if ((feedback != 0) && (feedback->state >= 8U) &&
                (feedback->state <= 14U)) {
                SetGlobalFault(services::FAULT_DM_MOTOR);
            } else {
                ResumePendingMotion(now_ms);
            }
        } else if ((now_ms - g_probeStartMs) > kProbeTimeoutMs) {
            if (g_pendingMotion.kind != PENDING_MOTION_NONE) {
                SetGlobalFault(services::FAULT_DM_TIMEOUT);
            } else {
                g_state.mode = DM_CONTROL_OFFLINE;
            }
            if ((g_pendingMotion.kind == PENDING_MOTION_NONE) &&
                (g_state.operation_result == DM_OPERATION_RUNNING)) {
                g_state.operation_result = DM_OPERATION_TARGET_TIMEOUT;
            }
        } else if ((g_lastTxMs == 0U) ||
                   ((now_ms - g_lastTxMs) >= kProbeIntervalMs)) {
            (void) SendMit(0, 0, 0, 0);
        }
        break;
    case DM_CONTROL_OFFLINE:
    case DM_CONTROL_READY:
        UpdateIdleFeedback(now_ms);
        break;
    case DM_CONTROL_ENABLING:
        UpdateEnabling(now_ms);
        break;
    case DM_CONTROL_POSITION:
        UpdatePosition(now_ms, false);
        break;
    case DM_CONTROL_HOLD:
        UpdatePosition(now_ms, true);
        break;
    case DM_CONTROL_SPEED:
        UpdateSpeed(now_ms, false);
        break;
    case DM_CONTROL_SPEED_STOPPING:
        UpdateSpeed(now_ms, true);
        break;
    case DM_CONTROL_ZEROING:
        BeginBurst(drivers::DM_G6220_COMMAND_SET_ZERO,
                   DM_CONTROL_ZEROING,
                   DM_CONTROL_PROBING,
                   false);
        g_protocol.feedback.valid = false;
        g_probeStartMs = now_ms;
        break;
    case DM_CONTROL_BOOT_CLEAR:
    case DM_CONTROL_DISABLING:
    case DM_CONTROL_FAULT_LOCKED:
    default:
        break;
    }
#endif
}

drivers::DmG6220FrameType DmG6220Controller_ProcessFrame(
    const drivers::CanFrame *frame,
    uint32_t now_ms)
{
    return drivers::DmG6220_ProcessFrame(&g_protocol, frame, now_ms);
}

drivers::DriverStatus DmG6220Controller_Probe(void)
{
    if (!g_state.initialized ||
        ((g_state.mode != DM_CONTROL_READY) &&
         (g_state.mode != DM_CONTROL_OFFLINE))) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    g_pendingMotion = {};
    g_state.operation_result = DM_OPERATION_RUNNING;
    StartProbe(true);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus DmG6220Controller_EnableHold(void)
{
    if (!CanAcceptMotionCommand()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    const uint32_t now_ms = services::Time_Millis();
    if (services::Fault_HasFault()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (!FeedbackFresh(now_ms)) {
        if (g_state.mode == DM_CONTROL_HOLD) {
            SetGlobalFault(services::FAULT_DM_TIMEOUT);
            return drivers::DRIVER_ERROR_TIMEOUT;
        }
        const PendingMotionRequest request = {
            PENDING_MOTION_HOLD, DM_POSITION_ABSOLUTE, 0, 0, 0U
        };
        StartPendingProbe(request);
        return drivers::DRIVER_OK;
    }
    StartHoldWithFeedback(now_ms);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus DmG6220Controller_StartPosition(
    DmG6220PositionFrame frame,
    int32_t target_mrad,
    int32_t max_velocity_mrad_s,
    uint32_t timeout_ms)
{
    if (!CanAcceptMotionCommand()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    const uint32_t now_ms = services::Time_Millis();
    const ConfigStoreParams *params = Params();
    if (services::Fault_HasFault()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if ((params == 0) ||
        ((frame != DM_POSITION_ABSOLUTE) &&
         (frame != DM_POSITION_RELATIVE)) ||
        (max_velocity_mrad_s <= 0) ||
        (max_velocity_mrad_s >
         static_cast<int32_t>(params->dm_max_velocity_mrad_s)) ||
        (timeout_ms < 50U) || (timeout_ms > 30000U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if ((frame == DM_POSITION_ABSOLUTE) &&
        ((target_mrad < -drivers::DM_G6220_POSITION_LIMIT_MRAD) ||
         (target_mrad > drivers::DM_G6220_POSITION_LIMIT_MRAD))) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    const PendingMotionRequest request = {
        PENDING_MOTION_POSITION, frame, target_mrad,
        max_velocity_mrad_s, timeout_ms
    };
    if (!FeedbackFresh(now_ms)) {
        if (g_state.mode == DM_CONTROL_HOLD) {
            SetGlobalFault(services::FAULT_DM_TIMEOUT);
            return drivers::DRIVER_ERROR_TIMEOUT;
        }
        StartPendingProbe(request);
        return drivers::DRIVER_OK;
    }
    (void) StartPositionWithFeedback(request, now_ms);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus DmG6220Controller_StartSpeed(
    int32_t velocity_mrad_s)
{
    if (!CanAcceptMotionCommand()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    const uint32_t now_ms = services::Time_Millis();
    const ConfigStoreParams *params = Params();
    if (services::Fault_HasFault()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if ((params == 0) || (velocity_mrad_s == 0) ||
        (AbsInt32(velocity_mrad_s) >
         static_cast<int32_t>(params->dm_max_velocity_mrad_s))) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (!FeedbackFresh(now_ms)) {
        if (g_state.mode == DM_CONTROL_HOLD) {
            SetGlobalFault(services::FAULT_DM_TIMEOUT);
            return drivers::DRIVER_ERROR_TIMEOUT;
        }
        const PendingMotionRequest request = {
            PENDING_MOTION_SPEED, DM_POSITION_ABSOLUTE,
            velocity_mrad_s, 0, 0U
        };
        StartPendingProbe(request);
        return drivers::DRIVER_OK;
    }
    StartSpeedWithFeedback(velocity_mrad_s, now_ms);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus DmG6220Controller_StopSpeedAndHold(void)
{
    if ((g_state.mode != DM_CONTROL_SPEED) &&
        (g_state.mode != DM_CONTROL_SPEED_STOPPING)) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (g_state.mode == DM_CONTROL_SPEED) {
        g_state.target_velocity_mrad_s = 0;
        g_state.mode = DM_CONTROL_SPEED_STOPPING;
        g_state.operation_result = DM_OPERATION_RUNNING;
        g_speedStopStartMs = services::Time_Millis();
        g_settlePending = false;
    }
    return drivers::DRIVER_OK;
}

drivers::DriverStatus DmG6220Controller_HoldCurrent(void)
{
    const uint32_t now_ms = services::Time_Millis();
    if (!CanAcceptMotionCommand()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (services::Fault_HasFault()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (!FeedbackFresh(now_ms)) {
        if (g_state.mode == DM_CONTROL_HOLD) {
            SetGlobalFault(services::FAULT_DM_TIMEOUT);
            return drivers::DRIVER_ERROR_TIMEOUT;
        }
        const PendingMotionRequest request = {
            PENDING_MOTION_HOLD, DM_POSITION_ABSOLUTE, 0, 0, 0U
        };
        StartPendingProbe(request);
        return drivers::DRIVER_OK;
    }
    StartHoldWithFeedback(now_ms);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus DmG6220Controller_Disable(void)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    g_pendingMotion = {};
    g_state.enabled = false;
    g_state.operation_result = DM_OPERATION_RUNNING;
    BeginBurst(drivers::DM_G6220_COMMAND_DISABLE,
               DM_CONTROL_DISABLING,
               FeedbackFresh(services::Time_Millis()) ?
                   DM_CONTROL_READY : DM_CONTROL_OFFLINE,
               false);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus DmG6220Controller_ClearError(void)
{
    if (!g_state.initialized || IsMotionMode(g_state.mode) ||
        services::Fault_HasFault()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    g_pendingMotion = {};
    g_state.operation_result = DM_OPERATION_RUNNING;
    BeginBurst(drivers::DM_G6220_COMMAND_CLEAR_ERROR,
               DM_CONTROL_BOOT_CLEAR,
               DM_CONTROL_BOOT_DISABLE,
               false);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus DmG6220Controller_SetZero(void)
{
    const uint32_t now_ms = services::Time_Millis();
    const drivers::DmG6220Feedback *feedback =
        drivers::DmG6220_GetFeedback(&g_protocol);
    if (!g_state.initialized || services::Fault_HasFault() ||
        (g_state.mode != DM_CONTROL_READY) || !FeedbackFresh(now_ms) ||
        (feedback == 0) || (feedback->state != 0U) ||
        (AbsInt32(feedback->velocity_mrad_s) > 50)) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    g_state.operation_result = DM_OPERATION_RUNNING;
    g_state.mode = DM_CONTROL_ZEROING;
    return drivers::DRIVER_OK;
}

void DmG6220Controller_EmergencyDisable(void)
{
    if (!g_state.initialized ||
        ((g_state.mode == DM_CONTROL_DISABLING) && g_emergencyBurst) ||
        (g_state.mode == DM_CONTROL_FAULT_LOCKED)) {
        return;
    }
    g_pendingMotion = {};
    g_state.enabled = false;
    BeginBurst(drivers::DM_G6220_COMMAND_DISABLE,
               DM_CONTROL_DISABLING,
               services::Fault_HasFault() ?
                   DM_CONTROL_FAULT_LOCKED : DM_CONTROL_READY,
               true);
}

bool DmG6220Controller_IsFeedbackFresh(uint32_t now_ms)
{
    return FeedbackFresh(now_ms);
}

bool DmG6220Controller_IsTxReserved(void)
{
    return IsMotionMode(g_state.mode) ||
           (g_state.special_remaining != 0U) ||
           (g_state.mode == DM_CONTROL_PROBING) ||
           (g_state.mode == DM_CONTROL_ZEROING);
}

const DmG6220ControlState *DmG6220Controller_GetState(void)
{
    return &g_state;
}

const drivers::DmG6220Feedback *DmG6220Controller_GetFeedback(void)
{
    return drivers::DmG6220_GetFeedback(&g_protocol);
}

const char *DmG6220Controller_ModeText(DmG6220ControlMode mode)
{
    switch (mode) {
    case DM_CONTROL_BOOT_WAIT: return "boot_wait";
    case DM_CONTROL_BOOT_CLEAR: return "boot_clear";
    case DM_CONTROL_BOOT_DISABLE: return "boot_disable";
    case DM_CONTROL_PROBING: return "probing";
    case DM_CONTROL_OFFLINE: return "offline";
    case DM_CONTROL_READY: return "ready";
    case DM_CONTROL_ENABLING: return "enabling";
    case DM_CONTROL_HOLD: return "hold";
    case DM_CONTROL_POSITION: return "position";
    case DM_CONTROL_SPEED: return "speed";
    case DM_CONTROL_SPEED_STOPPING: return "speed_stopping";
    case DM_CONTROL_DISABLING: return "disabling";
    case DM_CONTROL_ZEROING: return "zeroing";
    case DM_CONTROL_FAULT_LOCKED: return "fault";
    default: return "unknown";
    }
}

} /* namespace app */
