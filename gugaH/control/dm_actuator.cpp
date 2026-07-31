#include "control/dm_actuator.h"

#include "board/board_can.h"
#include "drivers/dm_g6220/dm_g6220.h"

namespace gugah {
namespace {

static const int32_t kPositionKpMilli = 4000;
static const int32_t kPositionKdMilli = 400;
/* The original 2 rad/s commissioning limit dominated the 100 Hz ball loop
 * and made a normal 0.5..0.7 rad correction take several tenths of a second.
 * Four rad/s follows the configured beam slew while retaining the
 * 250 mrad tracking-error guard. */
static const int32_t kNormalVelocityMradS = 4000;
static const int32_t kCatchupVelocityMradS = 6000;
static const int32_t kCatchupThresholdMrad = 250;
static const int32_t kMaximumTrackingErrorMrad = 300;
static const uint32_t kControlPeriodMs = 2U;
static const uint32_t kEnableSendIntervalMs = 20U;
static const uint32_t kClearSettleMs = 100U;
static const uint32_t kEnableTimeoutMs = 1000U;
static const uint32_t kFeedbackTimeoutMs = 120U;
static const uint8_t kEnableSendCount = 3U;

enum EnableStage : uint8_t {
    ENABLE_STAGE_IDLE = 0U,
    ENABLE_STAGE_CLEARING,
    ENABLE_STAGE_SENDING
};

const drivers::DmG6220Config g_config = { 0x001U, 0x000U, 1U };
drivers::DmG6220Context g_context = {};
bool g_ready = false;
bool g_enabled = false;
bool g_enable_requested = false;
EnableStage g_enable_stage = ENABLE_STAGE_IDLE;
uint8_t g_enable_sent = 0U;
uint32_t g_enable_start_ms = 0U;
uint32_t g_last_enable_send_ms = 0U;
uint32_t g_error_count = 0U;
uint32_t g_busy_count = 0U;
int32_t g_target_position_mrad = 0;
int32_t g_reference_position_mrad = 0;
uint32_t g_last_control_ms = 0U;
bool g_target_valid = false;
bool g_reference_valid = false;
drivers::DmG6220Command g_burst_command =
    drivers::DM_G6220_COMMAND_DISABLE;
uint8_t g_burst_remaining = 0U;
uint32_t g_last_burst_send_ms = 0U;
bool g_fault_latched = false;

int32_t Clamp32(int32_t value, int32_t minimum, int32_t maximum)
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
        const int32_t remaining = target - current;
        return current + ((remaining <= step) ? remaining : step);
    }
    if (current > target) {
        const int32_t remaining = current - target;
        return current - ((remaining <= step) ? remaining : step);
    }
    return current;
}

void CountSendStatus(drivers::DriverStatus status)
{
    if (status == drivers::DRIVER_ERROR_BUSY) {
        g_busy_count++;
    } else if (status != drivers::DRIVER_OK) {
        g_error_count++;
    }
}

drivers::DriverStatus SendSpecial(drivers::DmG6220Command command)
{
    drivers::CanFrame frame = {};
    drivers::DriverStatus status =
        drivers::DmG6220_PrepareSpecial(&g_context, command, &frame);
    if (status == drivers::DRIVER_OK) {
        status = board::Board_CanSend(&frame);
    }
    CountSendStatus(status);
    return status;
}

drivers::DriverStatus SendPosition(int32_t position_mrad)
{
    drivers::CanFrame frame = {};
    drivers::DriverStatus status = drivers::DmG6220_PrepareMit(
        &g_context, position_mrad, 0, kPositionKpMilli,
        kPositionKdMilli, 0, &frame);
    if (status == drivers::DRIVER_OK) {
        status = board::Board_CanSend(&frame);
    }
    CountSendStatus(status);
    return status;
}

void ScheduleBurst(drivers::DmG6220Command command)
{
    g_burst_command = command;
    g_burst_remaining = kEnableSendCount;
    g_last_burst_send_ms = 0U;
}

void LatchFaultAndDisable(void)
{
    if (!g_fault_latched) {
        g_error_count++;
    }
    g_fault_latched = true;
    g_enabled = false;
    g_enable_requested = false;
    g_enable_stage = ENABLE_STAGE_IDLE;
    g_enable_sent = 0U;
    g_target_valid = false;
    g_reference_valid = false;
    ScheduleBurst(drivers::DM_G6220_COMMAND_DISABLE);
}

} /* namespace */

drivers::DriverStatus DmActuator_Init(void)
{
    drivers::DriverStatus status = board::Board_CanInit();
    if (status == drivers::DRIVER_OK) {
        status = drivers::DmG6220_Init(&g_context, &g_config);
    }
    g_ready = (status == drivers::DRIVER_OK);
    g_enabled = false;
    g_enable_requested = false;
    g_enable_stage = ENABLE_STAGE_IDLE;
    g_enable_sent = 0U;
    g_enable_start_ms = 0U;
    g_last_enable_send_ms = 0U;
    g_target_valid = false;
    g_reference_valid = false;
    g_last_control_ms = 0U;
    g_error_count = 0U;
    g_busy_count = 0U;
    g_burst_remaining = 0U;
    g_last_burst_send_ms = 0U;
    g_fault_latched = false;
    return status;
}

drivers::DriverStatus DmActuator_Enable(void)
{
    if (!g_ready) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    /* Clear a fault left latched by the previous power cycle before the
     * three enable requests.  State 13 otherwise rejects every enable frame. */
    g_enabled = false;
    g_enable_requested = true;
    g_enable_stage = ENABLE_STAGE_CLEARING;
    g_enable_sent = 0U;
    g_enable_start_ms = 0U;
    g_last_enable_send_ms = 0U;
    ScheduleBurst(drivers::DM_G6220_COMMAND_CLEAR_ERROR);
    g_reference_valid = false;
    g_last_control_ms = 0U;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus DmActuator_Disable(void)
{
    if (!g_ready) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    /* Stop issuing MIT frames now; the scheduler retries all disable frames. */
    g_enabled = false;
    g_enable_requested = false;
    g_enable_stage = ENABLE_STAGE_IDLE;
    g_enable_sent = 0U;
    g_target_valid = false;
    g_reference_valid = false;
    ScheduleBurst(drivers::DM_G6220_COMMAND_DISABLE);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus DmActuator_ClearError(void)
{
    if (!g_ready || g_enabled || g_enable_requested) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    g_fault_latched = false;
    ScheduleBurst(drivers::DM_G6220_COMMAND_CLEAR_ERROR);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus DmActuator_HoldCurrent(void)
{
    const drivers::DmG6220Feedback *feedback =
        drivers::DmG6220_GetFeedback(&g_context);
    if (!DmActuator_IsReady() || (feedback == 0) || !feedback->valid) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    return DmActuator_SetPosition(feedback->position_mrad);
}

drivers::DriverStatus DmActuator_SetPosition(int32_t position_mrad)
{
    if (!g_ready || (!g_enabled && !g_enable_requested)) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if ((position_mrad < -drivers::DM_G6220_POSITION_LIMIT_MRAD) ||
        (position_mrad > drivers::DM_G6220_POSITION_LIMIT_MRAD)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    /* BallBalance updates the target; this 500 Hz layer owns CAN pacing. */
    g_target_position_mrad = position_mrad;
    g_target_valid = true;
    return drivers::DRIVER_OK;
}

void DmActuator_Update(uint32_t now_ms)
{
    drivers::CanFrame frame = {};
    while (board::Board_CanRead(&frame)) {
        (void)drivers::DmG6220_ProcessFrame(
            &g_context, &frame, now_ms);
    }
    const drivers::DmG6220Feedback *feedback =
        drivers::DmG6220_GetFeedback(&g_context);

    if (g_enabled && (feedback != 0) && feedback->valid) {
        const bool motor_fault =
            (feedback->state >= 8U) && (feedback->state <= 14U);
        const bool feedback_timeout =
            (now_ms - feedback->last_update_ms) > kFeedbackTimeoutMs;
        if (motor_fault || feedback_timeout) {
            LatchFaultAndDisable();
        }
    }

    if ((g_burst_remaining != 0U) &&
        ((g_last_burst_send_ms == 0U) ||
         ((now_ms - g_last_burst_send_ms) >=
          kEnableSendIntervalMs))) {
        const drivers::DriverStatus status =
            SendSpecial(g_burst_command);
        if (status == drivers::DRIVER_OK) {
            g_burst_remaining--;
            g_last_burst_send_ms = now_ms;
        }
    }

    if (g_enable_requested) {
        if (g_enable_stage == ENABLE_STAGE_CLEARING) {
            if ((g_burst_remaining == 0U) &&
                (g_last_burst_send_ms != 0U) &&
                ((now_ms - g_last_burst_send_ms) >= kClearSettleMs)) {
                g_enable_stage = ENABLE_STAGE_SENDING;
                g_enable_start_ms = now_ms;
                g_last_enable_send_ms = 0U;
                g_enable_sent = 0U;
            }
            return;
        }
        if ((feedback != 0) && feedback->valid &&
            (feedback->state == 1U)) {
            g_enabled = true;
            g_enable_requested = false;
            g_enable_stage = ENABLE_STAGE_IDLE;
            g_reference_position_mrad = feedback->position_mrad;
            g_reference_valid = true;
            if (!g_target_valid) {
                g_target_position_mrad = feedback->position_mrad;
                g_target_valid = true;
            }
            g_last_control_ms = now_ms;
        } else {
            if (g_enable_start_ms == 0U) {
                g_enable_start_ms = now_ms;
            }
            if ((now_ms - g_enable_start_ms) > kEnableTimeoutMs) {
                g_enable_requested = false;
                g_enable_stage = ENABLE_STAGE_IDLE;
                g_error_count++;
                return;
            }
            if ((g_enable_sent < kEnableSendCount) &&
                ((g_last_enable_send_ms == 0U) ||
                 ((now_ms - g_last_enable_send_ms) >=
                  kEnableSendIntervalMs))) {
                const drivers::DriverStatus status =
                    SendSpecial(drivers::DM_G6220_COMMAND_ENABLE);
                if (status == drivers::DRIVER_OK) {
                    g_enable_sent++;
                    g_last_enable_send_ms = now_ms;
                }
            }
            return;
        }
    }

    if (!g_enabled || !g_target_valid || (feedback == 0) ||
        !feedback->valid ||
        ((now_ms - g_last_control_ms) < kControlPeriodMs)) {
        return;
    }
    if (!g_reference_valid) {
        g_reference_position_mrad = feedback->position_mrad;
        g_reference_valid = true;
    }
    uint32_t elapsed_ms = now_ms - g_last_control_ms;
    if ((g_last_control_ms == 0U) || (elapsed_ms > 100U)) {
        elapsed_ms = kControlPeriodMs;
    }
    const int32_t reference_error =
        g_target_position_mrad - g_reference_position_mrad;
    const int32_t maximum_velocity =
        ((reference_error > kCatchupThresholdMrad) ||
         (reference_error < -kCatchupThresholdMrad))
        ? kCatchupVelocityMradS
        : kNormalVelocityMradS;
    int32_t step = static_cast<int32_t>(
        (static_cast<int64_t>(maximum_velocity) * elapsed_ms) /
        1000LL);
    if (step < 1) {
        step = 1;
    }
    int32_t next_reference = StepToward(
        g_reference_position_mrad, g_target_position_mrad, step);
    next_reference = Clamp32(
        next_reference,
        feedback->position_mrad - kMaximumTrackingErrorMrad,
        feedback->position_mrad + kMaximumTrackingErrorMrad);
    const drivers::DriverStatus status = SendPosition(next_reference);
    g_last_control_ms = now_ms;
    if (status == drivers::DRIVER_OK) {
        g_reference_position_mrad = next_reference;
    }
}

DmFeedback DmActuator_GetFeedback(void)
{
    DmFeedback result = {};
    const drivers::DmG6220Feedback *feedback =
        drivers::DmG6220_GetFeedback(&g_context);
    if (feedback != 0) {
        result.valid = feedback->valid;
        result.position_mrad = feedback->position_mrad;
        result.velocity_mrad_s = feedback->velocity_mrad_s;
        result.torque_mnm = feedback->torque_mnm;
        result.state = feedback->state;
        result.mos_temperature_c = feedback->mos_temperature_c;
        result.coil_temperature_c = feedback->coil_temperature_c;
        result.received_ms = feedback->last_update_ms;
    }
    return result;
}

bool DmActuator_IsReady(void)
{
    return g_ready && g_enabled;
}

uint32_t DmActuator_GetErrorCount(void)
{
    return g_error_count;
}

uint32_t DmActuator_GetBusyCount(void)
{
    return g_busy_count;
}

int32_t DmActuator_GetTargetPosition(void)
{
    return g_target_position_mrad;
}

int32_t DmActuator_GetReferencePosition(void)
{
    return g_reference_position_mrad;
}

bool DmActuator_IsEnabling(void)
{
    return g_enable_requested;
}

} /* namespace gugah */
