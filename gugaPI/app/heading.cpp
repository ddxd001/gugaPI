#include "app/heading.h"

#include "app/app_imu.h"
#include "app/chassis.h"
#include "app/config_store.h"
#include "drivers/common/driver_status.h"
#include "services/fault.h"
#include "services/time.h"

namespace app {
namespace {

/* Maps "increase yaw" to the left/right wheel command. Default +1 assumes
 * right-wheel-faster increases yaw; if bench testing shows hold/turn drives
 * the wrong way, set to -1. Confirmed against the actual robot + IMU axis
 * orientation during bring-up. */
static const int32_t kYawSign = 1;

static const int32_t kMaxErrorMdeg = 90000;   /* 90 deg -> stop (something wrong) */
static const int32_t kStaleUpdateMs = 200;    /* task starved -> IMU likely stale */
static const int32_t kTurnTimeoutMs = 8000;
static const int32_t kScale = 1000000;        /* correction_rpm = error_mdeg * kp / kScale */

HeadingState g_state = {
    HEADING_IDLE, 0, 0, 0, 0, false, 0, 0, 0, drivers::DRIVER_OK
};

int32_t AbsInt32(int32_t v)
{
    if (v < 0) {
        return (v == INT32_MIN) ? INT32_MAX : -v;
    }
    return v;
}

int32_t ClampInt32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/* Shortest signed angle from current to target, wrapped to [-180000, 180000]
 * milli-degrees. Handles the -180/180 seam. */
int32_t ShortestAngleDiff(int32_t target_mdeg, int32_t current_mdeg)
{
    int32_t d = target_mdeg - current_mdeg;
    while (d > 180000) {
        d -= 360000;
    }
    while (d <= -180000) {
        d += 360000;
    }
    return d;
}

int32_t WrapToSigned180(int32_t angle_mdeg)
{
    while (angle_mdeg > 180000) {
        angle_mdeg -= 360000;
    }
    while (angle_mdeg <= -180000) {
        angle_mdeg += 360000;
    }
    return angle_mdeg;
}

/* correction_rpm = error_mdeg * kp / kScale, in 64-bit to avoid overflow. */
int32_t GainToRpm(int32_t error_mdeg, int32_t kp)
{
    return static_cast<int32_t>(
        (static_cast<int64_t>(error_mdeg) * static_cast<int64_t>(kp)) /
        static_cast<int64_t>(kScale));
}

void SafetyStop(services::FaultCode code)
{
    g_state.mode = HEADING_IDLE;
    (void) Chassis_Stop();
    if (code != services::FAULT_NONE) {
        services::Fault_Set(code);
    }
}

} /* namespace */

void Heading_Init(void)
{
    g_state.mode = HEADING_IDLE;
    g_state.target_yaw_mdeg = 0;
    g_state.base_rpm = 0;
    g_state.correction_rpm = 0;
    g_state.error_mdeg = 0;
    g_state.at_target = false;
    g_state.at_target_since_ms = 0U;
    g_state.turn_start_ms = 0U;
    g_state.last_run_ms = 0U;
    g_state.last_status = drivers::DRIVER_OK;
}

drivers::DriverStatus Heading_HoldStart(int32_t base_rpm)
{
    if (services::Fault_HasFault()) {
        g_state.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    const AppImuData *imu = App_ImuGetData();
    if ((imu == 0) || (!imu->valid)) {
        g_state.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    g_state.mode = HEADING_HOLD;
    g_state.target_yaw_mdeg = imu->yaw_mdeg;   /* lock current heading */
    g_state.base_rpm = base_rpm;
    g_state.correction_rpm = 0;
    g_state.error_mdeg = 0;
    g_state.at_target = false;
    g_state.at_target_since_ms = 0U;
    g_state.last_run_ms = services::Time_Millis();
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Heading_TurnStart(int32_t delta_deg)
{
    if ((delta_deg < -180) || (delta_deg > 180)) {
        g_state.last_status = drivers::DRIVER_ERROR_INVALID_ARG;
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    if (services::Fault_HasFault()) {
        g_state.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    const AppImuData *imu = App_ImuGetData();
    if ((imu == 0) || (!imu->valid)) {
        g_state.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    const int32_t delta_mdeg = delta_deg * 1000;
    g_state.mode = HEADING_TURN;
    g_state.target_yaw_mdeg = WrapToSigned180(imu->yaw_mdeg + delta_mdeg);
    g_state.base_rpm = 0;
    g_state.correction_rpm = 0;
    g_state.error_mdeg = delta_mdeg;
    g_state.at_target = false;
    g_state.at_target_since_ms = 0U;
    g_state.turn_start_ms = services::Time_Millis();
    g_state.last_run_ms = g_state.turn_start_ms;
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Heading_Stop(void)
{
    g_state.mode = HEADING_IDLE;
    const drivers::DriverStatus s = Chassis_Stop();
    g_state.last_status = s;
    return s;
}

void Heading_Update(void)
{
    if (g_state.mode == HEADING_IDLE) {
        return;
    }

    const uint32_t now = services::Time_Millis();

    /* Scheduler-cadence watchdog: if this task was starved (e.g. scheduler
     * overload), the IMU data is likely stale -> stop and fault. */
    if (g_state.last_run_ms != 0U) {
        const int32_t since = static_cast<int32_t>(now - g_state.last_run_ms);
        if (since > kStaleUpdateMs) {
            g_state.last_run_ms = now;
            SafetyStop(services::FAULT_SENSOR_LOST);
            return;
        }
    }
    g_state.last_run_ms = now;

    if (services::Fault_HasFault()) {
        SafetyStop(services::FAULT_NONE);
        return;
    }

    const AppImuData *imu = App_ImuGetData();
    if ((imu == 0) || (!imu->valid)) {
        SafetyStop(services::FAULT_SENSOR_LOST);
        return;
    }

    const int32_t yaw = imu->yaw_mdeg;
    const int32_t error = ShortestAngleDiff(g_state.target_yaw_mdeg, yaw);
    g_state.error_mdeg = error;

    const ConfigStoreParams *params = ConfigStore_Get();
    if (params == 0) {
        SafetyStop(services::FAULT_UNKNOWN);
        return;
    }

    if (g_state.mode == HEADING_HOLD) {
        if (AbsInt32(error) > kMaxErrorMdeg) {
            SafetyStop(services::FAULT_SENSOR_LOST);
            return;
        }
        int32_t correction = GainToRpm(error, params->heading_kp);
        correction = ClampInt32(correction,
                                 -params->heading_max_correction_rpm,
                                 params->heading_max_correction_rpm);
        correction *= kYawSign;
        g_state.correction_rpm = correction;
        const int32_t left = g_state.base_rpm - correction;
        const int32_t right = g_state.base_rpm + correction;
        const drivers::DriverStatus s = Chassis_SetWheelRpm(left, right);
        g_state.last_status = s;
        if (s != drivers::DRIVER_OK) {
            SafetyStop(services::FAULT_NONE);
        }
        return;
    }

    if (g_state.mode == HEADING_TURN) {
        if (static_cast<int32_t>(now - g_state.turn_start_ms) > kTurnTimeoutMs) {
            SafetyStop(services::FAULT_DRIVER_TIMEOUT);
            return;
        }

        const int32_t abs_err = AbsInt32(error);
        if (abs_err < params->heading_tolerance_mdeg) {
            if (!g_state.at_target) {
                g_state.at_target = true;
                g_state.at_target_since_ms = now;
            }
            if (static_cast<int32_t>(now - g_state.at_target_since_ms) >=
                static_cast<int32_t>(params->heading_settle_ms)) {
                (void) Heading_Stop();
                return;
            }
            /* coast while settling */
            g_state.last_status = Chassis_Stop();
            return;
        }

        g_state.at_target = false;
        int32_t speed = GainToRpm(abs_err, params->heading_kp);
        speed = ClampInt32(speed,
                           params->heading_turn_min_rpm,
                           params->heading_turn_max_rpm);
        speed *= kYawSign;
        g_state.correction_rpm = speed;
        const int32_t dir = (error >= 0) ? 1 : -1;
        const int32_t left = -speed * dir;
        const int32_t right = speed * dir;
        const drivers::DriverStatus s = Chassis_SetWheelRpm(left, right);
        g_state.last_status = s;
        if (s != drivers::DRIVER_OK) {
            SafetyStop(services::FAULT_NONE);
        }
        return;
    }
}

const HeadingState *Heading_GetState(void)
{
    return &g_state;
}

} /* namespace app */
