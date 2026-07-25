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
static const int32_t kDistanceMaxMm = 10000;
static const int32_t kDistanceToleranceMm = 3;
static const int32_t kDistanceMinRpm = 15;
static const uint32_t kDistanceSettleMs = 100U;
static const uint32_t kFeedbackStaleMs = 100U;
static const uint32_t kDistanceMinTimeoutMs = 500U;
static const uint32_t kDistanceMaxTimeoutMs = 60000U;
static const int64_t kPiMicro = 3141593LL;

HeadingState g_state = {};

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

int32_t DivideRoundInt64(int64_t numerator, int64_t denominator)
{
    if (denominator <= 0) {
        return 0;
    }
    if (numerator >= 0) {
        numerator += denominator / 2;
    } else {
        numerator -= denominator / 2;
    }
    const int64_t result = numerator / denominator;
    if (result > INT32_MAX) {
        return INT32_MAX;
    }
    if (result < INT32_MIN) {
        return INT32_MIN;
    }
    return static_cast<int32_t>(result);
}

int32_t EncoderDelta(int32_t current, int32_t start)
{
    const uint32_t raw = static_cast<uint32_t>(current) -
                         static_cast<uint32_t>(start);
    if (raw <= static_cast<uint32_t>(INT32_MAX)) {
        return static_cast<int32_t>(raw);
    }
    return -1 - static_cast<int32_t>(UINT32_MAX - raw);
}

bool MillimetersToCounts(int32_t distance_mm,
                         uint32_t wheel_radius_mm,
                         uint32_t counts_per_rev,
                         int32_t *counts)
{
    if ((counts == 0) || (wheel_radius_mm == 0U) ||
        (counts_per_rev == 0U)) {
        return false;
    }
    const int64_t numerator =
        static_cast<int64_t>(distance_mm) *
        static_cast<int64_t>(counts_per_rev) * 1000000LL;
    const int64_t denominator =
        2LL * kPiMicro * static_cast<int64_t>(wheel_radius_mm);
    const int64_t rounded = (numerator >= 0)
        ? ((numerator + denominator / 2LL) / denominator)
        : ((numerator - denominator / 2LL) / denominator);
    if ((rounded > INT32_MAX) || (rounded < INT32_MIN)) {
        return false;
    }
    *counts = static_cast<int32_t>(rounded);
    return true;
}

int32_t CountsToMillimeters(int32_t counts,
                            uint32_t wheel_radius_mm,
                            uint32_t counts_per_rev)
{
    if ((wheel_radius_mm == 0U) || (counts_per_rev == 0U)) {
        return 0;
    }
    const int64_t numerator =
        static_cast<int64_t>(counts) * 2LL * kPiMicro *
        static_cast<int64_t>(wheel_radius_mm);
    const int64_t denominator =
        static_cast<int64_t>(counts_per_rev) * 1000000LL;
    return DivideRoundInt64(numerator, denominator);
}

bool IsFeedbackFresh(const ChassisState *chassis, uint32_t now_ms)
{
    return (chassis != 0) && chassis->initialized &&
           (chassis->last_feedback_status == drivers::DRIVER_OK) &&
           (chassis->feedback_sequence != 0U) &&
           ((now_ms - chassis->last_feedback_ms) <= kFeedbackStaleMs);
}

uint32_t CalculateDistanceTimeoutMs(int32_t distance_mm,
                                    int32_t max_rpm,
                                    uint32_t wheel_radius_mm)
{
    const int64_t numerator =
        static_cast<int64_t>(AbsInt32(distance_mm)) * 60000LL * 1000000LL;
    const int64_t denominator =
        2LL * kPiMicro * static_cast<int64_t>(wheel_radius_mm) *
        static_cast<int64_t>(max_rpm);
    uint32_t expected_ms = static_cast<uint32_t>(
        DivideRoundInt64(numerator, denominator));
    uint64_t timeout = static_cast<uint64_t>(expected_ms) * 3ULL + 2000ULL;
    if (timeout < 3000ULL) {
        timeout = 3000ULL;
    }
    if (timeout > kDistanceMaxTimeoutMs) {
        timeout = kDistanceMaxTimeoutMs;
    }
    return static_cast<uint32_t>(timeout);
}

void ResetDistanceState(void)
{
    g_state.target_distance_mm = 0;
    g_state.traveled_distance_mm = 0;
    g_state.remaining_distance_mm = 0;
    g_state.distance_max_rpm = 0;
    g_state.start_left_encoder_count = 0;
    g_state.start_right_encoder_count = 0;
    g_state.distance_start_ms = 0U;
    g_state.distance_timeout_ms = 0U;
    g_state.last_feedback_sequence = 0U;
}

bool IsImuFresh(const AppImuData *imu, uint32_t now_ms)
{
    return (imu != 0) && imu->valid &&
           ((now_ms - imu->last_update_ms) <=
            static_cast<uint32_t>(kStaleUpdateMs));
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
    g_state = {};
    g_state.mode = HEADING_IDLE;
    g_state.last_status = drivers::DRIVER_OK;
}

drivers::DriverStatus Heading_HoldStart(int32_t base_rpm)
{
    const AppImuData *imu = App_ImuGetData();
    if (!IsImuFresh(imu, services::Time_Millis())) {
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
    ResetDistanceState();
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

    const AppImuData *imu = App_ImuGetData();
    if (!IsImuFresh(imu, services::Time_Millis())) {
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
    ResetDistanceState();
    g_state.turn_start_ms = services::Time_Millis();
    g_state.last_run_ms = g_state.turn_start_ms;
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Heading_DistanceStart(int32_t distance_mm,
                                            int32_t max_rpm,
                                            uint32_t timeout_ms)
{
    const uint32_t now = services::Time_Millis();
    if ((distance_mm == 0) ||
        (AbsInt32(distance_mm) > kDistanceMaxMm) ||
        (max_rpm <= 0) ||
        ((timeout_ms != 0U) &&
         ((timeout_ms < kDistanceMinTimeoutMs) ||
          (timeout_ms > kDistanceMaxTimeoutMs)))) {
        g_state.last_status = drivers::DRIVER_ERROR_INVALID_ARG;
        return g_state.last_status;
    }

    const AppImuData *imu = App_ImuGetData();
    const ChassisState *chassis = Chassis_GetState();
    if ((!IsImuFresh(imu, now)) || (!IsFeedbackFresh(chassis, now))) {
        g_state.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return g_state.last_status;
    }
    if (max_rpm > static_cast<int32_t>(chassis->config.max_wheel_rpm)) {
        g_state.last_status = drivers::DRIVER_ERROR_INVALID_ARG;
        return g_state.last_status;
    }

    int32_t left_target_counts = 0;
    int32_t right_target_counts = 0;
    if ((!MillimetersToCounts(distance_mm,
                              chassis->config.wheel_radius_mm,
                              chassis->config.left_counts_per_rev,
                              &left_target_counts)) ||
        (!MillimetersToCounts(distance_mm,
                              chassis->config.wheel_radius_mm,
                              chassis->config.right_counts_per_rev,
                              &right_target_counts)) ||
        (left_target_counts == 0) || (right_target_counts == 0)) {
        g_state.last_status = drivers::DRIVER_ERROR_INVALID_ARG;
        return g_state.last_status;
    }

    g_state.mode = HEADING_DISTANCE;
    g_state.target_yaw_mdeg = imu->yaw_mdeg;
    g_state.base_rpm = 0;
    g_state.correction_rpm = 0;
    g_state.error_mdeg = 0;
    g_state.at_target = false;
    g_state.at_target_since_ms = 0U;
    g_state.turn_start_ms = 0U;
    g_state.target_distance_mm = distance_mm;
    g_state.traveled_distance_mm = 0;
    g_state.remaining_distance_mm = distance_mm;
    g_state.distance_max_rpm = max_rpm;
    g_state.start_left_encoder_count = chassis->left.encoder_count;
    g_state.start_right_encoder_count = chassis->right.encoder_count;
    g_state.distance_start_ms = now;
    g_state.distance_timeout_ms = (timeout_ms == 0U)
        ? CalculateDistanceTimeoutMs(distance_mm,
                                     max_rpm,
                                     chassis->config.wheel_radius_mm)
        : timeout_ms;
    g_state.last_feedback_sequence = chassis->feedback_sequence;
    g_state.last_run_ms = now;
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
    if (!IsImuFresh(imu, now)) {
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

    if (g_state.mode == HEADING_DISTANCE) {
        if ((now - g_state.distance_start_ms) >
            g_state.distance_timeout_ms) {
            g_state.last_status = drivers::DRIVER_ERROR_TIMEOUT;
            SafetyStop(services::FAULT_DRIVER_TIMEOUT);
            return;
        }
        if (AbsInt32(error) > kMaxErrorMdeg) {
            g_state.last_status = drivers::DRIVER_ERROR;
            SafetyStop(services::FAULT_SENSOR_LOST);
            return;
        }

        const ChassisState *chassis = Chassis_GetState();
        if (!IsFeedbackFresh(chassis, now)) {
            g_state.last_status = drivers::DRIVER_ERROR_TIMEOUT;
            SafetyStop(services::FAULT_SENSOR_LOST);
            return;
        }
        if (chassis->feedback_sequence == g_state.last_feedback_sequence) {
            return;
        }
        g_state.last_feedback_sequence = chassis->feedback_sequence;

        const int32_t left_counts = EncoderDelta(
            chassis->left.encoder_count,
            g_state.start_left_encoder_count);
        const int32_t right_counts = EncoderDelta(
            chassis->right.encoder_count,
            g_state.start_right_encoder_count);
        const int32_t left_mm = CountsToMillimeters(
            left_counts,
            chassis->config.wheel_radius_mm,
            chassis->config.left_counts_per_rev);
        const int32_t right_mm = CountsToMillimeters(
            right_counts,
            chassis->config.wheel_radius_mm,
            chassis->config.right_counts_per_rev);
        g_state.traveled_distance_mm = static_cast<int32_t>(
            (static_cast<int64_t>(left_mm) +
             static_cast<int64_t>(right_mm)) / 2LL);
        g_state.remaining_distance_mm =
            g_state.target_distance_mm - g_state.traveled_distance_mm;

        const bool left_reached =
            AbsInt32(g_state.target_distance_mm - left_mm) <=
            kDistanceToleranceMm;
        const bool right_reached =
            AbsInt32(g_state.target_distance_mm - right_mm) <=
            kDistanceToleranceMm;
        if (left_reached && right_reached) {
            if (!g_state.at_target) {
                g_state.at_target = true;
                g_state.at_target_since_ms = now;
                g_state.base_rpm = 0;
                g_state.correction_rpm = 0;
                g_state.last_status = Chassis_Stop();
                if (g_state.last_status != drivers::DRIVER_OK) {
                    SafetyStop(services::FAULT_DRIVER_TIMEOUT);
                    return;
                }
            }
            if ((now - g_state.at_target_since_ms) >= kDistanceSettleMs) {
                g_state.mode = HEADING_IDLE;
                g_state.last_status = drivers::DRIVER_OK;
            }
            return;
        }

        g_state.at_target = false;
        int32_t speed = AbsInt32(g_state.remaining_distance_mm);
        speed = ClampInt32(speed,
                           (g_state.distance_max_rpm < kDistanceMinRpm)
                               ? g_state.distance_max_rpm
                               : kDistanceMinRpm,
                           g_state.distance_max_rpm);
        const int32_t direction = (g_state.remaining_distance_mm != 0)
            ? ((g_state.remaining_distance_mm > 0) ? 1 : -1)
            : ((g_state.target_distance_mm > 0) ? 1 : -1);
        g_state.base_rpm = speed * direction;

        int32_t correction = GainToRpm(error, params->heading_kp);
        correction = ClampInt32(correction,
                                 -params->heading_max_correction_rpm,
                                 params->heading_max_correction_rpm);
        correction *= kYawSign;
        g_state.correction_rpm = correction;
        const int32_t wheel_limit =
            static_cast<int32_t>(chassis->config.max_wheel_rpm);
        const int32_t left = ClampInt32(g_state.base_rpm - correction,
                                        -wheel_limit,
                                        wheel_limit);
        const int32_t right = ClampInt32(g_state.base_rpm + correction,
                                         -wheel_limit,
                                         wheel_limit);
        g_state.last_status = Chassis_SetWheelRpm(left, right);
        if (g_state.last_status != drivers::DRIVER_OK) {
            SafetyStop(services::FAULT_DRIVER_TIMEOUT);
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
