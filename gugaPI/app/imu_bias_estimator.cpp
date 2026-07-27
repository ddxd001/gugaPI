#include "app/imu_bias_estimator.h"

#include <limits.h>

namespace app {
namespace {

static const int32_t kAccelMagnitudeToleranceMg = 80;
static const int32_t kGyroMotionLimitMdps = 1500;
static const int32_t kMeanLimitMdps = 1000;
static const uint32_t kStddevLimitMdps = 250U;
static const int32_t kMaintenanceStepLimitMdps = 250;
static const uint8_t kMaintenanceFilterShift = 3U; /* 1/8 */

ImuBiasEstimatorStatus g_status = {};
uint32_t g_collectionStartMs = 0U;
uint32_t g_cooldownStartMs = 0U;
int64_t g_sumZ = 0LL;
uint64_t g_sumSquaresZ = 0ULL;

int32_t AbsInt32(int32_t value)
{
    if (value >= 0) {
        return value;
    }
    return (value == INT32_MIN) ? INT32_MAX : -value;
}

int32_t ClampInt32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

uint32_t IntegerSqrt(uint64_t value)
{
    uint64_t result = 0ULL;
    uint64_t bit = 1ULL << 62U;

    while (bit > value) {
        bit >>= 2U;
    }
    while (bit != 0ULL) {
        if (value >= (result + bit)) {
            value -= result + bit;
            result = (result >> 1U) + bit;
        } else {
            result >>= 1U;
        }
        bit >>= 2U;
    }
    return (result > UINT32_MAX) ? UINT32_MAX :
        static_cast<uint32_t>(result);
}

bool HasElapsed(uint32_t start_ms, uint32_t interval_ms, uint32_t now_ms)
{
    return static_cast<uint32_t>(now_ms - start_ms) >= interval_ms;
}

void ClearCollection(void)
{
    g_collectionStartMs = 0U;
    g_sumZ = 0LL;
    g_sumSquaresZ = 0ULL;
    g_status.sample_count = 0U;
    g_status.collection_elapsed_ms = 0U;
    g_status.freeze_yaw = false;
}

bool EstimatorRequested(void)
{
    return g_status.auto_enabled || g_status.manual_requested;
}

void EnterWaiting(ImuBiasRejectReason reason, bool rejected)
{
    if (rejected) {
        g_status.rejected_windows++;
    }
    ClearCollection();
    g_status.reject_reason = reason;
    g_status.state = EstimatorRequested() ? IMU_BIAS_WAITING :
        IMU_BIAS_DISABLED;
}

bool AccelerationLooksStatic(const int32_t accel_mg[3])
{
    const int64_t x = accel_mg[0];
    const int64_t y = accel_mg[1];
    const int64_t z = accel_mg[2];
    const uint64_t magnitude_squared = static_cast<uint64_t>(
        (x * x) + (y * y) + (z * z));
    const int64_t minimum = 1000LL - kAccelMagnitudeToleranceMg;
    const int64_t maximum = 1000LL + kAccelMagnitudeToleranceMg;
    return (magnitude_squared >= static_cast<uint64_t>(minimum * minimum)) &&
           (magnitude_squared <= static_cast<uint64_t>(maximum * maximum));
}

bool GyroscopeLooksStatic(const int32_t gyro_mdps[3])
{
    return (AbsInt32(gyro_mdps[0]) <= kGyroMotionLimitMdps) &&
           (AbsInt32(gyro_mdps[1]) <= kGyroMotionLimitMdps) &&
           (AbsInt32(gyro_mdps[2]) <= kGyroMotionLimitMdps);
}

void FinishCollection(uint32_t now_ms)
{
    const int64_t sample_count = static_cast<int64_t>(g_status.sample_count);
    const int32_t mean = static_cast<int32_t>(g_sumZ / sample_count);
    const uint64_t mean_square =
        g_sumSquaresZ / static_cast<uint64_t>(g_status.sample_count);
    const int64_t signed_variance =
        static_cast<int64_t>(mean_square) -
        (static_cast<int64_t>(mean) * static_cast<int64_t>(mean));
    const uint64_t variance = (signed_variance > 0LL) ?
        static_cast<uint64_t>(signed_variance) : 0ULL;
    const uint32_t stddev = IntegerSqrt(variance);

    g_status.last_mean_z_mdps = mean;
    g_status.last_stddev_z_mdps = stddev;
    if ((AbsInt32(mean) > kMeanLimitMdps) ||
        (stddev > kStddevLimitMdps)) {
        EnterWaiting((stddev > kStddevLimitMdps) ?
                     IMU_BIAS_REJECT_NOISE : IMU_BIAS_REJECT_GYRO_MOTION,
                     true);
        return;
    }

    int32_t adjustment = mean;
    if (g_status.estimate_valid && (!g_status.manual_requested)) {
        adjustment = mean / static_cast<int32_t>(1U <<
                                                  kMaintenanceFilterShift);
        adjustment = ClampInt32(adjustment,
                                -kMaintenanceStepLimitMdps,
                                kMaintenanceStepLimitMdps);
    }
    g_status.runtime_bias_z_mdps = ClampInt32(
        g_status.runtime_bias_z_mdps + adjustment,
        -IMU_BIAS_RUNTIME_LIMIT_MDPS,
        IMU_BIAS_RUNTIME_LIMIT_MDPS);
    g_status.last_adjustment_z_mdps = adjustment;
    g_status.last_update_ms = now_ms;
    g_status.accepted_windows++;
    g_status.estimate_valid = true;
    g_status.manual_requested = false;
    ClearCollection();

    if (g_status.auto_enabled) {
        g_status.state = IMU_BIAS_COOLDOWN;
        g_status.reject_reason = IMU_BIAS_REJECT_COOLDOWN;
        g_cooldownStartMs = now_ms;
    } else {
        g_status.state = IMU_BIAS_DISABLED;
        g_status.reject_reason = IMU_BIAS_REJECT_DISABLED;
    }
}

} /* namespace */

void ImuBiasEstimator_Init(void)
{
    g_status = {};
    g_status.state = IMU_BIAS_WAITING;
    g_status.reject_reason = IMU_BIAS_REJECT_NONE;
    g_status.auto_enabled = true;
    ClearCollection();
    g_cooldownStartMs = 0U;
}

void ImuBiasEstimator_SetAutoEnabled(bool enabled)
{
    g_status.auto_enabled = enabled;
    if (!EstimatorRequested()) {
        EnterWaiting(IMU_BIAS_REJECT_DISABLED, false);
    } else if (g_status.state == IMU_BIAS_DISABLED) {
        g_status.state = IMU_BIAS_WAITING;
        g_status.reject_reason = IMU_BIAS_REJECT_NONE;
    }
}

void ImuBiasEstimator_RequestCalibration(void)
{
    g_status.manual_requested = true;
    EnterWaiting(IMU_BIAS_REJECT_NONE, false);
}

void ImuBiasEstimator_ResetRuntime(void)
{
    const bool auto_enabled = g_status.auto_enabled;
    g_status = {};
    g_status.auto_enabled = auto_enabled;
    g_status.state = auto_enabled ? IMU_BIAS_WAITING : IMU_BIAS_DISABLED;
    g_status.reject_reason = auto_enabled ? IMU_BIAS_REJECT_NONE :
        IMU_BIAS_REJECT_DISABLED;
    ClearCollection();
    g_cooldownStartMs = 0U;
}

void ImuBiasEstimator_CommitRuntimeToFixed(void)
{
    ImuBiasEstimator_ResetRuntime();
}

void ImuBiasEstimator_NotifyBlocked(ImuBiasRejectReason reason,
                                    uint32_t now_ms)
{
    (void) now_ms;
    const bool rejected = (g_status.state == IMU_BIAS_COLLECTING) &&
                          (g_status.sample_count != 0U);
    EnterWaiting(reason, rejected);
}

void ImuBiasEstimator_Update(const int32_t accel_mg[3],
                             const int32_t corrected_gyro_mdps[3],
                             uint32_t now_ms,
                             bool learning_allowed,
                             ImuBiasRejectReason blocked_reason)
{
    if ((accel_mg == 0) || (corrected_gyro_mdps == 0)) {
        ImuBiasEstimator_NotifyBlocked(IMU_BIAS_REJECT_SENSOR_INVALID,
                                       now_ms);
        return;
    }
    if (!EstimatorRequested()) {
        EnterWaiting(IMU_BIAS_REJECT_DISABLED, false);
        return;
    }
    if (g_status.state == IMU_BIAS_COOLDOWN) {
        if (!HasElapsed(g_cooldownStartMs, IMU_BIAS_COOLDOWN_MS, now_ms)) {
            g_status.reject_reason = IMU_BIAS_REJECT_COOLDOWN;
            return;
        }
        g_status.state = IMU_BIAS_WAITING;
        g_status.reject_reason = IMU_BIAS_REJECT_NONE;
    }
    if (!learning_allowed) {
        ImuBiasEstimator_NotifyBlocked(
            (blocked_reason == IMU_BIAS_REJECT_NONE) ?
                IMU_BIAS_REJECT_CONTROL_ACTIVE : blocked_reason,
            now_ms);
        return;
    }
    if (!AccelerationLooksStatic(accel_mg)) {
        ImuBiasEstimator_NotifyBlocked(IMU_BIAS_REJECT_ACCELERATION, now_ms);
        return;
    }
    if (!GyroscopeLooksStatic(corrected_gyro_mdps)) {
        ImuBiasEstimator_NotifyBlocked(IMU_BIAS_REJECT_GYRO_MOTION, now_ms);
        return;
    }

    if (g_status.state != IMU_BIAS_COLLECTING) {
        ClearCollection();
        g_status.state = IMU_BIAS_COLLECTING;
        g_status.reject_reason = IMU_BIAS_REJECT_NONE;
        g_collectionStartMs = now_ms;
    }

    const int64_t z = corrected_gyro_mdps[2];
    g_sumZ += z;
    g_sumSquaresZ += static_cast<uint64_t>(z * z);
    g_status.sample_count++;
    g_status.collection_elapsed_ms = now_ms - g_collectionStartMs;
    g_status.freeze_yaw = true;

    if ((g_status.sample_count >= IMU_BIAS_REQUIRED_SAMPLES) &&
        HasElapsed(g_collectionStartMs, IMU_BIAS_WINDOW_MS, now_ms)) {
        FinishCollection(now_ms);
    }
}

int32_t ImuBiasEstimator_GetRuntimeBiasZMdps(void)
{
    return g_status.runtime_bias_z_mdps;
}

bool ImuBiasEstimator_ShouldFreezeYaw(void)
{
    return g_status.freeze_yaw;
}

const ImuBiasEstimatorStatus *ImuBiasEstimator_GetStatus(void)
{
    return &g_status;
}

const char *ImuBiasEstimator_StateText(ImuBiasEstimatorState state)
{
    switch (state) {
    case IMU_BIAS_DISABLED: return "disabled";
    case IMU_BIAS_WAITING: return "waiting";
    case IMU_BIAS_COLLECTING: return "collecting";
    case IMU_BIAS_COOLDOWN: return "cooldown";
    default: return "unknown";
    }
}

const char *ImuBiasEstimator_RejectReasonText(ImuBiasRejectReason reason)
{
    switch (reason) {
    case IMU_BIAS_REJECT_NONE: return "none";
    case IMU_BIAS_REJECT_DISABLED: return "disabled";
    case IMU_BIAS_REJECT_SENSOR_INVALID: return "sensor_invalid";
    case IMU_BIAS_REJECT_CHASSIS_UNAVAILABLE: return "chassis_unavailable";
    case IMU_BIAS_REJECT_FEEDBACK_STALE: return "feedback_stale";
    case IMU_BIAS_REJECT_TARGET_ACTIVE: return "target_active";
    case IMU_BIAS_REJECT_WHEELS_MOVING: return "wheels_moving";
    case IMU_BIAS_REJECT_CONTROL_ACTIVE: return "control_active";
    case IMU_BIAS_REJECT_ACCELERATION: return "acceleration";
    case IMU_BIAS_REJECT_GYRO_MOTION: return "gyro_motion";
    case IMU_BIAS_REJECT_NOISE: return "noise";
    case IMU_BIAS_REJECT_COOLDOWN: return "cooldown";
    case IMU_BIAS_REJECT_CONFIG_CHANGED: return "config_changed";
    default: return "unknown";
    }
}

} /* namespace app */
