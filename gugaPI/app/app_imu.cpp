#include "app/app_imu.h"

#include <limits.h>

#include "app/action.h"
#include "app/app_main.h"
#include "app/chassis.h"
#include "app/config_store.h"
#include "app/heading.h"
#include "app/linefollow.h"
#include "board/board_imu.h"
#include "config/feature_config.h"
#include "drivers/common/driver_status.h"
#include "drivers/icm45686/icm45686.h"
#include "services/fault.h"
#include "services/time.h"

namespace app {
namespace {

AppImuData g_data = {};
uint32_t g_lastUpdateUs = 0U;
int64_t g_yawRemainder = 0LL;
int32_t g_lastFixedBiasZMdps = 0;

static const uint32_t kMaxYawIntegrationIntervalUs = 100000U;
static const int64_t kMicrosPerSecond = 1000000LL;
static const uint32_t kBiasFeedbackStaleMs = 100U;
static const int32_t kBiasMaximumStationaryRpm = 3;

int32_t AbsInt32(int32_t value)
{
    if (value >= 0) {
        return value;
    }
    return (value == INT32_MIN) ? INT32_MAX : -value;
}

ImuBiasRejectReason BiasLearningBlockReason(uint32_t now_ms)
{
    if (services::Fault_HasFault()) {
        return IMU_BIAS_REJECT_CONTROL_ACTIVE;
    }

#if FEATURE_ENABLE_MOTOR_DRIVER
    const ChassisState *chassis = Chassis_GetState();
    if ((chassis == 0) || (!chassis->initialized)) {
        return IMU_BIAS_REJECT_CHASSIS_UNAVAILABLE;
    }
    if ((chassis->last_feedback_status != drivers::DRIVER_OK) ||
        (static_cast<uint32_t>(now_ms - chassis->last_feedback_ms) >
         kBiasFeedbackStaleMs)) {
        return IMU_BIAS_REJECT_FEEDBACK_STALE;
    }

#if FEATURE_ENABLE_IMU
    const HeadingState *heading = Heading_GetState();
    if ((heading != 0) && (heading->mode != HEADING_IDLE)) {
        return IMU_BIAS_REJECT_CONTROL_ACTIVE;
    }
#endif
#if FEATURE_ENABLE_GRAYSCALE
    const LFState *line_follow = LF_GetState();
    if ((line_follow != 0) && (line_follow->mode != LF_IDLE)) {
        return IMU_BIAS_REJECT_CONTROL_ACTIVE;
    }
#endif
    const ActionRunnerState *runner = ActionRunner_GetState();
    if ((runner != 0) && runner->running) {
        return IMU_BIAS_REJECT_CONTROL_ACTIVE;
    }
    const AppState *app_state = App_GetState();
    if ((app_state != 0) &&
        ((app_state->mode == APP_MODE_FAULT) ||
         (app_state->mode == APP_MODE_COMPETITION_RUNNING))) {
        return IMU_BIAS_REJECT_CONTROL_ACTIVE;
    }

    if ((chassis->left.target_rpm != 0) ||
        (chassis->right.target_rpm != 0)) {
        return IMU_BIAS_REJECT_TARGET_ACTIVE;
    }
    if ((AbsInt32(chassis->left.actual_rpm) > kBiasMaximumStationaryRpm) ||
        (AbsInt32(chassis->right.actual_rpm) > kBiasMaximumStationaryRpm)) {
        return IMU_BIAS_REJECT_WHEELS_MOVING;
    }
#else
    (void) now_ms;
#endif

    return IMU_BIAS_REJECT_NONE;
}

/* CORDIC vectoring-mode atan2. Returns atan2(y, x) in milli-degrees,
 * range [-180000, 180000]. No floating point / libm dependency. */
int32_t Atan2MilliDeg(int32_t y, int32_t x)
{
    if ((x == 0) && (y == 0)) {
        return 0;
    }

    /* atan(2^-i) in micro-degrees (degrees * 1e6), i = 0..23. */
    static const int32_t kAtan[24] = {
        45000000, 26565051, 14036243, 7125016, 3576334, 1789911,
        895174, 447614, 223811, 111906, 55953, 27976,
        13988, 6994, 3497, 1748, 874, 437, 218, 109, 55, 27, 14, 7
    };

    const int64_t scale = 1LL << 20;
    int64_t xi = static_cast<int64_t>(x) * scale;
    int64_t yi = static_cast<int64_t>(y) * scale;
    int32_t z = 0;

    for (int32_t i = 0; i < 24; i++) {
        int64_t dx;
        int64_t dy;
        if (yi >= 0) {
            dx = xi + (yi >> i);
            dy = yi - (xi >> i);
            z += kAtan[static_cast<uint32_t>(i)];
        } else {
            dx = xi - (yi >> i);
            dy = yi + (xi >> i);
            z -= kAtan[static_cast<uint32_t>(i)];
        }
        xi = dx;
        yi = dy;
    }

    return z / 1000; /* micro-degrees -> milli-degrees */
}

int32_t Normalize360(int32_t angle_mdeg)
{
    while (angle_mdeg < 0) {
        angle_mdeg += 360000;
    }
    while (angle_mdeg >= 360000) {
        angle_mdeg -= 360000;
    }
    return angle_mdeg;
}

int32_t NormalizeSigned180(int32_t angle_mdeg)
{
    while (angle_mdeg > 180000) {
        angle_mdeg -= 360000;
    }
    while (angle_mdeg <= -180000) {
        angle_mdeg += 360000;
    }
    return angle_mdeg;
}

void UpdateYaw(uint32_t now_us)
{
    if (g_lastUpdateUs == 0U) {
        g_lastUpdateUs = now_us;
        return;
    }

    const uint32_t elapsed_us = now_us - g_lastUpdateUs;
    g_lastUpdateUs = now_us;
    if ((elapsed_us == 0U) ||
        (elapsed_us > kMaxYawIntegrationIntervalUs)) {
        g_yawRemainder = 0LL;
        return;
    }

    const int64_t scaled_delta =
        static_cast<int64_t>(g_data.gyro_mdps[2]) * elapsed_us +
        g_yawRemainder;
    const int32_t delta_mdeg =
        static_cast<int32_t>(scaled_delta / kMicrosPerSecond);
    g_yawRemainder = scaled_delta % kMicrosPerSecond;
    g_data.yaw_mdeg = NormalizeSigned180(g_data.yaw_mdeg + delta_mdeg);
}

void FreezeYaw(uint32_t now_us)
{
    /* Advance the timestamp while intentionally not integrating. Otherwise
     * the first moving sample would integrate the complete calibration gap. */
    g_lastUpdateUs = now_us;
    g_yawRemainder = 0LL;
}

} /* namespace */

void App_ImuInit(void)
{
    g_data.yaw_mdeg = 0;
    g_lastUpdateUs = 0U;
    g_yawRemainder = 0LL;
    g_data.valid = false;
    g_data.last_update_ms = 0U;
    g_data.error_count = 0U;
    g_data.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
    g_data.sequence = 0U;
    const ConfigStoreParams *params = ConfigStore_Get();
    g_lastFixedBiasZMdps = (params != 0) ?
        params->imu_gyro_bias_z_mdps : 0;
    ImuBiasEstimator_Init();
}

void App_ImuUpdate(void)
{
    if (!board::Board_Icm45686IsReady()) {
        g_data.valid = false;
        g_data.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        g_data.error_count++;
        ImuBiasEstimator_NotifyBlocked(IMU_BIAS_REJECT_SENSOR_INVALID,
                                       services::Time_Millis());
        return;
    }

    drivers::Icm45686SensorData raw = {};
    const drivers::DriverStatus status =
        board::Board_Icm45686ReadSensors(&raw);
    if (status != drivers::DRIVER_OK) {
        g_data.valid = false;
        g_data.last_status = status;
        g_data.error_count++;
        ImuBiasEstimator_NotifyBlocked(IMU_BIAS_REJECT_SENSOR_INVALID,
                                       services::Time_Millis());
        return;
    }

    const drivers::Icm45686Config *cfg = board::Board_Icm45686GetConfig();
    const uint8_t accel_fs = (cfg != 0) ? cfg->accel_fs : drivers::ICM45686_ACCEL_FS_4G;
    const uint8_t gyro_fs = (cfg != 0) ? cfg->gyro_fs : drivers::ICM45686_GYRO_FS_1000DPS;

    const int16_t accel_raw[3] = { raw.accel_x, raw.accel_y, raw.accel_z };
    const int16_t gyro_raw[3] = { raw.gyro_x, raw.gyro_y, raw.gyro_z };

    const ConfigStoreParams *params = ConfigStore_Get();
    const int32_t fixed_bias_z_mdps = (params != 0) ?
        params->imu_gyro_bias_z_mdps : 0;
    if (fixed_bias_z_mdps != g_lastFixedBiasZMdps) {
        ImuBiasEstimator_ResetRuntime();
        ImuBiasEstimator_NotifyBlocked(IMU_BIAS_REJECT_CONFIG_CHANGED,
                                       services::Time_Millis());
        g_lastFixedBiasZMdps = fixed_bias_z_mdps;
    }

    const int32_t runtime_bias_before =
        ImuBiasEstimator_GetRuntimeBiasZMdps();

    for (uint8_t i = 0U; i < 3U; i++) {
        int32_t accel_mg = drivers::Icm45686_AccelMilliG(accel_raw[i], accel_fs);
        int32_t gyro_mdps = drivers::Icm45686_GyroMilliDps(gyro_raw[i], gyro_fs);
        if (params != 0) {
            const int32_t accel_bias[3] = {
                params->imu_accel_bias_x_mg,
                params->imu_accel_bias_y_mg,
                params->imu_accel_bias_z_mg,
            };
            const int32_t gyro_bias[3] = {
                params->imu_gyro_bias_x_mdps,
                params->imu_gyro_bias_y_mdps,
                params->imu_gyro_bias_z_mdps,
            };
            accel_mg -= accel_bias[i];
            gyro_mdps -= gyro_bias[i];
        }
        g_data.accel_mg[i] = accel_mg;
        g_data.gyro_mdps[i] = gyro_mdps;
    }
    g_data.gyro_mdps[2] -= runtime_bias_before;

    g_data.temp_centi_c = drivers::Icm45686_TempCentiC(raw.temp);

    /* Tilt angles from the accelerometer (gravity reference), in degrees.
     * Pitch = atan2(-ax, az), Roll = atan2(ay, az); normalized to 0..360. */
    g_data.pitch_mdeg =
        Normalize360(Atan2MilliDeg(-g_data.accel_mg[0], g_data.accel_mg[2]));
    g_data.roll_mdeg =
        Normalize360(Atan2MilliDeg(g_data.accel_mg[1], g_data.accel_mg[2]));
    const uint32_t now_ms = services::Time_Millis();
    const ImuBiasRejectReason block_reason = BiasLearningBlockReason(now_ms);
    ImuBiasEstimator_Update(g_data.accel_mg,
                            g_data.gyro_mdps,
                            now_ms,
                            block_reason == IMU_BIAS_REJECT_NONE,
                            block_reason);

    const int32_t runtime_bias_after =
        ImuBiasEstimator_GetRuntimeBiasZMdps();
    g_data.gyro_mdps[2] -= runtime_bias_after - runtime_bias_before;

    const ImuBiasEstimatorStatus *bias_status =
        ImuBiasEstimator_GetStatus();
    const bool bias_updated_now =
        (bias_status != 0) && bias_status->estimate_valid &&
        (bias_status->last_update_ms == now_ms);
    const uint32_t now_us = services::Time_Micros();
    if (ImuBiasEstimator_ShouldFreezeYaw() || bias_updated_now) {
        FreezeYaw(now_us);
    } else {
        UpdateYaw(now_us);
    }
    g_data.last_update_ms = now_ms;
    g_data.last_status = drivers::DRIVER_OK;
    g_data.sequence++;
    g_data.valid = true;
}

const AppImuData *App_ImuGetData(void)
{
    return &g_data;
}

const ImuBiasEstimatorStatus *App_ImuGetBiasStatus(void)
{
    return ImuBiasEstimator_GetStatus();
}

void App_ImuBiasSetAutoEnabled(bool enabled)
{
    ImuBiasEstimator_SetAutoEnabled(enabled);
}

void App_ImuBiasRequestCalibration(void)
{
    ImuBiasEstimator_RequestCalibration();
}

void App_ImuBiasResetRuntime(void)
{
    ImuBiasEstimator_ResetRuntime();
}

drivers::DriverStatus App_ImuBiasSave(void)
{
    const ImuBiasEstimatorStatus *bias = ImuBiasEstimator_GetStatus();
    const ConfigStoreParams *params = ConfigStore_Get();
    if ((bias == 0) || (params == 0) || (!bias->estimate_valid)) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (BiasLearningBlockReason(services::Time_Millis()) !=
        IMU_BIAS_REJECT_NONE) {
        return drivers::DRIVER_ERROR_BUSY;
    }

    const int32_t old_fixed = params->imu_gyro_bias_z_mdps;
    const int64_t combined = static_cast<int64_t>(old_fixed) +
                             bias->runtime_bias_z_mdps;
    if ((combined < -2000000LL) || (combined > 2000000LL)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    drivers::DriverStatus status = ConfigStore_Set(
        "imu_gyro_bias_z_mdps", static_cast<int32_t>(combined));
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    status = ConfigStore_Save();
    if (status != drivers::DRIVER_OK) {
        /* Restore the runtime image if the FRAM write fails. The dynamic bias
         * remains active, so the total correction is unchanged. */
        (void) ConfigStore_Set("imu_gyro_bias_z_mdps", old_fixed);
        return status;
    }

    g_lastFixedBiasZMdps = static_cast<int32_t>(combined);
    ImuBiasEstimator_CommitRuntimeToFixed();
    return drivers::DRIVER_OK;
}

} /* namespace app */
