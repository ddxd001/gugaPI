#include "app/chassis.h"

#include <limits.h>

#include "app/app_ina219.h"
#include "app/config_store.h"
#include "app/local_motor_controller.h"
#include "config/feature_config.h"
#include "services/fault.h"
#include "services/time.h"

#if FEATURE_ENABLE_LOCAL_MOTOR

namespace app {
namespace {

static const int64_t kPiNumerator = 355LL;
static const int64_t kPiDenominator = 113LL;

ChassisState g_state = {
    false,
    0,
    0,
    { 0, 0, 0, 0, 0U },
    { 0, 0, 0, 0, 0U },
    { 33050U, 160U, 1456U, 1456U, 1000U },
    drivers::DRIVER_ERROR_NOT_INITIALIZED,
    drivers::DRIVER_ERROR_NOT_INITIALIZED,
    0U,
    0U
};

int32_t AbsInt32(int32_t value)
{
    if (value == INT32_MIN) {
        return INT32_MAX;
    }
    return (value < 0) ? -value : value;
}

int32_t SaturateInt32(int64_t value)
{
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return static_cast<int32_t>(value);
}

bool ChassisConfigsEqual(const ChassisConfig &left,
                         const ChassisConfig &right)
{
    return (left.wheel_radius_um == right.wheel_radius_um) &&
           (left.wheel_track_mm == right.wheel_track_mm) &&
           (left.left_counts_per_rev == right.left_counts_per_rev) &&
           (left.right_counts_per_rev == right.right_counts_per_rev) &&
           (left.max_wheel_rpm == right.max_wheel_rpm);
}

int32_t DivideRoundInt64(int64_t numerator, int64_t denominator)
{
    if (denominator == 0) {
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

int32_t WheelMmPerSecondToRpm(int32_t wheel_mm_s,
                              uint32_t wheel_radius_um)
{
    if (wheel_radius_um == 0U) {
        return 0;
    }
    const int64_t numerator =
        static_cast<int64_t>(wheel_mm_s) * 60LL * 1000LL * kPiDenominator;
    const int64_t denominator =
        2LL * kPiNumerator * static_cast<int64_t>(wheel_radius_um);
    return DivideRoundInt64(numerator, denominator);
}

int32_t AngularToWheelDeltaMmPerSecond(int32_t angular_mdeg_s,
                                       uint32_t wheel_track_mm)
{
    const int64_t numerator =
        static_cast<int64_t>(angular_mdeg_s) *
        static_cast<int64_t>(wheel_track_mm) * kPiNumerator;
    const int64_t denominator = 360000LL * kPiDenominator;
    return DivideRoundInt64(numerator, denominator);
}

drivers::DriverStatus BuildConfig(LocalMotorConfig *local_config)
{
    if (local_config == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    const ConfigStoreParams *params = ConfigStore_Get();
    if (params == 0) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    g_state.config.wheel_radius_um = params->wheel_radius_um;
    g_state.config.wheel_track_mm = params->wheel_track_mm;
    g_state.config.left_counts_per_rev = params->left_counts_per_rev;
    g_state.config.right_counts_per_rev = params->right_counts_per_rev;
    g_state.config.max_wheel_rpm = params->max_wheel_rpm;

    local_config->left_counts_per_rev = params->left_counts_per_rev;
    local_config->right_counts_per_rev = params->right_counts_per_rev;
    local_config->max_wheel_rpm = params->max_wheel_rpm;
    local_config->accel_rpm_per_s = params->speed_accel_rpm_s;
    local_config->decel_rpm_per_s = params->speed_decel_rpm_s;
    local_config->kp_q4_4 = params->speed_kp_q4_4;
    local_config->ki_q4_4 = params->speed_ki_q4_4;
    local_config->kd_q4_4 = params->speed_kd_q4_4;
    local_config->max_duty_percent = params->speed_max_duty;
    local_config->min_duty_percent = params->speed_min_duty;
    local_config->output_invert_flags =
        static_cast<uint8_t>(params->motor_output_invert_flags & 0x03U);
    local_config->encoder_invert_flags =
        static_cast<uint8_t>(params->motor_encoder_invert_flags & 0x03U);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus RefreshConfig(void)
{
    const ChassisConfig previous_config = g_state.config;
    LocalMotorConfig local_config = {};
    drivers::DriverStatus status = BuildConfig(&local_config);
    const LocalMotorState *local = LocalMotorController_GetState();
    if ((status == drivers::DRIVER_OK) && (local != 0) && local->active &&
        !ChassisConfigsEqual(previous_config, g_state.config)) {
        /* Geometry/CPR changes invalidate a live velocity command. The caller
         * will stop and requires a fresh command using the new parameters. */
        status = drivers::DRIVER_ERROR_BUSY;
    }
    if (status == drivers::DRIVER_OK) {
        status = LocalMotorController_Configure(&local_config);
    }
    return status;
}

void ClearCommandState(void)
{
    g_state.target_linear_mm_s = 0;
    g_state.target_angular_mdeg_s = 0;
    g_state.left.target_rpm = 0;
    g_state.right.target_rpm = 0;
}

void CopyFeedback(const LocalMotorState &local)
{
    g_state.left.target_rpm = local.left.target_rpm;
    g_state.left.actual_rpm = local.left.actual_rpm;
    g_state.left.encoder_count = local.left.encoder_count;
    g_state.left.encoder_counts_per_second =
        local.left.encoder_counts_per_second;
    g_state.left.encoder_state = local.left.encoder_state;

    g_state.right.target_rpm = local.right.target_rpm;
    g_state.right.actual_rpm = local.right.actual_rpm;
    g_state.right.encoder_count = local.right.encoder_count;
    g_state.right.encoder_counts_per_second =
        local.right.encoder_counts_per_second;
    g_state.right.encoder_state = local.right.encoder_state;
}

} /* namespace */

drivers::DriverStatus Chassis_Init(void)
{
    LocalMotorConfig config = {};
    drivers::DriverStatus status = BuildConfig(&config);
    if (status == drivers::DRIVER_OK) {
        status = LocalMotorController_Init(&config, services::Time_Millis());
    }

    ClearCommandState();
    g_state.initialized = (status == drivers::DRIVER_OK);
    g_state.last_status = status;
    g_state.last_feedback_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
    g_state.feedback_sequence = 0U;
    g_state.last_feedback_ms = 0U;
    return status;
}

drivers::DriverStatus Chassis_Stop(void)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    ClearCommandState();
    const drivers::DriverStatus status = LocalMotorController_Stop();
    g_state.last_status = status;
    return status;
}

drivers::DriverStatus Chassis_SetWheelRpm(int32_t left_rpm,
                                          int32_t right_rpm)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (services::Fault_HasFault()) {
        g_state.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return g_state.last_status;
    }
#if FEATURE_LOCAL_MOTOR_RIGHT_ONLY
    if (left_rpm != 0) {
        /* A two-wheel command must never be partially executed. Stop any
         * previous MR command, then report the capability mismatch. */
        const drivers::DriverStatus stop_status =
            LocalMotorController_Stop();
        ClearCommandState();
        g_state.last_status = (stop_status == drivers::DRIVER_OK) ?
            drivers::DRIVER_ERROR_UNSUPPORTED : stop_status;
        return g_state.last_status;
    }
#endif
#if FEATURE_ENABLE_INA219
    if (((left_rpm != 0) || (right_rpm != 0)) &&
        App_Ina219MotionInhibitRequested()) {
        g_state.last_status = drivers::DRIVER_ERROR_BUSY;
        return g_state.last_status;
    }
#endif

    drivers::DriverStatus status = RefreshConfig();
    if ((status == drivers::DRIVER_OK) &&
        ((AbsInt32(left_rpm) > g_state.config.max_wheel_rpm) ||
         (AbsInt32(right_rpm) > g_state.config.max_wheel_rpm))) {
        status = drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (status == drivers::DRIVER_OK) {
        status = LocalMotorController_SetTargets(left_rpm,
                                                 right_rpm,
                                                 services::Time_Millis());
    }
    if (status == drivers::DRIVER_OK) {
        g_state.target_linear_mm_s = 0;
        g_state.target_angular_mdeg_s = 0;
        g_state.left.target_rpm = left_rpm;
        g_state.right.target_rpm = right_rpm;
    } else {
        (void) LocalMotorController_Stop();
        ClearCommandState();
    }
    g_state.last_status = status;
    return status;
}

drivers::DriverStatus Chassis_SetVelocity(int32_t linear_mm_s,
                                          int32_t angular_mdeg_s)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
#if FEATURE_LOCAL_MOTOR_RIGHT_ONLY
    /* Linear/angular chassis kinematics require two driven wheels. Always
     * stop first so an unsupported request cannot leave an older MR command
     * active under its lease. */
    const drivers::DriverStatus stop_status = LocalMotorController_Stop();
    ClearCommandState();
    g_state.last_status = (stop_status == drivers::DRIVER_OK) ?
        drivers::DRIVER_ERROR_UNSUPPORTED : stop_status;
    return g_state.last_status;
#endif
    const drivers::DriverStatus config_status = RefreshConfig();
    if (config_status != drivers::DRIVER_OK) {
        (void) LocalMotorController_Stop();
        ClearCommandState();
        g_state.last_status = config_status;
        return config_status;
    }

    const int32_t delta_mm_s =
        AngularToWheelDeltaMmPerSecond(angular_mdeg_s,
                                       g_state.config.wheel_track_mm);
    const int32_t left_wheel_mm_s = SaturateInt32(
        static_cast<int64_t>(linear_mm_s) - delta_mm_s);
    const int32_t right_wheel_mm_s = SaturateInt32(
        static_cast<int64_t>(linear_mm_s) + delta_mm_s);
    const int32_t left_rpm = WheelMmPerSecondToRpm(
        left_wheel_mm_s, g_state.config.wheel_radius_um);
    const int32_t right_rpm = WheelMmPerSecondToRpm(
        right_wheel_mm_s, g_state.config.wheel_radius_um);
    const drivers::DriverStatus status =
        Chassis_SetWheelRpm(left_rpm, right_rpm);
    if (status == drivers::DRIVER_OK) {
        g_state.target_linear_mm_s = linear_mm_s;
        g_state.target_angular_mdeg_s = angular_mdeg_s;
    }
    g_state.last_status = status;
    return status;
}

drivers::DriverStatus Chassis_TrackMotorPosition(bool)
{
    return drivers::DRIVER_ERROR_UNSUPPORTED;
}

void Chassis_ReleaseMotorCommand(bool)
{
}

void Chassis_ReleaseAllMotorCommands(void)
{
    if (g_state.initialized) {
        (void) Chassis_Stop();
    }
}

drivers::DriverStatus Chassis_ControlUpdate(void)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    const drivers::DriverStatus status =
        LocalMotorController_Update(services::Time_Millis());
    g_state.last_status = status;
    return status;
}

drivers::DriverStatus Chassis_Service(void)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    drivers::DriverStatus status = RefreshConfig();
    if (status == drivers::DRIVER_OK) {
        status = LocalMotorController_RefreshLease(services::Time_Millis());
    }
    if (status != drivers::DRIVER_OK) {
        (void) LocalMotorController_Stop();
        ClearCommandState();
    }
    g_state.last_status = status;
    return status;
}

drivers::DriverStatus Chassis_Update(void)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    const LocalMotorState *local = LocalMotorController_GetState();
    if ((local == 0) || (!local->initialized)) {
        g_state.last_feedback_status =
            drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return g_state.last_feedback_status;
    }

    CopyFeedback(*local);
    g_state.last_feedback_status = local->last_status;
    if (local->last_status == drivers::DRIVER_OK) {
        g_state.last_feedback_ms = services::Time_Millis();
        if (g_state.feedback_sequence != UINT32_MAX) {
            g_state.feedback_sequence++;
        }
    }
    return g_state.last_feedback_status;
}

const ChassisState *Chassis_GetState(void)
{
    return &g_state;
}

} /* namespace app */

#elif !FEATURE_ENABLE_MOTOR_DRIVER

namespace app {
namespace {
ChassisState g_state = {};
}

drivers::DriverStatus Chassis_Init(void) { return drivers::DRIVER_ERROR_UNSUPPORTED; }
drivers::DriverStatus Chassis_Stop(void) { return drivers::DRIVER_ERROR_UNSUPPORTED; }
drivers::DriverStatus Chassis_SetWheelRpm(int32_t, int32_t) { return drivers::DRIVER_ERROR_UNSUPPORTED; }
drivers::DriverStatus Chassis_SetVelocity(int32_t, int32_t) { return drivers::DRIVER_ERROR_UNSUPPORTED; }
drivers::DriverStatus Chassis_TrackMotorPosition(bool) { return drivers::DRIVER_ERROR_UNSUPPORTED; }
void Chassis_ReleaseMotorCommand(bool) {}
void Chassis_ReleaseAllMotorCommands(void) {}
drivers::DriverStatus Chassis_ControlUpdate(void) { return drivers::DRIVER_ERROR_UNSUPPORTED; }
drivers::DriverStatus Chassis_Service(void) { return drivers::DRIVER_ERROR_UNSUPPORTED; }
drivers::DriverStatus Chassis_Update(void) { return drivers::DRIVER_ERROR_UNSUPPORTED; }
const ChassisState *Chassis_GetState(void) { return &g_state; }

} /* namespace app */

#endif
