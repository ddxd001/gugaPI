#include "app/chassis.h"

#include "app/config_store.h"
#include "app/motor_driver_client.h"
#include "config/feature_config.h"

namespace app {
namespace {

namespace motor = motor_driver_client;

static const uint32_t kPiMicro = 3141593U;

motor::Client g_motorClient = { motor::TRANSPORT_I2C,
                                motor::kI2cDefaultAddress };

ChassisState g_state = {
    false,
    0,
    0,
    { 0, 0, 0, 0, 0U },
    { 0, 0, 0, 0, 0U },
    {
        32U,
        160U,
        22400U,
        22400U,
        static_cast<uint16_t>(motor::kSpeedMaxRpm)
    },
    drivers::DRIVER_ERROR_NOT_INITIALIZED
};

bool g_motionLeaseActive = false;

int32_t AbsInt32(int32_t value)
{
    return (value < 0) ? -value : value;
}

drivers::DriverStatus ClampWheelRpm(int32_t rpm, uint16_t max_rpm)
{
    return (AbsInt32(rpm) <= static_cast<int32_t>(max_rpm)) ?
           drivers::DRIVER_OK : drivers::DRIVER_ERROR_INVALID_ARG;
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
    return static_cast<int32_t>(numerator / denominator);
}

int32_t WheelMmPerSecondToRpm(int32_t wheel_mm_s,
                              uint32_t wheel_radius_mm,
                              uint32_t counts_per_rev)
{
    if (counts_per_rev == 0U) {
        return 0;
    }

    const int64_t numerator =
        static_cast<int64_t>(wheel_mm_s) *
        static_cast<int64_t>(counts_per_rev) * 60LL * 1000000LL;
    const int64_t denominator =
        2LL * static_cast<int64_t>(kPiMicro) *
        static_cast<int64_t>(wheel_radius_mm) *
        static_cast<int64_t>(counts_per_rev);

    return DivideRoundInt64(numerator, denominator);
}

int32_t AngularToWheelDeltaMmPerSecond(int32_t angular_mdeg_s,
                                       uint32_t wheel_track_mm)
{
    const int64_t numerator =
        static_cast<int64_t>(angular_mdeg_s) *
        static_cast<int64_t>(wheel_track_mm) *
        static_cast<int64_t>(kPiMicro);
    const int64_t denominator = 360000LL * 1000000LL;

    return DivideRoundInt64(numerator, denominator);
}

drivers::DriverStatus SetOneWheelRpm(bool motor1, int32_t rpm)
{
    motor::MotionResult motion = {};
    const uint16_t target_rpm = static_cast<uint16_t>(AbsInt32(rpm));
    const bool reverse = (rpm < 0);

    const drivers::DriverStatus status =
        motor::SetSpeed(&g_motorClient, motor1, target_rpm, reverse, &motion);
    if (motion.target_status != drivers::DRIVER_OK) {
        return motion.target_status;
    }
    if (!motion.target_ack) {
        return drivers::DRIVER_ERROR;
    }
    if (motion.mode_status != drivers::DRIVER_OK) {
        return motion.mode_status;
    }
    return motion.mode_ack ? status : drivers::DRIVER_ERROR;
}

drivers::DriverStatus ReadWheelState(bool motor1,
                                     ChassisWheelState *wheel_state,
                                     int32_t actual_rpm)
{
    motor::EncoderData encoder = {};

    if (wheel_state == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    wheel_state->actual_rpm = actual_rpm;
    const drivers::DriverStatus status =
        motor::ReadEncoder(&g_motorClient, motor1, &encoder);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    wheel_state->encoder_count = encoder.count;
    wheel_state->encoder_counts_per_second = encoder.counts_per_second;
    wheel_state->encoder_state = encoder.state;
    return drivers::DRIVER_OK;
}

void SetLastStatus(drivers::DriverStatus status)
{
    g_state.last_status = status;
}

void ClearMotionCommandState(void)
{
    g_motionLeaseActive = false;
    g_state.target_linear_mm_s = 0;
    g_state.target_angular_mdeg_s = 0;
    g_state.left.target_rpm = 0;
    g_state.right.target_rpm = 0;
}

drivers::DriverStatus StopMotorsForSafety(void)
{
    motor::StopResult stop_result = {};

    ClearMotionCommandState();
    return motor::Stop(&g_motorClient, &stop_result);
}

void RefreshConfig(void)
{
    const ConfigStoreParams *params = ConfigStore_Get();
    if (params == 0) {
        return;
    }

    g_state.config.wheel_radius_mm = params->wheel_radius_mm;
    g_state.config.wheel_track_mm = params->wheel_track_mm;
    g_state.config.left_counts_per_rev = params->left_counts_per_rev;
    g_state.config.right_counts_per_rev = params->right_counts_per_rev;
    g_state.config.max_wheel_rpm = params->max_wheel_rpm;
}

drivers::DriverStatus ApplyPersistentMotorConfig(void)
{
    const ConfigStoreParams *params = ConfigStore_Get();
    if (params == 0) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    drivers::DriverStatus status =
        motor::SetCountsPerRev(&g_motorClient,
                               params->left_counts_per_rev,
                               params->right_counts_per_rev);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    const uint8_t speed_pid[motor::kSpeedPidLength] = {
        params->speed_kp_q4_4,
        params->speed_ki_q4_4,
        params->speed_kd_q4_4,
        params->speed_max_duty,
        params->speed_min_duty
    };
    status = motor::SetPid(&g_motorClient,
                           speed_pid,
                           motor::kSpeedPidLength);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    uint8_t position_pid[motor::kPositionPidLength] = {
        params->position_kp_q4_4,
        params->position_ki_q4_4,
        params->position_kd_q4_4,
        0U,
        0U,
        0U,
        0U
    };
    motor::EncodeUint16Le(params->position_max_rpm, &position_pid[3]);
    motor::EncodeUint16Le(params->position_tolerance_counts, &position_pid[5]);
    status = motor::SetPositionPid(&g_motorClient,
                                   position_pid,
                                   motor::kPositionPidLength);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    const motor::InvertConfig invert = {
        static_cast<uint8_t>(params->motor_output_invert_flags &
                             motor::kInvertValidMask),
        static_cast<uint8_t>(params->motor_encoder_invert_flags &
                             motor::kInvertValidMask),
    };
    return motor::SetInvertConfig(&g_motorClient, invert);
}

} /* namespace */

drivers::DriverStatus Chassis_Init(void)
{
#if FEATURE_ENABLE_MOTOR_DRIVER
    motor::Init(&g_motorClient);
    RefreshConfig();
    g_state.initialized = true;
    ClearMotionCommandState();
    g_state.last_status = ApplyPersistentMotorConfig();
    return g_state.last_status;
#else
    g_state.initialized = false;
    ClearMotionCommandState();
    g_state.last_status = drivers::DRIVER_ERROR_UNSUPPORTED;
    return drivers::DRIVER_ERROR_UNSUPPORTED;
#endif
}

drivers::DriverStatus Chassis_Stop(void)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    const drivers::DriverStatus status = StopMotorsForSafety();
    SetLastStatus(status);
    return status;
}

drivers::DriverStatus RefreshOneWheelRpm(bool motor1, int32_t rpm)
{
    bool ack = false;
    return motor::WriteTargetRpm(&g_motorClient,
                                 motor1,
                                 static_cast<uint16_t>(AbsInt32(rpm)),
                                 &ack);
}

drivers::DriverStatus Chassis_SetWheelRpm(int32_t left_rpm,
                                          int32_t right_rpm)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    RefreshConfig();

    drivers::DriverStatus status =
        ClampWheelRpm(left_rpm, g_state.config.max_wheel_rpm);
    if (status != drivers::DRIVER_OK) {
        SetLastStatus(status);
        return status;
    }
    status = ClampWheelRpm(right_rpm, g_state.config.max_wheel_rpm);
    if (status != drivers::DRIVER_OK) {
        SetLastStatus(status);
        return status;
    }

    status = SetOneWheelRpm(true, left_rpm);
    if (status != drivers::DRIVER_OK) {
        (void) StopMotorsForSafety();
        SetLastStatus(status);
        return status;
    }

    status = SetOneWheelRpm(false, right_rpm);
    if (status != drivers::DRIVER_OK) {
        (void) StopMotorsForSafety();
        SetLastStatus(status);
        return status;
    }

    g_motionLeaseActive = true;
    g_state.target_linear_mm_s = 0;
    g_state.target_angular_mdeg_s = 0;
    g_state.left.target_rpm = left_rpm;
    g_state.right.target_rpm = right_rpm;
    SetLastStatus(drivers::DRIVER_OK);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Chassis_SetVelocity(int32_t linear_mm_s,
                                          int32_t angular_mdeg_s)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    RefreshConfig();

    const int32_t delta_mm_s =
        AngularToWheelDeltaMmPerSecond(angular_mdeg_s,
                                       g_state.config.wheel_track_mm);
    const int32_t left_mm_s = linear_mm_s - delta_mm_s;
    const int32_t right_mm_s = linear_mm_s + delta_mm_s;
    const int32_t left_rpm =
        WheelMmPerSecondToRpm(left_mm_s,
                              g_state.config.wheel_radius_mm,
                              g_state.config.left_counts_per_rev);
    const int32_t right_rpm =
        WheelMmPerSecondToRpm(right_mm_s,
                              g_state.config.wheel_radius_mm,
                              g_state.config.right_counts_per_rev);

    const drivers::DriverStatus status =
        Chassis_SetWheelRpm(left_rpm, right_rpm);
    if (status == drivers::DRIVER_OK) {
        g_state.target_linear_mm_s = linear_mm_s;
        g_state.target_angular_mdeg_s = angular_mdeg_s;
    }
    SetLastStatus(status);
    return status;
}

drivers::DriverStatus Chassis_Service(void)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    if (!g_motionLeaseActive) {
        return drivers::DRIVER_OK;
    }

    drivers::DriverStatus status =
        RefreshOneWheelRpm(true, g_state.left.target_rpm);
    if (status == drivers::DRIVER_OK) {
        status = RefreshOneWheelRpm(false, g_state.right.target_rpm);
    }
    if (status != drivers::DRIVER_OK) {
        (void) StopMotorsForSafety();
        SetLastStatus(status);
        return status;
    }

    SetLastStatus(drivers::DRIVER_OK);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Chassis_Update(void)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    RefreshConfig();

    motor::RpmData rpm = {};
    drivers::DriverStatus status = motor::ReadRpm(&g_motorClient, &rpm);
    if (status != drivers::DRIVER_OK) {
        SetLastStatus(status);
        return status;
    }

    status = ReadWheelState(true, &g_state.left, rpm.actual_m1);
    if (status != drivers::DRIVER_OK) {
        SetLastStatus(status);
        return status;
    }

    status = ReadWheelState(false, &g_state.right, rpm.actual_m2);
    SetLastStatus(status);
    return status;
}

const ChassisState *Chassis_GetState(void)
{
    return &g_state;
}

} /* namespace app */
