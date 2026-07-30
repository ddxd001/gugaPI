/* Host-only regression test. Keep this outside the CCS project tree so its
 * main() is never linked into the target firmware. */
#include <assert.h>
#include <stdint.h>

#include "app/app_grayscale.h"
#include "app/app_infrared_sensor.h"
#include "app/chassis.h"
#include "app/config_store.h"
#include "app/line_sensor.h"
#include "app/linefollow.h"
#include "services/fault.h"
#include "services/time.h"

namespace {

uint32_t g_now;
app::LineSensorSnapshot g_line;
app::ChassisState g_chassis;
app::ConfigStoreParams g_params;
app::AppGrayscaleCalibrationStatus g_calibration;
services::FaultCode g_fault;
uint32_t g_stop_count;
uint32_t g_param_set_count;

void MakeStrong(int16_t position)
{
    g_line.valid = true;
    g_line.fresh = true;
    g_line.calibrated = true;
    g_line.line_detected = true;
    g_line.position_valid = true;
    g_line.line_position = position;
    g_line.position_confidence = 800U;
    g_line.channel_anomaly_mask = 0U;
    g_line.weak_tracking_frames = 0U;
    g_line.invalid_frames = 0U;
    g_line.position_source = drivers::GRAYSCALE_POSITION_CORE;
    g_line.track_state = drivers::GRAYSCALE_TRACK_VALID;
    g_line.road_type = app::GRAYSCALE_ROAD_STRAIGHT;
    g_line.road_phase = app::GRAYSCALE_ROAD_PHASE_NORMAL;
    g_line.road_observed_paths = 0U;
}

void MakeWeak(int16_t position, uint8_t weak_frames)
{
    MakeStrong(position);
    g_line.position_confidence = 250U;
    g_line.weak_tracking_frames = weak_frames;
}

void MakeLost(uint8_t invalid_frames)
{
    g_line.valid = true;
    g_line.fresh = true;
    g_line.calibrated = true;
    g_line.line_detected = false;
    g_line.position_valid = false;
    g_line.position_confidence = 0U;
    g_line.weak_tracking_frames = 0U;
    g_line.invalid_frames = invalid_frames;
    g_line.track_state = drivers::GRAYSCALE_TRACK_LOST;
    g_line.road_type = app::GRAYSCALE_ROAD_LOST;
    g_line.road_phase = app::GRAYSCALE_ROAD_PHASE_NORMAL;
    g_line.road_observed_paths = 0U;
}

void PublishAt(uint32_t now)
{
    g_now = now;
    g_line.last_update_ms = now;
    g_line.sequence++;
    app::LF_Update();
}

void ResetFixture(void)
{
    g_now = 1000U;
    g_line = {};
    g_line.source = app::LINE_SENSOR_ADC8;
    g_chassis = {};
    g_params = {};
    g_params.linefollow_kp = 4000;
    g_params.linefollow_kd = 600;
    g_params.linefollow_max_correction_rpm = 30U;
    g_params.linefollow_max_steering_permille = 400U;
    g_params.linefollow_correction_slew_permille_per_second = 25000U;
    g_params.linefollow_lost_hold_ms = 150U;
    g_params.linefollow_lost_stop_ms = 500U;
    g_params.infrared_linefollow_kp = 4000;
    g_params.infrared_linefollow_kd = 600;
    g_params.infrared_linefollow_max_correction_rpm = 30U;
    g_params.infrared_linefollow_correction_slew_permille_per_second = 25000U;
    g_calibration = {};
    g_fault = services::FAULT_NONE;
    g_stop_count = 0U;
    g_param_set_count = 0U;
    MakeStrong(1000);
    g_line.sequence = 1U;
    g_line.last_update_ms = g_now;
    app::LF_Init();
    assert(app::LF_Start(110, 0U) == drivers::DRIVER_OK);
    app::LF_Update();
    assert(g_chassis.left.target_rpm != 0);
    assert(g_chassis.right.target_rpm != 0);
}

} /* namespace */

namespace services {

uint32_t Time_Millis(void)
{
    return g_now;
}

bool Fault_HasFault(void)
{
    return g_fault != FAULT_NONE;
}

void Fault_Set(FaultCode code)
{
    if ((g_fault == FAULT_NONE) && (code != FAULT_NONE)) {
        g_fault = code;
    }
}

} /* namespace services */

namespace app {

const ConfigStoreParams *ConfigStore_Get(void)
{
    return &g_params;
}

drivers::DriverStatus ConfigStore_Set(const char *, int32_t)
{
    g_param_set_count++;
    return drivers::DRIVER_OK;
}

LineSensorSource LineSensor_GetSource(void)
{
    return g_line.source;
}

const LineSensorSnapshot *LineSensor_GetSnapshot(void)
{
    return &g_line;
}

drivers::DriverStatus Chassis_Stop(void)
{
    g_chassis.left.target_rpm = 0;
    g_chassis.right.target_rpm = 0;
    g_stop_count++;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Chassis_SetWheelRpm(int32_t left_rpm,
                                          int32_t right_rpm)
{
    g_chassis.left.target_rpm = left_rpm;
    g_chassis.right.target_rpm = right_rpm;
    return drivers::DRIVER_OK;
}

const ChassisState *Chassis_GetState(void)
{
    return &g_chassis;
}

const AppGrayscaleCalibrationStatus *App_GrayscaleGetCalibrationStatus(void)
{
    return &g_calibration;
}

drivers::DriverStatus App_GrayscaleStartSweepCalibration(uint32_t)
{
    return drivers::DRIVER_OK;
}

void App_GrayscaleCancelCalibration(void)
{
}

void App_InfraredSensorRecordControlLatency(uint32_t)
{
}

} /* namespace app */

int main()
{
    /* The final steering ratio is runtime configurable and scales the actual
     * wheel differential independently of the reference-speed maxcorr. */
    ResetFixture();
    app::LF_SetKp(1000000);
    app::LF_SetMaxCorrection(500);
    app::LF_SetCorrectionSlew(UINT16_MAX);
    app::LF_SetMaxSteeringRatio(500U);
    MakeStrong(3000);
    PublishAt(1007U);
    assert(app::LF_GetState()->max_steering_permille == 500U);
    assert(app::LF_GetState()->correction_rpm == 55);
    assert(g_chassis.left.target_rpm == 55);
    assert(g_chassis.right.target_rpm == 165);

    /* A weak but explicitly valid position remains in normal PID control. It
     * must not start the hold state that caused the captured false stop. */
    ResetFixture();
    const int32_t initial_left = g_chassis.left.target_rpm;
    const int32_t initial_right = g_chassis.right.target_rpm;
    MakeWeak(287, 4U);
    PublishAt(1007U);
    assert(app::LF_GetState()->recovery_mode == app::LF_RECOVERY_NONE);
    assert(app::LF_GetState()->mode == app::LF_FOLLOW);
    assert(app::LF_GetState()->error_mpos == 287);
    assert((g_chassis.left.target_rpm != initial_left) ||
           (g_chassis.right.target_rpm != initial_right));

    /* A true geometry loss repeats the exact wheel command that immediately
     * preceded it. It neither decelerates nor stops after the legacy timeout. */
    const int32_t reliable_left = g_chassis.left.target_rpm;
    const int32_t reliable_right = g_chassis.right.target_rpm;
    MakeLost(6U);
    PublishAt(1117U);
    assert(app::LF_GetState()->mode == app::LF_FOLLOW);
    assert(app::LF_GetState()->recovery_mode == app::LF_RECOVERY_HOLD);
    assert(app::LF_GetState()->recovery_elapsed_ms == 0U);
    assert(g_chassis.left.target_rpm == reliable_left);
    assert(g_chassis.right.target_rpm == reliable_right);
    assert(app::LF_IsLineDetected());
    PublishAt(5000U);
    assert(app::LF_GetState()->mode == app::LF_FOLLOW);
    assert(app::LF_GetState()->recovery_mode == app::LF_RECOVERY_HOLD);
    assert(app::LF_GetState()->recovery_elapsed_ms == 3883U);
    assert(g_chassis.left.target_rpm == reliable_left);
    assert(g_chassis.right.target_rpm == reliable_right);
    assert(g_stop_count == 0U);

    /* One or two valid frames cannot bounce the recovery latch. The third
     * valid frame, including a weak valid position, resumes PID with a cleared
     * derivative history. */
    MakeWeak(-300, 4U);
    PublishAt(5007U);
    assert(app::LF_GetState()->recovery_confirm_frames == 1U);
    assert(g_chassis.left.target_rpm == reliable_left);
    assert(g_chassis.right.target_rpm == reliable_right);
    PublishAt(5014U);
    assert(app::LF_GetState()->recovery_confirm_frames == 2U);
    assert(g_chassis.left.target_rpm == reliable_left);
    assert(g_chassis.right.target_rpm == reliable_right);
    PublishAt(5021U);
    assert(app::LF_GetState()->recovery_mode == app::LF_RECOVERY_NONE);
    assert(app::LF_GetState()->derivative_mpos_per_s == 0);
    assert(app::LF_GetState()->mode == app::LF_FOLLOW);
    assert(app::LF_GetState()->error_mpos == -300);

    /* Hardware freshness and anomaly failures remain immediate stops. */
    ResetFixture();
    g_line.channel_anomaly_mask = 0x01U;
    PublishAt(1010U);
    assert(app::LF_GetState()->mode == app::LF_IDLE);
    assert(g_fault == services::FAULT_SENSOR_LOST);
    assert(g_stop_count == 1U);

    ResetFixture();
    g_line.fresh = false;
    PublishAt(1010U);
    assert(app::LF_GetState()->mode == app::LF_IDLE);
    assert(g_fault == services::FAULT_SENSOR_LOST);
    assert(g_stop_count == 1U);

    /* A bounded forward road observation retains the existing pass-through
     * behavior and does not enter ordinary ADC8 recovery. */
    ResetFixture();
    MakeLost(1U);
    g_line.road_type = app::GRAYSCALE_ROAD_LEFT_BRANCH;
    g_line.road_observed_paths = app::GRAYSCALE_ROAD_PATH_FORWARD;
    PublishAt(1010U);
    assert(app::LF_GetState()->recovery_mode == app::LF_RECOVERY_NONE);
    assert(g_chassis.left.target_rpm == 110);
    assert(g_chassis.right.target_rpm == 110);

    /* No runtime or persistent parameter write is part of this behavior. */
    assert(g_param_set_count == 0U);
    return 0;
}
