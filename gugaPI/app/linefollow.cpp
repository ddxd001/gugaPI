#include "app/linefollow.h"

#include "app/app_grayscale.h"
#include "app/chassis.h"
#include "app/config_store.h"
#include "drivers/common/driver_status.h"
#include "services/fault.h"
#include "services/time.h"

namespace app {
namespace {

static const uint32_t kCalDurationMs = 2000U;
static const int32_t kControlScale = 1000000;
static const uint32_t kGrayscaleMaxAgeMs = 200U;

LFState g_state;

bool IsGrayscaleFresh(const AppGrayscaleData *data, uint32_t now_ms)
{
    return (data != 0) && data->valid && data->processed_valid &&
           ((now_ms - data->last_update_ms) <= kGrayscaleMaxAgeMs);
}

void SafetyStop(services::FaultCode code)
{
    g_state.mode = LF_IDLE;
    (void) Chassis_Stop();
    if (code != services::FAULT_NONE) {
        services::Fault_Set(code);
    }
}

drivers::DriverStatus ApplyWheelCommand(int32_t base_rpm,
                                        int32_t correction_rpm)
{
    const int32_t left = base_rpm - correction_rpm;
    const int32_t right = base_rpm + correction_rpm;
    const drivers::DriverStatus status = Chassis_SetWheelRpm(left, right);
    g_state.last_status = status;
    if (status != drivers::DRIVER_OK) {
        SafetyStop(services::FAULT_NONE);
    }
    return status;
}

void LoadConfig(void)
{
    const ConfigStoreParams *params = ConfigStore_Get();
    if (params == 0) {
        g_state.kp = 10000;
        g_state.kd = 0;
        g_state.max_correction_rpm = 30;
        g_state.lost_hold_ms = 150U;
        g_state.lost_timeout_ms = 500U;
        return;
    }
    g_state.kp = params->linefollow_kp;
    g_state.kd = params->linefollow_kd;
    g_state.max_correction_rpm =
        static_cast<int32_t>(params->linefollow_max_correction_rpm);
    g_state.lost_hold_ms = params->linefollow_lost_hold_ms;
    g_state.lost_timeout_ms = params->linefollow_lost_stop_ms;
}

void UpdateCalibrationMode(void)
{
    const AppGrayscaleCalibrationStatus *status =
        App_GrayscaleGetCalibrationStatus();
    if ((status != 0) && status->running) {
        return;
    }
    g_state.mode = LF_IDLE;
    if ((status != 0) && (status->last_status == drivers::DRIVER_OK)) {
        g_state.calibrated = true;
        g_state.last_status = drivers::DRIVER_OK;
    } else {
        g_state.calibrated = false;
        g_state.last_status = (status != 0)
            ? status->last_status
            : drivers::DRIVER_ERROR;
    }
}

int32_t CalculateCorrection(const AppGrayscaleData *data)
{
    const int32_t error = data->line_position;
    int32_t derivative = 0;
    if ((g_state.last_frame_ms != 0U) &&
        (data->last_update_ms != g_state.last_frame_ms)) {
        const uint32_t dt_ms = data->last_update_ms - g_state.last_frame_ms;
        const int64_t numerator =
            static_cast<int64_t>(error - g_state.last_error_mpos) * 1000LL;
        derivative = static_cast<int32_t>(numerator / dt_ms);
    }

    /* One-pole derivative filtering suppresses frame-to-frame ADC noise while
     * retaining the position term without additional latency. */
    g_state.derivative_mpos_per_s =
        (g_state.derivative_mpos_per_s * 3 + derivative) / 4;
    const int64_t proportional =
        static_cast<int64_t>(error) * static_cast<int64_t>(g_state.kp);
    const int64_t differential =
        static_cast<int64_t>(g_state.derivative_mpos_per_s) *
        static_cast<int64_t>(g_state.kd);
    const int64_t combined = (proportional + differential) / kControlScale;

    int32_t correction;
    if (combined > g_state.max_correction_rpm) {
        correction = g_state.max_correction_rpm;
    } else if (combined < -g_state.max_correction_rpm) {
        correction = -g_state.max_correction_rpm;
    } else {
        correction = static_cast<int32_t>(combined);
    }

    g_state.error_mpos = error;
    g_state.last_error_mpos = error;
    g_state.last_frame_ms = data->last_update_ms;
    g_state.correction_rpm = correction;
    return correction;
}

} /* namespace */

void LF_Init(void)
{
    g_state = {};
    g_state.mode = LF_IDLE;
    g_state.lost = true;
    g_state.road_type = GRAYSCALE_ROAD_UNKNOWN;
    g_state.last_status = drivers::DRIVER_OK;
    LoadConfig();
}

drivers::DriverStatus LF_CalibrateStart(void)
{
    if (g_state.mode != LF_IDLE) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    const drivers::DriverStatus status =
        App_GrayscaleStartSweepCalibration(kCalDurationMs);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    g_state.calibrated = false;
    g_state.mode = LF_CAL;
    g_state.last_status = drivers::DRIVER_ERROR_BUSY;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus LF_Start(int32_t base_rpm, uint32_t duration_ms)
{
    if (g_state.mode != LF_IDLE) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if ((!g_state.calibrated) &&
        App_GrayscaleCalibrationIsCommissioned()) {
        g_state.calibrated = true;
    }
    if (!g_state.calibrated) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (services::Fault_HasFault()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    const AppGrayscaleData *data = App_GrayscaleGetData();
    if (!IsGrayscaleFresh(data, services::Time_Millis())) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    g_state.mode = LF_FOLLOW;
    g_state.base_rpm = base_rpm;
    g_state.follow_start_ms = services::Time_Millis();
    g_state.follow_duration_ms = duration_ms;
    g_state.correction_rpm = 0;
    g_state.error_mpos = 0;
    g_state.last_error_mpos = 0;
    g_state.derivative_mpos_per_s = 0;
    g_state.last_sequence = 0U;
    g_state.last_frame_ms = 0U;
    g_state.processed_frame_count = 0U;
    g_state.lost_since_ms = 0U;
    g_state.lost = false;
    g_state.road_type = data->road_type;
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus LF_Stop(void)
{
    if (g_state.mode == LF_CAL) {
        App_GrayscaleCancelCalibration();
    }
    g_state.mode = LF_IDLE;
    const drivers::DriverStatus status = Chassis_Stop();
    g_state.last_status = status;
    return status;
}

void LF_Update(void)
{
    if (g_state.mode == LF_IDLE) {
        return;
    }
    if (g_state.mode == LF_CAL) {
        UpdateCalibrationMode();
        return;
    }

    const uint32_t now = services::Time_Millis();
    if (services::Fault_HasFault()) {
        SafetyStop(services::FAULT_NONE);
        return;
    }
    if ((g_state.follow_duration_ms != 0U) &&
        ((now - g_state.follow_start_ms) >= g_state.follow_duration_ms)) {
        (void) LF_Stop();
        return;
    }

    const AppGrayscaleData *data = App_GrayscaleGetData();
    if (!IsGrayscaleFresh(data, now)) {
        SafetyStop(services::FAULT_SENSOR_LOST);
        return;
    }
    if (data->sequence == g_state.last_sequence) {
        return;
    }
    g_state.last_sequence = data->sequence;
    g_state.processed_frame_count++;
    g_state.road_type = data->road_type;

    if (!data->line_detected) {
        g_state.lost = true;
        g_state.error_mpos = 0;
        if (g_state.lost_since_ms == 0U) {
            g_state.lost_since_ms = now;
        }
        const uint32_t lost_ms = now - g_state.lost_since_ms;
        if (lost_ms >= g_state.lost_timeout_ms) {
            (void) LF_Stop();
            return;
        }

        /* First slow down while holding the previous turn. After the hold
         * interval, use the same correction around zero base speed to search
         * in the last known line direction. */
        const int32_t search_base = (lost_ms <= g_state.lost_hold_ms)
            ? (g_state.base_rpm / 2)
            : 0;
        (void) ApplyWheelCommand(search_base, g_state.correction_rpm);
        return;
    }

    g_state.lost = false;
    g_state.lost_since_ms = 0U;
    const int32_t correction = CalculateCorrection(data);
    (void) ApplyWheelCommand(g_state.base_rpm, correction);
}

const LFState *LF_GetState(void)
{
    return &g_state;
}

bool LF_IsLineDetected(void)
{
    const AppGrayscaleData *data = App_GrayscaleGetData();
    return IsGrayscaleFresh(data, services::Time_Millis()) &&
           data->line_detected;
}

void LF_SetKp(int32_t kp)
{
    if (kp >= 0) {
        g_state.kp = kp;
        (void) ConfigStore_Set("lf_kp", kp);
    }
}

void LF_SetKd(int32_t kd)
{
    if (kd >= 0) {
        g_state.kd = kd;
        (void) ConfigStore_Set("lf_kd", kd);
    }
}

void LF_SetMaxCorrection(int32_t max_correction_rpm)
{
    if (max_correction_rpm >= 0) {
        g_state.max_correction_rpm = max_correction_rpm;
        (void) ConfigStore_Set("lf_maxcorr", max_correction_rpm);
    }
}

void LF_SetLostHold(uint32_t hold_ms)
{
    if (hold_ms <= g_state.lost_timeout_ms) {
        g_state.lost_hold_ms = hold_ms;
        (void) ConfigStore_Set("lf_lost_hold_ms",
                               static_cast<int32_t>(hold_ms));
    }
}

void LF_SetLostTimeout(uint32_t timeout_ms)
{
    if (timeout_ms >= g_state.lost_hold_ms) {
        g_state.lost_timeout_ms = timeout_ms;
        (void) ConfigStore_Set("lf_lost_stop_ms",
                               static_cast<int32_t>(timeout_ms));
    }
}

} /* namespace app */
