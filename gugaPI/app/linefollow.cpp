#include "app/linefollow.h"

#include "app/app_grayscale.h"
#include "app/app_infrared_sensor.h"
#include "app/chassis.h"
#include "app/config_store.h"
#include "app/line_sensor.h"
#include "app/linefollow_road_handoff.h"
#include "drivers/common/driver_status.h"
#include "services/fault.h"
#include "services/time.h"

namespace app {
namespace {

static const uint32_t kCalDurationMs = 2000U;
static const int32_t kControlScale = 1000000;

LFState g_state;

void ClearRecovery(void)
{
    g_state.recovery_mode = LF_RECOVERY_NONE;
    g_state.recovery_start_ms = 0U;
    g_state.recovery_elapsed_ms = 0U;
    g_state.recovery_confirm_frames = 0U;
}

bool IsSensorFresh(const LineSensorSnapshot *data)
{
    return (data != 0) && data->valid && data->fresh;
}

bool IsTrackValid(const LineSensorSnapshot *data)
{
    return (data != 0) && data->line_detected && data->position_valid &&
           (data->track_state == drivers::GRAYSCALE_TRACK_VALID) &&
           (data->channel_anomaly_mask == 0U);
}

bool IsStrongTrack(const LineSensorSnapshot *data)
{
    return IsTrackValid(data) && (data->weak_tracking_frames == 0U) &&
           (data->position_confidence >= LF_STRONG_CONFIDENCE_MIN);
}

void SafetyStop(services::FaultCode code)
{
    g_state.mode = LF_IDLE;
    ClearRecovery();
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
    if (status == drivers::DRIVER_OK) {
        /* Keep the exact latest wheel command as the ADC8 search command.
         * Re-applying it during recovery is intentional and does not change
         * the saved pair. */
        g_state.recovery_base_rpm = base_rpm;
        g_state.recovery_correction_rpm = correction_rpm;
    }
    if (LineSensor_GetSource() == LINE_SENSOR_IR3) {
        App_InfraredSensorRecordControlLatency(
            LineSensor_GetSnapshot()->sequence);
    }
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
        g_state.kp = 3800;
        g_state.kd = 600;
        g_state.max_correction_rpm = 30;
        g_state.max_steering_permille =
            LF_DEFAULT_MAX_STEERING_PERMILLE;
        g_state.correction_slew_permille_per_second =
            LF_DEFAULT_CORRECTION_SLEW_PERMILLE_PER_SECOND;
        g_state.lost_hold_ms = 150U;
        g_state.lost_timeout_ms = 500U;
        return;
    }
    if (LineSensor_GetSource() == LINE_SENSOR_IR3) {
        g_state.kp = params->infrared_linefollow_kp;
        g_state.kd = params->infrared_linefollow_kd;
        g_state.max_correction_rpm = static_cast<int32_t>(
            params->infrared_linefollow_max_correction_rpm);
        g_state.correction_slew_permille_per_second =
            params->
                infrared_linefollow_correction_slew_permille_per_second;
    } else {
        g_state.kp = params->linefollow_kp;
        g_state.kd = params->linefollow_kd;
        g_state.max_correction_rpm =
            static_cast<int32_t>(params->linefollow_max_correction_rpm);
        g_state.correction_slew_permille_per_second =
            params->linefollow_correction_slew_permille_per_second;
    }
    g_state.max_steering_permille =
        params->linefollow_max_steering_permille;
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
    ClearRecovery();
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

int32_t AbsoluteInt32(int32_t value)
{
    if (value >= 0) {
        return value;
    }
    return (value == INT32_MIN) ? INT32_MAX : -value;
}

int32_t ClampToMagnitude(int32_t value, int32_t magnitude)
{
    if (value > magnitude) {
        return magnitude;
    }
    if (value < -magnitude) {
        return -magnitude;
    }
    return value;
}

int32_t ApplyCorrectionSlew(int32_t requested,
                            int32_t base_rpm,
                            uint32_t dt_ms,
                            bool first_frame)
{
    if (first_frame || (dt_ms == 0U)) {
        return requested;
    }

    const int32_t base_magnitude = AbsoluteInt32(base_rpm);
    const int64_t numerator =
        static_cast<int64_t>(base_magnitude) *
        static_cast<int64_t>(
            g_state.correction_slew_permille_per_second) *
        static_cast<int64_t>(dt_ms);
    int32_t maximum_delta = static_cast<int32_t>(
        (numerator + 999999LL) / 1000000LL);
    if ((base_magnitude != 0) && (maximum_delta < 1)) {
        maximum_delta = 1;
    }

    const int32_t previous = g_state.correction_rpm;
    if (requested > previous + maximum_delta) {
        return previous + maximum_delta;
    }
    if (requested < previous - maximum_delta) {
        return previous - maximum_delta;
    }
    return requested;
}

int32_t CalculateCorrection(const LineSensorSnapshot *data,
                            int32_t base_rpm,
                            bool reset_derivative)
{
    const int32_t measured_error = data->line_position;
    const int32_t error =
        (AbsoluteInt32(measured_error) <= LF_ERROR_DEADBAND_MPOS)
        ? 0
        : measured_error;
    const bool first_frame = (g_state.last_frame_ms == 0U);
    uint32_t dt_ms = 0U;
    int32_t derivative = 0;
    if ((!first_frame) &&
        (data->last_update_ms != g_state.last_frame_ms)) {
        dt_ms = data->last_update_ms - g_state.last_frame_ms;
        if (!reset_derivative) {
            const int64_t numerator =
                static_cast<int64_t>(error - g_state.last_error_mpos) *
                1000LL;
            derivative = static_cast<int32_t>(numerator / dt_ms);
        }
    }

    /* Backward-Euler one-pole filtering keeps the derivative cutoff stable
     * when the grayscale frame period changes. */
    if (reset_derivative) {
        g_state.derivative_mpos_per_s = 0;
    } else if ((!first_frame) && (dt_ms != 0U)) {
        const int64_t filtered_numerator =
            static_cast<int64_t>(g_state.derivative_mpos_per_s) *
                static_cast<int64_t>(LF_DERIVATIVE_FILTER_TAU_MS) +
            static_cast<int64_t>(derivative) * static_cast<int64_t>(dt_ms);
        g_state.derivative_mpos_per_s = static_cast<int32_t>(
            filtered_numerator /
            static_cast<int64_t>(LF_DERIVATIVE_FILTER_TAU_MS + dt_ms));
    } else {
        g_state.derivative_mpos_per_s = 0;
    }
    const int64_t proportional =
        static_cast<int64_t>(error) * static_cast<int64_t>(g_state.kp);
    const int64_t differential =
        static_cast<int64_t>(g_state.derivative_mpos_per_s) *
        static_cast<int64_t>(g_state.kd);
    const int64_t combined = (proportional + differential) / kControlScale;

    int32_t reference_correction;
    if (combined > g_state.max_correction_rpm) {
        reference_correction = g_state.max_correction_rpm;
    } else if (combined < -g_state.max_correction_rpm) {
        reference_correction = -g_state.max_correction_rpm;
    } else {
        reference_correction = static_cast<int32_t>(combined);
    }

    const int32_t base_magnitude = AbsoluteInt32(base_rpm);
    const int64_t speed_scaled =
        static_cast<int64_t>(reference_correction) *
        static_cast<int64_t>(base_magnitude);
    int32_t requested = static_cast<int32_t>(
        speed_scaled / static_cast<int64_t>(LF_REFERENCE_RPM));
    const int32_t ratio_limit = static_cast<int32_t>(
        (static_cast<int64_t>(base_magnitude) *
         static_cast<int64_t>(g_state.max_steering_permille)) / 1000LL);
    requested = ClampToMagnitude(requested, ratio_limit);
    const int32_t correction =
        ApplyCorrectionSlew(requested, base_rpm, dt_ms, first_frame);

    g_state.error_mpos = measured_error;
    g_state.last_error_mpos = error;
    g_state.last_frame_ms = data->last_update_ms;
    g_state.correction_rpm = correction;
    return correction;
}

void RecordTrackPosition(const LineSensorSnapshot *data)
{
    g_state.last_strong_position_mpos = data->line_position;
}

void BeginAdcRecovery(uint32_t now)
{
    g_state.recovery_mode = LF_RECOVERY_HOLD;
    g_state.recovery_start_ms = now;
    g_state.recovery_elapsed_ms = 0U;
    g_state.recovery_confirm_frames = 0U;
    g_state.lost_since_ms = now;
}

bool ApplyAdcRecovery(uint32_t now, uint32_t frame_ms)
{
    g_state.recovery_elapsed_ms = now - g_state.recovery_start_ms;
    g_state.recovery_mode = LF_RECOVERY_HOLD;
    g_state.correction_rpm = g_state.recovery_correction_rpm;
    g_state.last_frame_ms = frame_ms;
    return ApplyWheelCommand(g_state.recovery_base_rpm,
                             g_state.recovery_correction_rpm) ==
           drivers::DRIVER_OK;
}

bool IsForwardJunctionPassThrough(const LineSensorSnapshot *data)
{
    if ((data == 0) ||
        ((data->road_observed_paths & GRAYSCALE_ROAD_PATH_FORWARD) == 0U)) {
        return false;
    }
    return (data->road_type == GRAYSCALE_ROAD_LEFT_BRANCH) ||
           (data->road_type == GRAYSCALE_ROAD_RIGHT_BRANCH) ||
           (data->road_type == GRAYSCALE_ROAD_CROSS);
}

} /* namespace */

void LF_Init(void)
{
    g_state = {};
    g_state.mode = LF_IDLE;
    g_state.lost = true;
    g_state.road_type = GRAYSCALE_ROAD_UNKNOWN;
    g_state.last_status = drivers::DRIVER_OK;
    ClearRecovery();
    LoadConfig();
}

void LF_ReloadConfig(void)
{
    if (g_state.mode == LF_IDLE) {
        LoadConfig();
        g_state.calibrated = LineSensor_GetSnapshot()->calibrated;
        g_state.last_error_mpos = 0;
        g_state.derivative_mpos_per_s = 0;
        g_state.correction_rpm = 0;
        g_state.error_mpos = 0;
        g_state.lost_since_ms = 0U;
        g_state.lost = true;
        g_state.last_sequence = 0U;
        g_state.last_frame_ms = 0U;
        ClearRecovery();
    }
}

drivers::DriverStatus LF_CalibrateStart(void)
{
    if (g_state.mode != LF_IDLE) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (LineSensor_GetSource() != LINE_SENSOR_ADC8) {
        return drivers::DRIVER_ERROR_UNSUPPORTED;
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
    LoadConfig();
    const LineSensorSnapshot *data = LineSensor_GetSnapshot();
    g_state.calibrated = (data != 0) && data->calibrated;
    if (!g_state.calibrated) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (services::Fault_HasFault()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    if (!IsSensorFresh(data)) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if ((!data->line_detected) || (!data->position_valid) ||
        (data->track_state != drivers::GRAYSCALE_TRACK_VALID) ||
        (data->channel_anomaly_mask != 0U)) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if ((data->source == LINE_SENSOR_ADC8) && !IsStrongTrack(data)) {
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
    g_state.position_valid = data->position_valid;
    g_state.selected_mask = data->selected_mask;
    g_state.position_confidence = data->position_confidence;
    g_state.position_source = data->position_source;
    g_state.track_state = data->track_state;
    g_state.weak_tracking_frames = data->weak_tracking_frames;
    g_state.invalid_frames = data->invalid_frames;
    ClearRecovery();
    g_state.recovery_base_rpm = base_rpm;
    g_state.recovery_correction_rpm = 0;
    g_state.last_strong_position_mpos = data->line_position;
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus LF_Stop(void)
{
    if (g_state.mode == LF_CAL) {
        App_GrayscaleCancelCalibration();
    }
    g_state.mode = LF_IDLE;
    ClearRecovery();
    const drivers::DriverStatus status = Chassis_Stop();
    g_state.last_status = status;
    return status;
}

drivers::DriverStatus LF_ContinueForMotionHandoff(
    int32_t base_rpm,
    uint32_t duration_ms)
{
    if ((g_state.mode != LF_FOLLOW) || (base_rpm <= 0)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (services::Fault_HasFault()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    g_state.base_rpm = base_rpm;
    g_state.follow_start_ms = services::Time_Millis();
    g_state.follow_duration_ms = duration_ms;
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus LF_ReleaseForMotionHandoff(void)
{
    if (g_state.mode != LF_FOLLOW) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    g_state.mode = LF_IDLE;
    ClearRecovery();
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
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

    const LineSensorSnapshot *data = LineSensor_GetSnapshot();
    if (!IsSensorFresh(data)) {
        /* IR transport health is recoverable and already retained in its
         * diagnostics. Stop motion, but do not latch the global vehicle fault
         * for a transient disconnect; the operator must restart LF manually. */
        SafetyStop((data != 0) && (data->source == LINE_SENSOR_IR3)
            ? services::FAULT_NONE : services::FAULT_SENSOR_LOST);
        return;
    }
    if (data->sequence == g_state.last_sequence) {
        return;
    }
    g_state.last_sequence = data->sequence;
    g_state.processed_frame_count++;
    g_state.road_type = data->road_type;
    g_state.position_valid = data->position_valid;
    g_state.selected_mask = data->selected_mask;
    g_state.position_confidence = data->position_confidence;
    g_state.position_source = data->position_source;
    g_state.track_state = data->track_state;
    g_state.weak_tracking_frames = data->weak_tracking_frames;
    g_state.invalid_frames = data->invalid_frames;

    if (data->channel_anomaly_mask != 0U) {
        SafetyStop(services::FAULT_SENSOR_LOST);
        return;
    }
    const bool track_valid = IsTrackValid(data);
    if (!track_valid) {
        /* A branch/crossing can temporarily make the analogue geometry wide
         * while a forward path exists. A corner can also lose its center one
         * classifier frame before the confirmed event is published. Keep
         * moving straight only during those bounded classifier windows so
         * RoadEventController_Update() can pre-empt LF before LF_Stop(). */
        if (IsForwardJunctionPassThrough(data) ||
            linefollow_road_handoff::ShouldHoldForPendingRoadEvent(
                data->road_phase)) {
            g_state.lost = false;
            g_state.lost_since_ms = 0U;
            g_state.error_mpos = 0;
            g_state.correction_rpm = 0;
            ClearRecovery();
            (void) ApplyWheelCommand(g_state.base_rpm, 0);
            return;
        }
        g_state.lost = true;
        g_state.error_mpos = 0;
        if (data->source == LINE_SENSOR_ADC8) {
            if (g_state.recovery_mode == LF_RECOVERY_NONE) {
                BeginAdcRecovery(now);
            } else {
                g_state.recovery_confirm_frames = 0U;
            }
            (void) ApplyAdcRecovery(now, data->last_update_ms);
            return;
        }
        if (g_state.lost_since_ms == 0U) {
            g_state.lost_since_ms = now;
        }
        if (data->source == LINE_SENSOR_IR3) {
            const uint32_t lost_ms = now - g_state.lost_since_ms;
            if (lost_ms < g_state.lost_hold_ms) {
                return;
            }
            if (lost_ms < g_state.lost_timeout_ms) {
                (void) ApplyWheelCommand(g_state.base_rpm, 0);
                return;
            }
        }
        (void) LF_Stop();
        return;
    }

    bool reset_derivative = false;
    if ((data->source == LINE_SENSOR_ADC8) &&
        (g_state.recovery_mode != LF_RECOVERY_NONE)) {
        if (g_state.recovery_confirm_frames < UINT8_MAX) {
            g_state.recovery_confirm_frames++;
        }
        if (g_state.recovery_confirm_frames < LF_RECOVERY_CONFIRM_FRAMES) {
            (void) ApplyAdcRecovery(now, data->last_update_ms);
            return;
        }
        reset_derivative = true;
        ClearRecovery();
    }

    g_state.lost = false;
    g_state.lost_since_ms = 0U;
    const int32_t correction =
        CalculateCorrection(data, g_state.base_rpm, reset_derivative);
    if (ApplyWheelCommand(g_state.base_rpm, correction) == drivers::DRIVER_OK) {
        RecordTrackPosition(data);
    }
}

const LFState *LF_GetState(void)
{
    return &g_state;
}

bool LF_IsLineDetected(void)
{
    const LineSensorSnapshot *data = LineSensor_GetSnapshot();
    if (!IsSensorFresh(data) || (data->channel_anomaly_mask != 0U)) {
        return false;
    }
    if ((g_state.mode == LF_FOLLOW) &&
        (data->source == LINE_SENSOR_ADC8) &&
        (g_state.recovery_mode != LF_RECOVERY_NONE)) {
        return true;
    }
    return
           data->line_detected && data->position_valid &&
           (data->track_state == drivers::GRAYSCALE_TRACK_VALID) &&
           (data->channel_anomaly_mask == 0U);
}

void LF_SetKp(int32_t kp)
{
    if (kp >= 0) {
        g_state.kp = kp;
        (void) ConfigStore_Set(
            (LineSensor_GetSource() == LINE_SENSOR_IR3)
                ? "ir_lf_kp" : "lf_kp",
            kp);
    }
}

void LF_SetKd(int32_t kd)
{
    if (kd >= 0) {
        g_state.kd = kd;
        (void) ConfigStore_Set(
            (LineSensor_GetSource() == LINE_SENSOR_IR3)
                ? "ir_lf_kd" : "lf_kd",
            kd);
    }
}

void LF_SetMaxCorrection(int32_t max_correction_rpm)
{
    if (max_correction_rpm >= 0) {
        g_state.max_correction_rpm = max_correction_rpm;
        (void) ConfigStore_Set(
            (LineSensor_GetSource() == LINE_SENSOR_IR3)
                ? "ir_lf_maxcorr" : "lf_maxcorr",
            max_correction_rpm);
    }
}

void LF_SetMaxSteeringRatio(uint32_t permille)
{
    if ((permille >= LF_MIN_MAX_STEERING_PERMILLE) &&
        (permille <= LF_MAX_MAX_STEERING_PERMILLE)) {
        g_state.max_steering_permille =
            static_cast<uint16_t>(permille);
        (void) ConfigStore_Set("lf_max_ratio_permille",
                               static_cast<int32_t>(permille));
    }
}

void LF_SetCorrectionSlew(uint32_t permille_per_second)
{
    if ((permille_per_second > 0U) &&
        (permille_per_second <= UINT16_MAX)) {
        g_state.correction_slew_permille_per_second = permille_per_second;
        (void) ConfigStore_Set(
            (LineSensor_GetSource() == LINE_SENSOR_IR3)
                ? "ir_lf_slew_permille_s" : "lf_slew_permille_s",
            static_cast<int32_t>(permille_per_second));
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
