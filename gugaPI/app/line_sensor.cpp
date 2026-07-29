#include "app/line_sensor.h"

#include "app/action.h"
#include "app/app_grayscale.h"
#include "app/app_infrared_sensor.h"
#include "app/app_main.h"
#include "app/config_store.h"
#include "app/linefollow.h"
#include "app/road_event_controller.h"
#include "services/time.h"

namespace app {
namespace {

static const uint32_t kAdcMaximumAgeMs = 200U;

LineSensorSource g_source = LINE_SENSOR_IR3;
LineSensorSnapshot g_snapshot = {};

bool MotionControllerBusy(void)
{
    const LFState *linefollow = LF_GetState();
    const ActionRunnerState *action = ActionRunner_GetState();
    const RoadControlState *road = RoadEventController_GetState();
    const AppMode mode = App_GetState()->mode;
    return ((linefollow != 0) && (linefollow->mode != LF_IDLE)) ||
           ((action != 0) && action->running) ||
           RoadEventController_IsRouteActive() ||
           ((road != 0) &&
            (road->phase != ROAD_CONTROL_PHASE_IDLE) &&
            (road->phase != ROAD_CONTROL_PHASE_STOPPED)) ||
           (mode == APP_MODE_COMPETITION_ARMED) ||
           (mode == APP_MODE_COMPETITION_RUNNING);
}

void ResetAfterSourceChange(void)
{
    g_snapshot = {};
    g_snapshot.source = g_source;
    LF_ReloadConfig();
    (void) RoadEventController_SetMode(ROAD_CONTROL_DETECT_ONLY);
    RoadEventController_ClearEvent();
}

void FillAdcSnapshot(uint32_t now)
{
    const AppGrayscaleData *data = App_GrayscaleGetData();
    if (data == 0) {
        g_snapshot.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return;
    }
    g_snapshot.valid = data->valid && data->processed_valid;
    g_snapshot.fresh = g_snapshot.valid &&
        ((now - data->last_update_ms) <= kAdcMaximumAgeMs);
    g_snapshot.calibrated = App_GrayscaleCalibrationIsCommissioned();
    g_snapshot.line_detected = data->line_detected;
    g_snapshot.position_valid = data->position_valid;
    g_snapshot.road_capable = true;
    g_snapshot.raw_offset = data->line_position;
    g_snapshot.line_position = data->line_position;
    g_snapshot.line_strength = data->line_strength;
    g_snapshot.position_confidence = data->position_confidence;
    g_snapshot.selected_mask = data->selected_mask;
    g_snapshot.channel_anomaly_mask = data->channel_anomaly_mask;
    g_snapshot.weak_tracking_frames = data->weak_tracking_frames;
    g_snapshot.invalid_frames = data->invalid_frames;
    g_snapshot.position_source = data->position_source;
    g_snapshot.track_state = data->track_state;
    g_snapshot.road_type = data->road_type;
    g_snapshot.road_phase = data->road_phase;
    g_snapshot.road_observed_paths = data->road_observed_paths;
    g_snapshot.road_event_sequence = data->road_event_sequence;
    g_snapshot.road_event_type = data->road_event_type;
    g_snapshot.road_event_paths = data->road_event_paths;
    g_snapshot.road_event_confidence = data->road_event_confidence;
    g_snapshot.sequence = data->sequence;
    g_snapshot.last_update_ms = data->last_update_ms;
    g_snapshot.age_ms = (data->sequence != 0U)
        ? static_cast<uint32_t>(now - data->last_update_ms) : 0U;
    g_snapshot.last_status = data->processing_status;
}

void FillInfraredSnapshot(uint32_t now)
{
    const AppInfraredSensorData *data = App_InfraredSensorGetData();
    if (data == 0) {
        g_snapshot.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return;
    }
    g_snapshot.valid = data->valid;
    g_snapshot.fresh = App_InfraredSensorIsFresh(now);
    g_snapshot.calibrated = data->calibrated;
    g_snapshot.line_detected = data->line_detected;
    g_snapshot.all_black = data->all_black;
    g_snapshot.position_valid = data->position_valid;
    g_snapshot.road_capable = false;
    g_snapshot.raw_offset = data->frame.offset;
    g_snapshot.line_position = data->position_mpos;
    const uint32_t strength = static_cast<uint32_t>(data->frame.adc[0]) +
        data->frame.adc[1] + data->frame.adc[2];
    g_snapshot.line_strength = static_cast<uint16_t>(
        (strength > UINT16_MAX) ? UINT16_MAX : strength);
    g_snapshot.position_confidence = data->confidence;
    g_snapshot.selected_mask = data->line_detected ? 0x07U : 0U;
    g_snapshot.channel_anomaly_mask = 0U;
    g_snapshot.weak_tracking_frames = 0U;
    g_snapshot.invalid_frames = data->position_valid ? 0U : 1U;
    g_snapshot.position_source = data->position_valid
        ? drivers::GRAYSCALE_POSITION_CORE
        : drivers::GRAYSCALE_POSITION_NONE;
    g_snapshot.track_state = data->position_valid
        ? drivers::GRAYSCALE_TRACK_VALID
        : drivers::GRAYSCALE_TRACK_LOST;
    g_snapshot.road_type = GRAYSCALE_ROAD_UNKNOWN;
    g_snapshot.road_phase = GRAYSCALE_ROAD_PHASE_NORMAL;
    g_snapshot.road_event_type = GRAYSCALE_ROAD_UNKNOWN;
    g_snapshot.sequence = data->frame.sequence;
    g_snapshot.last_update_ms = data->frame.received_ms;
    g_snapshot.age_ms = (data->frame.sequence != 0U)
        ? static_cast<uint32_t>(now - data->frame.received_ms) : 0U;
    g_snapshot.last_status = data->last_status;
}

} /* namespace */

void LineSensor_Init(void)
{
    const ConfigStoreParams *params = ConfigStore_Get();
    g_source = ((params != 0) &&
                (params->line_sensor_source == LINE_SENSOR_SOURCE_ADC8))
        ? LINE_SENSOR_ADC8 : LINE_SENSOR_IR3;
    g_snapshot = {};
    g_snapshot.source = g_source;
}

drivers::DriverStatus LineSensor_SetSource(LineSensorSource source)
{
    if ((source != LINE_SENSOR_ADC8) && (source != LINE_SENSOR_IR3)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (MotionControllerBusy()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    const drivers::DriverStatus status = ConfigStore_Set(
        "line_sensor_source",
        static_cast<int32_t>(source));
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    g_source = source;
    ResetAfterSourceChange();
    return drivers::DRIVER_OK;
}

drivers::DriverStatus LineSensor_ApplyConfiguredSource(void)
{
    if (MotionControllerBusy()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    const ConfigStoreParams *params = ConfigStore_Get();
    if (params == 0) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    g_source = (params->line_sensor_source == LINE_SENSOR_SOURCE_ADC8)
        ? LINE_SENSOR_ADC8 : LINE_SENSOR_IR3;
    ResetAfterSourceChange();
    return drivers::DRIVER_OK;
}

LineSensorSource LineSensor_GetSource(void)
{
    return g_source;
}

const LineSensorSnapshot *LineSensor_GetSnapshot(void)
{
    g_snapshot = {};
    g_snapshot.source = g_source;
    g_snapshot.road_type = GRAYSCALE_ROAD_UNKNOWN;
    g_snapshot.road_event_type = GRAYSCALE_ROAD_UNKNOWN;
    g_snapshot.track_state = drivers::GRAYSCALE_TRACK_UNKNOWN;
    g_snapshot.position_source = drivers::GRAYSCALE_POSITION_NONE;
    const uint32_t now = services::Time_Millis();
    if (g_source == LINE_SENSOR_ADC8) {
        FillAdcSnapshot(now);
    } else {
        FillInfraredSnapshot(now);
    }
    return &g_snapshot;
}

bool LineSensor_IsRoadCapable(void)
{
    return g_source == LINE_SENSOR_ADC8;
}

bool LineSensor_IsReadyForMotion(void)
{
    const LineSensorSnapshot *snapshot = LineSensor_GetSnapshot();
    return snapshot->valid && snapshot->fresh && snapshot->calibrated &&
           snapshot->line_detected && snapshot->position_valid &&
           (snapshot->last_status == drivers::DRIVER_OK);
}

const char *LineSensor_SourceText(LineSensorSource source)
{
    return (source == LINE_SENSOR_ADC8) ? "adc8" : "ir3";
}

} /* namespace app */
