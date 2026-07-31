#include "app/line_sensor.h"

#include "app/app_grayscale.h"
#include "services/time.h"

namespace app {
namespace {

static const uint32_t kAdcMaximumAgeMs = 200U;

LineSensorSnapshot g_snapshot = {};

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

} /* namespace */

void LineSensor_Init(void)
{
    g_snapshot = {};
}

const LineSensorSnapshot *LineSensor_GetSnapshot(void)
{
    g_snapshot = {};
    g_snapshot.road_type = GRAYSCALE_ROAD_UNKNOWN;
    g_snapshot.road_event_type = GRAYSCALE_ROAD_UNKNOWN;
    g_snapshot.track_state = drivers::GRAYSCALE_TRACK_UNKNOWN;
    g_snapshot.position_source = drivers::GRAYSCALE_POSITION_NONE;
    FillAdcSnapshot(services::Time_Millis());
    return &g_snapshot;
}

bool LineSensor_IsReadyForMotion(void)
{
    const LineSensorSnapshot *snapshot = LineSensor_GetSnapshot();
    return snapshot->valid && snapshot->fresh && snapshot->calibrated &&
           snapshot->line_detected && snapshot->position_valid &&
           (snapshot->last_status == drivers::DRIVER_OK);
}

} /* namespace app */
