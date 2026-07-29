#ifndef APP_LINE_SENSOR_H_
#define APP_LINE_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "app/grayscale_road.h"
#include "drivers/common/driver_status.h"
#include "drivers/grayscale/grayscale_processing.h"

namespace app {

enum LineSensorSource : uint8_t {
    LINE_SENSOR_ADC8 = 0U,
    LINE_SENSOR_IR3 = 1U
};

struct LineSensorSnapshot {
    LineSensorSource source;
    bool valid;
    bool fresh;
    bool calibrated;
    bool line_detected;
    bool all_black;
    bool position_valid;
    bool road_capable;
    int16_t raw_offset;
    int16_t line_position;
    uint16_t line_strength;
    uint16_t position_confidence;
    uint8_t selected_mask;
    uint8_t channel_anomaly_mask;
    uint8_t weak_tracking_frames;
    uint8_t invalid_frames;
    drivers::GrayscalePositionSource position_source;
    drivers::GrayscaleTrackState track_state;
    GrayscaleRoadType road_type;
    GrayscaleRoadPhase road_phase;
    uint8_t road_observed_paths;
    uint32_t road_event_sequence;
    GrayscaleRoadType road_event_type;
    uint8_t road_event_paths;
    uint16_t road_event_confidence;
    uint32_t sequence;
    uint32_t last_update_ms;
    uint32_t age_ms;
    drivers::DriverStatus last_status;
};

void LineSensor_Init(void);
drivers::DriverStatus LineSensor_SetSource(LineSensorSource source);
drivers::DriverStatus LineSensor_ApplyConfiguredSource(void);
LineSensorSource LineSensor_GetSource(void);
const LineSensorSnapshot *LineSensor_GetSnapshot(void);
bool LineSensor_IsRoadCapable(void);
bool LineSensor_IsReadyForMotion(void);
const char *LineSensor_SourceText(LineSensorSource source);

} /* namespace app */

#endif /* APP_LINE_SENSOR_H_ */
