#ifndef APP_APP_GRAYSCALE_H_
#define APP_APP_GRAYSCALE_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "drivers/grayscale/grayscale_processing.h"
#include "app/grayscale_road.h"

namespace app {

struct AppGrayscaleData {
    uint16_t raw[8];   /* 0..4095 per channel, PA15 ADC */
    bool valid;
    uint32_t last_update_ms;
    uint32_t error_count;
    drivers::DriverStatus last_status;
    uint32_t sequence;
    uint16_t normalized[drivers::GRAYSCALE_CHANNEL_COUNT];
    bool processed_valid;
    uint8_t active_mask;
    uint8_t usable_mask;
    uint8_t track_mask;
    uint8_t selected_mask;
    uint8_t calibration_fault_mask;
    uint8_t saturation_mask;
    uint8_t channel_anomaly_mask;
    bool line_detected;
    bool position_valid;
    int16_t line_position;
    uint16_t line_strength;
    uint16_t position_confidence;
    drivers::GrayscalePositionSource position_source;
    drivers::GrayscaleTrackState track_state;
    uint8_t weak_tracking_frames;
    uint8_t invalid_frames;
    GrayscaleRoadType road_type;
    uint8_t road_confirm_count;
    GrayscaleRoadPhase road_phase;
    uint8_t road_observed_paths;
    uint32_t road_event_sequence;
    GrayscaleRoadType road_event_type;
    uint8_t road_event_paths;
    uint16_t road_event_confidence;
    uint8_t road_event_entry_mask;
    uint8_t road_event_peak_mask;
    uint8_t road_event_exit_mask;
    uint16_t threshold_on;
    uint16_t threshold_off;
    uint32_t frame_period_ms;
    uint32_t processing_error_count;
    drivers::DriverStatus processing_status;
};

enum AppGrayscaleCalibrationMode : uint8_t {
    APP_GRAYSCALE_CAL_IDLE = 0U,
    APP_GRAYSCALE_CAL_SWEEP,
    APP_GRAYSCALE_CAL_WHITE,
    APP_GRAYSCALE_CAL_BLACK
};

struct AppGrayscaleCalibrationStatus {
    AppGrayscaleCalibrationMode mode;
    bool running;
    bool white_ready;
    bool black_ready;
    uint16_t sample_count;
    uint16_t target_samples;
    uint8_t fault_mask;
    drivers::DriverStatus last_status;
};

void App_GrayscaleInit(void);
void App_GrayscaleUpdate(void);
const AppGrayscaleData *App_GrayscaleGetData(void);
void App_GrayscaleClearRoadEvent(void);
drivers::DriverStatus App_GrayscaleReloadCalibration(void);
drivers::DriverStatus App_GrayscaleSetCalibration(
    const drivers::GrayscaleCalibration *calibration);
const drivers::GrayscaleCalibration *App_GrayscaleGetCalibration(void);
bool App_GrayscaleCalibrationIsCommissioned(void);
drivers::DriverStatus App_GrayscaleStartSweepCalibration(uint32_t duration_ms);
drivers::DriverStatus App_GrayscaleStartWhiteCalibration(uint16_t frames);
drivers::DriverStatus App_GrayscaleStartBlackCalibration(uint16_t frames);
drivers::DriverStatus App_GrayscaleCommitCalibration(void);
void App_GrayscaleCancelCalibration(void);
const AppGrayscaleCalibrationStatus *App_GrayscaleGetCalibrationStatus(void);

} /* namespace app */

#endif /* APP_APP_GRAYSCALE_H_ */
