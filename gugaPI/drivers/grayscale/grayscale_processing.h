#ifndef DRIVERS_GRAYSCALE_GRAYSCALE_PROCESSING_H_
#define DRIVERS_GRAYSCALE_GRAYSCALE_PROCESSING_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace drivers {

static const uint8_t GRAYSCALE_CHANNEL_COUNT = 8U;
static const uint16_t GRAYSCALE_ADC_MAX = 4095U;
static const uint16_t GRAYSCALE_NORMALIZED_MAX = 1000U;
static const uint8_t GRAYSCALE_ALL_CHANNEL_MASK = 0xFFU;
static const uint8_t GRAYSCALE_DEFAULT_TRACK_MASK = 0x3CU;

enum GrayscalePositionSource : uint8_t {
    GRAYSCALE_POSITION_NONE = 0U,
    GRAYSCALE_POSITION_CORE,
    GRAYSCALE_POSITION_LEFT_EDGE,
    GRAYSCALE_POSITION_RIGHT_EDGE,
    GRAYSCALE_POSITION_HELD
};

struct GrayscaleCalibration {
    uint16_t white[GRAYSCALE_CHANNEL_COUNT];
    uint16_t black[GRAYSCALE_CHANNEL_COUNT];
    uint16_t threshold;       /* hysteresis center, 1..999 */
    uint16_t hysteresis;      /* on/off separation, 0..1000 */
    uint16_t position_floor;  /* normalized values at/below this are noise */
    uint16_t min_line_strength;
    uint8_t track_mask;       /* channels used for continuous line position */
};

struct GrayscaleProcessingState {
    uint8_t active_mask;      /* previous hysteresis state */
    int16_t last_position;
    uint8_t last_selected_mask;
    bool position_valid;
};

struct GrayscaleProcessedData {
    uint16_t normalized[GRAYSCALE_CHANNEL_COUNT]; /* 0=white, 1000=black */
    uint8_t active_mask;       /* normalized >= threshold */
    uint8_t usable_mask;
    uint8_t track_mask;
    uint8_t selected_mask;     /* contiguous line segment used for position */
    uint8_t calibration_fault_mask;
    uint8_t saturation_mask;   /* raw ADC exactly 0 or 4095 */
    bool line_detected;
    bool position_valid;
    int16_t line_position;     /* +3000 left .. -3000 right */
    uint16_t line_strength;    /* sum of active normalized channels */
    uint16_t position_confidence; /* 0..1000 */
    GrayscalePositionSource position_source;
};

DriverStatus Grayscale_ValidateCalibration(
    const GrayscaleCalibration *calibration,
    uint8_t *fault_mask);
DriverStatus Grayscale_Normalize(
    const uint16_t raw[GRAYSCALE_CHANNEL_COUNT],
    const GrayscaleCalibration *calibration,
    uint16_t normalized[GRAYSCALE_CHANNEL_COUNT],
    uint8_t *calibration_fault_mask,
    uint8_t *saturation_mask);
DriverStatus Grayscale_CalculatePosition(
    const uint16_t normalized[GRAYSCALE_CHANNEL_COUNT],
    uint16_t threshold,
    GrayscaleProcessedData *result);
DriverStatus Grayscale_Process(
    const uint16_t raw[GRAYSCALE_CHANNEL_COUNT],
    const GrayscaleCalibration *calibration,
    GrayscaleProcessedData *result);
DriverStatus Grayscale_ProcessWithState(
    const uint16_t raw[GRAYSCALE_CHANNEL_COUNT],
    const GrayscaleCalibration *calibration,
    GrayscaleProcessingState *state,
    GrayscaleProcessedData *result);
uint16_t Grayscale_GetThresholdOn(const GrayscaleCalibration *calibration);
uint16_t Grayscale_GetThresholdOff(const GrayscaleCalibration *calibration);

} /* namespace drivers */

#endif /* DRIVERS_GRAYSCALE_GRAYSCALE_PROCESSING_H_ */
