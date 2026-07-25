#include "drivers/grayscale/grayscale_processing.h"

namespace drivers {
namespace {

static const int16_t kChannelPosition[GRAYSCALE_CHANNEL_COUNT] = {
    -3000, -2000, -1500, -1000, 1000, 1500, 2000, 3000
};

void ClearProcessed(GrayscaleProcessedData *result)
{
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        result->normalized[i] = 0U;
    }
    result->active_mask = 0U;
    result->usable_mask = 0U;
    result->track_mask = 0U;
    result->calibration_fault_mask = 0U;
    result->saturation_mask = 0U;
    result->line_detected = false;
    result->line_position = 0;
    result->line_strength = 0U;
}

DriverStatus CalculateStatefulPosition(
    const uint16_t normalized[GRAYSCALE_CHANNEL_COUNT],
    const GrayscaleCalibration *calibration,
    uint8_t previous_active_mask,
    GrayscaleProcessedData *result)
{
    const uint16_t threshold_on = Grayscale_GetThresholdOn(calibration);
    const uint16_t threshold_off = Grayscale_GetThresholdOff(calibration);
    uint8_t active_mask = 0U;
    uint32_t strength = 0U;
    int32_t weighted_sum = 0;

    result->usable_mask = GRAYSCALE_ALL_CHANNEL_MASK;
    result->track_mask = calibration->track_mask;
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        const uint8_t bit = static_cast<uint8_t>(1U << i);
        const bool was_active = (previous_active_mask & bit) != 0U;
        const bool is_active = was_active
            ? (normalized[i] >= threshold_off)
            : (normalized[i] >= threshold_on);
        if (is_active) {
            active_mask = static_cast<uint8_t>(active_mask | bit);
        }

        if (((calibration->track_mask & bit) != 0U) &&
            (normalized[i] > calibration->position_floor)) {
            const uint16_t effective = static_cast<uint16_t>(
                normalized[i] - calibration->position_floor);
            strength += effective;
            weighted_sum += static_cast<int32_t>(kChannelPosition[i]) *
                            static_cast<int32_t>(effective);
        }
    }

    result->active_mask = active_mask;
    if (strength > 0xFFFFU) {
        return DRIVER_ERROR;
    }
    result->line_strength = static_cast<uint16_t>(strength);
    if (strength < calibration->min_line_strength) {
        return DRIVER_OK;
    }

    result->line_detected = true;
    result->line_position = static_cast<int16_t>(
        weighted_sum / static_cast<int32_t>(strength));
    return DRIVER_OK;
}

} /* namespace */

DriverStatus Grayscale_ValidateCalibration(
    const GrayscaleCalibration *calibration,
    uint8_t *fault_mask)
{
    if (calibration == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    uint8_t faults = 0U;
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        if ((calibration->white[i] > GRAYSCALE_ADC_MAX) ||
            (calibration->black[i] > GRAYSCALE_ADC_MAX) ||
            (calibration->white[i] == calibration->black[i])) {
            faults = static_cast<uint8_t>(faults | (1U << i));
        }
    }
    if ((calibration->threshold == 0U) ||
        (calibration->threshold >= GRAYSCALE_NORMALIZED_MAX) ||
        (calibration->hysteresis > GRAYSCALE_NORMALIZED_MAX) ||
        (calibration->position_floor >= GRAYSCALE_NORMALIZED_MAX) ||
        (calibration->min_line_strength == 0U) ||
        (calibration->min_line_strength >
         (GRAYSCALE_NORMALIZED_MAX * GRAYSCALE_CHANNEL_COUNT)) ||
        (calibration->track_mask == 0U) ||
        (Grayscale_GetThresholdOff(calibration) == 0U) ||
        (Grayscale_GetThresholdOn(calibration) >=
         GRAYSCALE_NORMALIZED_MAX)) {
        faults = 0xFFU;
    }

    if (fault_mask != 0) {
        *fault_mask = faults;
    }
    return (faults == 0U) ? DRIVER_OK : DRIVER_ERROR_INVALID_ARG;
}

DriverStatus Grayscale_Normalize(
    const uint16_t raw[GRAYSCALE_CHANNEL_COUNT],
    const GrayscaleCalibration *calibration,
    uint16_t normalized[GRAYSCALE_CHANNEL_COUNT],
    uint8_t *calibration_fault_mask,
    uint8_t *saturation_mask)
{
    if ((raw == 0) || (calibration == 0) || (normalized == 0) ||
        (calibration_fault_mask == 0) || (saturation_mask == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    uint8_t faults = 0U;
    const DriverStatus calibration_status =
        Grayscale_ValidateCalibration(calibration, &faults);
    uint8_t saturation = 0U;

    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        const uint8_t bit = static_cast<uint8_t>(1U << i);
        normalized[i] = 0U;

        if ((raw[i] == 0U) || (raw[i] >= GRAYSCALE_ADC_MAX)) {
            saturation = static_cast<uint8_t>(saturation | bit);
        }
        if (raw[i] > GRAYSCALE_ADC_MAX) {
            faults = static_cast<uint8_t>(faults | bit);
        }
        if ((faults & bit) != 0U) {
            continue;
        }

        const int32_t numerator =
            (static_cast<int32_t>(raw[i]) -
             static_cast<int32_t>(calibration->white[i])) *
            static_cast<int32_t>(GRAYSCALE_NORMALIZED_MAX);
        const int32_t denominator =
            static_cast<int32_t>(calibration->black[i]) -
            static_cast<int32_t>(calibration->white[i]);
        int32_t value = numerator / denominator;
        if (value < 0) {
            value = 0;
        } else if (value > GRAYSCALE_NORMALIZED_MAX) {
            value = GRAYSCALE_NORMALIZED_MAX;
        }
        normalized[i] = static_cast<uint16_t>(value);
    }

    *calibration_fault_mask = faults;
    *saturation_mask = saturation;
    if (faults != 0U) {
        return DRIVER_ERROR_INVALID_ARG;
    }
    return calibration_status;
}

DriverStatus Grayscale_CalculatePosition(
    const uint16_t normalized[GRAYSCALE_CHANNEL_COUNT],
    uint16_t threshold,
    GrayscaleProcessedData *result)
{
    if ((normalized == 0) || (result == 0) || (threshold == 0U) ||
        (threshold > GRAYSCALE_NORMALIZED_MAX)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ClearProcessed(result);
    result->usable_mask = GRAYSCALE_ALL_CHANNEL_MASK;
    result->track_mask = GRAYSCALE_ALL_CHANNEL_MASK;

    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        result->normalized[i] = 0U;
        if (normalized[i] > GRAYSCALE_NORMALIZED_MAX) {
            return DRIVER_ERROR_INVALID_ARG;
        }
    }

    int32_t weighted_sum = 0;
    uint32_t strength = 0U;
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        result->normalized[i] = normalized[i];
        if (normalized[i] >= threshold) {
            result->active_mask = static_cast<uint8_t>(
                result->active_mask | (1U << i));
            strength += normalized[i];
            weighted_sum += static_cast<int32_t>(kChannelPosition[i]) *
                            static_cast<int32_t>(normalized[i]);
        }
    }

    if (strength == 0U) {
        return DRIVER_OK;
    }

    result->line_detected = true;
    result->line_strength = static_cast<uint16_t>(strength);
    result->line_position = static_cast<int16_t>(
        weighted_sum / static_cast<int32_t>(strength));
    return DRIVER_OK;
}

DriverStatus Grayscale_Process(
    const uint16_t raw[GRAYSCALE_CHANNEL_COUNT],
    const GrayscaleCalibration *calibration,
    GrayscaleProcessedData *result)
{
    if ((calibration == 0) || (result == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    /* Preserve the original stateless API for callers that only populate
     * white/black/threshold. The application uses ProcessWithState below. */
    GrayscaleCalibration compatible = *calibration;
    if (compatible.threshold >= GRAYSCALE_NORMALIZED_MAX) {
        compatible.threshold = GRAYSCALE_NORMALIZED_MAX - 1U;
    }
    if (compatible.track_mask == 0U) {
        compatible.hysteresis = 0U;
        compatible.position_floor = 0U;
        compatible.min_line_strength = 1U;
        compatible.track_mask = GRAYSCALE_ALL_CHANNEL_MASK;
    }

    uint16_t normalized[GRAYSCALE_CHANNEL_COUNT] = {};
    uint8_t calibration_faults = 0U;
    uint8_t saturation = 0U;
    DriverStatus status = Grayscale_Normalize(raw,
                                               &compatible,
                                               normalized,
                                               &calibration_faults,
                                               &saturation);
    if (status != DRIVER_OK) {
        for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
            result->normalized[i] = normalized[i];
        }
        result->active_mask = 0U;
        result->calibration_fault_mask = calibration_faults;
        result->saturation_mask = saturation;
        result->line_detected = false;
        result->line_position = 0;
        result->line_strength = 0U;
        return status;
    }

    status = Grayscale_CalculatePosition(normalized,
                                         compatible.threshold,
                                         result);
    result->calibration_fault_mask = calibration_faults;
    result->saturation_mask = saturation;
    return status;
}

DriverStatus Grayscale_ProcessWithState(
    const uint16_t raw[GRAYSCALE_CHANNEL_COUNT],
    const GrayscaleCalibration *calibration,
    GrayscaleProcessingState *state,
    GrayscaleProcessedData *result)
{
    if ((state == 0) || (result == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ClearProcessed(result);
    uint16_t normalized[GRAYSCALE_CHANNEL_COUNT] = {};
    uint8_t calibration_faults = 0U;
    uint8_t saturation = 0U;
    DriverStatus status = Grayscale_Normalize(raw,
                                               calibration,
                                               normalized,
                                               &calibration_faults,
                                               &saturation);
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        result->normalized[i] = normalized[i];
    }
    result->calibration_fault_mask = calibration_faults;
    result->saturation_mask = saturation;
    result->usable_mask = static_cast<uint8_t>(
        GRAYSCALE_ALL_CHANNEL_MASK &
        static_cast<uint8_t>(~calibration_faults));
    result->track_mask = (calibration != 0) ? calibration->track_mask : 0U;
    if (status != DRIVER_OK) {
        state->active_mask = 0U;
        return status;
    }

    status = CalculateStatefulPosition(normalized,
                                       calibration,
                                       state->active_mask,
                                       result);
    if (status == DRIVER_OK) {
        state->active_mask = result->active_mask;
    }
    return status;
}

uint16_t Grayscale_GetThresholdOn(const GrayscaleCalibration *calibration)
{
    if (calibration == 0) {
        return GRAYSCALE_NORMALIZED_MAX;
    }
    const uint16_t upper_half = static_cast<uint16_t>(
        (calibration->hysteresis + 1U) / 2U);
    const uint32_t value = static_cast<uint32_t>(calibration->threshold) +
                           static_cast<uint32_t>(upper_half);
    return (value > GRAYSCALE_NORMALIZED_MAX)
        ? GRAYSCALE_NORMALIZED_MAX
        : static_cast<uint16_t>(value);
}

uint16_t Grayscale_GetThresholdOff(const GrayscaleCalibration *calibration)
{
    if (calibration == 0) {
        return 0U;
    }
    const uint16_t lower_half = static_cast<uint16_t>(
        calibration->hysteresis / 2U);
    return (calibration->threshold > lower_half)
        ? static_cast<uint16_t>(calibration->threshold - lower_half)
        : 0U;
}

} /* namespace drivers */
