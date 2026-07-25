#include "drivers/grayscale/grayscale_processing.h"

namespace drivers {
namespace {

static const int16_t kChannelPosition[GRAYSCALE_CHANNEL_COUNT] = {
    3000, 2000, 1500, 1000, -1000, -1500, -2000, -3000
};

static const uint16_t kMinimumPositionConfidence = 300U;
static const uint8_t kMaximumUnambiguousWidth = 4U;
static const uint16_t kMinimumSegmentContrast = 50U;

struct LineSegment {
    uint8_t mask;
    uint8_t count;
    uint32_t strength;
    int32_t weighted_sum;
    int16_t position;
};

void ClearProcessed(GrayscaleProcessedData *result)
{
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        result->normalized[i] = 0U;
    }
    result->active_mask = 0U;
    result->usable_mask = 0U;
    result->track_mask = 0U;
    result->selected_mask = 0U;
    result->calibration_fault_mask = 0U;
    result->saturation_mask = 0U;
    result->line_detected = false;
    result->position_valid = false;
    result->line_position = 0;
    result->line_strength = 0U;
    result->position_confidence = 0U;
    result->position_source = GRAYSCALE_POSITION_NONE;
}

uint8_t GetLeftExtensionMask(uint8_t track_mask)
{
    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        const uint8_t bit = static_cast<uint8_t>(1U << i);
        if ((track_mask & bit) != 0U) {
            break;
        }
        mask = static_cast<uint8_t>(mask | bit);
    }
    return mask;
}

uint8_t GetRightExtensionMask(uint8_t track_mask)
{
    uint8_t mask = 0U;
    for (int8_t i = static_cast<int8_t>(GRAYSCALE_CHANNEL_COUNT - 1U);
         i >= 0;
         i--) {
        const uint8_t bit = static_cast<uint8_t>(1U << i);
        if ((track_mask & bit) != 0U) {
            break;
        }
        mask = static_cast<uint8_t>(mask | bit);
    }
    return mask;
}

GrayscalePositionSource GetPositionSource(uint8_t selected_mask,
                                          uint8_t track_mask)
{
    const bool includes_left =
        (selected_mask & GetLeftExtensionMask(track_mask)) != 0U;
    const bool includes_right =
        (selected_mask & GetRightExtensionMask(track_mask)) != 0U;
    if (includes_left && includes_right) {
        return GRAYSCALE_POSITION_HELD;
    }
    if (includes_left) {
        return GRAYSCALE_POSITION_LEFT_EDGE;
    }
    if (includes_right) {
        return GRAYSCALE_POSITION_RIGHT_EDGE;
    }
    return GRAYSCALE_POSITION_CORE;
}

uint16_t CalculateConfidence(const LineSegment &segment,
                             uint8_t segment_count,
                             const GrayscaleCalibration *calibration,
                             const GrayscaleProcessingState *state)
{
    const uint32_t full_strength =
        static_cast<uint32_t>(calibration->min_line_strength) * 2U;
    uint32_t confidence = (segment.strength >= full_strength)
        ? GRAYSCALE_NORMALIZED_MAX
        : ((segment.strength * GRAYSCALE_NORMALIZED_MAX) / full_strength);

    if (segment.count > 3U) {
        const uint32_t width_penalty =
            static_cast<uint32_t>(segment.count - 3U) * 120U;
        confidence = (confidence > width_penalty)
            ? (confidence - width_penalty)
            : 0U;
    }
    if (segment_count > 1U) {
        const uint32_t split_penalty =
            static_cast<uint32_t>(segment_count - 1U) * 180U;
        confidence = (confidence > split_penalty)
            ? (confidence - split_penalty)
            : 0U;
    }
    if ((state != 0) && state->position_valid) {
        int32_t jump = static_cast<int32_t>(segment.position) -
                       static_cast<int32_t>(state->last_position);
        if (jump < 0) {
            jump = -jump;
        }
        if (jump > 800) {
            uint32_t jump_penalty = static_cast<uint32_t>(jump - 800) / 5U;
            if (jump_penalty > 300U) {
                jump_penalty = 300U;
            }
            confidence = (confidence > jump_penalty)
                ? (confidence - jump_penalty)
                : 0U;
        }
    }
    return static_cast<uint16_t>(confidence);
}

DriverStatus CalculateStatefulPosition(
    const uint16_t normalized[GRAYSCALE_CHANNEL_COUNT],
    const GrayscaleCalibration *calibration,
    GrayscaleProcessingState *state,
    GrayscaleProcessedData *result)
{
    const uint16_t threshold_on = Grayscale_GetThresholdOn(calibration);
    const uint16_t threshold_off = Grayscale_GetThresholdOff(calibration);
    uint8_t active_mask = 0U;
    uint16_t effective[GRAYSCALE_CHANNEL_COUNT] = {};
    LineSegment segments[GRAYSCALE_CHANNEL_COUNT] = {};
    uint8_t segment_count = 0U;
    LineSegment current = {};
    uint16_t frame_minimum = GRAYSCALE_NORMALIZED_MAX;

    result->usable_mask = GRAYSCALE_ALL_CHANNEL_MASK;
    result->track_mask = calibration->track_mask;
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        const uint8_t bit = static_cast<uint8_t>(1U << i);
        const bool was_active = (state->active_mask & bit) != 0U;
        const bool is_active = was_active
            ? (normalized[i] >= threshold_off)
            : (normalized[i] >= threshold_on);
        if (is_active) {
            active_mask = static_cast<uint8_t>(active_mask | bit);
        }
        if (normalized[i] < frame_minimum) {
            frame_minimum = normalized[i];
        }
    }

    uint16_t segment_floor = calibration->position_floor;
    if (frame_minimum < threshold_on) {
        const uint16_t adaptive_floor = static_cast<uint16_t>(
            frame_minimum + kMinimumSegmentContrast);
        if (adaptive_floor > segment_floor) {
            segment_floor = adaptive_floor;
        }
    }

    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        if (normalized[i] > segment_floor) {
            effective[i] = static_cast<uint16_t>(
                normalized[i] - calibration->position_floor);
        }
    }

    result->active_mask = active_mask;

    for (uint8_t i = 0U; i <= GRAYSCALE_CHANNEL_COUNT; i++) {
        const bool contributes = (i < GRAYSCALE_CHANNEL_COUNT) &&
                                 (effective[i] > 0U);
        if (contributes) {
            const uint8_t bit = static_cast<uint8_t>(1U << i);
            current.mask = static_cast<uint8_t>(current.mask | bit);
            current.count++;
            current.strength += effective[i];
            current.weighted_sum +=
                static_cast<int32_t>(kChannelPosition[i]) *
                static_cast<int32_t>(effective[i]);
            continue;
        }
        if (current.count == 0U) {
            continue;
        }
        current.position = static_cast<int16_t>(
            current.weighted_sum / static_cast<int32_t>(current.strength));
        if (current.strength >= calibration->min_line_strength) {
            segments[segment_count] = current;
            segment_count++;
        }
        current = LineSegment{};
    }

    if (segment_count == 0U) {
        return DRIVER_OK;
    }

    uint8_t selected_index = 0U;
    if (state->position_valid) {
        int32_t closest_distance = 0x7FFFFFFF;
        for (uint8_t i = 0U; i < segment_count; i++) {
            int32_t distance = static_cast<int32_t>(segments[i].position) -
                               static_cast<int32_t>(state->last_position);
            if (distance < 0) {
                distance = -distance;
            }
            if (distance < closest_distance) {
                closest_distance = distance;
                selected_index = i;
            }
        }
    } else {
        for (uint8_t i = 1U; i < segment_count; i++) {
            if (segments[i].strength > segments[selected_index].strength) {
                selected_index = i;
            }
        }
    }

    const LineSegment &selected = segments[selected_index];
    result->line_detected = true;
    result->selected_mask = selected.mask;
    result->line_strength = static_cast<uint16_t>(selected.strength);
    result->position_source = GetPositionSource(selected.mask,
                                                calibration->track_mask);
    result->position_confidence = CalculateConfidence(selected,
                                                      segment_count,
                                                      calibration,
                                                      state);

    const bool ambiguous_width = selected.count > kMaximumUnambiguousWidth;
    if (ambiguous_width) {
        result->position_source = GRAYSCALE_POSITION_HELD;
        if (result->position_confidence > 250U) {
            result->position_confidence = 250U;
        }
        result->line_position = state->position_valid
            ? state->last_position
            : selected.position;
        return DRIVER_OK;
    }

    result->line_position = selected.position;
    result->position_valid =
        result->position_confidence >= kMinimumPositionConfidence;
    if (result->position_valid) {
        state->last_position = result->line_position;
        state->last_selected_mask = result->selected_mask;
        state->position_valid = true;
    }
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
    result->position_valid = true;
    result->selected_mask = result->active_mask;
    result->line_strength = static_cast<uint16_t>(strength);
    result->line_position = static_cast<int16_t>(
        weighted_sum / static_cast<int32_t>(strength));
    result->position_confidence = GRAYSCALE_NORMALIZED_MAX;
    result->position_source = GRAYSCALE_POSITION_CORE;
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
        result->selected_mask = 0U;
        result->calibration_fault_mask = calibration_faults;
        result->saturation_mask = saturation;
        result->line_detected = false;
        result->position_valid = false;
        result->line_position = 0;
        result->line_strength = 0U;
        result->position_confidence = 0U;
        result->position_source = GRAYSCALE_POSITION_NONE;
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
        state->last_position = 0;
        state->last_selected_mask = 0U;
        state->position_valid = false;
        return status;
    }

    status = CalculateStatefulPosition(normalized,
                                       calibration,
                                       state,
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
