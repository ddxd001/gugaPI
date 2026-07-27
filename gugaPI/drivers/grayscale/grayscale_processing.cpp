#include "drivers/grayscale/grayscale_processing.h"

namespace drivers {
namespace {

/* Preserve the existing coordinate convention: positive is left. Position
 * control intentionally uses only the configured tracking core (normally
 * channels 1..6). The outermost channels remain available in active_mask for
 * road/crossing classification, but can no longer pull the steering centroid
 * away from the physical line. */
static const int16_t kChannelPosition[GRAYSCALE_CHANNEL_COUNT] = {
    3000, 2000, 1500, 1000, -1000, -1500, -2000, -3000
};

static const uint8_t kWideCoreActiveChannels = 4U;
static const uint16_t kValidConfidenceFloor = 300U;
static const uint16_t kWeakStrengthDivisor = 4U;

void ClearProcessed(GrayscaleProcessedData *result)
{
    *result = {};
    result->position_source = GRAYSCALE_POSITION_NONE;
    result->track_state = GRAYSCALE_TRACK_UNKNOWN;
}

uint8_t CountBits(uint8_t value)
{
    uint8_t count = 0U;
    while (value != 0U) {
        count = static_cast<uint8_t>(count + (value & 1U));
        value = static_cast<uint8_t>(value >> 1U);
    }
    return count;
}

uint8_t CountRuns(uint8_t mask)
{
    uint8_t runs = 0U;
    bool inside_run = false;
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        const bool set = (mask & static_cast<uint8_t>(1U << i)) != 0U;
        if (set && !inside_run) {
            runs++;
        }
        inside_run = set;
    }
    return runs;
}

bool HasAdjacentBits(uint8_t mask)
{
    return (mask & static_cast<uint8_t>(mask >> 1U)) != 0U;
}

bool TouchesPreviousSegment(uint8_t current_mask, uint8_t previous_mask)
{
    const uint8_t expanded_previous = static_cast<uint8_t>(
        previous_mask |
        static_cast<uint8_t>(previous_mask << 1U) |
        static_cast<uint8_t>(previous_mask >> 1U));
    return (current_mask & expanded_previous) != 0U;
}

bool CanContinueWeakTracking(uint8_t selected_mask,
                             uint32_t strength,
                             const GrayscaleCalibration *calibration,
                             const GrayscaleProcessingState *state)
{
    uint16_t weak_minimum = static_cast<uint16_t>(
        calibration->min_line_strength / kWeakStrengthDivisor);
    if (weak_minimum == 0U) {
        weak_minimum = 1U;
    }

    return state->weak_recovery_eligible &&
           (state->weak_recovery_frames <
            GRAYSCALE_MAX_WEAK_TRACKING_FRAMES) &&
           (selected_mask != 0U) &&
           (CountRuns(selected_mask) == 1U) &&
           (strength >= weak_minimum) &&
           TouchesPreviousSegment(selected_mask,
                                  state->last_selected_mask);
}

uint16_t ClampStrength(uint32_t strength)
{
    return (strength > 0xFFFFU)
        ? 0xFFFFU
        : static_cast<uint16_t>(strength);
}

uint16_t CalculateConfidence(uint32_t strength,
                             uint16_t minimum_strength,
                             GrayscaleTrackState track_state)
{
    if ((track_state == GRAYSCALE_TRACK_LOST) ||
        (track_state == GRAYSCALE_TRACK_SENSOR_FAULT) ||
        (minimum_strength == 0U)) {
        return 0U;
    }

    const uint32_t full_strength =
        static_cast<uint32_t>(minimum_strength) * 2U;
    uint32_t confidence = (strength >= full_strength)
        ? GRAYSCALE_NORMALIZED_MAX
        : ((strength * GRAYSCALE_NORMALIZED_MAX) / full_strength);

    if (track_state == GRAYSCALE_TRACK_VALID) {
        if (confidence < kValidConfidenceFloor) {
            confidence = kValidConfidenceFloor;
        }
    } else if (confidence > 250U) {
        confidence = 250U;
    }
    return static_cast<uint16_t>(confidence);
}

void RecordInvalidFrame(GrayscaleProcessingState *state,
                        GrayscaleProcessedData *result,
                        GrayscaleTrackState track_state)
{
    if (state->invalid_frames != 0xFFU) {
        state->invalid_frames++;
    }
    state->last_track_state = track_state;
    state->position_valid = false;
    state->weak_tracking_frames = 0U;

    /* A completely white frame can occur while a narrow line crosses the
     * physical gap between two sensors. Keep the last spatial segment as a
     * recovery anchor, but charge every lost frame against the same bounded
     * budget as accepted weak frames. Ambiguous geometry and hardware faults
     * cancel recovery immediately. */
    if ((track_state == GRAYSCALE_TRACK_LOST) &&
        state->weak_recovery_eligible &&
        (state->weak_recovery_frames <
         GRAYSCALE_MAX_WEAK_TRACKING_FRAMES)) {
        state->weak_recovery_frames++;
        if (state->weak_recovery_frames >=
            GRAYSCALE_MAX_WEAK_TRACKING_FRAMES) {
            state->weak_recovery_eligible = false;
        }
    } else if (track_state != GRAYSCALE_TRACK_LOST) {
        state->weak_recovery_frames = 0U;
        state->weak_recovery_eligible = false;
    }

    result->track_state = track_state;
    result->weak_tracking_frames = 0U;
    result->invalid_frames = state->invalid_frames;
    result->position_valid = false;
    if (state->last_selected_mask != 0U) {
        result->line_position = state->last_position;
        result->position_source = GRAYSCALE_POSITION_HELD;
    }
}

DriverStatus CalculateCorePosition(
    const uint16_t normalized[GRAYSCALE_CHANNEL_COUNT],
    const GrayscaleCalibration *calibration,
    GrayscaleProcessingState *state,
    GrayscaleProcessedData *result)
{
    const uint16_t threshold_on = Grayscale_GetThresholdOn(calibration);
    const uint16_t threshold_off = Grayscale_GetThresholdOff(calibration);
    uint8_t active_mask = 0U;
    uint8_t selected_mask = 0U;
    uint8_t evidence_mask = 0U;
    uint8_t moderate_mask = 0U;
    uint32_t strength = 0U;
    int32_t weighted_sum = 0;

    result->usable_mask = GRAYSCALE_ALL_CHANNEL_MASK;
    result->track_mask = calibration->track_mask;

    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        const uint8_t bit = static_cast<uint8_t>(1U << i);
        const bool was_active = (state->active_mask & bit) != 0U;
        const bool active = was_active
            ? (normalized[i] >= threshold_off)
            : (normalized[i] >= threshold_on);
        if (active) {
            active_mask = static_cast<uint8_t>(active_mask | bit);
        }

        if ((calibration->track_mask & bit) == 0U) {
            continue;
        }
        if (active) {
            evidence_mask = static_cast<uint8_t>(evidence_mask | bit);
        }
        if (normalized[i] >= threshold_off) {
            moderate_mask = static_cast<uint8_t>(moderate_mask | bit);
        }
        if (normalized[i] <= calibration->position_floor) {
            continue;
        }

        const uint16_t signal = static_cast<uint16_t>(
            normalized[i] - calibration->position_floor);
        selected_mask = static_cast<uint8_t>(selected_mask | bit);
        strength += signal;
        weighted_sum += static_cast<int32_t>(kChannelPosition[i]) *
                        static_cast<int32_t>(signal);
    }

    result->active_mask = active_mask;
    result->selected_mask = selected_mask;
    result->line_strength = ClampStrength(strength);

    GrayscaleTrackState track_state = GRAYSCALE_TRACK_VALID;
    if ((evidence_mask == 0U) &&
        (!HasAdjacentBits(moderate_mask) ||
         (strength < calibration->min_line_strength))) {
        track_state = GRAYSCALE_TRACK_LOST;
    } else if (CountRuns(evidence_mask) > 1U) {
        track_state = GRAYSCALE_TRACK_MULTIPLE;
    } else if (CountBits(static_cast<uint8_t>(
                   active_mask & calibration->track_mask)) >=
               kWideCoreActiveChannels) {
        track_state = GRAYSCALE_TRACK_WIDE;
    }

    /* The reference implementation continuously interpolates analogue core
     * values and effectively retains tracking through the physical gap
     * between sensors. Keep that useful behavior through a short intervening
     * blank interval, but only while the total recovery window is bounded and
     * the weak segment is spatially connected to the last strong line. A cold
     * weak signal cannot acquire a line. */
    const bool weak_tracking =
        (track_state == GRAYSCALE_TRACK_LOST) &&
        CanContinueWeakTracking(selected_mask,
                                strength,
                                calibration,
                                state);
    if (weak_tracking) {
        track_state = GRAYSCALE_TRACK_VALID;
    }

    result->line_detected = (track_state != GRAYSCALE_TRACK_LOST);
    result->position_confidence = CalculateConfidence(
        strength, calibration->min_line_strength, track_state);
    if (weak_tracking && (result->position_confidence > 250U)) {
        result->position_confidence = 250U;
    }

    if (track_state != GRAYSCALE_TRACK_VALID) {
        RecordInvalidFrame(state, result, track_state);
        return DRIVER_OK;
    }
    if ((strength == 0U) || (selected_mask == 0U)) {
        RecordInvalidFrame(state, result, GRAYSCALE_TRACK_LOST);
        result->line_detected = false;
        result->position_confidence = 0U;
        return DRIVER_OK;
    }

    result->line_position = static_cast<int16_t>(
        weighted_sum / static_cast<int32_t>(strength));
    result->position_valid = true;
    result->position_source = GRAYSCALE_POSITION_CORE;
    result->track_state = GRAYSCALE_TRACK_VALID;
    result->weak_tracking_frames = weak_tracking
        ? static_cast<uint8_t>(state->weak_recovery_frames + 1U)
        : 0U;
    result->invalid_frames = 0U;

    state->last_position = result->line_position;
    state->last_selected_mask = selected_mask;
    state->weak_tracking_frames = result->weak_tracking_frames;
    state->weak_recovery_frames = result->weak_tracking_frames;
    state->weak_recovery_eligible = true;
    state->position_valid = true;
    state->last_track_state = GRAYSCALE_TRACK_VALID;
    state->invalid_frames = 0U;
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
    return (faults == 0U) ? calibration_status : DRIVER_ERROR_INVALID_ARG;
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
    uint32_t strength = 0U;
    int32_t weighted_sum = 0;
    for (uint8_t i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++) {
        if (normalized[i] > GRAYSCALE_NORMALIZED_MAX) {
            return DRIVER_ERROR_INVALID_ARG;
        }
        result->normalized[i] = normalized[i];
        if (normalized[i] >= threshold) {
            const uint8_t bit = static_cast<uint8_t>(1U << i);
            result->active_mask = static_cast<uint8_t>(
                result->active_mask | bit);
            strength += normalized[i];
            weighted_sum += static_cast<int32_t>(kChannelPosition[i]) *
                            static_cast<int32_t>(normalized[i]);
        }
    }

    if (strength == 0U) {
        result->track_state = GRAYSCALE_TRACK_LOST;
        result->invalid_frames = 1U;
        return DRIVER_OK;
    }

    result->line_detected = true;
    result->position_valid = true;
    result->selected_mask = result->active_mask;
    result->line_strength = ClampStrength(strength);
    result->line_position = static_cast<int16_t>(
        weighted_sum / static_cast<int32_t>(strength));
    result->position_confidence = GRAYSCALE_NORMALIZED_MAX;
    result->position_source = GRAYSCALE_POSITION_CORE;
    result->track_state = GRAYSCALE_TRACK_VALID;
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

    GrayscaleCalibration compatible = *calibration;
    if (compatible.track_mask == 0U) {
        compatible.hysteresis = 0U;
        compatible.position_floor = 0U;
        compatible.min_line_strength = 1U;
        compatible.track_mask = GRAYSCALE_ALL_CHANNEL_MASK;
    }
    GrayscaleProcessingState state = {};
    return Grayscale_ProcessWithState(raw, &compatible, &state, result);
}

DriverStatus Grayscale_ProcessWithState(
    const uint16_t raw[GRAYSCALE_CHANNEL_COUNT],
    const GrayscaleCalibration *calibration,
    GrayscaleProcessingState *state,
    GrayscaleProcessedData *result)
{
    if ((raw == 0) || (calibration == 0) ||
        (state == 0) || (result == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ClearProcessed(result);
    uint16_t normalized[GRAYSCALE_CHANNEL_COUNT] = {};
    uint8_t calibration_faults = 0U;
    uint8_t saturation = 0U;
    const DriverStatus status = Grayscale_Normalize(raw,
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
    result->track_mask = calibration->track_mask;
    if (status != DRIVER_OK) {
        state->active_mask = 0U;
        RecordInvalidFrame(state, result, GRAYSCALE_TRACK_SENSOR_FAULT);
        return status;
    }

    const DriverStatus position_status = CalculateCorePosition(normalized,
                                                                calibration,
                                                                state,
                                                                result);
    if (position_status == DRIVER_OK) {
        state->active_mask = result->active_mask;
    }
    return position_status;
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
