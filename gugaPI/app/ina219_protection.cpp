#include "app/ina219_protection.h"

#include <limits.h>
#include <string.h>

namespace app {
namespace {

void IncrementSaturated(uint8_t *value)
{
    if (*value != UINT8_MAX) {
        (*value)++;
    }
}

void IncrementSaturated(uint32_t *value)
{
    if (*value != UINT32_MAX) {
        (*value)++;
    }
}

uint32_t AbsCurrentUa(int32_t current_ua)
{
    if (current_ua >= 0) {
        return static_cast<uint32_t>(current_ua);
    }
    if (current_ua == INT32_MIN) {
        return static_cast<uint32_t>(INT32_MAX) + 1U;
    }
    return static_cast<uint32_t>(-current_ua);
}

void RecomputeMotionInhibit(Ina219ProtectionState *state)
{
    const uint8_t faults = static_cast<uint8_t>(
        state->active_fault_mask | state->latched_fault_mask);
    state->motion_inhibit_requested =
        state->config.motion_inhibit_enabled && (faults != 0U);
}

void ActivateFault(Ina219ProtectionState *state, uint8_t fault)
{
    state->active_fault_mask = static_cast<uint8_t>(
        state->active_fault_mask | fault);
    if (state->config.latch_faults) {
        state->latched_fault_mask = static_cast<uint8_t>(
            state->latched_fault_mask | fault);
    }
}

void UpdateFault(Ina219ProtectionState *state,
                 uint8_t fault,
                 bool trip_condition,
                 bool release_condition,
                 uint8_t trip_limit,
                 uint8_t *trip_count,
                 uint8_t *release_count)
{
    const bool active = (state->active_fault_mask & fault) != 0U;
    if (!active) {
        *release_count = 0U;
        if (!trip_condition) {
            *trip_count = 0U;
            return;
        }
        IncrementSaturated(trip_count);
        if (*trip_count >= trip_limit) {
            ActivateFault(state, fault);
            *trip_count = 0U;
        }
        return;
    }

    *trip_count = 0U;
    if (!release_condition) {
        *release_count = 0U;
        return;
    }
    IncrementSaturated(release_count);
    if (*release_count >= state->config.release_samples) {
        state->active_fault_mask = static_cast<uint8_t>(
            state->active_fault_mask & static_cast<uint8_t>(~fault));
        *release_count = 0U;
    }
}

void RecordCommunicationSuccess(Ina219ProtectionState *state)
{
    state->communication_fail_count = 0U;
    UpdateFault(state,
                INA219_PROTECTION_FAULT_COMMUNICATION,
                false,
                true,
                state->config.communication_fail_samples,
                &state->communication_fail_count,
                &state->communication_release_count);
}

} /* namespace */

drivers::DriverStatus Ina219Protection_ValidateConfig(
    const Ina219ProtectionConfig *config)
{
    if ((config == 0) || (config->undervoltage_trip_mv == 0U) ||
        (config->undervoltage_release_mv >
         INA219_PROTECTION_MAX_BUS_VOLTAGE_MV) ||
        (config->undervoltage_release_mv <= config->undervoltage_trip_mv) ||
        (config->overcurrent_trip_ma == 0U) ||
        (config->overcurrent_trip_ma > INA219_PROTECTION_MAX_CURRENT_MA) ||
        (config->overcurrent_release_ma >= config->overcurrent_trip_ma) ||
        (config->trip_samples == 0U) ||
        (config->trip_samples > INA219_PROTECTION_MAX_SAMPLE_COUNT) ||
        (config->release_samples == 0U) ||
        (config->release_samples > INA219_PROTECTION_MAX_SAMPLE_COUNT) ||
        (config->communication_fail_samples == 0U) ||
        (config->communication_fail_samples >
         INA219_PROTECTION_MAX_SAMPLE_COUNT)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Ina219Protection_Init(
    Ina219ProtectionState *state,
    const Ina219ProtectionConfig *config)
{
    if (state == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    const drivers::DriverStatus status =
        Ina219Protection_ValidateConfig(config);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    (void) memset(state, 0, sizeof(*state));
    state->config = *config;
    state->last_read_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
    RecomputeMotionInhibit(state);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Ina219Protection_Configure(
    Ina219ProtectionState *state,
    const Ina219ProtectionConfig *config)
{
    if (state == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    const drivers::DriverStatus status =
        Ina219Protection_ValidateConfig(config);
    if (status != drivers::DRIVER_OK) {
        return status;
    }
    state->config = *config;
    RecomputeMotionInhibit(state);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Ina219Protection_RecordSample(
    Ina219ProtectionState *state,
    const Ina219ProtectionSample *sample)
{
    if ((state == 0) || (sample == 0)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    state->last_sample = *sample;
    state->sample_valid = sample->conversion_ready && !sample->math_overflow;
    state->last_read_status = drivers::DRIVER_OK;
    IncrementSaturated(&state->successful_sample_count);
    RecordCommunicationSuccess(state);

    UpdateFault(state, INA219_PROTECTION_FAULT_MATH_OVERFLOW,
                sample->math_overflow, !sample->math_overflow,
                state->config.trip_samples, &state->overflow_trip_count,
                &state->overflow_release_count);
    UpdateFault(state, INA219_PROTECTION_FAULT_NOT_READY,
                !sample->conversion_ready, sample->conversion_ready,
                state->config.trip_samples, &state->not_ready_trip_count,
                &state->not_ready_release_count);

    if (state->sample_valid) {
        const uint32_t absolute_current_ua = AbsCurrentUa(sample->current_ua);
        UpdateFault(state, INA219_PROTECTION_FAULT_UNDERVOLTAGE,
                    sample->bus_voltage_mv <=
                        static_cast<int32_t>(state->config.undervoltage_trip_mv),
                    sample->bus_voltage_mv >= static_cast<int32_t>(
                        state->config.undervoltage_release_mv),
                    state->config.trip_samples,
                    &state->undervoltage_trip_count,
                    &state->undervoltage_release_count);
        UpdateFault(state, INA219_PROTECTION_FAULT_OVERCURRENT,
                    absolute_current_ua >=
                        static_cast<uint32_t>(
                            state->config.overcurrent_trip_ma) * 1000U,
                    absolute_current_ua <=
                        static_cast<uint32_t>(
                            state->config.overcurrent_release_ma) * 1000U,
                    state->config.trip_samples,
                    &state->overcurrent_trip_count,
                    &state->overcurrent_release_count);
    } else {
        /* A quality-invalid reading breaks both voltage/current debounce
         * sequences; only consecutive valid measurements may trip/release. */
        state->undervoltage_trip_count = 0U;
        state->undervoltage_release_count = 0U;
        state->overcurrent_trip_count = 0U;
        state->overcurrent_release_count = 0U;
    }

    RecomputeMotionInhibit(state);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Ina219Protection_RecordReadError(
    Ina219ProtectionState *state,
    drivers::DriverStatus read_status)
{
    if ((state == 0) || (read_status == drivers::DRIVER_OK)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    state->sample_valid = false;
    state->last_read_status = read_status;
    IncrementSaturated(&state->read_error_count);
    state->communication_release_count = 0U;
    UpdateFault(state, INA219_PROTECTION_FAULT_COMMUNICATION, true, false,
                state->config.communication_fail_samples,
                &state->communication_fail_count,
                &state->communication_release_count);
    RecomputeMotionInhibit(state);
    return drivers::DRIVER_OK;
}

void Ina219Protection_ClearLatchedFaults(Ina219ProtectionState *state)
{
    if (state != 0) {
        state->latched_fault_mask = 0U;
        RecomputeMotionInhibit(state);
    }
}

bool Ina219Protection_MotionInhibitRequested(
    const Ina219ProtectionState *state)
{
    return (state != 0) && state->motion_inhibit_requested;
}

} /* namespace app */
