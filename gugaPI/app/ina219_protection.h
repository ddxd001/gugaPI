#ifndef APP_INA219_PROTECTION_H_
#define APP_INA219_PROTECTION_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace app {

static const uint16_t INA219_PROTECTION_MAX_BUS_VOLTAGE_MV = 26000U;
static const uint16_t INA219_PROTECTION_MAX_CURRENT_MA = 6500U;
static const uint8_t INA219_PROTECTION_MAX_SAMPLE_COUNT = 100U;

enum Ina219ProtectionFault : uint8_t {
    INA219_PROTECTION_FAULT_NONE = 0U,
    INA219_PROTECTION_FAULT_UNDERVOLTAGE = 1U << 0U,
    INA219_PROTECTION_FAULT_OVERCURRENT = 1U << 1U,
    INA219_PROTECTION_FAULT_COMMUNICATION = 1U << 2U,
    INA219_PROTECTION_FAULT_MATH_OVERFLOW = 1U << 3U,
    INA219_PROTECTION_FAULT_NOT_READY = 1U << 4U
};

struct Ina219ProtectionConfig {
    uint16_t undervoltage_trip_mv;
    uint16_t undervoltage_release_mv;
    uint16_t overcurrent_trip_ma;
    uint16_t overcurrent_release_ma;
    uint8_t trip_samples;
    uint8_t release_samples;
    uint8_t communication_fail_samples;
    bool latch_faults;
    bool motion_inhibit_enabled;
};

struct Ina219ProtectionSample {
    int32_t bus_voltage_mv;
    int32_t current_ua;
    bool conversion_ready;
    bool math_overflow;
};

struct Ina219ProtectionState {
    Ina219ProtectionConfig config;
    Ina219ProtectionSample last_sample;
    bool sample_valid;
    drivers::DriverStatus last_read_status;
    uint8_t active_fault_mask;
    uint8_t latched_fault_mask;
    bool motion_inhibit_requested;
    uint8_t undervoltage_trip_count;
    uint8_t undervoltage_release_count;
    uint8_t overcurrent_trip_count;
    uint8_t overcurrent_release_count;
    uint8_t communication_fail_count;
    uint8_t communication_release_count;
    uint8_t overflow_trip_count;
    uint8_t overflow_release_count;
    uint8_t not_ready_trip_count;
    uint8_t not_ready_release_count;
    uint32_t successful_sample_count;
    uint32_t read_error_count;
};

drivers::DriverStatus Ina219Protection_ValidateConfig(
    const Ina219ProtectionConfig *config);
drivers::DriverStatus Ina219Protection_Init(
    Ina219ProtectionState *state,
    const Ina219ProtectionConfig *config);
drivers::DriverStatus Ina219Protection_Configure(
    Ina219ProtectionState *state,
    const Ina219ProtectionConfig *config);
drivers::DriverStatus Ina219Protection_RecordSample(
    Ina219ProtectionState *state,
    const Ina219ProtectionSample *sample);
drivers::DriverStatus Ina219Protection_RecordReadError(
    Ina219ProtectionState *state,
    drivers::DriverStatus read_status);
void Ina219Protection_ClearLatchedFaults(Ina219ProtectionState *state);
bool Ina219Protection_MotionInhibitRequested(
    const Ina219ProtectionState *state);

} /* namespace app */

#endif /* APP_INA219_PROTECTION_H_ */
