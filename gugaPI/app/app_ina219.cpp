#include "app/app_ina219.h"

#include "app/config_store.h"
#include "board/board_ina219.h"
#include "services/time.h"

namespace app {
namespace {

static const uint32_t kProtectionPollPeriodMs = 100U;

AppIna219Data g_data = {};
bool g_firstRun = true;

Ina219ProtectionConfig ConfigFromStore(void)
{
    const ConfigStoreParams *params = ConfigStore_Get();
    Ina219ProtectionConfig config = {};
    if (params != 0) {
        config.undervoltage_trip_mv = params->ina219_undervoltage_trip_mv;
        config.undervoltage_release_mv = params->ina219_undervoltage_release_mv;
        config.overcurrent_trip_ma = params->ina219_overcurrent_trip_ma;
        config.overcurrent_release_ma = params->ina219_overcurrent_release_ma;
        config.trip_samples = params->ina219_trip_samples;
        config.release_samples = params->ina219_release_samples;
        config.communication_fail_samples = params->ina219_comm_fail_samples;
        config.latch_faults = params->ina219_latch_faults != 0U;
        config.motion_inhibit_enabled =
            params->ina219_motion_inhibit_enable != 0U;
    }
    return config;
}

bool ConfigEqual(const Ina219ProtectionConfig &left,
                 const Ina219ProtectionConfig &right)
{
    return (left.undervoltage_trip_mv == right.undervoltage_trip_mv) &&
           (left.undervoltage_release_mv == right.undervoltage_release_mv) &&
           (left.overcurrent_trip_ma == right.overcurrent_trip_ma) &&
           (left.overcurrent_release_ma == right.overcurrent_release_ma) &&
           (left.trip_samples == right.trip_samples) &&
           (left.release_samples == right.release_samples) &&
           (left.communication_fail_samples ==
            right.communication_fail_samples) &&
           (left.latch_faults == right.latch_faults) &&
           (left.motion_inhibit_enabled == right.motion_inhibit_enabled);
}

} /* namespace */

drivers::DriverStatus App_Ina219Init(void)
{
    const Ina219ProtectionConfig config = ConfigFromStore();
    const drivers::DriverStatus status =
        Ina219Protection_Init(&g_data.protection, &config);
    g_data.initialized = status == drivers::DRIVER_OK;
    if (!g_data.initialized) {
        g_data.protection.last_read_status = status;
    }
    g_data.last_attempt_ms = 0U;
    g_data.last_update_ms = 0U;
    g_firstRun = true;
    return status;
}

void App_Ina219Run(void)
{
    if (!g_data.initialized) {
        return;
    }
    if (!g_firstRun &&
        !services::Time_HasElapsed(g_data.last_attempt_ms,
                                   kProtectionPollPeriodMs)) {
        return;
    }

    g_firstRun = false;
    g_data.last_attempt_ms = services::Time_Millis();
    const Ina219ProtectionConfig latest_config = ConfigFromStore();
    if (!ConfigEqual(latest_config, g_data.protection.config)) {
        if (Ina219Protection_Configure(&g_data.protection,
                                       &latest_config) != drivers::DRIVER_OK) {
            return;
        }
    }

    drivers::Ina219Measurement measurement = {};
    const drivers::DriverStatus status =
        board::Board_Ina219ReadMeasurement(&measurement);
    if (status != drivers::DRIVER_OK) {
        (void) Ina219Protection_RecordReadError(&g_data.protection, status);
        return;
    }

    const Ina219ProtectionSample sample = {
        measurement.bus_voltage_mv,
        measurement.current_ua,
        measurement.conversion_ready,
        measurement.math_overflow
    };
    (void) Ina219Protection_RecordSample(&g_data.protection, &sample);
    if (g_data.protection.sample_valid) {
        g_data.last_update_ms = g_data.last_attempt_ms;
    }
}

drivers::DriverStatus App_Ina219ReloadConfig(void)
{
    const Ina219ProtectionConfig config = ConfigFromStore();
    const drivers::DriverStatus status = g_data.initialized ?
        Ina219Protection_Configure(&g_data.protection, &config) :
        Ina219Protection_Init(&g_data.protection, &config);
    if (status == drivers::DRIVER_OK) {
        g_data.initialized = true;
        g_firstRun = true;
    }
    return status;
}

const AppIna219Data *App_Ina219GetData(void)
{
    return &g_data;
}

void App_Ina219ClearLatchedFaults(void)
{
    Ina219Protection_ClearLatchedFaults(&g_data.protection);
}

bool App_Ina219MotionInhibitRequested(void)
{
    return g_data.initialized &&
           Ina219Protection_MotionInhibitRequested(&g_data.protection);
}

} /* namespace app */
