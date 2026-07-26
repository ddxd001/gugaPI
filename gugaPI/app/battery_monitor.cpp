#include "app/battery_monitor.h"

#include <stdint.h>

namespace app {
namespace {

static const uint8_t kSeriesCells = 3U;
static const uint16_t kRatedCapacityMah = 3000U;
static const uint32_t kRatedCapacityUah =
    (uint32_t) kRatedCapacityMah * 1000U;
static const uint8_t kInitialVoltageSampleCount = 20U;
static const uint32_t kMaximumIntegrationGapMs = 1000U;
static const int32_t kChargingThresholdUa = -10000;
static const uint16_t kLowVoltageMv = 10800U;
static const uint16_t kLowVoltageReleaseMv = 11100U;
static const uint16_t kCriticalVoltageMv = 9900U;
static const uint16_t kCriticalVoltageReleaseMv = 10200U;
static const int64_t kUaMsPerUah = 3600000LL;

struct VoltageSocPoint {
    uint16_t pack_voltage_mv;
    uint8_t soc_percent;
};

static const VoltageSocPoint kVoltageSocTable[] = {
    { 9900U, 0U },
    { 10350U, 10U },
    { 10800U, 20U },
    { 11040U, 30U },
    { 11190U, 40U },
    { 11370U, 50U },
    { 11550U, 60U },
    { 11760U, 70U },
    { 12000U, 80U },
    { 12300U, 90U },
    { 12600U, 100U }
};

BatteryMonitorStatus g_status = {};
uint32_t g_initialVoltageSumMv = 0U;
uint8_t g_initialVoltageSamples = 0U;
uint16_t g_filteredVoltageMv = 0U;
uint32_t g_lastIntegrationMs = 0U;
bool g_hasIntegrationTime = false;
int64_t g_integrationRemainderUaMs = 0LL;

uint8_t EstimateSocFromVoltage(uint16_t pack_voltage_mv)
{
    const uint32_t point_count =
        sizeof(kVoltageSocTable) / sizeof(kVoltageSocTable[0]);
    if (pack_voltage_mv <= kVoltageSocTable[0].pack_voltage_mv) {
        return kVoltageSocTable[0].soc_percent;
    }

    for (uint32_t i = 1U; i < point_count; i++) {
        const VoltageSocPoint &high = kVoltageSocTable[i];
        if (pack_voltage_mv <= high.pack_voltage_mv) {
            const VoltageSocPoint &low = kVoltageSocTable[i - 1U];
            const uint32_t voltage_span =
                (uint32_t) high.pack_voltage_mv - low.pack_voltage_mv;
            const uint32_t soc_span =
                (uint32_t) high.soc_percent - low.soc_percent;
            const uint32_t voltage_offset =
                (uint32_t) pack_voltage_mv - low.pack_voltage_mv;
            return (uint8_t) (low.soc_percent +
                ((voltage_offset * soc_span) / voltage_span));
        }
    }

    return 100U;
}

void UpdateSocPercent(void)
{
    const uint32_t rounded =
        (g_status.remaining_uah * 100U) + (kRatedCapacityUah / 2U);
    g_status.soc_percent = (uint8_t) (rounded / kRatedCapacityUah);
    if (g_status.soc_percent > 100U) {
        g_status.soc_percent = 100U;
    }
    g_status.consumed_uah = kRatedCapacityUah - g_status.remaining_uah;
}

void UpdateVoltageFlags(uint16_t pack_voltage_mv)
{
    if (g_filteredVoltageMv == 0U) {
        g_filteredVoltageMv = pack_voltage_mv;
    } else {
        const int32_t difference =
            (int32_t) pack_voltage_mv - (int32_t) g_filteredVoltageMv;
        g_filteredVoltageMv = (uint16_t) ((int32_t) g_filteredVoltageMv +
                                           (difference / 8));
    }

    if (g_status.critical_battery) {
        if (g_filteredVoltageMv >= kCriticalVoltageReleaseMv) {
            g_status.critical_battery = false;
        }
    } else if (g_filteredVoltageMv <= kCriticalVoltageMv) {
        g_status.critical_battery = true;
    }

    if (g_status.low_battery) {
        if ((!g_status.critical_battery) &&
            (g_filteredVoltageMv >= kLowVoltageReleaseMv)) {
            g_status.low_battery = false;
        }
    } else if ((g_filteredVoltageMv <= kLowVoltageMv) ||
               g_status.critical_battery) {
        g_status.low_battery = true;
    }
}

void InitializeSocFromVoltage(uint16_t pack_voltage_mv)
{
    const uint8_t soc_percent = EstimateSocFromVoltage(pack_voltage_mv);
    g_status.remaining_uah =
        ((uint32_t) soc_percent * kRatedCapacityUah) / 100U;
    g_status.soc_ready = true;
    g_status.soc_source = BATTERY_SOC_SOURCE_VOLTAGE;
    g_hasIntegrationTime = false;
    g_integrationRemainderUaMs = 0LL;
    UpdateSocPercent();
}

void IntegrateCurrent(int32_t current_ua, uint32_t now_ms)
{
    if (!g_status.soc_ready) {
        return;
    }
    if (!g_hasIntegrationTime) {
        g_lastIntegrationMs = now_ms;
        g_hasIntegrationTime = true;
        return;
    }

    const uint32_t elapsed_ms = now_ms - g_lastIntegrationMs;
    g_lastIntegrationMs = now_ms;
    if ((elapsed_ms == 0U) || (elapsed_ms > kMaximumIntegrationGapMs)) {
        g_integrationRemainderUaMs = 0LL;
        return;
    }

    g_integrationRemainderUaMs +=
        (int64_t) current_ua * (int64_t) elapsed_ms;
    const int64_t delta_uah =
        g_integrationRemainderUaMs / kUaMsPerUah;
    g_integrationRemainderUaMs -= delta_uah * kUaMsPerUah;

    int64_t remaining_uah =
        (int64_t) g_status.remaining_uah - delta_uah;
    if (remaining_uah < 0LL) {
        remaining_uah = 0LL;
        g_integrationRemainderUaMs = 0LL;
    } else if (remaining_uah > (int64_t) kRatedCapacityUah) {
        remaining_uah = (int64_t) kRatedCapacityUah;
        g_integrationRemainderUaMs = 0LL;
    }
    g_status.remaining_uah = (uint32_t) remaining_uah;
    UpdateSocPercent();
}

} /* namespace */

void BatteryMonitor_Init(void)
{
    g_status = {};
    g_status.initialized = true;
    g_status.series_cells = kSeriesCells;
    g_status.rated_capacity_mah = kRatedCapacityMah;
    g_status.last_read_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
    g_status.soc_source = BATTERY_SOC_SOURCE_ESTIMATING;
    g_initialVoltageSumMv = 0U;
    g_initialVoltageSamples = 0U;
    g_filteredVoltageMv = 0U;
    g_lastIntegrationMs = 0U;
    g_hasIntegrationTime = false;
    g_integrationRemainderUaMs = 0LL;
}

void BatteryMonitor_RecordMeasurement(
    const drivers::Ina219Measurement *measurement,
    uint32_t now_ms)
{
    if ((!g_status.initialized) || (measurement == 0)) {
        return;
    }

    g_status.last_read_status = drivers::DRIVER_OK;
    g_status.sample_valid = measurement->conversion_ready &&
                            (!measurement->math_overflow) &&
                            (measurement->bus_voltage_mv > 0) &&
                            (measurement->bus_voltage_mv <= 26000);
    if (measurement->math_overflow) {
        g_status.overflow_count++;
    }
    if (!g_status.sample_valid) {
        g_hasIntegrationTime = false;
        return;
    }

    const uint16_t pack_voltage_mv =
        (uint16_t) measurement->bus_voltage_mv;
    g_status.pack_voltage_mv = pack_voltage_mv;
    g_status.average_cell_voltage_mv =
        (uint16_t) (pack_voltage_mv / kSeriesCells);
    g_status.current_ua = measurement->current_ua;
    g_status.power_mw = measurement->power_mw;
    g_status.charging = measurement->current_ua < kChargingThresholdUa;
    g_status.successful_sample_count++;
    g_status.last_update_ms = now_ms;

    const int64_t current_ua = measurement->current_ua;
    const uint32_t absolute_current_ma = (uint32_t) (
        ((current_ua < 0LL) ? -current_ua : current_ua) / 1000LL);
    if (absolute_current_ma > g_status.peak_current_ma) {
        g_status.peak_current_ma = absolute_current_ma;
    }

    UpdateVoltageFlags(pack_voltage_mv);

    if (!g_status.soc_ready) {
        g_initialVoltageSumMv += pack_voltage_mv;
        g_initialVoltageSamples++;
        if (g_initialVoltageSamples >= kInitialVoltageSampleCount) {
            InitializeSocFromVoltage((uint16_t) (
                g_initialVoltageSumMv / g_initialVoltageSamples));
        }
        return;
    }

    IntegrateCurrent(measurement->current_ua, now_ms);
}

void BatteryMonitor_RecordReadError(drivers::DriverStatus status)
{
    if (!g_status.initialized) {
        return;
    }
    g_status.sample_valid = false;
    g_status.last_read_status = status;
    g_status.read_error_count++;
    g_hasIntegrationTime = false;
}

void BatteryMonitor_SetFull(void)
{
    if (!g_status.initialized) {
        BatteryMonitor_Init();
    }
    g_status.remaining_uah = kRatedCapacityUah;
    g_status.soc_ready = true;
    g_status.soc_source = BATTERY_SOC_SOURCE_MANUAL_FULL;
    g_initialVoltageSumMv = 0U;
    g_initialVoltageSamples = 0U;
    g_hasIntegrationTime = false;
    g_integrationRemainderUaMs = 0LL;
    UpdateSocPercent();
}

void BatteryMonitor_ResetEstimate(void)
{
    BatteryMonitor_Init();
}

const BatteryMonitorStatus *BatteryMonitor_GetStatus(void)
{
    return &g_status;
}

} /* namespace app */
