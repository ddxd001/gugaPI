#ifndef APP_BATTERY_MONITOR_H_
#define APP_BATTERY_MONITOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "drivers/ina219/ina219.h"

namespace app {

enum BatterySocSource {
    BATTERY_SOC_SOURCE_ESTIMATING = 0,
    BATTERY_SOC_SOURCE_VOLTAGE,
    BATTERY_SOC_SOURCE_MANUAL_FULL
};

struct BatteryMonitorStatus {
    bool initialized;
    bool soc_ready;
    bool sample_valid;
    bool low_battery;
    bool critical_battery;
    bool charging;
    uint8_t series_cells;
    uint8_t soc_percent;
    uint16_t rated_capacity_mah;
    uint16_t pack_voltage_mv;
    uint16_t average_cell_voltage_mv;
    int32_t current_ua;
    int32_t power_mw;
    uint32_t remaining_uah;
    uint32_t consumed_uah;
    uint32_t peak_current_ma;
    uint32_t successful_sample_count;
    uint32_t overflow_count;
    uint32_t read_error_count;
    uint32_t last_update_ms;
    drivers::DriverStatus last_read_status;
    BatterySocSource soc_source;
};

void BatteryMonitor_Init(void);
void BatteryMonitor_RecordMeasurement(
    const drivers::Ina219Measurement *measurement,
    uint32_t now_ms);
void BatteryMonitor_RecordReadError(drivers::DriverStatus status);
void BatteryMonitor_SetFull(void);
void BatteryMonitor_ResetEstimate(void);
const BatteryMonitorStatus *BatteryMonitor_GetStatus(void);

} /* namespace app */

#endif /* APP_BATTERY_MONITOR_H_ */
