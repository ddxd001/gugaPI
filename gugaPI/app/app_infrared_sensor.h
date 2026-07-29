#ifndef APP_APP_INFRARED_SENSOR_H_
#define APP_APP_INFRARED_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "drivers/infrared_line/infrared_line_protocol.h"

namespace app {

enum InfraredCalibrationStep : uint8_t {
    IR_CAL_STEP_NONE = 0U,
    IR_CAL_STEP_WHITE,
    IR_CAL_STEP_BLACK,
    IR_CAL_STEP_CENTER,
    IR_CAL_STEP_LEFT,
    IR_CAL_STEP_RIGHT
};

enum AppInfraredCommState : uint8_t {
    IR_COMM_STARTING = 0U,
    IR_COMM_HEALTHY,
    IR_COMM_DEGRADED,
    IR_COMM_FAULT
};

struct AppInfraredSensorData {
    drivers::InfraredLineFrame frame;
    drivers::InfraredLineParserStats parser_stats;
    bool valid;
    bool calibrated;
    bool line_detected;
    bool all_black;
    bool position_valid;
    int16_t position_mpos;
    uint16_t confidence;
    uint16_t threshold_on;
    uint16_t threshold_off;
    uint32_t stale_timeout_ms;
    uint32_t white_since_ms;
    uint32_t uart_dropped_count;
    uint32_t uart_error_count;
    uint32_t rx_timeout_count;
    uint32_t overrun_error_count;
    uint32_t framing_error_count;
    uint32_t parity_error_count;
    uint32_t noise_error_count;
    uint32_t dma_wrap_count;
    uint32_t dma_produced_count;
    uint32_t dma_consumed_count;
    uint32_t dma_current_lag;
    uint32_t dma_maximum_lag;
    uint32_t dma_overwrite_count;
    uint32_t dma_fault_count;
    uint32_t control_latency_us;
    uint32_t maximum_control_latency_us;
    uint8_t error_streak;
    uint8_t recovery_streak;
    AppInfraredCommState communication_state;
    drivers::DriverStatus last_status;
};

struct AppInfraredCalibrationStatus {
    bool active;
    InfraredCalibrationStep capturing;
    uint16_t sample_count;
    uint16_t target_samples;
    bool white_ready;
    bool black_ready;
    bool center_ready;
    bool left_ready;
    bool right_ready;
    uint16_t white_level;
    uint16_t black_level;
    int16_t center_offset;
    int16_t left_offset;
    int16_t right_offset;
    drivers::DriverStatus last_status;
};

void App_InfraredSensorInit(void);
void App_InfraredSensorUpdate(void);
const AppInfraredSensorData *App_InfraredSensorGetData(void);
bool App_InfraredSensorIsFresh(uint32_t now_ms);
void App_InfraredSensorClearStats(void);
void App_InfraredSensorRecordControlLatency(uint32_t frame_sequence);
const char *App_InfraredCommStateText(AppInfraredCommState state);
drivers::DriverStatus App_InfraredCalibrationBegin(void);
drivers::DriverStatus App_InfraredCalibrationCapture(
    InfraredCalibrationStep step);
drivers::DriverStatus App_InfraredCalibrationCommit(void);
void App_InfraredCalibrationCancel(void);
const AppInfraredCalibrationStatus *App_InfraredCalibrationGetStatus(void);
const char *App_InfraredCalibrationStepText(InfraredCalibrationStep step);

} /* namespace app */

#endif /* APP_APP_INFRARED_SENSOR_H_ */
