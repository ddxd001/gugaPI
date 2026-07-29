#include "app/app_infrared_sensor.h"

#include <limits.h>

#include "app/config_store.h"
#include "board/board_infrared_sensor.h"
#include "services/time.h"

namespace app {
namespace {

static const uint16_t kCalibrationSamples = 64U;
static const uint16_t kMinimumAdcGap = 200U;
static const uint16_t kMinimumHysteresis = 20U;
static const uint16_t kMaximumHysteresis = 500U;
static const uint32_t kLineLostDebounceMs = 20U;

drivers::InfraredLineParser g_parser;
AppInfraredSensorData g_data = {};
AppInfraredCalibrationStatus g_calibration = {};
uint32_t g_previousValidFrames = 0U;
uint32_t g_previousCrcErrors = 0U;
uint8_t g_consecutiveCrcFailures = 0U;

struct CaptureAccumulator {
    int64_t offset_sum;
    int16_t offset_min;
    int16_t offset_max;
    uint32_t adc_sum[3];
    uint16_t adc_min[3];
    uint16_t adc_max[3];
};

CaptureAccumulator g_capture = {};

int32_t Abs32(int32_t value)
{
    return (value >= 0) ? value : ((value == INT32_MIN) ? INT32_MAX : -value);
}

uint16_t ClampU16(uint32_t value, uint16_t minimum, uint16_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return static_cast<uint16_t>(value);
}

void ResetCapture(void)
{
    g_capture = {};
    g_capture.offset_min = INT16_MAX;
    g_capture.offset_max = INT16_MIN;
    for (uint8_t i = 0U; i < 3U; i++) {
        g_capture.adc_min[i] = UINT16_MAX;
    }
    g_calibration.sample_count = 0U;
    g_calibration.target_samples = kCalibrationSamples;
}

uint32_t StaleTimeoutMs(void)
{
    const uint32_t average = g_parser.stats.average_period_ms;
    if (average == 0U) {
        return 100U;
    }
    uint32_t timeout = average * 4U;
    if (timeout < 30U) {
        timeout = 30U;
    }
    if (timeout > 200U) {
        timeout = 200U;
    }
    return timeout;
}

void RefreshCalibrationFields(void)
{
    const ConfigStoreParams *params = ConfigStore_Get();
    g_data.calibrated = (params != 0) &&
        (params->infrared_position_span_raw != 0U) &&
        (params->infrared_adc_threshold != 0U);
    if (!g_data.calibrated) {
        g_data.threshold_on = 0U;
        g_data.threshold_off = 0U;
        return;
    }
    const uint16_t lower = static_cast<uint16_t>(
        params->infrared_adc_hysteresis / 2U);
    const uint16_t upper = static_cast<uint16_t>(
        (params->infrared_adc_hysteresis + 1U) / 2U);
    g_data.threshold_on = static_cast<uint16_t>(
        params->infrared_adc_threshold + upper);
    g_data.threshold_off = static_cast<uint16_t>(
        params->infrared_adc_threshold - lower);
}

void UpdateProcessedFrame(uint32_t now_ms)
{
    RefreshCalibrationFields();
    g_data.all_black = (g_data.frame.all_black == 1U);
    g_data.stale_timeout_ms = StaleTimeoutMs();
    if (!g_data.calibrated) {
        g_data.line_detected = false;
        g_data.position_valid = false;
        g_data.position_mpos = 0;
        g_data.confidence = 0U;
        g_data.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return;
    }

    uint16_t maximum_adc = g_data.frame.adc[0];
    for (uint8_t i = 1U; i < 3U; i++) {
        if (g_data.frame.adc[i] > maximum_adc) {
            maximum_adc = g_data.frame.adc[i];
        }
    }
    if (g_data.all_black || (maximum_adc >= g_data.threshold_on)) {
        g_data.line_detected = true;
        g_data.white_since_ms = 0U;
    } else if (maximum_adc <= g_data.threshold_off) {
        if (g_data.white_since_ms == 0U) {
            g_data.white_since_ms = now_ms;
        } else if ((now_ms - g_data.white_since_ms) >=
                   kLineLostDebounceMs) {
            g_data.line_detected = false;
        }
    }

    const ConfigStoreParams *params = ConfigStore_Get();
    int32_t position = static_cast<int32_t>(g_data.frame.offset) * 3000;
    position /= static_cast<int32_t>(params->infrared_position_span_raw);
    if (params->infrared_position_invert != 0U) {
        position = -position;
    }
    if (position > 3500) {
        position = 3500;
    } else if (position < -3500) {
        position = -3500;
    }
    g_data.position_mpos = static_cast<int16_t>(position);
    g_data.position_valid = g_data.line_detected;
    if (maximum_adc <= g_data.threshold_off) {
        g_data.confidence = 0U;
    } else {
        const uint32_t range = 4095U - g_data.threshold_off;
        g_data.confidence = (range == 0U) ? 1000U : ClampU16(
            (static_cast<uint32_t>(maximum_adc - g_data.threshold_off) *
             1000U) / range,
            0U,
            1000U);
    }
    g_data.last_status = drivers::DRIVER_OK;
}

void AccumulateCalibrationFrame(const drivers::InfraredLineFrame &frame)
{
    if ((g_calibration.capturing == IR_CAL_STEP_BLACK) &&
        (frame.all_black != 1U)) {
        return;
    }
    if ((g_calibration.capturing != IR_CAL_STEP_BLACK) &&
        (frame.all_black == 1U)) {
        return;
    }

    g_capture.offset_sum += frame.offset;
    if (frame.offset < g_capture.offset_min) {
        g_capture.offset_min = frame.offset;
    }
    if (frame.offset > g_capture.offset_max) {
        g_capture.offset_max = frame.offset;
    }
    for (uint8_t i = 0U; i < 3U; i++) {
        g_capture.adc_sum[i] += frame.adc[i];
        if (frame.adc[i] < g_capture.adc_min[i]) {
            g_capture.adc_min[i] = frame.adc[i];
        }
        if (frame.adc[i] > g_capture.adc_max[i]) {
            g_capture.adc_max[i] = frame.adc[i];
        }
    }
    g_calibration.sample_count++;
    if (g_calibration.sample_count < g_calibration.target_samples) {
        return;
    }

    const uint32_t divisor = g_calibration.target_samples - 2U;
    const int32_t offset_average = static_cast<int32_t>(
        (g_capture.offset_sum - g_capture.offset_min -
         g_capture.offset_max) / static_cast<int32_t>(divisor));
    uint16_t adc_average[3];
    for (uint8_t i = 0U; i < 3U; i++) {
        adc_average[i] = static_cast<uint16_t>(
            (g_capture.adc_sum[i] - g_capture.adc_min[i] -
             g_capture.adc_max[i]) / divisor);
    }

    switch (g_calibration.capturing) {
    case IR_CAL_STEP_WHITE:
        g_calibration.white_level = adc_average[0];
        for (uint8_t i = 1U; i < 3U; i++) {
            if (adc_average[i] > g_calibration.white_level) {
                g_calibration.white_level = adc_average[i];
            }
        }
        g_calibration.white_ready = true;
        break;
    case IR_CAL_STEP_BLACK:
        g_calibration.black_level = adc_average[0];
        for (uint8_t i = 1U; i < 3U; i++) {
            if (adc_average[i] < g_calibration.black_level) {
                g_calibration.black_level = adc_average[i];
            }
        }
        g_calibration.black_ready = true;
        break;
    case IR_CAL_STEP_CENTER:
        g_calibration.center_offset = static_cast<int16_t>(offset_average);
        g_calibration.center_ready = true;
        break;
    case IR_CAL_STEP_LEFT:
        g_calibration.left_offset = static_cast<int16_t>(offset_average);
        g_calibration.left_ready = true;
        break;
    case IR_CAL_STEP_RIGHT:
        g_calibration.right_offset = static_cast<int16_t>(offset_average);
        g_calibration.right_ready = true;
        break;
    default:
        break;
    }
    g_calibration.capturing = IR_CAL_STEP_NONE;
    g_calibration.last_status = drivers::DRIVER_OK;
}

} /* namespace */

void App_InfraredSensorInit(void)
{
    InfraredLineParser_Init(&g_parser);
    g_data = {};
    g_calibration = {};
    g_data.stale_timeout_ms = 100U;
    g_previousValidFrames = 0U;
    g_previousCrcErrors = 0U;
    g_consecutiveCrcFailures = 0U;
    g_data.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
    RefreshCalibrationFields();
}

void App_InfraredSensorUpdate(void)
{
    uint8_t byte = 0U;
    const uint32_t now = services::Time_Millis();
    board::Board_InfraredSensorServiceRx();
    while (board::Board_InfraredSensorReadByte(&byte)) {
        drivers::InfraredLineFrame frame = {};
        if (InfraredLineParser_FeedByte(&g_parser, byte, now, &frame)) {
            g_data.frame = frame;
            g_data.valid = true;
            UpdateProcessedFrame(now);
            if (g_calibration.active &&
                (g_calibration.capturing != IR_CAL_STEP_NONE)) {
                AccumulateCalibrationFrame(frame);
            }
        }
    }
    g_data.parser_stats = g_parser.stats;
    const uint32_t dropped = board::Board_InfraredSensorGetDroppedCount();
    const uint32_t uart_errors =
        board::Board_InfraredSensorGetUartErrorCount();
    const uint32_t overrun_errors =
        board::Board_InfraredSensorGetOverrunErrorCount();
    const bool transport_fault =
        (dropped > g_data.uart_dropped_count) ||
        (overrun_errors > g_data.overrun_error_count);
    if (g_parser.stats.valid_frames != g_previousValidFrames) {
        g_consecutiveCrcFailures = 0U;
    } else if (g_parser.stats.crc_errors > g_previousCrcErrors) {
        const uint32_t delta =
            g_parser.stats.crc_errors - g_previousCrcErrors;
        const uint32_t total = g_consecutiveCrcFailures + delta;
        g_consecutiveCrcFailures = static_cast<uint8_t>(
            (total > UINT8_MAX) ? UINT8_MAX : total);
    }
    if (transport_fault || (g_consecutiveCrcFailures >= 3U)) {
        /* Make the unified snapshot fail immediately. LF_Update performs the
         * actual motor stop in its 2 ms task; no PID or chassis call occurs
         * in the UART ISR. A later valid frame restores diagnostic validity,
         * but a stopped LF session is never restarted automatically. */
        g_data.valid = false;
        g_data.position_valid = false;
        g_data.last_status = drivers::DRIVER_ERROR;
    }
    g_previousValidFrames = g_parser.stats.valid_frames;
    g_previousCrcErrors = g_parser.stats.crc_errors;
    g_data.uart_dropped_count = dropped;
    g_data.uart_error_count = uart_errors;
    g_data.rx_timeout_count =
        board::Board_InfraredSensorGetRxTimeoutCount();
    g_data.overrun_error_count = overrun_errors;
    g_data.framing_error_count =
        board::Board_InfraredSensorGetFramingErrorCount();
    g_data.parity_error_count =
        board::Board_InfraredSensorGetParityErrorCount();
    g_data.noise_error_count =
        board::Board_InfraredSensorGetNoiseErrorCount();
    g_data.stale_timeout_ms = StaleTimeoutMs();
}

const AppInfraredSensorData *App_InfraredSensorGetData(void)
{
    return &g_data;
}

bool App_InfraredSensorIsFresh(uint32_t now_ms)
{
    return g_data.valid &&
           ((now_ms - g_data.frame.received_ms) <= g_data.stale_timeout_ms);
}

void App_InfraredSensorClearStats(void)
{
    InfraredLineParser_Init(&g_parser);
    board::Board_InfraredSensorClear();
    g_data.parser_stats = {};
    g_data.uart_dropped_count = 0U;
    g_data.uart_error_count = 0U;
    g_data.rx_timeout_count = 0U;
    g_data.overrun_error_count = 0U;
    g_data.framing_error_count = 0U;
    g_data.parity_error_count = 0U;
    g_data.noise_error_count = 0U;
    g_previousValidFrames = 0U;
    g_previousCrcErrors = 0U;
    g_consecutiveCrcFailures = 0U;
}

drivers::DriverStatus App_InfraredCalibrationBegin(void)
{
    g_calibration = {};
    g_calibration.active = true;
    g_calibration.target_samples = kCalibrationSamples;
    g_calibration.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus App_InfraredCalibrationCapture(
    InfraredCalibrationStep step)
{
    if ((!g_calibration.active) || (step == IR_CAL_STEP_NONE)) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (g_calibration.capturing != IR_CAL_STEP_NONE) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    ResetCapture();
    g_calibration.capturing = step;
    g_calibration.last_status = drivers::DRIVER_ERROR_BUSY;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus App_InfraredCalibrationCommit(void)
{
    if ((!g_calibration.active) ||
        (!g_calibration.white_ready) || (!g_calibration.black_ready) ||
        (!g_calibration.center_ready) || (!g_calibration.left_ready) ||
        (!g_calibration.right_ready) ||
        (g_calibration.black_level <
         static_cast<uint16_t>(g_calibration.white_level + kMinimumAdcGap))) {
        g_calibration.last_status = drivers::DRIVER_ERROR_INVALID_ARG;
        return g_calibration.last_status;
    }
    const int32_t left = g_calibration.left_offset;
    const int32_t right = g_calibration.right_offset;
    if (((left <= 0) && (right <= 0)) || ((left >= 0) && (right >= 0))) {
        g_calibration.last_status = drivers::DRIVER_ERROR_INVALID_ARG;
        return g_calibration.last_status;
    }
    const uint32_t span = static_cast<uint32_t>(
        (Abs32(left) + Abs32(right)) / 2);
    if ((span == 0U) || (span > 32767U) ||
        (static_cast<uint32_t>(Abs32(g_calibration.center_offset)) * 10U >
         span)) {
        g_calibration.last_status = drivers::DRIVER_ERROR_INVALID_ARG;
        return g_calibration.last_status;
    }
    const uint16_t gap = static_cast<uint16_t>(
        g_calibration.black_level - g_calibration.white_level);
    const uint16_t threshold = static_cast<uint16_t>(
        (static_cast<uint32_t>(g_calibration.black_level) +
         g_calibration.white_level) / 2U);
    const uint16_t hysteresis = ClampU16(
        static_cast<uint32_t>(gap) / 5U,
        kMinimumHysteresis,
        kMaximumHysteresis);
    const uint8_t invert = (left < 0) ? 1U : 0U;
    const drivers::DriverStatus status =
        ConfigStore_SetInfraredCalibration(
            invert,
            static_cast<uint16_t>(span),
            threshold,
            hysteresis);
    g_calibration.last_status = status;
    if (status == drivers::DRIVER_OK) {
        g_calibration.active = false;
        g_calibration.capturing = IR_CAL_STEP_NONE;
        RefreshCalibrationFields();
    }
    return status;
}

void App_InfraredCalibrationCancel(void)
{
    g_calibration = {};
}

const AppInfraredCalibrationStatus *App_InfraredCalibrationGetStatus(void)
{
    return &g_calibration;
}

const char *App_InfraredCalibrationStepText(InfraredCalibrationStep step)
{
    switch (step) {
    case IR_CAL_STEP_WHITE: return "white";
    case IR_CAL_STEP_BLACK: return "black";
    case IR_CAL_STEP_CENTER: return "center";
    case IR_CAL_STEP_LEFT: return "left";
    case IR_CAL_STEP_RIGHT: return "right";
    default: return "none";
    }
}

} /* namespace app */
