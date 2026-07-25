#include "app/app_grayscale.h"

#include <limits.h>

#include "app/config_store.h"
#include "board/board_grayscale.h"
#include "services/time.h"

namespace app {
namespace {

static const uint16_t kMinimumCalibrationSpan = 200U;
static const uint16_t kDefaultCaptureFrames = 16U;
static const uint32_t kDefaultSweepDurationMs = 2000U;
// Grayscale calibration first became persistent in ConfigStore payload v5.
static const uint16_t kFirstCalibrationPayloadLength = 137U;

AppGrayscaleData g_data = {};
uint16_t g_workingRaw[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
drivers::GrayscaleCalibration g_calibration = {};
drivers::GrayscaleProcessingState g_processingState = {};
GrayscaleRoadClassifierState g_roadState = {};
static const GrayscaleRoadClassifierConfig kRoadConfig = {
    0xC0U,
    0x3CU,
    0x03U,
    2U
};
AppGrayscaleCalibrationStatus g_calibrationStatus = {};
uint32_t g_calibrationStartMs = 0U;
uint32_t g_calibrationDurationMs = 0U;
uint32_t g_calibrationSum[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
uint16_t g_calibrationMin[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
uint16_t g_calibrationMax[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
uint16_t g_stagedWhite[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
uint16_t g_stagedBlack[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
bool g_calibrationCommissioned = false;
uint8_t g_nextChannel = 0U;

void IncrementSaturated(uint32_t *value)
{
    if (*value != UINT32_MAX) {
        (*value)++;
    }
}

void MarkFailure(drivers::DriverStatus status)
{
    g_data.valid = false;
    g_data.processed_valid = false;
    g_data.last_status = status;
    g_data.processing_status = status;
    IncrementSaturated(&g_data.error_count);
    g_nextChannel = 0U;
    g_processingState.active_mask = 0U;
    GrayscaleRoad_Init(&g_roadState);
}

void MarkProcessingFailure(drivers::DriverStatus status)
{
    g_data.processed_valid = false;
    g_data.processing_status = status;
    IncrementSaturated(&g_data.processing_error_count);
}

void ResetChannelDiagnostics(void)
{
    g_data.channel_anomaly_mask = 0U;
}

void ResetProcessingState(void)
{
    g_processingState.active_mask = 0U;
    GrayscaleRoad_Init(&g_roadState);
    g_data.road_type = GRAYSCALE_ROAD_UNKNOWN;
    g_data.road_confirm_count = 0U;
}

void PublishProcessed(const drivers::GrayscaleProcessedData &processed)
{
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        g_data.normalized[i] = processed.normalized[i];
    }
    g_data.active_mask = processed.active_mask;
    g_data.usable_mask = processed.usable_mask;
    g_data.track_mask = processed.track_mask;
    g_data.calibration_fault_mask = processed.calibration_fault_mask;
    g_data.saturation_mask = processed.saturation_mask;
    g_data.line_detected = processed.line_detected;
    g_data.line_position = processed.line_position;
    g_data.line_strength = processed.line_strength;
    g_data.threshold_on = drivers::Grayscale_GetThresholdOn(&g_calibration);
    g_data.threshold_off = drivers::Grayscale_GetThresholdOff(&g_calibration);
}

void ProcessPublishedFrame(void)
{
    drivers::GrayscaleProcessedData processed = {};
    const drivers::DriverStatus status =
        drivers::Grayscale_ProcessWithState(g_data.raw,
                                            &g_calibration,
                                            &g_processingState,
                                            &processed);
    PublishProcessed(processed);
    g_data.channel_anomaly_mask = processed.calibration_fault_mask;
    if (status != drivers::DRIVER_OK) {
        MarkProcessingFailure(status);
        return;
    }
    if (g_data.channel_anomaly_mask != 0U) {
        MarkProcessingFailure(drivers::DRIVER_ERROR);
        return;
    }
    g_data.road_type = GrayscaleRoad_Update(&g_roadState,
                                            &kRoadConfig,
                                            processed.active_mask,
                                            g_data.sequence);
    g_data.road_confirm_count = g_roadState.candidate_count;
    g_data.processed_valid = true;
    g_data.processing_status = drivers::DRIVER_OK;
}

void ResetCalibrationAccumulator(void)
{
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        g_calibrationSum[i] = 0U;
        g_calibrationMin[i] = drivers::GRAYSCALE_ADC_MAX;
        g_calibrationMax[i] = 0U;
    }
    g_calibrationStatus.sample_count = 0U;
    g_calibrationStatus.fault_mask = 0U;
}

uint8_t ValidateCalibrationSpan(const uint16_t white[drivers::GRAYSCALE_CHANNEL_COUNT],
                                const uint16_t black[drivers::GRAYSCALE_CHANNEL_COUNT])
{
    uint8_t fault_mask = 0U;
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        const uint16_t span = (white[i] >= black[i])
            ? static_cast<uint16_t>(white[i] - black[i])
            : static_cast<uint16_t>(black[i] - white[i]);
        if (span < kMinimumCalibrationSpan) {
            fault_mask = static_cast<uint8_t>(fault_mask | (1U << i));
        }
    }
    return fault_mask;
}

void FinishCalibration(drivers::DriverStatus status, uint8_t fault_mask)
{
    g_calibrationStatus.running = false;
    g_calibrationStatus.mode = APP_GRAYSCALE_CAL_IDLE;
    g_calibrationStatus.fault_mask = fault_mask;
    g_calibrationStatus.last_status = status;
}

void ApplySweepCalibration(void)
{
    const uint8_t fault_mask =
        ValidateCalibrationSpan(g_calibrationMax, g_calibrationMin);
    if (fault_mask != 0U) {
        FinishCalibration(drivers::DRIVER_ERROR_INVALID_ARG, fault_mask);
        return;
    }

    drivers::GrayscaleCalibration calibration = g_calibration;
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        /* Compatibility sweep assumes the common sensor polarity: white gives
         * the larger ADC value and black gives the smaller value. Explicit
         * white/black capture supports either polarity. */
        calibration.white[i] = g_calibrationMax[i];
        calibration.black[i] = g_calibrationMin[i];
    }
    const drivers::DriverStatus status =
        App_GrayscaleSetCalibration(&calibration);
    FinishCalibration(status, (status == drivers::DRIVER_OK) ? 0U : 0xFFU);
}

void UpdateCalibration(void)
{
    if (!g_calibrationStatus.running) {
        return;
    }

    if (g_calibrationStatus.mode == APP_GRAYSCALE_CAL_SWEEP) {
        for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
            if (g_data.raw[i] < g_calibrationMin[i]) {
                g_calibrationMin[i] = g_data.raw[i];
            }
            if (g_data.raw[i] > g_calibrationMax[i]) {
                g_calibrationMax[i] = g_data.raw[i];
            }
        }
        if (g_calibrationStatus.sample_count != 0xFFFFU) {
            g_calibrationStatus.sample_count++;
        }
        if ((services::Time_Millis() - g_calibrationStartMs) >=
            g_calibrationDurationMs) {
            ApplySweepCalibration();
        }
        return;
    }

    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        g_calibrationSum[i] += g_data.raw[i];
    }
    g_calibrationStatus.sample_count++;
    if (g_calibrationStatus.sample_count <
        g_calibrationStatus.target_samples) {
        return;
    }

    uint16_t *destination =
        (g_calibrationStatus.mode == APP_GRAYSCALE_CAL_WHITE)
        ? g_stagedWhite
        : g_stagedBlack;
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        destination[i] = static_cast<uint16_t>(
            g_calibrationSum[i] / g_calibrationStatus.sample_count);
    }
    if (g_calibrationStatus.mode == APP_GRAYSCALE_CAL_WHITE) {
        g_calibrationStatus.white_ready = true;
    } else {
        g_calibrationStatus.black_ready = true;
    }
    FinishCalibration(drivers::DRIVER_OK, 0U);
}

drivers::DriverStatus StartPointCalibration(
    AppGrayscaleCalibrationMode mode,
    uint16_t frames)
{
    if (g_calibrationStatus.running) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (frames == 0U) {
        frames = kDefaultCaptureFrames;
    }
    if (frames > 128U) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    ResetCalibrationAccumulator();
    g_calibrationStatus.mode = mode;
    g_calibrationStatus.running = true;
    g_calibrationStatus.target_samples = frames;
    g_calibrationStatus.last_status = drivers::DRIVER_ERROR_BUSY;
    return drivers::DRIVER_OK;
}

} /* namespace */

void App_GrayscaleInit(void)
{
    g_data = {};
    g_data.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
    g_data.processing_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
    g_nextChannel = 0U;
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        g_workingRaw[i] = 0U;
    }
    ResetChannelDiagnostics();
    ResetProcessingState();
    g_calibrationStatus = {};
    g_calibrationStatus.mode = APP_GRAYSCALE_CAL_IDLE;
    g_calibrationStatus.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
    g_calibrationCommissioned = false;
    (void) App_GrayscaleReloadCalibration();
}

void App_GrayscaleUpdate(void)
{
    if (!board::Board_GrayscaleIsReady()) {
        MarkFailure(drivers::DRIVER_ERROR_NOT_INITIALIZED);
        return;
    }

    uint16_t raw = 0U;
    const drivers::DriverStatus status =
        board::Board_GrayscaleReadChannel(g_nextChannel, &raw);
    if (status != drivers::DRIVER_OK) {
        MarkFailure(status);
        return;
    }

    g_workingRaw[g_nextChannel] = raw;
    g_nextChannel++;
    if (g_nextChannel < drivers::GRAYSCALE_CHANNEL_COUNT) {
        return;
    }

    g_nextChannel = 0U;
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        g_data.raw[i] = g_workingRaw[i];
    }
    const uint32_t now = services::Time_Millis();
    g_data.frame_period_ms = (g_data.sequence == 0U)
        ? 0U
        : (now - g_data.last_update_ms);
    g_data.last_update_ms = now;
    g_data.last_status = drivers::DRIVER_OK;
    IncrementSaturated(&g_data.sequence);
    g_data.valid = true;
    UpdateCalibration();
    ProcessPublishedFrame();
}

const AppGrayscaleData *App_GrayscaleGetData(void)
{
    return &g_data;
}

drivers::DriverStatus App_GrayscaleReloadCalibration(void)
{
    const ConfigStoreParams *params = ConfigStore_Get();
    if (params == 0) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    drivers::GrayscaleCalibration calibration = {};
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        calibration.white[i] = params->grayscale_white[i];
        calibration.black[i] = params->grayscale_black[i];
    }
    calibration.threshold = params->grayscale_threshold;
    calibration.hysteresis = params->grayscale_hysteresis;
    calibration.position_floor = params->grayscale_position_floor;
    calibration.min_line_strength = params->grayscale_min_line_strength;
    calibration.track_mask = params->grayscale_track_mask;
    g_calibration = calibration;

    uint8_t fault_mask = 0U;
    const drivers::DriverStatus status =
        drivers::Grayscale_ValidateCalibration(&calibration, &fault_mask);
    if (status != drivers::DRIVER_OK) {
        g_data.calibration_fault_mask = fault_mask;
        MarkProcessingFailure(status);
        return status;
    }

    g_data.calibration_fault_mask = 0U;
    g_data.processed_valid = false;
    g_data.processing_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
    ResetChannelDiagnostics();
    ResetProcessingState();
    const ConfigStoreStatus *store_status = ConfigStore_GetStatus();
    if ((store_status != 0) && store_status->loaded_from_fram &&
        (store_status->stored_length >= kFirstCalibrationPayloadLength)) {
        g_calibrationCommissioned = true;
    }
    return drivers::DRIVER_OK;
}

drivers::DriverStatus App_GrayscaleSetCalibration(
    const drivers::GrayscaleCalibration *calibration)
{
    uint8_t fault_mask = 0U;
    drivers::DriverStatus status =
        drivers::Grayscale_ValidateCalibration(calibration, &fault_mask);
    if (status != drivers::DRIVER_OK) {
        g_data.calibration_fault_mask = fault_mask;
        return status;
    }

    status = ConfigStore_SetGrayscaleCalibration(calibration->white,
                                                 calibration->black,
                                                 calibration->threshold);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    g_calibration = *calibration;
    g_calibrationCommissioned = true;
    g_data.calibration_fault_mask = 0U;
    g_data.processed_valid = false;
    g_data.processing_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
    ResetChannelDiagnostics();
    ResetProcessingState();
    return drivers::DRIVER_OK;
}

const drivers::GrayscaleCalibration *App_GrayscaleGetCalibration(void)
{
    return &g_calibration;
}

bool App_GrayscaleCalibrationIsCommissioned(void)
{
    return g_calibrationCommissioned;
}

drivers::DriverStatus App_GrayscaleStartSweepCalibration(uint32_t duration_ms)
{
    if (g_calibrationStatus.running) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (duration_ms == 0U) {
        duration_ms = kDefaultSweepDurationMs;
    }
    if ((duration_ms < 500U) || (duration_ms > 10000U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    ResetCalibrationAccumulator();
    g_calibrationStatus.mode = APP_GRAYSCALE_CAL_SWEEP;
    g_calibrationStatus.running = true;
    g_calibrationStatus.target_samples = 0U;
    g_calibrationStatus.last_status = drivers::DRIVER_ERROR_BUSY;
    g_calibrationStartMs = services::Time_Millis();
    g_calibrationDurationMs = duration_ms;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus App_GrayscaleStartWhiteCalibration(uint16_t frames)
{
    return StartPointCalibration(APP_GRAYSCALE_CAL_WHITE, frames);
}

drivers::DriverStatus App_GrayscaleStartBlackCalibration(uint16_t frames)
{
    return StartPointCalibration(APP_GRAYSCALE_CAL_BLACK, frames);
}

drivers::DriverStatus App_GrayscaleCommitCalibration(void)
{
    if (g_calibrationStatus.running) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if ((!g_calibrationStatus.white_ready) ||
        (!g_calibrationStatus.black_ready)) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    const uint8_t fault_mask =
        ValidateCalibrationSpan(g_stagedWhite, g_stagedBlack);
    if (fault_mask != 0U) {
        g_calibrationStatus.fault_mask = fault_mask;
        g_calibrationStatus.last_status = drivers::DRIVER_ERROR_INVALID_ARG;
        return g_calibrationStatus.last_status;
    }

    drivers::GrayscaleCalibration calibration = g_calibration;
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        calibration.white[i] = g_stagedWhite[i];
        calibration.black[i] = g_stagedBlack[i];
    }
    const drivers::DriverStatus status =
        App_GrayscaleSetCalibration(&calibration);
    g_calibrationStatus.last_status = status;
    g_calibrationStatus.fault_mask =
        (status == drivers::DRIVER_OK) ? 0U : 0xFFU;
    return status;
}

void App_GrayscaleCancelCalibration(void)
{
    if (g_calibrationStatus.running) {
        FinishCalibration(drivers::DRIVER_ERROR, 0U);
    }
}

const AppGrayscaleCalibrationStatus *App_GrayscaleGetCalibrationStatus(void)
{
    return &g_calibrationStatus;
}

} /* namespace app */
