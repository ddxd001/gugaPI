#include "app/app_grayscale.h"

#include <limits.h>

#include "app/config_store.h"
#include "board/board_grayscale.h"
#include "services/time.h"

namespace app {
namespace {

static const uint16_t kMinimumCalibrationSpan = 400U;
static const uint16_t kDefaultCaptureFrames = 64U;
static const uint16_t kMaximumCaptureFrames = 128U;
static const uint16_t kCalibrationNoiseMultiplier = 8U;
static const uint16_t kDefaultThreshold = 500U;
static const uint16_t kDefaultHysteresis = 300U;
static const uint16_t kDefaultPositionFloor = 100U;
static const uint16_t kDefaultMinimumLineStrength = 600U;
static const uint8_t kDefaultTrackMask = 0x7EU;
static const uint32_t kDefaultSweepDurationMs = 2000U;
static const uint16_t kSaturationFaultFrames = 8U;
static const uint16_t kStuckFaultFrames = 125U;
static const uint16_t kChannelChangeThreshold = 0U;
// Grayscale calibration first became persistent in ConfigStore payload v5.
static const uint16_t kFirstCalibrationPayloadLength = 137U;

AppGrayscaleData g_data = {};
uint16_t g_workingRaw[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
drivers::GrayscaleCalibration g_calibration = {};
drivers::GrayscaleProcessingState g_processingState = {};
GrayscaleRoadClassifierState g_roadState = {};
static const GrayscaleRoadClassifierConfig kRoadConfig = {
    0x03U, /* left-side road evidence: channels 0..1 */
    0x3CU, /* forward road evidence: channels 2..5 */
    0xC0U, /* right-side road evidence: channels 6..7 */
    2U,    /* no-forward confirmation frames */
    3U,    /* side-exit confirmation frames */
    6U,    /* centered frames required before rearming */
    24U    /* bounded observation window */
};
AppGrayscaleCalibrationStatus g_calibrationStatus = {};
uint32_t g_calibrationStartMs = 0U;
uint32_t g_calibrationDurationMs = 0U;
uint16_t g_calibrationSamples[drivers::GRAYSCALE_CHANNEL_COUNT]
                             [kMaximumCaptureFrames];
uint16_t g_calibrationMin[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
uint16_t g_calibrationMax[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
uint16_t g_stagedWhite[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
uint16_t g_stagedBlack[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
uint16_t g_stagedWhiteNoise[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
uint16_t g_stagedBlackNoise[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
bool g_calibrationCommissioned = false;
/* One outermost road-classification sample is interleaved before each
 * complete six-channel tracking scan. This produces a coherent 1..6 position
 * frame every seven scheduler calls (normally 7 ms), while channels 0 and 7
 * both refresh within 14 ms. The scheduler API and task period are unchanged. */
static const uint8_t kScanSequence[] = {
    0U, 1U, 2U, 3U, 4U, 5U, 6U,
    7U, 1U, 2U, 3U, 4U, 5U, 6U
};
static const uint8_t kScanSequenceLength =
    static_cast<uint8_t>(sizeof(kScanSequence) / sizeof(kScanSequence[0]));
uint8_t g_scanPhase = 0U;
uint8_t g_initializedChannelMask = 0U;
uint8_t g_pendingFreshMask = 0U;
uint16_t g_previousRaw[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
uint16_t g_unchangedActivityFrames[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
uint16_t g_saturationFrames[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
bool g_previousRawValid = false;

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
    g_scanPhase = 0U;
    g_initializedChannelMask = 0U;
    g_pendingFreshMask = 0U;
    g_processingState.active_mask = 0U;
    g_processingState.last_position = 0;
    g_processingState.last_selected_mask = 0U;
    g_processingState.weak_tracking_frames = 0U;
    g_processingState.weak_recovery_frames = 0U;
    g_processingState.weak_recovery_eligible = false;
    g_processingState.position_valid = false;
    g_processingState.last_track_state = drivers::GRAYSCALE_TRACK_UNKNOWN;
    g_processingState.invalid_frames = 0U;
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
    g_previousRawValid = false;
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        g_previousRaw[i] = 0U;
        g_unchangedActivityFrames[i] = 0U;
        g_saturationFrames[i] = 0U;
    }
}

void ResetProcessingState(void)
{
    g_processingState.active_mask = 0U;
    g_processingState.last_position = 0;
    g_processingState.last_selected_mask = 0U;
    g_processingState.weak_tracking_frames = 0U;
    g_processingState.weak_recovery_frames = 0U;
    g_processingState.weak_recovery_eligible = false;
    g_processingState.position_valid = false;
    g_processingState.last_track_state = drivers::GRAYSCALE_TRACK_UNKNOWN;
    g_processingState.invalid_frames = 0U;
    GrayscaleRoad_Init(&g_roadState);
    g_data.road_type = GRAYSCALE_ROAD_UNKNOWN;
    g_data.road_confirm_count = 0U;
    g_data.road_phase = GRAYSCALE_ROAD_PHASE_NORMAL;
    g_data.road_observed_paths = 0U;
    g_data.road_event_sequence = 0U;
    g_data.road_event_type = GRAYSCALE_ROAD_UNKNOWN;
    g_data.road_event_paths = 0U;
    g_data.road_event_confidence = 0U;
    g_data.road_event_entry_mask = 0U;
    g_data.road_event_peak_mask = 0U;
    g_data.road_event_exit_mask = 0U;
}

void PublishProcessed(const drivers::GrayscaleProcessedData &processed)
{
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        g_data.normalized[i] = processed.normalized[i];
    }
    g_data.active_mask = processed.active_mask;
    g_data.usable_mask = processed.usable_mask;
    g_data.track_mask = processed.track_mask;
    g_data.selected_mask = processed.selected_mask;
    g_data.calibration_fault_mask = processed.calibration_fault_mask;
    g_data.saturation_mask = processed.saturation_mask;
    g_data.line_detected = processed.line_detected;
    g_data.position_valid = processed.position_valid;
    g_data.line_position = processed.line_position;
    g_data.line_strength = processed.line_strength;
    g_data.position_confidence = processed.position_confidence;
    g_data.position_source = processed.position_source;
    g_data.track_state = processed.track_state;
    g_data.weak_tracking_frames = processed.weak_tracking_frames;
    g_data.invalid_frames = processed.invalid_frames;
    g_data.threshold_on = drivers::Grayscale_GetThresholdOn(&g_calibration);
    g_data.threshold_off = drivers::Grayscale_GetThresholdOff(&g_calibration);
}

uint16_t AbsoluteDifference(uint16_t first, uint16_t second)
{
    return (first >= second)
        ? static_cast<uint16_t>(first - second)
        : static_cast<uint16_t>(second - first);
}

void UpdateChannelDiagnostics(
    const drivers::GrayscaleProcessedData &processed,
    uint8_t fresh_mask)
{
    uint8_t changed_mask = 0U;
    if (g_previousRawValid) {
        for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
            const uint8_t bit = static_cast<uint8_t>(1U << i);
            if (((fresh_mask & bit) != 0U) &&
                (AbsoluteDifference(g_data.raw[i], g_previousRaw[i]) >
                 kChannelChangeThreshold)) {
                changed_mask = static_cast<uint8_t>(changed_mask | (1U << i));
            }
        }
    }

    uint8_t runtime_fault_mask = 0U;
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        const uint8_t bit = static_cast<uint8_t>(1U << i);
        if ((!g_previousRawValid) || ((fresh_mask & bit) != 0U)) {
            if ((processed.saturation_mask & bit) != 0U) {
                if (g_saturationFrames[i] != UINT16_MAX) {
                    g_saturationFrames[i]++;
                }
            } else {
                g_saturationFrames[i] = 0U;
            }

            const bool channel_changed = (changed_mask & bit) != 0U;
            const bool another_channel_changed =
                (changed_mask & static_cast<uint8_t>(~bit)) != 0U;
            if ((!g_previousRawValid) || channel_changed) {
                g_unchangedActivityFrames[i] = 0U;
            } else if (another_channel_changed &&
                       (g_unchangedActivityFrames[i] != UINT16_MAX)) {
                g_unchangedActivityFrames[i]++;
            }
            g_previousRaw[i] = g_data.raw[i];
        }

        if ((g_saturationFrames[i] >= kSaturationFaultFrames) ||
            (g_unchangedActivityFrames[i] >= kStuckFaultFrames)) {
            runtime_fault_mask = static_cast<uint8_t>(runtime_fault_mask | bit);
        }
    }
    g_previousRawValid = true;
    g_data.channel_anomaly_mask = static_cast<uint8_t>(
        processed.calibration_fault_mask | runtime_fault_mask);
}

void ProcessPublishedFrame(uint8_t fresh_mask)
{
    drivers::GrayscaleProcessedData processed = {};
    const drivers::DriverStatus status =
        drivers::Grayscale_ProcessWithState(g_data.raw,
                                            &g_calibration,
                                            &g_processingState,
                                            &processed);
    PublishProcessed(processed);
    UpdateChannelDiagnostics(processed, fresh_mask);
    if (status != drivers::DRIVER_OK) {
        MarkProcessingFailure(status);
        return;
    }
    if (processed.calibration_fault_mask != 0U) {
        MarkProcessingFailure(drivers::DRIVER_ERROR);
        return;
    }
    uint8_t road_mask = processed.active_mask;
    if (processed.position_valid) {
        /* The hysteretic active mask remains authoritative for branches and
         * crossings. A valid analogue segment may still fall between two
         * core sensors and leave both below the digital-on threshold; in that
         * case preserve only center-line presence for road classification. */
        road_mask = static_cast<uint8_t>(
            road_mask |
            (processed.selected_mask & processed.track_mask));
    }
    g_data.road_type = GrayscaleRoad_Update(&g_roadState,
                                            &kRoadConfig,
                                            road_mask,
                                            g_data.sequence);
    g_data.road_confirm_count = g_roadState.candidate_count;
    g_data.road_phase = g_roadState.phase;
    g_data.road_observed_paths = g_roadState.observed_paths;
    const GrayscaleRoadEvent *event =
        GrayscaleRoad_GetLastEvent(&g_roadState);
    if ((event != 0) && event->valid) {
        g_data.road_event_sequence = event->sequence;
        g_data.road_event_type = event->type;
        g_data.road_event_paths = event->observed_paths;
        g_data.road_event_confidence = event->confidence;
        g_data.road_event_entry_mask = event->entry_mask;
        g_data.road_event_peak_mask = event->peak_mask;
        g_data.road_event_exit_mask = event->exit_mask;
    }
    g_data.processed_valid = true;
    g_data.processing_status = drivers::DRIVER_OK;
}

void ResetCalibrationAccumulator(void)
{
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        g_calibrationMin[i] = drivers::GRAYSCALE_ADC_MAX;
        g_calibrationMax[i] = 0U;
    }
    g_calibrationStatus.sample_count = 0U;
    g_calibrationStatus.fault_mask = 0U;
}

void ApplyProcessingDefaults(drivers::GrayscaleCalibration *calibration)
{
    calibration->threshold = kDefaultThreshold;
    calibration->hysteresis = kDefaultHysteresis;
    calibration->position_floor = kDefaultPositionFloor;
    calibration->min_line_strength = kDefaultMinimumLineStrength;
    calibration->track_mask = kDefaultTrackMask;
}

void SortCaptureSamples(uint8_t channel, uint16_t count)
{
    for (uint16_t i = 1U; i < count; i++) {
        const uint16_t value = g_calibrationSamples[channel][i];
        uint16_t insert = i;
        while ((insert > 0U) &&
               (g_calibrationSamples[channel][insert - 1U] > value)) {
            g_calibrationSamples[channel][insert] =
                g_calibrationSamples[channel][insert - 1U];
            insert--;
        }
        g_calibrationSamples[channel][insert] = value;
    }
}

void CalculateRobustCapture(
    uint16_t destination[drivers::GRAYSCALE_CHANNEL_COUNT],
    uint16_t noise[drivers::GRAYSCALE_CHANNEL_COUNT],
    uint16_t count)
{
    const uint16_t trim = (count >= 8U)
        ? static_cast<uint16_t>(count / 8U)
        : 0U;
    for (uint8_t channel = 0U;
         channel < drivers::GRAYSCALE_CHANNEL_COUNT;
         channel++) {
        SortCaptureSamples(channel, count);
        const uint16_t first = trim;
        const uint16_t last = static_cast<uint16_t>(count - trim);
        uint32_t sum = 0U;
        for (uint16_t sample = first; sample < last; sample++) {
            sum += g_calibrationSamples[channel][sample];
        }
        const uint16_t retained = static_cast<uint16_t>(last - first);
        destination[channel] = static_cast<uint16_t>(sum / retained);
        noise[channel] = static_cast<uint16_t>(
            g_calibrationSamples[channel][last - 1U] -
            g_calibrationSamples[channel][first]);
    }
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

uint8_t ValidatePointCalibration(void)
{
    uint8_t fault_mask = ValidateCalibrationSpan(g_stagedWhite,
                                                  g_stagedBlack);
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        const uint16_t span = AbsoluteDifference(g_stagedWhite[i],
                                                  g_stagedBlack[i]);
        const uint16_t noise = (g_stagedWhiteNoise[i] > g_stagedBlackNoise[i])
            ? g_stagedWhiteNoise[i]
            : g_stagedBlackNoise[i];
        if (static_cast<uint32_t>(span) <
            (static_cast<uint32_t>(noise) * kCalibrationNoiseMultiplier)) {
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
    ApplyProcessingDefaults(&calibration);
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

    const uint16_t sample_index = g_calibrationStatus.sample_count;
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        g_calibrationSamples[i][sample_index] = g_data.raw[i];
    }
    g_calibrationStatus.sample_count++;
    if (g_calibrationStatus.sample_count <
        g_calibrationStatus.target_samples) {
        return;
    }

    if (g_calibrationStatus.mode == APP_GRAYSCALE_CAL_WHITE) {
        CalculateRobustCapture(g_stagedWhite,
                               g_stagedWhiteNoise,
                               g_calibrationStatus.sample_count);
        g_calibrationStatus.white_ready = true;
    } else {
        CalculateRobustCapture(g_stagedBlack,
                               g_stagedBlackNoise,
                               g_calibrationStatus.sample_count);
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
    if (frames > kMaximumCaptureFrames) {
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
    g_scanPhase = 0U;
    g_initializedChannelMask = 0U;
    g_pendingFreshMask = 0U;
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
    (void) board::Board_GrayscalePrepareChannel(kScanSequence[0]);
}

void App_GrayscaleUpdate(void)
{
    if (!board::Board_GrayscaleIsReady()) {
        MarkFailure(drivers::DRIVER_ERROR_NOT_INITIALIZED);
        return;
    }

    uint8_t completed_channel = 0U;
    uint16_t raw = 0U;
    const drivers::DriverStatus take_status =
        board::Board_GrayscaleTakeCompletedConversion(&completed_channel,
                                                      &raw);
    bool frame_complete = false;
    if (take_status == drivers::DRIVER_OK) {
        const uint8_t expected_channel = kScanSequence[g_scanPhase];
        if (completed_channel != expected_channel) {
            MarkFailure(drivers::DRIVER_ERROR);
            (void) board::Board_GrayscalePrepareChannel(kScanSequence[0]);
            return;
        }

        g_workingRaw[completed_channel] = raw;
        g_initializedChannelMask = static_cast<uint8_t>(
            g_initializedChannelMask | (1U << completed_channel));
        g_pendingFreshMask = static_cast<uint8_t>(
            g_pendingFreshMask | (1U << completed_channel));
        g_scanPhase++;
        if (g_scanPhase >= kScanSequenceLength) {
            g_scanPhase = 0U;
        }
        frame_complete =
            (completed_channel == 6U) &&
            (g_initializedChannelMask == drivers::GRAYSCALE_ALL_CHANNEL_MASK);
    } else if (take_status != drivers::DRIVER_ERROR_BUSY) {
        MarkFailure(take_status);
        (void) board::Board_GrayscalePrepareChannel(kScanSequence[0]);
        return;
    }

    const uint8_t next_channel = kScanSequence[g_scanPhase];
    uint8_t following_phase = static_cast<uint8_t>(g_scanPhase + 1U);
    if (following_phase >= kScanSequenceLength) {
        following_phase = 0U;
    }
    const uint8_t following_channel = kScanSequence[following_phase];
    const drivers::DriverStatus start_status =
        board::Board_GrayscaleStartPreparedConversion(next_channel,
                                                      following_channel);
    if ((start_status != drivers::DRIVER_OK) &&
        (start_status != drivers::DRIVER_ERROR_BUSY)) {
        MarkFailure(start_status);
        (void) board::Board_GrayscalePrepareChannel(kScanSequence[0]);
        return;
    }

    if (!frame_complete) {
        return;
    }

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
    ProcessPublishedFrame(g_pendingFreshMask);
    g_pendingFreshMask = 0U;
}

const AppGrayscaleData *App_GrayscaleGetData(void)
{
    return &g_data;
}

void App_GrayscaleClearRoadEvent(void)
{
    GrayscaleRoad_ClearLastEvent(&g_roadState);
    g_data.road_event_sequence = 0U;
    g_data.road_event_type = GRAYSCALE_ROAD_UNKNOWN;
    g_data.road_event_paths = 0U;
    g_data.road_event_confidence = 0U;
    g_data.road_event_entry_mask = 0U;
    g_data.road_event_peak_mask = 0U;
    g_data.road_event_exit_mask = 0U;
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

    /* Migrate only the exact early commissioning tuple. This keeps deliberate
     * user tuning intact while preventing the known 200/40/20/50 settings
     * from treating normal floor variation as line evidence. */
    const bool legacy_processing_tuple =
        (calibration.threshold == 200U) &&
        (calibration.hysteresis == 40U) &&
        (calibration.position_floor == 20U) &&
        (calibration.min_line_strength == 50U) &&
        (calibration.track_mask == kDefaultTrackMask);
    if (legacy_processing_tuple) {
        ApplyProcessingDefaults(&calibration);
        (void) ConfigStore_SetGrayscaleCalibration(
            calibration.white,
            calibration.black,
            calibration.threshold,
            calibration.hysteresis,
            calibration.position_floor,
            calibration.min_line_strength,
            calibration.track_mask);
    }
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

    status = ConfigStore_SetGrayscaleCalibration(
        calibration->white,
        calibration->black,
        calibration->threshold,
        calibration->hysteresis,
        calibration->position_floor,
        calibration->min_line_strength,
        calibration->track_mask);
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
    const uint8_t fault_mask = ValidatePointCalibration();
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
    ApplyProcessingDefaults(&calibration);
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
