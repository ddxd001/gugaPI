/* Host-only regression test for the non-blocking ADC8 calibration session. */
#include <assert.h>
#include <stdint.h>

#include "app/app_grayscale.h"
#include "app/config_store.h"
#include "board/board_grayscale.h"
#include "services/time.h"

namespace {

const uint8_t kScanSequence[] = {
    0U, 1U, 2U, 3U, 4U, 5U, 6U,
    7U, 1U, 2U, 3U, 4U, 5U, 6U
};

uint32_t g_now_ms = 0U;
bool g_board_ready = true;
bool g_conversion_pending = false;
uint8_t g_pending_channel = 0U;
uint16_t g_pending_raw = 0U;
uint8_t g_host_scan_phase = 0U;
uint16_t g_frame[drivers::GRAYSCALE_CHANNEL_COUNT] = {};
app::ConfigStoreParams g_params = {};
app::ConfigStoreStatus g_store_status = {};
uint32_t g_calibration_set_count = 0U;

void FillFrame(uint16_t base, uint16_t jitter)
{
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        g_frame[i] = static_cast<uint16_t>(base + i + jitter);
    }
}

void QueueNextConversion(void)
{
    const uint8_t channel = kScanSequence[g_host_scan_phase];
    g_pending_channel = channel;
    g_pending_raw = g_frame[channel];
    g_conversion_pending = true;
    g_host_scan_phase = static_cast<uint8_t>(g_host_scan_phase + 1U);
    if (g_host_scan_phase >= sizeof(kScanSequence)) {
        g_host_scan_phase = 0U;
    }
}

void FeedPublishedFrame(void)
{
    const uint32_t old_sequence = app::App_GrayscaleGetData()->sequence;
    uint8_t guard = 0U;
    while (app::App_GrayscaleGetData()->sequence == old_sequence) {
        QueueNextConversion();
        g_now_ms++;
        app::App_GrayscaleUpdate();
        guard++;
        assert(guard <= 14U);
    }
}

void InitializeHarness(void)
{
    g_now_ms = 0U;
    g_board_ready = true;
    g_conversion_pending = false;
    g_host_scan_phase = 0U;
    g_calibration_set_count = 0U;
    g_params = {};
    g_store_status = {};
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        g_params.grayscale_white[i] = 3800U;
        g_params.grayscale_black[i] = 400U;
    }
    g_params.grayscale_threshold = 500U;
    g_params.grayscale_hysteresis = 300U;
    g_params.grayscale_position_floor = 100U;
    g_params.grayscale_min_line_strength = 600U;
    g_params.grayscale_track_mask = 0x7EU;
    app::App_GrayscaleInit();
}

void CapturePoint(bool white, uint16_t base, uint16_t frames)
{
    const drivers::DriverStatus status = white
        ? app::App_GrayscaleStartWhiteCalibration(frames)
        : app::App_GrayscaleStartBlackCalibration(frames);
    assert(status == drivers::DRIVER_OK);
    for (uint16_t sample = 0U; sample < frames; sample++) {
        FillFrame(base, static_cast<uint16_t>(sample % 5U));
        FeedPublishedFrame();
    }
    const app::AppGrayscaleCalibrationStatus *capture =
        app::App_GrayscaleGetCalibrationStatus();
    assert(!capture->running);
    assert(capture->last_status == drivers::DRIVER_OK);
}

} /* namespace */

namespace services {

uint32_t Time_Millis(void)
{
    return g_now_ms;
}

} /* namespace services */

namespace board {

bool Board_GrayscaleIsReady(void)
{
    return g_board_ready;
}

drivers::DriverStatus Board_GrayscalePrepareChannel(uint8_t)
{
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_GrayscaleStartPreparedConversion(uint8_t, uint8_t)
{
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_GrayscaleTakeCompletedConversion(uint8_t *channel,
                                                             uint16_t *raw)
{
    if (!g_conversion_pending) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    *channel = g_pending_channel;
    *raw = g_pending_raw;
    g_conversion_pending = false;
    return drivers::DRIVER_OK;
}

} /* namespace board */

namespace app {

const ConfigStoreParams *ConfigStore_Get(void)
{
    return &g_params;
}

const ConfigStoreStatus *ConfigStore_GetStatus(void)
{
    return &g_store_status;
}

drivers::DriverStatus ConfigStore_SetGrayscaleCalibration(
    const uint16_t white[CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT],
    const uint16_t black[CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT],
    uint16_t threshold,
    uint16_t hysteresis,
    uint16_t position_floor,
    uint16_t min_line_strength,
    uint8_t track_mask)
{
    for (uint8_t i = 0U; i < CONFIG_STORE_GRAYSCALE_CHANNEL_COUNT; i++) {
        g_params.grayscale_white[i] = white[i];
        g_params.grayscale_black[i] = black[i];
    }
    g_params.grayscale_threshold = threshold;
    g_params.grayscale_hysteresis = hysteresis;
    g_params.grayscale_position_floor = position_floor;
    g_params.grayscale_min_line_strength = min_line_strength;
    g_params.grayscale_track_mask = track_mask;
    g_store_status.dirty = true;
    g_calibration_set_count++;
    return drivers::DRIVER_OK;
}

} /* namespace app */

int main()
{
    InitializeHarness();
    assert(app::App_GrayscaleBeginCalibration() ==
           drivers::DRIVER_ERROR_NOT_INITIALIZED);

    FillFrame(1000U, 0U);
    FeedPublishedFrame();
    assert(app::App_GrayscaleBeginCalibration() == drivers::DRIVER_OK);

    /* Explicit white/black capture accepts reverse ADC polarity. */
    CapturePoint(true, 700U, 64U);
    CapturePoint(false, 3300U, 64U);
    const app::AppGrayscaleCalibrationPreview *preview =
        app::App_GrayscaleGetCalibrationPreview();
    assert(preview->fault_mask == 0U);
    for (uint8_t i = 0U; i < drivers::GRAYSCALE_CHANNEL_COUNT; i++) {
        assert(preview->span[i] >= 2500U);
        assert(preview->white_noise[i] <= 4U);
        assert(preview->black_noise[i] <= 4U);
    }
    assert(app::App_GrayscaleCommitCalibration() == drivers::DRIVER_OK);
    assert(g_calibration_set_count == 1U);
    assert(g_store_status.dirty);
    assert(!app::App_GrayscaleGetCalibrationStatus()->session_active);

    /* One low-span channel is reported and cannot be committed. */
    assert(app::App_GrayscaleBeginCalibration() == drivers::DRIVER_OK);
    CapturePoint(true, 800U, 8U);
    assert(app::App_GrayscaleStartBlackCalibration(8U) == drivers::DRIVER_OK);
    for (uint16_t sample = 0U; sample < 8U; sample++) {
        FillFrame(3300U, static_cast<uint16_t>(sample % 3U));
        g_frame[3] = static_cast<uint16_t>(900U + (sample % 3U));
        FeedPublishedFrame();
    }
    preview = app::App_GrayscaleGetCalibrationPreview();
    assert((preview->fault_mask & (1U << 3U)) != 0U);
    assert(app::App_GrayscaleCommitCalibration() ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(g_calibration_set_count == 1U);
    app::App_GrayscaleCancelCalibration();
    assert(!app::App_GrayscaleGetCalibrationStatus()->session_active);

    /* Losing complete ADC frames releases running/busy after 500 ms. */
    FillFrame(1000U, 0U);
    FeedPublishedFrame();
    assert(app::App_GrayscaleBeginCalibration() == drivers::DRIVER_OK);
    assert(app::App_GrayscaleStartWhiteCalibration(64U) == drivers::DRIVER_OK);
    g_now_ms += 501U;
    app::App_GrayscaleUpdate();
    const app::AppGrayscaleCalibrationStatus *timed_out =
        app::App_GrayscaleGetCalibrationStatus();
    assert(!timed_out->running);
    assert(timed_out->last_status == drivers::DRIVER_ERROR_TIMEOUT);
    assert(timed_out->fault_mask == 0xFFU);

    return 0;
}
