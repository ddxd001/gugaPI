#include "app/app_grayscale.h"

#include "board/board_grayscale.h"
#include "drivers/common/driver_status.h"

namespace app {
namespace {

static const uint8_t kGrayscaleChannelCount = 8U;

AppGrayscaleData g_data = { { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U }, false };

/*
 * Round-robin sampling: read ONE channel per call instead of all eight. The
 * 8-channel ReadAll took ~18 ms on a 10 ms task (180% load) and starved every
 * other fast task (imu, chassis_fb). One channel is ~2.25 ms, well within the
 * 10 ms budget. A full frame refreshes every 8 calls (80 ms at 10 ms). valid
 * stays false until the first complete sweep so a consumer never reads a
 * half-populated frame.
 */
uint8_t g_nextChannel = 0U;
bool g_sweepComplete = false;

} /* namespace */

void App_GrayscaleInit(void)
{
    g_data.valid = false;
    g_nextChannel = 0U;
    g_sweepComplete = false;
}

void App_GrayscaleUpdate(void)
{
    if (!board::Board_GrayscaleIsReady()) {
        return;
    }

    uint16_t raw = 0U;
    const drivers::DriverStatus status =
        board::Board_GrayscaleReadChannel(g_nextChannel, &raw);
    if (status != drivers::DRIVER_OK) {
        return;
    }

    g_data.raw[g_nextChannel] = raw;
    g_nextChannel++;
    if (g_nextChannel >= kGrayscaleChannelCount) {
        g_nextChannel = 0U;
        g_sweepComplete = true;
    }
    g_data.valid = g_sweepComplete;
}

const AppGrayscaleData *App_GrayscaleGetData(void)
{
    return &g_data;
}

} /* namespace app */
