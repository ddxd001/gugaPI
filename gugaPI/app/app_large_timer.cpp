#include "app/app_large_timer.h"

#include <limits.h>

#include "board/board_oled.h"
#include "drivers/oled/oled_ssd1306.h"
#include "services/fault.h"
#include "services/time.h"

namespace app {
namespace {

static const uint16_t kInvalidTenths = UINT16_MAX;
static const uint32_t kMaximumElapsedMs = 599900U; /* 9:59.9 */
static const uint8_t kWidth = 128U;
static const uint8_t kHeight = 32U;

static const uint8_t kSegments[10] = {
    0x3FU, 0x06U, 0x5BU, 0x4FU, 0x66U,
    0x6DU, 0x7DU, 0x07U, 0x7FU, 0x6FU
};

AppLargeTimerState g_state = {};
uint32_t g_started_ms = 0U;
uint32_t g_accumulated_ms = 0U;
uint16_t g_last_rendered_tenths = kInvalidTenths;
bool g_display_preempted = false;
uint8_t g_frame[drivers::OLED_SSD1306_FRAME_BYTES] = {};

bool DisplayPreempted(void)
{
    return services::Fault_HasFault();
}

uint32_t CurrentElapsedMs(uint32_t now_ms)
{
    uint32_t elapsed = g_accumulated_ms;
    if (g_state.running) {
        const uint32_t delta = now_ms - g_started_ms;
        if (delta >= (kMaximumElapsedMs - elapsed)) {
            return kMaximumElapsedMs;
        }
        elapsed += delta;
    }
    return (elapsed > kMaximumElapsedMs) ? kMaximumElapsedMs : elapsed;
}

void ClearFrame(void)
{
    for (uint16_t i = 0U; i < drivers::OLED_SSD1306_FRAME_BYTES; i++) {
        g_frame[i] = 0U;
    }
}

void SetPixel(uint8_t x, uint8_t y)
{
    if ((x >= kWidth) || (y >= kHeight)) {
        return;
    }
    const uint16_t index = static_cast<uint16_t>(
        (y / 8U) * kWidth + x);
    g_frame[index] = static_cast<uint8_t>(
        g_frame[index] | (1U << (y & 7U)));
}

void FillRect(uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
    for (uint8_t row = 0U; row < height; row++) {
        for (uint8_t column = 0U; column < width; column++) {
            SetPixel(static_cast<uint8_t>(x + column),
                     static_cast<uint8_t>(y + row));
        }
    }
}

void DrawDigit(uint8_t x, uint8_t digit)
{
    const uint8_t segments = kSegments[digit % 10U];
    if ((segments & 0x01U) != 0U) FillRect(x + 4U, 0U, 16U, 4U);
    if ((segments & 0x02U) != 0U) FillRect(x + 20U, 4U, 4U, 11U);
    if ((segments & 0x04U) != 0U) FillRect(x + 20U, 17U, 4U, 11U);
    if ((segments & 0x08U) != 0U) FillRect(x + 4U, 28U, 16U, 4U);
    if ((segments & 0x10U) != 0U) FillRect(x, 17U, 4U, 11U);
    if ((segments & 0x20U) != 0U) FillRect(x, 4U, 4U, 11U);
    if ((segments & 0x40U) != 0U) FillRect(x + 4U, 14U, 16U, 4U);
}

drivers::DriverStatus Render(uint16_t tenths)
{
    const uint8_t minutes = static_cast<uint8_t>(tenths / 600U);
    const uint8_t seconds = static_cast<uint8_t>((tenths / 10U) % 60U);
    const uint8_t decimal = static_cast<uint8_t>(tenths % 10U);

    ClearFrame();
    DrawDigit(0U, minutes);
    FillRect(29U, 8U, 6U, 6U);
    FillRect(29U, 20U, 6U, 6U);
    DrawDigit(39U, static_cast<uint8_t>(seconds / 10U));
    DrawDigit(67U, static_cast<uint8_t>(seconds % 10U));
    FillRect(95U, 27U, 5U, 5U);
    DrawDigit(104U, decimal);
    return board::Board_OledWriteBuffer(g_frame, sizeof(g_frame));
}

} /* namespace */

void App_LargeTimerInit(void)
{
    g_state = {};
    g_state.displayed_tenths = 0U;
    g_state.last_display_status = drivers::DRIVER_OK;
    g_started_ms = 0U;
    g_accumulated_ms = 0U;
    g_last_rendered_tenths = kInvalidTenths;
    g_display_preempted = false;
    ClearFrame();
}

drivers::DriverStatus App_LargeTimerStart(void)
{
    if (DisplayPreempted()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (!board::Board_OledIsReady()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    g_state.visible = true;
    g_state.running = true;
    g_state.saturated = false;
    g_started_ms = services::Time_Millis();
    g_accumulated_ms = 0U;
    g_last_rendered_tenths = kInvalidTenths;
    g_display_preempted = false;
    App_LargeTimerUpdate();
    return g_state.last_display_status;
}

drivers::DriverStatus App_LargeTimerStop(void)
{
    if (!g_state.visible) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (g_state.running) {
        g_accumulated_ms = CurrentElapsedMs(services::Time_Millis());
        g_state.running = false;
    }
    g_last_rendered_tenths = kInvalidTenths;
    App_LargeTimerUpdate();
    return g_state.last_display_status;
}

drivers::DriverStatus App_LargeTimerResume(void)
{
    if ((!g_state.visible) || g_state.saturated) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (DisplayPreempted()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (!g_state.running) {
        g_started_ms = services::Time_Millis();
        g_state.running = true;
    }
    return drivers::DRIVER_OK;
}

drivers::DriverStatus App_LargeTimerReset(void)
{
    g_accumulated_ms = 0U;
    g_state.saturated = false;
    if (g_state.running) {
        g_started_ms = services::Time_Millis();
    }
    g_last_rendered_tenths = kInvalidTenths;
    if (g_state.visible) {
        App_LargeTimerUpdate();
        return g_state.last_display_status;
    }
    return drivers::DRIVER_OK;
}

drivers::DriverStatus App_LargeTimerHide(void)
{
    if (g_state.running) {
        g_accumulated_ms = CurrentElapsedMs(services::Time_Millis());
    }
    g_state.running = false;
    g_state.visible = false;
    g_display_preempted = false;
    g_last_rendered_tenths = kInvalidTenths;
    if (!board::Board_OledIsReady()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    g_state.last_display_status = board::Board_OledClear();
    return g_state.last_display_status;
}

void App_LargeTimerUpdate(void)
{
    if (!g_state.visible) {
        return;
    }
    if (DisplayPreempted()) {
        g_display_preempted = true;
        return;
    }
    if (!board::Board_OledIsReady()) {
        g_state.last_display_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return;
    }
    if (g_display_preempted) {
        g_display_preempted = false;
        g_last_rendered_tenths = kInvalidTenths;
    }

    const uint32_t now = services::Time_Millis();
    const uint32_t elapsed = CurrentElapsedMs(now);
    if (elapsed >= kMaximumElapsedMs) {
        g_accumulated_ms = kMaximumElapsedMs;
        g_state.running = false;
        g_state.saturated = true;
    }
    g_state.elapsed_ms = elapsed;
    const uint16_t tenths = static_cast<uint16_t>(elapsed / 100U);
    g_state.displayed_tenths = tenths;
    if (tenths == g_last_rendered_tenths) {
        return;
    }

    g_state.last_display_status = Render(tenths);
    if (g_state.last_display_status == drivers::DRIVER_OK) {
        g_last_rendered_tenths = tenths;
        g_state.display_updates++;
    }
}

bool App_LargeTimerOwnsDisplay(void)
{
    return g_state.visible;
}

const AppLargeTimerState *App_LargeTimerGetState(void)
{
    g_state.elapsed_ms = CurrentElapsedMs(services::Time_Millis());
    return &g_state;
}

} /* namespace app */
