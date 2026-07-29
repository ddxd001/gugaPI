#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include <vector>

#include "app/app_large_timer.h"
#include "app/app_main.h"
#include "board/board_oled.h"
#include "drivers/oled/oled_ssd1306.h"
#include "services/fault.h"
#include "services/time.h"

namespace {

uint32_t g_now_ms = 1000U;
app::AppState g_app_state = {app::APP_MODE_IDLE, 0U};
bool g_fault = false;
bool g_oled_ready = true;
uint32_t g_frame_writes = 0U;
uint32_t g_clears = 0U;
std::vector<uint8_t> g_frame;
std::vector<uint8_t> g_previous_frame;

} /* namespace */

namespace services {

uint32_t Time_Millis(void) { return g_now_ms; }
bool Fault_HasFault(void) { return g_fault; }

} /* namespace services */

namespace app {

const AppState *App_GetState(void) { return &g_app_state; }

} /* namespace app */

namespace board {

bool Board_OledIsReady(void) { return g_oled_ready; }

drivers::DriverStatus Board_OledWriteBuffer(const uint8_t *buffer,
                                            uint16_t length)
{
    assert(buffer != 0);
    assert(length == drivers::OLED_SSD1306_FRAME_BYTES);
    g_previous_frame = g_frame;
    g_frame.assign(buffer, buffer + length);
    g_frame_writes++;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_OledClear(void)
{
    g_clears++;
    return drivers::DRIVER_OK;
}

} /* namespace board */

int main(void)
{
    app::App_LargeTimerInit();
    assert(app::App_LargeTimerStart() == drivers::DRIVER_OK);
    assert(g_frame_writes == 1U);

    g_now_ms = 1099U;
    app::App_LargeTimerUpdate();
    assert(g_frame_writes == 1U);

    g_now_ms = 1100U;
    app::App_LargeTimerUpdate();
    assert(g_frame_writes == 2U);
    const app::AppLargeTimerState *state = app::App_LargeTimerGetState();
    assert(state->displayed_tenths == 1U);

    /* A normal 0.1-second update only changes the rightmost digit region. */
    uint16_t first_changed = drivers::OLED_SSD1306_FRAME_BYTES;
    uint16_t last_changed = 0U;
    for (uint16_t i = 0U; i < g_frame.size(); i++) {
        if (g_frame[i] != g_previous_frame[i]) {
            const uint16_t column = i % 128U;
            if (column < first_changed) first_changed = column;
            if (column > last_changed) last_changed = column;
        }
    }
    assert(first_changed >= 104U);
    assert(last_changed <= 127U);

    /* 200.1 seconds is rendered as the requested 3:20.1 format. */
    g_now_ms = 201100U;
    app::App_LargeTimerUpdate();
    state = app::App_LargeTimerGetState();
    assert(state->displayed_tenths == 2001U);
    assert(state->elapsed_ms == 200100U);

    assert(app::App_LargeTimerStop() == drivers::DRIVER_OK);
    const uint32_t stopped_writes = g_frame_writes;
    g_now_ms += 5000U;
    app::App_LargeTimerUpdate();
    assert(g_frame_writes == stopped_writes);
    assert(app::App_LargeTimerGetState()->elapsed_ms == 200100U);

    assert(app::App_LargeTimerResume() == drivers::DRIVER_OK);
    g_now_ms += 100U;
    app::App_LargeTimerUpdate();
    assert(app::App_LargeTimerGetState()->displayed_tenths == 2002U);

    assert(app::App_LargeTimerHide() == drivers::DRIVER_OK);
    assert(g_clears == 1U);
    assert(!app::App_LargeTimerOwnsDisplay());

    g_app_state.mode = app::APP_MODE_COMPETITION_ARMED;
    assert(app::App_LargeTimerStart() == drivers::DRIVER_OK);
    assert(app::App_LargeTimerHide() == drivers::DRIVER_OK);
    g_app_state.mode = app::APP_MODE_IDLE;

    g_fault = true;
    assert(app::App_LargeTimerStart() == drivers::DRIVER_ERROR_BUSY);
    g_fault = false;

    g_now_ms = 1000000U;
    assert(app::App_LargeTimerStart() == drivers::DRIVER_OK);
    g_now_ms += 600000U;
    app::App_LargeTimerUpdate();
    state = app::App_LargeTimerGetState();
    assert(state->displayed_tenths == 5999U);
    assert(state->saturated);
    assert(!state->running);

    puts("large OLED timer ok");
    return 0;
}
