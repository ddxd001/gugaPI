#include <assert.h>
#include <stdint.h>

#define CONFIG_FEATURE_CONFIG_H_
#define FEATURE_ENABLE_CAN 1
#define FEATURE_ENABLE_DM_G6220_CAN 0
#include "../gugaPI/app/app_can_bus.cpp"

namespace {

drivers::CanFrame g_frames[64] = {};
uint16_t g_frameCount = 0U;
uint16_t g_frameRead = 0U;

void QueueFrames(uint16_t first_id, uint16_t count)
{
    assert(count <= 64U);
    g_frameCount = count;
    g_frameRead = 0U;
    for (uint16_t i = 0U; i < count; i++) {
        g_frames[i] = {};
        g_frames[i].id = static_cast<uint32_t>(first_id + i);
        g_frames[i].length = 1U;
        g_frames[i].data[0] = static_cast<uint8_t>(i);
    }
}

void DrainHardware(void)
{
    while (g_frameRead < g_frameCount) {
        app::AppCanBus_Update();
    }
}

} /* namespace */

namespace board {

bool Board_CanRead(drivers::CanFrame *frame)
{
    if ((frame == 0) || (g_frameRead >= g_frameCount)) {
        return false;
    }
    *frame = g_frames[g_frameRead++];
    return true;
}

} /* namespace board */

namespace services {

uint32_t Time_Millis(void)
{
    return 0U;
}

} /* namespace services */

int main()
{
    app::AppCanBus_Init();

    /* A full cache uses all 32 entries. New frames evict the oldest cached
     * copies so diagnostics always show the most recent bus traffic. */
    QueueFrames(0U, 40U);
    DrainHardware();
    assert(app::AppCanBus_GetRawAvailable() == 32U);
    assert(app::AppCanBus_GetRawDropped() == 8U);

    drivers::CanFrame frame = {};
    for (uint16_t expected = 8U; expected < 40U; expected++) {
        assert(app::AppCanBus_ReadRaw(&frame));
        assert(frame.id == expected);
    }
    assert(!app::AppCanBus_ReadRaw(&frame));

    app::AppCanBus_Clear();
    QueueFrames(100U, 32U);
    DrainHardware();
    assert(app::AppCanBus_GetRawAvailable() == 32U);
    assert(app::AppCanBus_GetRawDropped() == 0U);
    for (uint16_t expected = 100U; expected < 132U; expected++) {
        assert(app::AppCanBus_ReadRaw(&frame));
        assert(frame.id == expected);
    }

    return 0;
}
