#include "app/app_can_bus.h"

#include "app/dm_g6220_controller.h"
#include "board/board_can.h"
#include "config/feature_config.h"
#include "services/time.h"

namespace app {
namespace {

#if FEATURE_ENABLE_CAN
static const uint8_t kFramesPerUpdate = 16U;
static const uint16_t kRawQueueSize = 32U;

drivers::CanFrame g_rawQueue[kRawQueueSize] = {};
uint16_t g_rawHead = 0U;
uint16_t g_rawTail = 0U;
uint16_t g_rawCount = 0U;
uint32_t g_rawDropped = 0U;

uint16_t NextRawIndex(uint16_t index)
{
    index++;
    return (index >= kRawQueueSize) ? 0U : index;
}

void PushRaw(const drivers::CanFrame &frame)
{
    if (g_rawCount == kRawQueueSize) {
        /* Diagnostics must show recent bus traffic. Drop the oldest cached
         * copy when full; protocol dispatch below remains independent. */
        g_rawTail = NextRawIndex(g_rawTail);
        g_rawDropped++;
    } else {
        g_rawCount++;
    }
    g_rawQueue[g_rawHead] = frame;
    g_rawHead = NextRawIndex(g_rawHead);
}
#endif

} /* namespace */

void AppCanBus_Init(void)
{
#if FEATURE_ENABLE_CAN
    g_rawHead = 0U;
    g_rawTail = 0U;
    g_rawCount = 0U;
    g_rawDropped = 0U;
#if FEATURE_ENABLE_DM_G6220_CAN
    DmG6220Controller_Init();
#endif
#endif
}

void AppCanBus_Update(void)
{
#if FEATURE_ENABLE_CAN
    drivers::CanFrame frame = {};
#if FEATURE_ENABLE_DM_G6220_CAN
    const uint32_t now_ms = services::Time_Millis();
#endif
    for (uint8_t count = 0U; count < kFramesPerUpdate; count++) {
        if (!board::Board_CanRead(&frame)) {
            break;
        }
        PushRaw(frame);
#if FEATURE_ENABLE_DM_G6220_CAN
        (void) DmG6220Controller_ProcessFrame(&frame, now_ms);
#endif
    }
#if FEATURE_ENABLE_DM_G6220_CAN
    DmG6220Controller_Update();
#endif
#endif
}

bool AppCanBus_ReadRaw(drivers::CanFrame *frame)
{
#if FEATURE_ENABLE_CAN
    if ((frame == 0) || (g_rawCount == 0U)) {
        return false;
    }
    *frame = g_rawQueue[g_rawTail];
    g_rawTail = NextRawIndex(g_rawTail);
    g_rawCount--;
    return true;
#else
    (void) frame;
    return false;
#endif
}

uint16_t AppCanBus_GetRawAvailable(void)
{
#if FEATURE_ENABLE_CAN
    return g_rawCount;
#else
    return 0U;
#endif
}

uint32_t AppCanBus_GetRawDropped(void)
{
#if FEATURE_ENABLE_CAN
    return g_rawDropped;
#else
    return 0U;
#endif
}

void AppCanBus_Clear(void)
{
#if FEATURE_ENABLE_CAN
    g_rawHead = 0U;
    g_rawTail = 0U;
    g_rawCount = 0U;
    g_rawDropped = 0U;
#endif
}

} /* namespace app */
