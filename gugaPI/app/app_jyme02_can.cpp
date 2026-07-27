#include "app/app_jyme02_can.h"

#include "board/board_can.h"
#include "config/feature_config.h"
#include "services/time.h"

namespace app {
namespace {

#if FEATURE_ENABLE_CAN && FEATURE_ENABLE_JYME02_CAN
const uint16_t kDefaultAddress = 0x50U;
const uint16_t kDefaultSampleTime100us = 1000U;
const uint32_t kStaleTimeoutMs = 500U;
const uint8_t kFramesPerUpdate = 8U;
const uint16_t kRawQueueSize = 32U;

drivers::JYME02CanContext g_context = {};
drivers::CanFrame g_rawQueue[kRawQueueSize] = {};
uint16_t g_rawHead = 0U;
uint16_t g_rawTail = 0U;
uint32_t g_rawDropped = 0U;

uint16_t NextRawIndex(uint16_t index)
{
    index++;
    return (index >= kRawQueueSize) ? 0U : index;
}

void PushRaw(const drivers::CanFrame &frame)
{
    const uint16_t next = NextRawIndex(g_rawHead);
    if (next == g_rawTail) {
        g_rawDropped++;
        return;
    }
    g_rawQueue[g_rawHead] = frame;
    g_rawHead = next;
}
#endif

} /* namespace */

void AppJyme02Can_Init(void)
{
#if FEATURE_ENABLE_CAN && FEATURE_ENABLE_JYME02_CAN
    const drivers::JYME02CanConfig config = {
        kDefaultAddress,
        kDefaultSampleTime100us,
        kStaleTimeoutMs
    };
    (void) drivers::JYME02Can_Init(&g_context, &config);
    g_rawHead = 0U;
    g_rawTail = 0U;
    g_rawDropped = 0U;
#endif
}

void AppJyme02Can_Update(void)
{
#if FEATURE_ENABLE_CAN && FEATURE_ENABLE_JYME02_CAN
    drivers::CanFrame frame = {};
    const uint32_t now_ms = services::Time_Millis();
    for (uint8_t count = 0U; count < kFramesPerUpdate; count++) {
        if (!board::Board_CanRead(&frame)) {
            break;
        }
        PushRaw(frame);
        (void) drivers::JYME02Can_ProcessFrame(
            &g_context, &frame, now_ms);
    }
#endif
}

const drivers::JYME02CanData *AppJyme02Can_GetData(void)
{
#if FEATURE_ENABLE_CAN && FEATURE_ENABLE_JYME02_CAN
    return drivers::JYME02Can_GetData(&g_context);
#else
    return 0;
#endif
}

bool AppJyme02Can_IsMeasurementFresh(uint32_t now_ms)
{
#if FEATURE_ENABLE_CAN && FEATURE_ENABLE_JYME02_CAN
    return drivers::JYME02Can_IsMeasurementFresh(&g_context, now_ms);
#else
    (void) now_ms;
    return false;
#endif
}

bool AppJyme02Can_IsTemperatureFresh(uint32_t now_ms)
{
#if FEATURE_ENABLE_CAN && FEATURE_ENABLE_JYME02_CAN
    return drivers::JYME02Can_IsTemperatureFresh(&g_context, now_ms);
#else
    (void) now_ms;
    return false;
#endif
}

drivers::DriverStatus AppJyme02Can_SetAddress(uint16_t address)
{
#if FEATURE_ENABLE_CAN && FEATURE_ENABLE_JYME02_CAN
    return drivers::JYME02Can_SetAddress(&g_context, address);
#else
    (void) address;
    return drivers::DRIVER_ERROR_UNSUPPORTED;
#endif
}

drivers::DriverStatus AppJyme02Can_SetSampleTime(uint16_t sample_time_100us)
{
#if FEATURE_ENABLE_CAN && FEATURE_ENABLE_JYME02_CAN
    return drivers::JYME02Can_SetSampleTime(
        &g_context, sample_time_100us);
#else
    (void) sample_time_100us;
    return drivers::DRIVER_ERROR_UNSUPPORTED;
#endif
}

drivers::DriverStatus AppJyme02Can_ReadRegister(uint8_t register_address)
{
#if FEATURE_ENABLE_CAN && FEATURE_ENABLE_JYME02_CAN
    drivers::CanFrame frame = {};
    const drivers::DriverStatus prepare =
        drivers::JYME02Can_PrepareReadRegister(
            &g_context, register_address, &frame);
    if (prepare != drivers::DRIVER_OK) {
        return prepare;
    }
    return board::Board_CanSend(&frame);
#else
    (void) register_address;
    return drivers::DRIVER_ERROR_UNSUPPORTED;
#endif
}

bool AppJyme02Can_ReadRaw(drivers::CanFrame *frame)
{
#if FEATURE_ENABLE_CAN && FEATURE_ENABLE_JYME02_CAN
    if ((frame == 0) || (g_rawHead == g_rawTail)) {
        return false;
    }
    *frame = g_rawQueue[g_rawTail];
    g_rawTail = NextRawIndex(g_rawTail);
    return true;
#else
    (void) frame;
    return false;
#endif
}

uint16_t AppJyme02Can_GetRawAvailable(void)
{
#if FEATURE_ENABLE_CAN && FEATURE_ENABLE_JYME02_CAN
    if (g_rawHead >= g_rawTail) {
        return static_cast<uint16_t>(g_rawHead - g_rawTail);
    }
    return static_cast<uint16_t>(kRawQueueSize - g_rawTail + g_rawHead);
#else
    return 0U;
#endif
}

uint32_t AppJyme02Can_GetRawDropped(void)
{
#if FEATURE_ENABLE_CAN && FEATURE_ENABLE_JYME02_CAN
    return g_rawDropped;
#else
    return 0U;
#endif
}

void AppJyme02Can_Clear(void)
{
#if FEATURE_ENABLE_CAN && FEATURE_ENABLE_JYME02_CAN
    drivers::JYME02Can_ClearStatistics(&g_context);
    g_rawHead = 0U;
    g_rawTail = 0U;
    g_rawDropped = 0U;
#endif
}

} /* namespace app */
