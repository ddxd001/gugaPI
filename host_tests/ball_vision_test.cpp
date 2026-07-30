#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "app/ball_vision.h"

namespace {

uint32_t g_now_ms = 0U;
uint8_t g_rx[64] = {};
uint16_t g_rx_length = 0U;
uint16_t g_rx_index = 0U;
uint32_t g_clear_count = 0U;

void Queue(const uint8_t *data, uint16_t length)
{
    assert(length <= sizeof(g_rx));
    for (uint16_t i = 0U; i < length; i++) {
        g_rx[i] = data[i];
    }
    g_rx_length = length;
    g_rx_index = 0U;
}

} /* namespace */

namespace services {

uint32_t Time_Millis(void)
{
    return g_now_ms;
}

} /* namespace services */

namespace board {

bool Board_BallVisionReadByte(uint8_t *data)
{
    if ((data == 0) || (g_rx_index >= g_rx_length)) {
        return false;
    }
    *data = g_rx[g_rx_index++];
    return true;
}

void Board_BallVisionClear(void)
{
    g_rx_index = 0U;
    g_rx_length = 0U;
    g_clear_count++;
}

bool Board_BallVisionIsReady(void)
{
    return true;
}

uint32_t Board_BallVisionGetDroppedBytes(void)
{
    return 2U;
}

uint32_t Board_BallVisionGetUartErrors(void)
{
    return 3U;
}

uint32_t Board_BallVisionGetIrqCount(void)
{
    return 4U;
}

} /* namespace board */

int main(void)
{
    using namespace app;
    const uint8_t positive[drivers::BALL_VISION_FRAME_SIZE] = {
        0xA5U, 0x5AU, 0x2AU, 0x07U, 0xF4U, 0x01U,
        0x52U, 0x03U, 0x12U, 0x00U, 0xD4U, 0x3DU
    };

    BallVision_Init();
    assert(BallVision_GetData()->state == BALL_VISION_OFFLINE);
    assert(!BallVision_IsCommunicationOnline(0U));

    Queue(positive, sizeof(positive));
    g_now_ms = 100U;
    BallVision_Update();
    const BallVisionData *data = BallVision_GetData();
    assert(data->state == BALL_VISION_FRESH);
    assert(data->frame.position_0p1mm == 500);
    assert(data->ball_frame.position_0p1mm == 500);
    assert(data->uart_dropped_bytes == 2U);
    assert(data->uart_errors == 3U);
    assert(data->uart_irq_count == 4U);
    assert(BallVision_IsCommunicationOnline(g_now_ms));
    assert(BallVision_IsUsable(g_now_ms));

    g_now_ms = 161U;
    BallVision_Update();
    assert(data->state == BALL_VISION_DEGRADED);
    assert(BallVision_IsUsable(g_now_ms));

    g_now_ms = 201U;
    BallVision_Update();
    assert(data->state == BALL_VISION_OFFLINE);
    assert(!BallVision_IsCommunicationOnline(g_now_ms));
    assert(BallVision_IsUsable(g_now_ms));

    g_now_ms = 221U;
    BallVision_Update();
    assert(data->state == BALL_VISION_OFFLINE);
    assert(!BallVision_IsUsable(g_now_ms));

    assert(BallVision_Inject(-1250, 1000U, 300U) ==
           drivers::DRIVER_OK);
    assert(data->state == BALL_VISION_FRESH);
    assert(data->injected);
    assert(data->frame.position_0p1mm == -1250);
    assert(BallVision_Inject(1251, 1000U, 300U) ==
           drivers::DRIVER_ERROR_INVALID_ARG);

    BallVision_Clear();
    assert(g_clear_count == 1U);
    assert(data->state == BALL_VISION_OFFLINE);
    assert(data->parser_stats.valid_frames == 0U);

    puts("ball vision app ok");
    return 0;
}
