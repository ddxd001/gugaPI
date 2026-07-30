#include "app/ball_vision.h"

#include "board/board_ball_vision.h"
#include "services/time.h"

namespace app {
namespace {

static const uint32_t kCommunicationTimeoutMs = 100U;
static const uint32_t kFreshBallTimeoutMs = 60U;
static const uint32_t kBallFailureTimeoutMs = 120U;

drivers::BallVisionParser g_parser;
BallVisionData g_data = {};

bool FrameReportsUsableBall(const drivers::BallVisionFrame &frame)
{
    const uint8_t required =
        drivers::BALL_VISION_FLAG_BALL_FOUND |
        drivers::BALL_VISION_FLAG_CALIBRATION_VALID |
        drivers::BALL_VISION_FLAG_CAMERA_OK;
    return ((frame.flags & required) == required) &&
           (frame.confidence != 0U);
}

void RefreshState(uint32_t now_ms)
{
    if (g_parser.stats.valid_frames == 0U) {
        g_data.frame_age_ms = UINT32_MAX;
        g_data.ball_age_ms = UINT32_MAX;
        g_data.state = BALL_VISION_OFFLINE;
        g_data.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return;
    }

    const uint32_t age = now_ms - g_data.frame.received_ms;
    const bool current_ball = FrameReportsUsableBall(g_data.frame);
    g_data.frame_age_ms = age;
    g_data.ball_age_ms = g_data.has_ball_sample
        ? static_cast<uint32_t>(now_ms - g_data.ball_frame.received_ms)
        : UINT32_MAX;
    if (age > kCommunicationTimeoutMs) {
        g_data.state = BALL_VISION_OFFLINE;
        g_data.last_status = drivers::DRIVER_ERROR_TIMEOUT;
    } else if (!current_ball) {
        if (!g_data.has_ball_sample) {
            g_data.state = BALL_VISION_NO_BALL;
            g_data.last_status = drivers::DRIVER_ERROR;
        } else if (g_data.ball_age_ms <= kBallFailureTimeoutMs) {
            g_data.state = BALL_VISION_DEGRADED;
            g_data.last_status = drivers::DRIVER_ERROR_BUSY;
        } else {
            g_data.state = BALL_VISION_STALE;
            g_data.last_status = drivers::DRIVER_ERROR_TIMEOUT;
        }
    } else if (g_data.ball_age_ms <= kFreshBallTimeoutMs) {
        g_data.state = BALL_VISION_FRESH;
        g_data.last_status = drivers::DRIVER_OK;
    } else if (g_data.ball_age_ms <= kBallFailureTimeoutMs) {
        g_data.state = BALL_VISION_DEGRADED;
        g_data.last_status = drivers::DRIVER_ERROR_BUSY;
    }
}

} /* namespace */

void BallVision_Init(void)
{
    drivers::BallVisionParser_Init(&g_parser);
    g_data = {};
    g_data.state = BALL_VISION_OFFLINE;
    g_data.frame_age_ms = UINT32_MAX;
    g_data.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
}

void BallVision_Update(void)
{
    const uint32_t now_ms = services::Time_Millis();
    uint8_t byte = 0U;
    drivers::BallVisionFrame frame = {};
    while (board::Board_BallVisionReadByte(&byte)) {
        if (drivers::BallVisionParser_FeedByte(
                &g_parser, byte, now_ms, &frame)) {
            g_data.frame = frame;
            if (FrameReportsUsableBall(frame)) {
                g_data.ball_frame = frame;
                g_data.has_ball_sample = true;
            }
            g_data.injected = false;
        }
    }
    g_data.parser_stats = g_parser.stats;
    g_data.uart_dropped_bytes = board::Board_BallVisionGetDroppedBytes();
    g_data.uart_errors = board::Board_BallVisionGetUartErrors();
    g_data.uart_irq_count = board::Board_BallVisionGetIrqCount();
    RefreshState(now_ms);
}

const BallVisionData *BallVision_GetData(void)
{
    return &g_data;
}

bool BallVision_IsCommunicationOnline(uint32_t now_ms)
{
    return (g_parser.stats.valid_frames != 0U) &&
           ((now_ms - g_data.frame.received_ms) <=
            kCommunicationTimeoutMs);
}

bool BallVision_IsUsable(uint32_t now_ms)
{
    return g_data.has_ball_sample &&
           ((now_ms - g_data.ball_frame.received_ms) <=
            kBallFailureTimeoutMs);
}

drivers::DriverStatus BallVision_Inject(int16_t position_0p1mm,
                                        uint16_t confidence,
                                        uint32_t now_ms)
{
    if ((position_0p1mm < -1250) || (position_0p1mm > 1250) ||
        (confidence > 1000U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    g_data.frame.sequence++;
    g_data.frame.flags =
        drivers::BALL_VISION_FLAG_BALL_FOUND |
        drivers::BALL_VISION_FLAG_CALIBRATION_VALID |
        drivers::BALL_VISION_FLAG_CAMERA_OK;
    g_data.frame.position_0p1mm = position_0p1mm;
    g_data.frame.confidence = confidence;
    g_data.frame.source_delay_ms = 0U;
    g_data.frame.received_ms = now_ms;
    g_data.ball_frame = g_data.frame;
    g_data.has_ball_sample = true;
    if (g_parser.stats.valid_frames != UINT32_MAX) {
        g_parser.stats.valid_frames++;
    }
    g_data.injected = true;
    g_data.parser_stats = g_parser.stats;
    RefreshState(now_ms);
    return drivers::DRIVER_OK;
}

void BallVision_Clear(void)
{
    board::Board_BallVisionClear();
    drivers::BallVisionParser_Init(&g_parser);
    g_data = {};
    g_data.state = BALL_VISION_OFFLINE;
    g_data.frame_age_ms = UINT32_MAX;
    g_data.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
}

const char *BallVision_StateText(BallVisionState state)
{
    switch (state) {
    case BALL_VISION_OFFLINE:
        return "offline";
    case BALL_VISION_NO_BALL:
        return "no-ball";
    case BALL_VISION_FRESH:
        return "fresh";
    case BALL_VISION_DEGRADED:
        return "degraded";
    case BALL_VISION_STALE:
        return "stale";
    default:
        return "unknown";
    }
}

} /* namespace app */
