#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "drivers/ball_vision/ball_vision_protocol.h"

namespace {

bool Feed(drivers::BallVisionParser *parser,
          const uint8_t *data,
          uint8_t length,
          uint32_t now_ms,
          drivers::BallVisionFrame *frame)
{
    bool received = false;
    for (uint8_t i = 0U; i < length; i++) {
        received = drivers::BallVisionParser_FeedByte(
            parser, data[i], now_ms, frame) || received;
    }
    return received;
}

void AppendCrc(uint8_t frame[drivers::BALL_VISION_FRAME_SIZE])
{
    const uint16_t crc = drivers::BallVision_ModbusCrc16(frame, 10U);
    frame[10] = static_cast<uint8_t>(crc & 0xFFU);
    frame[11] = static_cast<uint8_t>(crc >> 8U);
}

} /* namespace */

int main(void)
{
    using namespace drivers;

    const uint8_t center[BALL_VISION_FRAME_SIZE] = {
        0xA5U, 0x5AU, 0x01U, 0x07U, 0x00U, 0x00U,
        0xE8U, 0x03U, 0x14U, 0x00U, 0x99U, 0x9AU
    };
    const uint8_t positive[BALL_VISION_FRAME_SIZE] = {
        0xA5U, 0x5AU, 0x2AU, 0x07U, 0xF4U, 0x01U,
        0x52U, 0x03U, 0x12U, 0x00U, 0xD4U, 0x3DU
    };
    const uint8_t negative[BALL_VISION_FRAME_SIZE] = {
        0xA5U, 0x5AU, 0x2BU, 0x07U, 0x0CU, 0xFEU,
        0x84U, 0x03U, 0x16U, 0x00U, 0x2EU, 0xD5U
    };
    const uint8_t no_ball[BALL_VISION_FRAME_SIZE] = {
        0xA5U, 0x5AU, 0x2CU, 0x06U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x14U, 0x00U, 0x8FU, 0x7BU
    };

    assert(BallVision_ModbusCrc16(center, 10U) == 0x9A99U);

    BallVisionParser parser;
    BallVisionParser_Init(&parser);
    BallVisionFrame frame = {};
    assert(!Feed(&parser, center, 5U, 10U, &frame));
    assert(Feed(&parser, &center[5], 7U, 11U, &frame));
    assert(frame.sequence == 1U);
    assert(frame.flags == 7U);
    assert(frame.position_0p1mm == 0);
    assert(frame.confidence == 1000U);
    assert(frame.source_delay_ms == 20U);
    assert(frame.received_ms == 11U);

    assert(Feed(&parser, positive, BALL_VISION_FRAME_SIZE, 20U, &frame));
    assert(frame.position_0p1mm == 500);
    assert(frame.confidence == 850U);
    assert(parser.stats.sequence_gaps == 40U);
    assert(Feed(&parser, positive, BALL_VISION_FRAME_SIZE, 21U, &frame));
    assert(parser.stats.duplicate_frames == 1U);
    assert(Feed(&parser, negative, BALL_VISION_FRAME_SIZE, 22U, &frame));
    assert(frame.position_0p1mm == -500);

    assert(Feed(&parser, no_ball, BALL_VISION_FRAME_SIZE, 23U, &frame));
    assert((frame.flags & BALL_VISION_FLAG_BALL_FOUND) == 0U);
    assert(frame.confidence == 0U);

    uint8_t bad[BALL_VISION_FRAME_SIZE];
    for (uint8_t i = 0U; i < BALL_VISION_FRAME_SIZE; i++) {
        bad[i] = center[i];
    }
    bad[6] ^= 0x01U;
    assert(!Feed(&parser, bad, BALL_VISION_FRAME_SIZE, 30U, &frame));
    assert(parser.stats.crc_errors == 1U);
    assert(Feed(&parser, positive, BALL_VISION_FRAME_SIZE, 31U, &frame));

    uint8_t invalid[BALL_VISION_FRAME_SIZE] = {
        0xA5U, 0x5AU, 0x30U, 0x87U, 0x00U, 0x00U,
        0xE8U, 0x03U, 0x14U, 0x00U, 0x00U, 0x00U
    };
    AppendCrc(invalid);
    assert(!Feed(&parser, invalid, BALL_VISION_FRAME_SIZE, 40U, &frame));
    assert(parser.stats.payload_errors == 1U);

    const uint8_t noise[] = {0x00U, 0xA5U, 0x00U, 0xA5U};
    assert(!Feed(&parser, noise, sizeof(noise), 50U, &frame));
    assert(Feed(&parser, &center[1], 11U, 51U, &frame));
    assert(parser.stats.valid_frames == 7U);

    BallVisionParser_ClearStats(&parser);
    assert(parser.stats.valid_frames == 0U);
    assert(!parser.has_previous_sequence);

    puts("ball vision protocol ok");
    return 0;
}
