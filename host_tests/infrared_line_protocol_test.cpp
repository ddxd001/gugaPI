#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "drivers/infrared_line/infrared_line_protocol.h"

namespace {

void AppendCrc(uint8_t frame[drivers::INFRARED_LINE_FRAME_SIZE])
{
    const uint16_t crc = drivers::InfraredLine_ModbusCrc16(frame, 13U);
    frame[13] = static_cast<uint8_t>(crc & 0xFFU);
    frame[14] = static_cast<uint8_t>(crc >> 8U);
}

bool Feed(drivers::InfraredLineParser *parser,
          const uint8_t *data,
          uint8_t length,
          uint32_t now,
          drivers::InfraredLineFrame *out)
{
    bool received = false;
    for (uint8_t i = 0U; i < length; i++) {
        received = drivers::InfraredLineParser_FeedByte(
            parser, data[i], now, now * 1000U, out) || received;
    }
    return received;
}

} /* namespace */

int main(void)
{
    using namespace drivers;
    const uint8_t example[INFRARED_LINE_FRAME_SIZE] = {
        0x20U, 0x03U, 0x0AU, 0xE8U, 0xFFU,
        0x01U, 0x00U, 0xCFU, 0x0EU, 0xA2U,
        0x0EU, 0xADU, 0x0EU, 0x9FU, 0xC2U
    };
    assert(InfraredLine_ModbusCrc16(example, 13U) == 0xC29FU);

    InfraredLineParser parser;
    InfraredLineParser_Init(&parser);
    InfraredLineFrame frame = {};
    assert(!Feed(&parser, example, 7U, 10U, &frame));
    assert(Feed(&parser, &example[7], 8U, 11U, &frame));
    assert(frame.offset == -24);
    assert(frame.all_black == 1U);
    assert(frame.adc[0] == 3791U);
    assert(frame.adc[1] == 3746U);
    assert(frame.adc[2] == 3757U);
    assert(frame.sequence == 1U);

    uint8_t second[INFRARED_LINE_FRAME_SIZE] = {
        0x20U, 0x03U, 0x0AU, 0x2AU, 0x00U,
        0x00U, 0x00U, 0x34U, 0x03U, 0x78U,
        0x05U, 0xBCU, 0x0AU, 0U, 0U
    };
    AppendCrc(second);
    const uint8_t noise[] = {0x00U, 0x20U, 0x04U, 0x20U};
    assert(!Feed(&parser, noise, sizeof(noise), 20U, &frame));
    assert(Feed(&parser, &second[1], 14U, 21U, &frame));
    assert(frame.offset == 42);
    assert(frame.sequence == 2U);
    assert(parser.stats.average_period_ms == 10U);

    uint8_t bad[INFRARED_LINE_FRAME_SIZE];
    for (uint8_t i = 0U; i < INFRARED_LINE_FRAME_SIZE; i++) {
        bad[i] = example[i];
    }
    bad[8] ^= 0x01U;
    assert(!Feed(&parser, bad, INFRARED_LINE_FRAME_SIZE, 30U, &frame));
    assert(parser.stats.crc_errors == 1U);
    assert(Feed(&parser, second, INFRARED_LINE_FRAME_SIZE, 31U, &frame));
    assert(frame.sequence == 3U);
    assert(parser.stats.valid_frames == 3U);
    assert(parser.stats.header_errors >= 2U);

    /* A lone 0x20 in corrupted payload is not enough to retain a false
     * header. The following valid frame must still decode exactly once. */
    bad[4] = 0x20U;
    AppendCrc(bad);
    bad[14] ^= 0x80U;
    assert(!Feed(&parser, bad, INFRARED_LINE_FRAME_SIZE, 40U, &frame));
    assert(Feed(&parser, second, INFRARED_LINE_FRAME_SIZE, 41U, &frame));
    assert(frame.sequence == 4U);

    uint8_t semantic[INFRARED_LINE_FRAME_SIZE];
    for (uint8_t i = 0U; i < INFRARED_LINE_FRAME_SIZE; i++) {
        semantic[i] = second[i];
    }
    semantic[5] = 2U;
    semantic[6] = 0U;
    AppendCrc(semantic);
    assert(!Feed(&parser, semantic, INFRARED_LINE_FRAME_SIZE, 50U, &frame));
    assert(parser.stats.payload_errors == 1U);

    const uint32_t sequence_before_clear = frame.sequence;
    InfraredLineParser_ClearStats(&parser);
    assert(parser.stats.valid_frames == 0U);
    assert(Feed(&parser, second, INFRARED_LINE_FRAME_SIZE, 60U, &frame));
    assert(frame.sequence == sequence_before_clear + 1U);

    puts("infrared line protocol ok");
    return 0;
}
