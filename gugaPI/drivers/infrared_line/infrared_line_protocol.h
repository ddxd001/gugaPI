#ifndef DRIVERS_INFRARED_LINE_INFRARED_LINE_PROTOCOL_H_
#define DRIVERS_INFRARED_LINE_INFRARED_LINE_PROTOCOL_H_

#include <stdbool.h>
#include <stdint.h>

namespace drivers {

static const uint8_t INFRARED_LINE_FRAME_SIZE = 15U;

struct InfraredLineFrame {
    int16_t offset;
    uint16_t all_black;
    uint16_t adc[3];
    uint32_t sequence;
    uint32_t received_ms;
};

struct InfraredLineParserStats {
    uint32_t bytes_received;
    uint32_t valid_frames;
    uint32_t header_errors;
    uint32_t crc_errors;
    uint32_t last_period_ms;
    uint32_t average_period_ms;
    uint32_t minimum_period_ms;
    uint32_t maximum_period_ms;
};

struct InfraredLineParser {
    uint8_t buffer[INFRARED_LINE_FRAME_SIZE];
    uint8_t length;
    uint32_t last_frame_ms;
    InfraredLineParserStats stats;
};

void InfraredLineParser_Init(InfraredLineParser *parser);
bool InfraredLineParser_FeedByte(InfraredLineParser *parser,
                                 uint8_t byte,
                                 uint32_t now_ms,
                                 InfraredLineFrame *frame);
uint16_t InfraredLine_ModbusCrc16(const uint8_t *data, uint8_t length);

} /* namespace drivers */

#endif /* DRIVERS_INFRARED_LINE_INFRARED_LINE_PROTOCOL_H_ */
