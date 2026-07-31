#ifndef DRIVERS_BALL_VISION_BALL_VISION_PROTOCOL_H_
#define DRIVERS_BALL_VISION_BALL_VISION_PROTOCOL_H_

#include <stdbool.h>
#include <stdint.h>

namespace drivers {

static const uint8_t BALL_VISION_FRAME_SIZE = 12U;
static const uint8_t BALL_VISION_FLAG_BALL_FOUND = (1U << 0);
static const uint8_t BALL_VISION_FLAG_CALIBRATION_VALID = (1U << 1);
static const uint8_t BALL_VISION_FLAG_CAMERA_OK = (1U << 2);

struct BallVisionFrame {
    uint8_t sequence;
    uint8_t flags;
    int16_t position_0p1mm;
    uint16_t confidence;
    uint16_t source_delay_ms;
    uint32_t received_ms;
};

struct BallVisionParserStats {
    uint32_t bytes_received;
    uint32_t valid_frames;
    uint32_t header_errors;
    uint32_t crc_errors;
    uint32_t payload_errors;
    uint32_t resync_bytes;
    uint32_t sequence_gaps;
    uint32_t duplicate_frames;
};

struct BallVisionParser {
    uint8_t buffer[BALL_VISION_FRAME_SIZE];
    uint8_t length;
    uint8_t previous_sequence;
    bool has_previous_sequence;
    BallVisionParserStats stats;
};

void BallVisionParser_Init(BallVisionParser *parser);
void BallVisionParser_ClearStats(BallVisionParser *parser);
void BallVisionParser_ResetStream(BallVisionParser *parser);
bool BallVisionParser_FeedByte(BallVisionParser *parser,
                               uint8_t byte,
                               uint32_t now_ms,
                               BallVisionFrame *frame);
uint16_t BallVision_ModbusCrc16(const uint8_t *data, uint8_t length);

} /* namespace drivers */

#endif /* DRIVERS_BALL_VISION_BALL_VISION_PROTOCOL_H_ */
