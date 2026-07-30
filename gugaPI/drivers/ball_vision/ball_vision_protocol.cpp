#include "drivers/ball_vision/ball_vision_protocol.h"

#include <string.h>

namespace drivers {
namespace {

static const uint8_t kHeader0 = 0xA5U;
static const uint8_t kHeader1 = 0x5AU;
static const uint8_t kAllowedFlags =
    BALL_VISION_FLAG_BALL_FOUND |
    BALL_VISION_FLAG_CALIBRATION_VALID |
    BALL_VISION_FLAG_CAMERA_OK;
static const int16_t kMaximumPosition0p1mm = 1250;
static const uint16_t kMaximumConfidence = 1000U;

uint16_t ReadU16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

void KeepPossibleHeader(BallVisionParser *parser, uint8_t byte)
{
    if (byte == kHeader0) {
        parser->buffer[0] = byte;
        parser->length = 1U;
    } else {
        parser->length = 0U;
    }
}

void ResyncAfterBadFrame(BallVisionParser *parser)
{
    for (uint8_t start = 1U; start < parser->length; start++) {
        const uint8_t remaining =
            static_cast<uint8_t>(parser->length - start);
        if (parser->buffer[start] != kHeader0) {
            continue;
        }
        if ((remaining >= 2U) && (parser->buffer[start + 1U] != kHeader1)) {
            continue;
        }
        (void) memmove(parser->buffer, &parser->buffer[start], remaining);
        parser->length = remaining;
        parser->stats.resync_bytes += start;
        return;
    }
    parser->stats.resync_bytes += parser->length;
    parser->length = 0U;
}

void UpdateSequenceStats(BallVisionParser *parser, uint8_t sequence)
{
    if (parser->has_previous_sequence) {
        const uint8_t delta =
            static_cast<uint8_t>(sequence - parser->previous_sequence);
        if (delta == 0U) {
            parser->stats.duplicate_frames++;
        } else if (delta > 1U) {
            parser->stats.sequence_gaps +=
                static_cast<uint32_t>(delta - 1U);
        }
    }
    parser->previous_sequence = sequence;
    parser->has_previous_sequence = true;
}

} /* namespace */

uint16_t BallVision_ModbusCrc16(const uint8_t *data, uint8_t length)
{
    if ((data == 0) && (length != 0U)) {
        return 0U;
    }
    uint16_t crc = 0xFFFFU;
    for (uint8_t i = 0U; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 1U) != 0U) {
                crc = static_cast<uint16_t>((crc >> 1U) ^ 0xA001U);
            } else {
                crc = static_cast<uint16_t>(crc >> 1U);
            }
        }
    }
    return crc;
}

void BallVisionParser_Init(BallVisionParser *parser)
{
    if (parser != 0) {
        (void) memset(parser, 0, sizeof(*parser));
    }
}

void BallVisionParser_ClearStats(BallVisionParser *parser)
{
    if (parser == 0) {
        return;
    }
    (void) memset(&parser->stats, 0, sizeof(parser->stats));
    parser->has_previous_sequence = false;
}

void BallVisionParser_ResetStream(BallVisionParser *parser)
{
    if (parser != 0) {
        parser->length = 0U;
    }
}

bool BallVisionParser_FeedByte(BallVisionParser *parser,
                               uint8_t byte,
                               uint32_t now_ms,
                               BallVisionFrame *frame)
{
    if ((parser == 0) || (frame == 0)) {
        return false;
    }
    parser->stats.bytes_received++;

    if (parser->length == 0U) {
        if (byte == kHeader0) {
            parser->buffer[0] = byte;
            parser->length = 1U;
        } else {
            parser->stats.header_errors++;
        }
        return false;
    }
    if (parser->length == 1U) {
        if (byte != kHeader1) {
            parser->stats.header_errors++;
            KeepPossibleHeader(parser, byte);
            return false;
        }
        parser->buffer[parser->length++] = byte;
        return false;
    }

    parser->buffer[parser->length++] = byte;
    if (parser->length < BALL_VISION_FRAME_SIZE) {
        return false;
    }

    const uint16_t received_crc = ReadU16(&parser->buffer[10]);
    const uint16_t calculated_crc =
        BallVision_ModbusCrc16(parser->buffer, 10U);
    if (received_crc != calculated_crc) {
        parser->stats.crc_errors++;
        ResyncAfterBadFrame(parser);
        return false;
    }

    const uint8_t flags = parser->buffer[3];
    const int16_t position =
        static_cast<int16_t>(ReadU16(&parser->buffer[4]));
    const uint16_t confidence = ReadU16(&parser->buffer[6]);
    if (((flags & static_cast<uint8_t>(~kAllowedFlags)) != 0U) ||
        (position < -kMaximumPosition0p1mm) ||
        (position > kMaximumPosition0p1mm) ||
        (confidence > kMaximumConfidence)) {
        parser->stats.payload_errors++;
        ResyncAfterBadFrame(parser);
        return false;
    }

    frame->sequence = parser->buffer[2];
    frame->flags = flags;
    frame->position_0p1mm = position;
    frame->confidence = confidence;
    frame->source_delay_ms = ReadU16(&parser->buffer[8]);
    frame->received_ms = now_ms;
    parser->stats.valid_frames++;
    UpdateSequenceStats(parser, frame->sequence);
    parser->length = 0U;
    return true;
}

} /* namespace drivers */
