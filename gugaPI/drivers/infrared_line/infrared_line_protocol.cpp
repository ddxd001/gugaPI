#include "drivers/infrared_line/infrared_line_protocol.h"

#include <string.h>

namespace drivers {
namespace {

static const uint8_t kAddress = 0x20U;
static const uint8_t kFunction = 0x03U;
static const uint8_t kPayloadLength = 0x0AU;

uint16_t ReadU16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

void UpdatePeriod(InfraredLineParser *parser, uint32_t now_ms)
{
    if (parser->last_frame_ms != 0U) {
        const uint32_t period = now_ms - parser->last_frame_ms;
        parser->stats.last_period_ms = period;
        if ((parser->stats.minimum_period_ms == 0U) ||
            (period < parser->stats.minimum_period_ms)) {
            parser->stats.minimum_period_ms = period;
        }
        if (period > parser->stats.maximum_period_ms) {
            parser->stats.maximum_period_ms = period;
        }
        if (parser->stats.average_period_ms == 0U) {
            parser->stats.average_period_ms = period;
        } else {
            parser->stats.average_period_ms =
                (parser->stats.average_period_ms * 7U + period + 4U) / 8U;
        }
    }
    parser->last_frame_ms = now_ms;
}

void KeepPossibleHeader(InfraredLineParser *parser, uint8_t byte)
{
    if (byte == kAddress) {
        parser->buffer[0] = byte;
        parser->length = 1U;
    } else {
        parser->length = 0U;
    }
}

void ResyncAfterBadFrame(InfraredLineParser *parser)
{
    for (uint8_t start = 1U; start < parser->length; start++) {
        if (parser->buffer[start] != kAddress) {
            continue;
        }
        const uint8_t remaining = static_cast<uint8_t>(
            parser->length - start);
        (void) memmove(parser->buffer, &parser->buffer[start], remaining);
        parser->length = remaining;
        return;
    }
    parser->length = 0U;
}

} /* namespace */

uint16_t InfraredLine_ModbusCrc16(const uint8_t *data, uint8_t length)
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

void InfraredLineParser_Init(InfraredLineParser *parser)
{
    if (parser != 0) {
        (void) memset(parser, 0, sizeof(*parser));
    }
}

bool InfraredLineParser_FeedByte(InfraredLineParser *parser,
                                 uint8_t byte,
                                 uint32_t now_ms,
                                 InfraredLineFrame *frame)
{
    if ((parser == 0) || (frame == 0)) {
        return false;
    }
    parser->stats.bytes_received++;

    if (parser->length == 0U) {
        if (byte == kAddress) {
            parser->buffer[0] = byte;
            parser->length = 1U;
        } else {
            parser->stats.header_errors++;
        }
        return false;
    }
    if (parser->length == 1U) {
        if (byte != kFunction) {
            parser->stats.header_errors++;
            KeepPossibleHeader(parser, byte);
            return false;
        }
        parser->buffer[parser->length++] = byte;
        return false;
    }
    if (parser->length == 2U) {
        if (byte != kPayloadLength) {
            parser->stats.header_errors++;
            KeepPossibleHeader(parser, byte);
            return false;
        }
        parser->buffer[parser->length++] = byte;
        return false;
    }

    parser->buffer[parser->length++] = byte;
    if (parser->length < INFRARED_LINE_FRAME_SIZE) {
        return false;
    }

    const uint16_t received_crc = ReadU16(&parser->buffer[13]);
    const uint16_t calculated_crc = InfraredLine_ModbusCrc16(
        parser->buffer,
        13U);
    if (received_crc != calculated_crc) {
        parser->stats.crc_errors++;
        ResyncAfterBadFrame(parser);
        return false;
    }

    frame->offset = static_cast<int16_t>(ReadU16(&parser->buffer[3]));
    frame->all_black = ReadU16(&parser->buffer[5]);
    frame->adc[0] = ReadU16(&parser->buffer[7]);
    frame->adc[1] = ReadU16(&parser->buffer[9]);
    frame->adc[2] = ReadU16(&parser->buffer[11]);
    parser->stats.valid_frames++;
    frame->sequence = parser->stats.valid_frames;
    frame->received_ms = now_ms;
    UpdatePeriod(parser, now_ms);
    parser->length = 0U;
    return true;
}

} /* namespace drivers */
