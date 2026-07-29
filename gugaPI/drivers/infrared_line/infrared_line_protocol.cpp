#include "drivers/infrared_line/infrared_line_protocol.h"

#include <string.h>

namespace drivers {
namespace {

static const uint8_t kAddress = 0x20U;
static const uint8_t kFunction = 0x03U;
static const uint8_t kPayloadLength = 0x0AU;
static const uint8_t kHeader[] = { kAddress, kFunction, kPayloadLength };

uint16_t ReadU16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

void UpdatePeriod(InfraredLineParser *parser,
                  uint32_t now_ms,
                  uint32_t now_us)
{
    if (parser->last_frame_us != 0U) {
        const uint32_t period_us = now_us - parser->last_frame_us;
        const uint32_t period = (period_us + 500U) / 1000U;
        parser->stats.last_period_us = period_us;
        parser->stats.last_period_ms = period;
        if ((parser->stats.minimum_period_ms == 0U) ||
            (period < parser->stats.minimum_period_ms)) {
            parser->stats.minimum_period_ms = period;
        }
        if (period > parser->stats.maximum_period_ms) {
            parser->stats.maximum_period_ms = period;
        }
        if (parser->stats.average_period_us == 0U) {
            parser->stats.average_period_us = period_us;
            parser->stats.average_period_ms = period;
        } else {
            parser->stats.average_period_us =
                (parser->stats.average_period_us * 7U + period_us + 4U) /
                8U;
            parser->stats.average_period_ms =
                (parser->stats.average_period_us + 500U) / 1000U;
        }
        if (parser->stats.period_samples != UINT32_MAX) {
            parser->stats.period_samples++;
        }
    }
    parser->last_frame_ms = now_ms;
    parser->last_frame_us = now_us;
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
        const uint8_t remaining = static_cast<uint8_t>(
            parser->length - start);
        const uint8_t compare_length = (remaining < sizeof(kHeader))
            ? remaining : static_cast<uint8_t>(sizeof(kHeader));
        if (memcmp(&parser->buffer[start], kHeader, compare_length) != 0) {
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

void RecordInvalidFrame(InfraredLineParser *parser)
{
    if (parser->stats.consecutive_invalid_frames != UINT8_MAX) {
        parser->stats.consecutive_invalid_frames++;
    }
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

void InfraredLineParser_ClearStats(InfraredLineParser *parser)
{
    if (parser == 0) {
        return;
    }
    (void) memset(&parser->stats, 0, sizeof(parser->stats));
    parser->last_frame_ms = 0U;
    parser->last_frame_us = 0U;
}

void InfraredLineParser_ResetStream(InfraredLineParser *parser)
{
    if (parser != 0) {
        parser->length = 0U;
    }
}

bool InfraredLineParser_FeedByte(InfraredLineParser *parser,
                                 uint8_t byte,
                                 uint32_t now_ms,
                                 uint32_t now_us,
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
        RecordInvalidFrame(parser);
        ResyncAfterBadFrame(parser);
        return false;
    }

    const uint16_t all_black = ReadU16(&parser->buffer[5]);
    const uint16_t adc0 = ReadU16(&parser->buffer[7]);
    const uint16_t adc1 = ReadU16(&parser->buffer[9]);
    const uint16_t adc2 = ReadU16(&parser->buffer[11]);
    if ((all_black > 1U) || (adc0 > 4095U) ||
        (adc1 > 4095U) || (adc2 > 4095U)) {
        parser->stats.payload_errors++;
        RecordInvalidFrame(parser);
        ResyncAfterBadFrame(parser);
        return false;
    }

    frame->offset = static_cast<int16_t>(ReadU16(&parser->buffer[3]));
    frame->all_black = all_black;
    frame->adc[0] = adc0;
    frame->adc[1] = adc1;
    frame->adc[2] = adc2;
    parser->stats.valid_frames++;
    parser->sequence++;
    frame->sequence = parser->sequence;
    frame->received_ms = now_ms;
    frame->received_us = now_us;
    parser->stats.consecutive_invalid_frames = 0U;
    UpdatePeriod(parser, now_ms, now_us);
    parser->length = 0U;
    return true;
}

} /* namespace drivers */
