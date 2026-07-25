#include "app/lora_protocol.h"

#include <limits.h>
#include <string.h>

namespace app {
namespace {

static const uint16_t kCrcInitialValue = 0xFFFFU;
static const uint16_t kCrcPolynomial = 0x1021U;
static const uint8_t kEncodedVersionIndex = 2U;
static const uint8_t kEncodedTypeIndex = 3U;
static const uint8_t kEncodedFlagsIndex = 4U;
static const uint8_t kEncodedSequenceIndex = 5U;
static const uint8_t kEncodedLengthLowIndex = 6U;
static const uint8_t kEncodedLengthHighIndex = 7U;
void IncrementSaturated(uint32_t *value)
{
    if (*value != UINT32_MAX) {
        (*value)++;
    }
}

uint16_t UpdateCrc(uint16_t crc, uint8_t data)
{
    crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data) << 8U));
    for (uint8_t bit = 0U; bit < 8U; bit++) {
        crc = ((crc & 0x8000U) != 0U) ?
            static_cast<uint16_t>((crc << 1U) ^ kCrcPolynomial) :
            static_cast<uint16_t>(crc << 1U);
    }
    return crc;
}

bool ConfigIsValid(const LoraProtocolConfig *config)
{
    return (config != 0) && (config->write != 0) &&
           (config->acknowledgment_timeout_ms != 0U);
}

void ResetParser(LoraProtocolContext *context)
{
    context->parser_state = LORA_PROTOCOL_PARSE_SOF_0;
    context->parser_header_index = 0U;
    context->parser_payload_index = 0U;
    context->parser_expected_length = 0U;
    context->parser_crc = kCrcInitialValue;
    context->parser_received_crc = 0U;
    context->parser_frame.type = 0U;
    context->parser_frame.flags = 0U;
    context->parser_frame.sequence = 0U;
    context->parser_frame.length = 0U;
}

void ResetParserWithPossibleSof(LoraProtocolContext *context, uint8_t data)
{
    ResetParser(context);
    if (data == LORA_PROTOCOL_SOF_0) {
        context->parser_state = LORA_PROTOCOL_PARSE_SOF_1;
    }
}

uint16_t EncodeFrame(uint8_t type,
                     uint8_t flags,
                     uint8_t sequence,
                     const uint8_t *payload,
                     uint16_t length,
                     uint8_t encoded[LORA_PROTOCOL_MAX_FRAME_LENGTH])
{
    encoded[0] = LORA_PROTOCOL_SOF_0;
    encoded[1] = LORA_PROTOCOL_SOF_1;
    encoded[kEncodedVersionIndex] = LORA_PROTOCOL_VERSION;
    encoded[kEncodedTypeIndex] = type;
    encoded[kEncodedFlagsIndex] = flags;
    encoded[kEncodedSequenceIndex] = sequence;
    encoded[kEncodedLengthLowIndex] = static_cast<uint8_t>(length & 0xFFU);
    encoded[kEncodedLengthHighIndex] =
        static_cast<uint8_t>((length >> 8U) & 0xFFU);

    if (length != 0U) {
        (void) memcpy(&encoded[LORA_PROTOCOL_HEADER_LENGTH], payload, length);
    }

    const uint16_t crc_length = static_cast<uint16_t>(
        (LORA_PROTOCOL_HEADER_LENGTH - 2U) + length);
    const uint16_t crc = LoraProtocol_Crc16Ccitt(
        &encoded[kEncodedVersionIndex], crc_length);
    const uint16_t crc_index = static_cast<uint16_t>(
        LORA_PROTOCOL_HEADER_LENGTH + length);
    encoded[crc_index] = static_cast<uint8_t>(crc & 0xFFU);
    encoded[crc_index + 1U] = static_cast<uint8_t>((crc >> 8U) & 0xFFU);
    return static_cast<uint16_t>(crc_index + LORA_PROTOCOL_CRC_LENGTH);
}

drivers::DriverStatus WriteEncoded(LoraProtocolContext *context,
                                   const uint8_t *encoded,
                                   uint16_t length)
{
    const drivers::DriverStatus status = context->config.write(
        encoded,
        length,
        context->config.write_context);
    if (status != drivers::DRIVER_OK) {
        IncrementSaturated(&context->statistics.tx_errors);
    }
    return status;
}

drivers::DriverStatus SendAcknowledgment(LoraProtocolContext *context,
                                         uint8_t sequence)
{
    uint8_t encoded[LORA_PROTOCOL_MAX_FRAME_LENGTH];
    const uint16_t length = EncodeFrame(LORA_PROTOCOL_TYPE_ACK,
                                        0U,
                                        sequence,
                                        0,
                                        0U,
                                        encoded);
    const drivers::DriverStatus status = WriteEncoded(context,
                                                       encoded,
                                                       length);
    if (status == drivers::DRIVER_OK) {
        IncrementSaturated(&context->statistics.tx_ack_frames);
    }
    return status;
}

bool IsDuplicate(const LoraProtocolContext *context, uint16_t frame_crc)
{
    return context->has_last_rx_signature &&
           (context->last_rx_sequence == context->parser_frame.sequence) &&
           (context->last_rx_crc == frame_crc);
}

drivers::DriverStatus QueueParsedFrame(LoraProtocolContext *context)
{
    if (context->rx_queue_count >= LORA_PROTOCOL_RX_QUEUE_DEPTH) {
        IncrementSaturated(&context->statistics.rx_queue_drops);
        return drivers::DRIVER_ERROR_BUSY;
    }

    context->rx_queue[context->rx_queue_head] = context->parser_frame;
    context->rx_queue_head++;
    if (context->rx_queue_head >= LORA_PROTOCOL_RX_QUEUE_DEPTH) {
        context->rx_queue_head = 0U;
    }
    context->rx_queue_count++;
    IncrementSaturated(&context->statistics.rx_delivered_frames);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus HandleCompletedFrame(LoraProtocolContext *context,
                                           uint16_t frame_crc,
                                           uint32_t now_ms)
{
    (void) now_ms;
    IncrementSaturated(&context->statistics.rx_valid_frames);

    if (context->parser_frame.type == LORA_PROTOCOL_TYPE_ACK) {
        if ((context->parser_frame.flags != 0U) ||
            (context->parser_frame.length != 0U)) {
            IncrementSaturated(&context->statistics.rx_format_errors);
            return drivers::DRIVER_ERROR;
        }

        IncrementSaturated(&context->statistics.rx_ack_frames);
        if (context->awaiting_ack &&
            (context->pending_sequence == context->parser_frame.sequence)) {
            context->awaiting_ack = false;
            context->pending_retry_count = 0U;
            context->pending_encoded_length = 0U;
            context->last_tx_status = drivers::DRIVER_OK;
            return drivers::DRIVER_OK;
        }

        IncrementSaturated(&context->statistics.rx_unexpected_acks);
        return drivers::DRIVER_ERROR;
    }

    const bool acknowledgment_required =
        (context->parser_frame.flags & LORA_PROTOCOL_FLAG_ACK_REQUIRED) != 0U;
    if (IsDuplicate(context, frame_crc)) {
        IncrementSaturated(&context->statistics.rx_duplicates);
        return acknowledgment_required ?
            SendAcknowledgment(context, context->parser_frame.sequence) :
            drivers::DRIVER_OK;
    }

    const drivers::DriverStatus queue_status = QueueParsedFrame(context);
    if (queue_status != drivers::DRIVER_OK) {
        /* Do not ACK a frame that the application could not retain. */
        return queue_status;
    }

    context->has_last_rx_signature = true;
    context->last_rx_sequence = context->parser_frame.sequence;
    context->last_rx_crc = frame_crc;

    return acknowledgment_required ?
        SendAcknowledgment(context, context->parser_frame.sequence) :
        drivers::DRIVER_OK;
}

drivers::DriverStatus ConsumeHeaderByte(LoraProtocolContext *context,
                                        uint8_t data)
{
    context->parser_crc = UpdateCrc(context->parser_crc, data);

    switch (context->parser_header_index) {
    case 0U:
        if (data != LORA_PROTOCOL_VERSION) {
            IncrementSaturated(&context->statistics.rx_format_errors);
            ResetParserWithPossibleSof(context, data);
            return drivers::DRIVER_ERROR_UNSUPPORTED;
        }
        break;
    case 1U:
        if (data == 0U) {
            IncrementSaturated(&context->statistics.rx_format_errors);
            ResetParserWithPossibleSof(context, data);
            return drivers::DRIVER_ERROR_INVALID_ARG;
        }
        context->parser_frame.type = data;
        break;
    case 2U:
        if ((data & static_cast<uint8_t>(~LORA_PROTOCOL_FLAG_ACK_REQUIRED)) !=
            0U) {
            IncrementSaturated(&context->statistics.rx_format_errors);
            ResetParserWithPossibleSof(context, data);
            return drivers::DRIVER_ERROR_INVALID_ARG;
        }
        context->parser_frame.flags = data;
        break;
    case 3U:
        context->parser_frame.sequence = data;
        break;
    case 4U:
        context->parser_expected_length = data;
        break;
    case 5U:
        context->parser_expected_length = static_cast<uint16_t>(
            context->parser_expected_length |
            (static_cast<uint16_t>(data) << 8U));
        if (context->parser_expected_length >
            LORA_PROTOCOL_MAX_PAYLOAD_LENGTH) {
            IncrementSaturated(&context->statistics.rx_length_errors);
            ResetParserWithPossibleSof(context, data);
            return drivers::DRIVER_ERROR_INVALID_ARG;
        }
        context->parser_frame.length = context->parser_expected_length;
        context->parser_payload_index = 0U;
        context->parser_state = (context->parser_expected_length == 0U) ?
            LORA_PROTOCOL_PARSE_CRC_0 : LORA_PROTOCOL_PARSE_PAYLOAD;
        break;
    default:
        ResetParser(context);
        return drivers::DRIVER_ERROR;
    }

    context->parser_header_index++;
    return drivers::DRIVER_OK;
}

} /* namespace */

uint16_t LoraProtocol_Crc16Ccitt(const uint8_t *data, uint16_t length)
{
    if ((data == 0) && (length != 0U)) {
        return 0U;
    }

    uint16_t crc = kCrcInitialValue;
    for (uint16_t i = 0U; i < length; i++) {
        crc = UpdateCrc(crc, data[i]);
    }
    return crc;
}

drivers::DriverStatus LoraProtocol_Init(LoraProtocolContext *context,
                                        const LoraProtocolConfig *config,
                                        uint8_t initial_sequence)
{
    if ((context == 0) || (!ConfigIsValid(config))) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    (void) memset(context, 0, sizeof(*context));
    context->config = *config;
    context->initialized = true;
    context->next_tx_sequence = initial_sequence;
    context->last_tx_status = drivers::DRIVER_OK;
    ResetParser(context);
    return drivers::DRIVER_OK;
}

drivers::DriverStatus LoraProtocol_Reset(LoraProtocolContext *context,
                                         uint8_t initial_sequence)
{
    if ((context == 0) || (!context->initialized) ||
        (!ConfigIsValid(&context->config))) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    const LoraProtocolConfig config = context->config;
    return LoraProtocol_Init(context, &config, initial_sequence);
}

drivers::DriverStatus LoraProtocol_Send(LoraProtocolContext *context,
                                        uint8_t type,
                                        const uint8_t *payload,
                                        uint16_t length,
                                        bool acknowledgment_required,
                                        uint32_t now_ms,
                                        uint8_t *sequence)
{
    if ((context == 0) || (!context->initialized)) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if ((type == 0U) || (type == LORA_PROTOCOL_TYPE_ACK) ||
        (length > LORA_PROTOCOL_MAX_PAYLOAD_LENGTH) ||
        ((payload == 0) && (length != 0U))) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (context->awaiting_ack) {
        return drivers::DRIVER_ERROR_BUSY;
    }

    const uint8_t tx_sequence = context->next_tx_sequence;
    const uint8_t flags = acknowledgment_required ?
        LORA_PROTOCOL_FLAG_ACK_REQUIRED : 0U;
    uint8_t encoded[LORA_PROTOCOL_MAX_FRAME_LENGTH];
    const uint16_t encoded_length = EncodeFrame(type,
                                                flags,
                                                tx_sequence,
                                                payload,
                                                length,
                                                encoded);
    const drivers::DriverStatus status = WriteEncoded(context,
                                                       encoded,
                                                       encoded_length);
    context->last_tx_status = status;
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    IncrementSaturated(&context->statistics.tx_frames);
    context->next_tx_sequence++;
    if (sequence != 0) {
        *sequence = tx_sequence;
    }

    if (acknowledgment_required) {
        (void) memcpy(context->pending_encoded, encoded, encoded_length);
        context->awaiting_ack = true;
        context->pending_sequence = tx_sequence;
        context->pending_retry_count = 0U;
        context->pending_since_ms = now_ms;
        context->pending_encoded_length = encoded_length;
    }
    return drivers::DRIVER_OK;
}

drivers::DriverStatus LoraProtocol_ProcessByte(LoraProtocolContext *context,
                                               uint8_t data,
                                               uint32_t now_ms)
{
    if ((context == 0) || (!context->initialized)) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    switch (context->parser_state) {
    case LORA_PROTOCOL_PARSE_SOF_0:
        if (data == LORA_PROTOCOL_SOF_0) {
            context->parser_state = LORA_PROTOCOL_PARSE_SOF_1;
        }
        return drivers::DRIVER_OK;

    case LORA_PROTOCOL_PARSE_SOF_1:
        if (data == LORA_PROTOCOL_SOF_1) {
            context->parser_state = LORA_PROTOCOL_PARSE_HEADER;
            context->parser_header_index = 0U;
            context->parser_crc = kCrcInitialValue;
        } else if (data != LORA_PROTOCOL_SOF_0) {
            context->parser_state = LORA_PROTOCOL_PARSE_SOF_0;
        }
        return drivers::DRIVER_OK;

    case LORA_PROTOCOL_PARSE_HEADER:
        return ConsumeHeaderByte(context, data);

    case LORA_PROTOCOL_PARSE_PAYLOAD:
        context->parser_frame.payload[context->parser_payload_index] = data;
        context->parser_payload_index++;
        context->parser_crc = UpdateCrc(context->parser_crc, data);
        if (context->parser_payload_index >= context->parser_expected_length) {
            context->parser_state = LORA_PROTOCOL_PARSE_CRC_0;
        }
        return drivers::DRIVER_OK;

    case LORA_PROTOCOL_PARSE_CRC_0:
        context->parser_received_crc = data;
        context->parser_state = LORA_PROTOCOL_PARSE_CRC_1;
        return drivers::DRIVER_OK;

    case LORA_PROTOCOL_PARSE_CRC_1: {
        context->parser_received_crc = static_cast<uint16_t>(
            context->parser_received_crc | (static_cast<uint16_t>(data) << 8U));
        if (context->parser_received_crc != context->parser_crc) {
            IncrementSaturated(&context->statistics.rx_crc_errors);
            ResetParserWithPossibleSof(context, data);
            return drivers::DRIVER_ERROR;
        }

        const uint16_t frame_crc = context->parser_crc;
        const drivers::DriverStatus status = HandleCompletedFrame(context,
                                                                  frame_crc,
                                                                  now_ms);
        ResetParser(context);
        return status;
    }

    default:
        ResetParserWithPossibleSof(context, data);
        return drivers::DRIVER_ERROR;
    }
}

drivers::DriverStatus LoraProtocol_ProcessTimeouts(
    LoraProtocolContext *context,
    uint32_t now_ms)
{
    if ((context == 0) || (!context->initialized)) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (!context->awaiting_ack) {
        return drivers::DRIVER_OK;
    }
    if (static_cast<uint32_t>(now_ms - context->pending_since_ms) <
        context->config.acknowledgment_timeout_ms) {
        return drivers::DRIVER_OK;
    }

    if (context->pending_retry_count >= context->config.max_retries) {
        context->awaiting_ack = false;
        context->pending_encoded_length = 0U;
        context->last_tx_status = drivers::DRIVER_ERROR_TIMEOUT;
        IncrementSaturated(&context->statistics.tx_retry_exhausted);
        return drivers::DRIVER_ERROR_TIMEOUT;
    }

    const drivers::DriverStatus status = WriteEncoded(
        context,
        context->pending_encoded,
        context->pending_encoded_length);
    context->pending_retry_count++;
    context->pending_since_ms = now_ms;
    context->last_tx_status = status;
    if (status == drivers::DRIVER_OK) {
        IncrementSaturated(&context->statistics.tx_retries);
    }
    return status;
}

bool LoraProtocol_ReadFrame(LoraProtocolContext *context,
                            LoraProtocolFrame *frame)
{
    if ((context == 0) || (!context->initialized) || (frame == 0) ||
        (context->rx_queue_count == 0U)) {
        return false;
    }

    *frame = context->rx_queue[context->rx_queue_tail];
    context->rx_queue_tail++;
    if (context->rx_queue_tail >= LORA_PROTOCOL_RX_QUEUE_DEPTH) {
        context->rx_queue_tail = 0U;
    }
    context->rx_queue_count--;
    return true;
}

uint8_t LoraProtocol_GetQueuedFrameCount(const LoraProtocolContext *context)
{
    return ((context != 0) && context->initialized) ?
        context->rx_queue_count : 0U;
}

} /* namespace app */

