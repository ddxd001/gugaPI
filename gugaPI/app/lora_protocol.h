#ifndef APP_LORA_PROTOCOL_H_
#define APP_LORA_PROTOCOL_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace app {

static const uint8_t LORA_PROTOCOL_SOF_0 = 0xA5U;
static const uint8_t LORA_PROTOCOL_SOF_1 = 0x5AU;
static const uint8_t LORA_PROTOCOL_VERSION = 1U;
static const uint16_t LORA_PROTOCOL_MAX_PAYLOAD_LENGTH = 128U;
static const uint16_t LORA_PROTOCOL_HEADER_LENGTH = 8U;
static const uint16_t LORA_PROTOCOL_CRC_LENGTH = 2U;
static const uint16_t LORA_PROTOCOL_MAX_FRAME_LENGTH =
    LORA_PROTOCOL_HEADER_LENGTH + LORA_PROTOCOL_MAX_PAYLOAD_LENGTH +
    LORA_PROTOCOL_CRC_LENGTH;
static const uint8_t LORA_PROTOCOL_RX_QUEUE_DEPTH = 4U;
static const uint8_t LORA_PROTOCOL_TYPE_DATA = 0x01U;
static const uint8_t LORA_PROTOCOL_TYPE_ACK = 0x02U;
static const uint8_t LORA_PROTOCOL_FLAG_ACK_REQUIRED = 0x01U;

typedef drivers::DriverStatus (*LoraProtocolWriteFunction)(
    const uint8_t *data,
    uint16_t length,
    void *user_context);

struct LoraProtocolConfig {
    LoraProtocolWriteFunction write;
    void *write_context;
    uint32_t acknowledgment_timeout_ms;
    uint8_t max_retries;
};

struct LoraProtocolFrame {
    uint8_t type;
    uint8_t flags;
    uint8_t sequence;
    uint16_t length;
    uint8_t payload[LORA_PROTOCOL_MAX_PAYLOAD_LENGTH];
};

struct LoraProtocolStatistics {
    uint32_t rx_valid_frames;
    uint32_t rx_delivered_frames;
    uint32_t rx_crc_errors;
    uint32_t rx_format_errors;
    uint32_t rx_length_errors;
    uint32_t rx_duplicates;
    uint32_t rx_queue_drops;
    uint32_t rx_ack_frames;
    uint32_t rx_unexpected_acks;
    uint32_t tx_frames;
    uint32_t tx_ack_frames;
    uint32_t tx_retries;
    uint32_t tx_errors;
    uint32_t tx_retry_exhausted;
};

enum LoraProtocolParserState : uint8_t {
    LORA_PROTOCOL_PARSE_SOF_0 = 0U,
    LORA_PROTOCOL_PARSE_SOF_1,
    LORA_PROTOCOL_PARSE_HEADER,
    LORA_PROTOCOL_PARSE_PAYLOAD,
    LORA_PROTOCOL_PARSE_CRC_0,
    LORA_PROTOCOL_PARSE_CRC_1
};

struct LoraProtocolContext {
    LoraProtocolConfig config;
    bool initialized;
    uint8_t next_tx_sequence;
    drivers::DriverStatus last_tx_status;

    bool awaiting_ack;
    uint8_t pending_sequence;
    uint8_t pending_retry_count;
    uint32_t pending_since_ms;
    uint16_t pending_encoded_length;
    uint8_t pending_encoded[LORA_PROTOCOL_MAX_FRAME_LENGTH];

    LoraProtocolParserState parser_state;
    uint8_t parser_header_index;
    uint16_t parser_payload_index;
    uint16_t parser_expected_length;
    uint16_t parser_crc;
    uint16_t parser_received_crc;
    LoraProtocolFrame parser_frame;

    bool has_last_rx_signature;
    uint8_t last_rx_sequence;
    uint16_t last_rx_crc;

    LoraProtocolFrame rx_queue[LORA_PROTOCOL_RX_QUEUE_DEPTH];
    uint8_t rx_queue_head;
    uint8_t rx_queue_tail;
    uint8_t rx_queue_count;

    LoraProtocolStatistics statistics;
};

drivers::DriverStatus LoraProtocol_Init(LoraProtocolContext *context,
                                        const LoraProtocolConfig *config,
                                        uint8_t initial_sequence);
drivers::DriverStatus LoraProtocol_Reset(LoraProtocolContext *context,
                                         uint8_t initial_sequence);
drivers::DriverStatus LoraProtocol_Send(LoraProtocolContext *context,
                                        uint8_t type,
                                        const uint8_t *payload,
                                        uint16_t length,
                                        bool acknowledgment_required,
                                        uint32_t now_ms,
                                        uint8_t *sequence);
drivers::DriverStatus LoraProtocol_ProcessByte(LoraProtocolContext *context,
                                               uint8_t data,
                                               uint32_t now_ms);
drivers::DriverStatus LoraProtocol_ProcessTimeouts(
    LoraProtocolContext *context,
    uint32_t now_ms);
bool LoraProtocol_ReadFrame(LoraProtocolContext *context,
                            LoraProtocolFrame *frame);
uint8_t LoraProtocol_GetQueuedFrameCount(const LoraProtocolContext *context);
uint16_t LoraProtocol_Crc16Ccitt(const uint8_t *data, uint16_t length);

} /* namespace app */

#endif /* APP_LORA_PROTOCOL_H_ */

