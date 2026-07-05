#ifndef PROTOCOL_SERIAL_PROTOCOL_H_
#define PROTOCOL_SERIAL_PROTOCOL_H_

#include <stdint.h>

#include "protocol/motor_registers.h"
#include "protocol/register_map.h"

namespace protocol {

constexpr uint8_t kSerialMaxFrameLength =
    static_cast<uint8_t>(4U + kMaxPayloadLength + 1U);

struct SerialResponse {
    uint8_t bytes[kSerialMaxFrameLength];
    uint8_t length;
};

class SerialProtocol {
public:
    void Init(RegisterMap *registers);
    bool ProcessByte(uint8_t value,
                     SerialResponse *response,
                     bool *write_committed);
    static uint8_t Crc8Update(uint8_t crc, uint8_t value);

private:
    enum class ParserState : uint8_t {
        WaitSof,
        Command,
        Register,
        Length,
        Data,
        Crc,
    };

    void ResetParser(void);
    bool HandleFrame(SerialResponse *response, bool *write_committed);
    void BuildResponse(uint8_t request_cmd,
                       uint8_t reg,
                       const uint8_t *data,
                       uint8_t length,
                       SerialResponse *response) const;
    void BuildStatusResponse(uint8_t request_cmd,
                             uint8_t reg,
                             uint8_t status,
                             SerialResponse *response) const;
    RegisterMap *registers_;
    ParserState state_;
    uint8_t crc_;
    uint8_t cmd_;
    uint8_t reg_;
    uint8_t length_;
    uint8_t data_index_;
    uint8_t data_[kMaxPayloadLength];
};

uint8_t SerialCrc8(const uint8_t *data, uint8_t length);

}  // namespace protocol

#endif  // PROTOCOL_SERIAL_PROTOCOL_H_
