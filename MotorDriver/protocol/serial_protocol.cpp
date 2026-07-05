#include "protocol/serial_protocol.h"

namespace protocol {

void SerialProtocol::Init(RegisterMap *registers)
{
    registers_ = registers;
    ResetParser();
}

bool SerialProtocol::ProcessByte(uint8_t value,
                                 SerialResponse *response,
                                 bool *write_committed)
{
    if (response != 0) {
        response->length = 0U;
    }
    if (write_committed != 0) {
        *write_committed = false;
    }

    switch (state_) {
    case ParserState::WaitSof:
        if (value == kSerialSof) {
            crc_ = Crc8Update(0U, value);
            state_ = ParserState::Command;
        }
        return false;

    case ParserState::Command:
        cmd_ = value;
        crc_ = Crc8Update(crc_, value);
        state_ = ParserState::Register;
        return false;

    case ParserState::Register:
        reg_ = value;
        crc_ = Crc8Update(crc_, value);
        state_ = ParserState::Length;
        return false;

    case ParserState::Length:
        length_ = value;
        data_index_ = 0U;
        crc_ = Crc8Update(crc_, value);
        if (length_ > kMaxPayloadLength) {
            if (response != 0) {
                BuildStatusResponse(cmd_, reg_, kSerialStatusError, response);
            }
            ResetParser();
            return response != 0;
        }
        if ((cmd_ == kSerialCmdWrite) && (length_ > 0U)) {
            state_ = ParserState::Data;
        } else {
            state_ = ParserState::Crc;
        }
        return false;

    case ParserState::Data:
        data_[data_index_] = value;
        data_index_++;
        crc_ = Crc8Update(crc_, value);
        if (data_index_ >= length_) {
            state_ = ParserState::Crc;
        }
        return false;

    case ParserState::Crc:
        if (crc_ != value) {
            ResetParser();
            return false;
        }
        {
            const bool ready = HandleFrame(response, write_committed);
            ResetParser();
            return ready;
        }

    default:
        ResetParser();
        return false;
    }
}

void SerialProtocol::ResetParser(void)
{
    state_ = ParserState::WaitSof;
    crc_ = 0U;
    cmd_ = 0U;
    reg_ = 0U;
    length_ = 0U;
    data_index_ = 0U;
}

bool SerialProtocol::HandleFrame(SerialResponse *response, bool *write_committed)
{
    if ((response == 0) || (registers_ == 0)) {
        return false;
    }

    if (cmd_ == kSerialCmdHeartbeat) {
        if (length_ == 0U) {
            BuildStatusResponse(cmd_, reg_, kSerialStatusOk, response);
        } else {
            BuildStatusResponse(cmd_, reg_, kSerialStatusError, response);
        }
        return true;
    }

    if (cmd_ == kSerialCmdRead) {
        uint8_t response_data[kMaxPayloadLength];
        if ((length_ > 0U) &&
            registers_->ReadBlock(reg_, length_, response_data)) {
            BuildResponse(cmd_, reg_, response_data, length_, response);
        } else {
            BuildStatusResponse(cmd_, reg_, kSerialStatusError, response);
        }
        return true;
    }

    if (cmd_ == kSerialCmdWrite) {
        bool control_changed = false;
        if ((length_ > 0U) &&
            registers_->CommitWrite(reg_, data_, length_, &control_changed)) {
            (void) control_changed;
            if (write_committed != 0) {
                *write_committed = true;
            }
            BuildStatusResponse(cmd_, reg_, kSerialStatusOk, response);
        } else {
            BuildStatusResponse(cmd_, reg_, kSerialStatusError, response);
        }
        return true;
    }

    BuildStatusResponse(cmd_, reg_, kSerialStatusError, response);
    return true;
}

void SerialProtocol::BuildResponse(uint8_t request_cmd,
                                   uint8_t reg,
                                   const uint8_t *data,
                                   uint8_t length,
                                   SerialResponse *response) const
{
    uint8_t crc = 0U;
    uint8_t out_index = 0U;

    response->bytes[out_index++] = kSerialSof;
    response->bytes[out_index++] =
        static_cast<uint8_t>(request_cmd | kSerialResponseFlag);
    response->bytes[out_index++] = reg;
    response->bytes[out_index++] = length;

    for (uint8_t index = 0U; index < out_index; index++) {
        crc = Crc8Update(crc, response->bytes[index]);
    }

    for (uint8_t index = 0U; index < length; index++) {
        const uint8_t value = (data == 0) ? 0U : data[index];
        response->bytes[out_index++] = value;
        crc = Crc8Update(crc, value);
    }

    response->bytes[out_index++] = crc;
    response->length = out_index;
}

void SerialProtocol::BuildStatusResponse(uint8_t request_cmd,
                                         uint8_t reg,
                                         uint8_t status,
                                         SerialResponse *response) const
{
    BuildResponse(request_cmd, reg, &status, 1U, response);
}

uint8_t SerialProtocol::Crc8Update(uint8_t crc, uint8_t value)
{
    crc = static_cast<uint8_t>(crc ^ value);

    for (uint8_t bit = 0U; bit < 8U; bit++) {
        if ((crc & 0x80U) != 0U) {
            crc = static_cast<uint8_t>((crc << 1U) ^ 0x07U);
        } else {
            crc = static_cast<uint8_t>(crc << 1U);
        }
    }

    return crc;
}

uint8_t SerialCrc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;

    if (data == 0) {
        return 0U;
    }

    for (uint8_t index = 0U; index < length; index++) {
        crc = SerialProtocol::Crc8Update(crc, data[index]);
    }

    return crc;
}

}  // namespace protocol
