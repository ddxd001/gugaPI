#include "drivers/uart_control.h"

#include "ti_msp_dl_config.h"

namespace {

constexpr uint8_t kSof = 0xAA;
constexpr uint8_t kCmdRead = 0x01;
constexpr uint8_t kCmdWrite = 0x02;
constexpr uint8_t kCmdHeartbeat = 0x03;
constexpr uint8_t kResponseFlag = 0x80;
constexpr uint8_t kStatusOk = 0x00;
constexpr uint8_t kStatusError = 0x01;
constexpr uint8_t kMaxPayloadLength = 32;
constexpr uint16_t kRxBufferSize = 128;
constexpr bool kEnableEchoTest = false;

enum class ParserState : uint8_t {
    WaitSof,
    Cmd,
    Reg,
    Len,
    Data,
    Crc,
};

MotorRegisterMap* g_registers = nullptr;
volatile bool g_writeActivity = false;

#if defined(UART_CONTROL_INST)
volatile uint16_t g_rxHead = 0;
volatile uint16_t g_rxTail = 0;
volatile uint32_t g_rxDropped = 0;
uint8_t g_rxBuffer[kRxBufferSize];

ParserState g_state = ParserState::WaitSof;
uint8_t g_cmd = 0;
uint8_t g_reg = 0;
uint8_t g_len = 0;
uint8_t g_dataIndex = 0;
uint8_t g_data[kMaxPayloadLength];

uint16_t nextRxIndex(uint16_t index)
{
    ++index;
    if (index >= kRxBufferSize) {
        index = 0;
    }
    return index;
}

void pushRxFromIsr(uint8_t value)
{
    const uint16_t next = nextRxIndex(g_rxHead);
    if (next == g_rxTail) {
        ++g_rxDropped;
        return;
    }

    g_rxBuffer[g_rxHead] = value;
    g_rxHead = next;
}

bool popRx(uint8_t* value)
{
    if ((value == nullptr) || (g_rxHead == g_rxTail)) {
        return false;
    }

    *value = g_rxBuffer[g_rxTail];
    g_rxTail = nextRxIndex(g_rxTail);
    return true;
}

uint8_t crc8Update(uint8_t crc, uint8_t value)
{
    crc ^= value;
    for (uint8_t bit = 0; bit < 8; ++bit) {
        if ((crc & 0x80U) != 0U) {
            crc = static_cast<uint8_t>((crc << 1U) ^ 0x07U);
        } else {
            crc = static_cast<uint8_t>(crc << 1U);
        }
    }
    return crc;
}

uint8_t crc8(const uint8_t* data, uint8_t length)
{
    uint8_t crc = 0;
    for (uint8_t index = 0; index < length; ++index) {
        crc = crc8Update(crc, data[index]);
    }
    return crc;
}

bool waitTxSpace(void)
{
    uint32_t timeout = 100000;
    while (DL_UART_Main_isTXFIFOFull(UART_CONTROL_INST)) {
        if (timeout == 0U) {
            return false;
        }
        --timeout;
    }
    return true;
}

void sendByte(uint8_t value)
{
    if (waitTxSpace()) {
        DL_UART_Main_transmitData(UART_CONTROL_INST, value);
    }
}

void sendFrame(uint8_t cmd, uint8_t reg, const uint8_t* data, uint8_t length)
{
    uint8_t header[4] = { kSof, static_cast<uint8_t>(cmd | kResponseFlag), reg, length };
    uint8_t crc = 0;

    delay_cycles(32000);

    for (uint8_t value : header) {
        sendByte(value);
        crc = crc8Update(crc, value);
    }

    for (uint8_t index = 0; index < length; ++index) {
        const uint8_t value = (data == nullptr) ? 0 : data[index];
        sendByte(value);
        crc = crc8Update(crc, value);
    }

    sendByte(crc);
}

bool isRangeValid(uint8_t reg, uint8_t length)
{
    if ((g_registers == nullptr) || (length > kMaxPayloadLength)) {
        return false;
    }

    return (reg < MotorProtocol::kRegisterCount) &&
           (length <= (MotorProtocol::kRegisterCount - reg));
}

void handleFrame(void)
{
    if (g_registers == nullptr) {
        uint8_t status = kStatusError;
        sendFrame(g_cmd, g_reg, &status, 1);
        return;
    }

    if (g_cmd == kCmdHeartbeat) {
        uint8_t status = kStatusOk;
        sendFrame(g_cmd, 0, &status, 1);
        return;
    }

    if ((g_cmd == kCmdRead) && (g_len > 0U) && isRangeValid(g_reg, g_len)) {
        uint8_t response[kMaxPayloadLength];
        for (uint8_t index = 0; index < g_len; ++index) {
            response[index] = g_registers->read(static_cast<uint8_t>(g_reg + index));
        }
        sendFrame(g_cmd, g_reg, response, g_len);
        return;
    }

    if ((g_cmd == kCmdWrite) && (g_len > 0U) && isRangeValid(g_reg, g_len)) {
        for (uint8_t index = 0; index < g_len; ++index) {
            g_registers->write(static_cast<uint8_t>(g_reg + index), g_data[index]);
        }
        g_writeActivity = true;
        uint8_t status = kStatusOk;
        sendFrame(g_cmd, g_reg, &status, 1);
        return;
    }

    uint8_t status = kStatusError;
    sendFrame(g_cmd, g_reg, &status, 1);
}

void resetParser(void)
{
    g_state = ParserState::WaitSof;
    g_cmd = 0;
    g_reg = 0;
    g_len = 0;
    g_dataIndex = 0;
}

void processByte(uint8_t value)
{
    switch (g_state) {
        case ParserState::WaitSof:
            if (value == kSof) {
                g_state = ParserState::Cmd;
            }
            break;

        case ParserState::Cmd:
            g_cmd = value;
            g_state = ParserState::Reg;
            break;

        case ParserState::Reg:
            g_reg = value;
            g_state = ParserState::Len;
            break;

        case ParserState::Len:
            g_len = value;
            g_dataIndex = 0;
            if (g_len > kMaxPayloadLength) {
                resetParser();
            } else if ((g_len == 0U) || (g_cmd == kCmdRead) || (g_cmd == kCmdHeartbeat)) {
                g_state = ParserState::Crc;
            } else {
                g_state = ParserState::Data;
            }
            break;

        case ParserState::Data:
            g_data[g_dataIndex] = value;
            ++g_dataIndex;
            if (g_dataIndex >= g_len) {
                g_state = ParserState::Crc;
            }
            break;

        case ParserState::Crc: {
            uint8_t frame[kMaxPayloadLength + 4];
            frame[0] = kSof;
            frame[1] = g_cmd;
            frame[2] = g_reg;
            frame[3] = g_len;
            for (uint8_t index = 0; index < g_len; ++index) {
                frame[4U + index] = g_data[index];
            }

            if (crc8(frame, static_cast<uint8_t>(g_len + 4U)) == value) {
                handleFrame();
            }
            resetParser();
            break;
        }

        default:
            resetParser();
            break;
    }
}
#endif

}  // namespace

void UartControl_init(MotorRegisterMap* registers)
{
    g_registers = registers;
    g_writeActivity = false;

#if defined(UART_CONTROL_INST)
    g_rxHead = 0;
    g_rxTail = 0;
    g_rxDropped = 0;
    resetParser();

    NVIC_DisableIRQ(UART_CONTROL_INST_INT_IRQN);
    DL_UART_Main_disableInterrupt(UART_CONTROL_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    DL_UART_Main_clearInterruptStatus(UART_CONTROL_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    NVIC_ClearPendingIRQ(UART_CONTROL_INST_INT_IRQN);
#endif
}

void UartControl_process(void)
{
#if defined(UART_CONTROL_INST)
    while (!DL_UART_Main_isRXFIFOEmpty(UART_CONTROL_INST)) {
        const uint8_t value = DL_UART_Main_receiveData(UART_CONTROL_INST);
        if (kEnableEchoTest) {
            sendByte(value);
        } else {
            processByte(value);
        }
    }
    DL_UART_Main_clearInterruptStatus(UART_CONTROL_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);

    if (kEnableEchoTest) {
        return;
    }

    uint8_t value = 0;
    while (popRx(&value)) {
        processByte(value);
    }
#endif
}

bool UartControl_consumeWriteActivity(void)
{
    const bool activity = g_writeActivity;
    g_writeActivity = false;
    return activity;
}

#if defined(UART_CONTROL_INST)
extern "C" void UART_CONTROL_INST_IRQHandler(void)
{
    (void) DL_UART_Main_getPendingInterrupt(UART_CONTROL_INST);

    while (!DL_UART_Main_isRXFIFOEmpty(UART_CONTROL_INST)) {
        const uint8_t value = DL_UART_Main_receiveData(UART_CONTROL_INST);
        if (kEnableEchoTest) {
            sendByte(value);
        } else {
            pushRxFromIsr(value);
        }
    }

    DL_UART_Main_clearInterruptStatus(UART_CONTROL_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
}
#endif
