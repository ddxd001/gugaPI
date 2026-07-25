#include "board/board_i2c_target.h"

#include "board/board_motor_pins.h"

namespace board {
namespace {

constexpr uint8_t kRxBufferLength =
    static_cast<uint8_t>(protocol::kMaxPayloadLength + 1U);
constexpr uint8_t kWriteQueueLength = 8U;
constexpr uint32_t kErrorRxOverflow = 0x00000001U;
constexpr uint32_t kErrorDroppedWrite = 0x00000002U;
constexpr uint32_t kErrorTxUnderflow = 0x00000004U;
constexpr uint32_t kErrorArbitrationLost = 0x00000008U;
constexpr uint32_t kErrorInterruptOverflow = 0x00000010U;
constexpr uint32_t kErrorBusStuckRecovery = 0x00000020U;
constexpr uint32_t kBusStuckRecoveryMs = 50U;

const protocol::RegisterMap *g_registers = 0;
volatile uint8_t g_selected_reg = 0U;
volatile uint8_t g_rx_count = 0U;
volatile uint8_t g_rx_buffer[kRxBufferLength];
volatile bool g_rx_overflow_current = false;
volatile uint8_t g_write_head = 0U;
volatile uint8_t g_write_tail = 0U;
volatile uint8_t g_write_count = 0U;
volatile uint8_t g_write_reg[kWriteQueueLength];
volatile uint8_t g_write_length[kWriteQueueLength];
volatile uint8_t g_write_data[kWriteQueueLength][protocol::kMaxPayloadLength];
volatile uint32_t g_dropped_writes = 0U;
volatile uint32_t g_error_flags = 0U;
uint32_t g_sda_low_since_ms = 0U;
uint32_t g_recovery_count = 0U;
uint8_t g_i2c_address = kI2cTargetAddress;
bool g_sda_low_tracking = false;

void ResetRxTransaction(void)
{
    g_rx_count = 0U;
    g_rx_overflow_current = false;
}

uint8_t ReadSelectedRegister(void)
{
    const uint8_t reg = g_selected_reg;
    const uint8_t value = (g_registers == 0) ? 0U : g_registers->Read(reg);

    if (g_selected_reg != 0xFFU) {
        g_selected_reg = static_cast<uint8_t>(g_selected_reg + 1U);
    }

    return value;
}

void FillTxFifo(void)
{
    while (!DL_I2C_isTargetTXFIFOFull(BOARD_I2C_INST)) {
        const uint8_t value = ReadSelectedRegister();
        if (!DL_I2C_transmitTargetDataCheck(BOARD_I2C_INST, value)) {
            break;
        }
    }
}

void StoreRxByte(uint8_t value)
{
    if (g_rx_count == 0U) {
        g_selected_reg = value;
    }

    if (g_rx_count < kRxBufferLength) {
        g_rx_buffer[g_rx_count] = value;
        g_rx_count = static_cast<uint8_t>(g_rx_count + 1U);
    } else {
        g_rx_overflow_current = true;
        g_error_flags |= kErrorRxOverflow;
    }
}

void DrainRxFifo(void)
{
    uint8_t value = 0U;

    while (DL_I2C_receiveTargetDataCheck(BOARD_I2C_INST, &value)) {
        StoreRxByte(value);
    }
}

void QueueWriteFromRx(void)
{
    if ((g_rx_count <= 1U) || g_rx_overflow_current) {
        return;
    }

    if (g_write_count >= kWriteQueueLength) {
        g_dropped_writes++;
        g_error_flags |= kErrorDroppedWrite;
        return;
    }

    const uint8_t slot = g_write_tail;
    const uint8_t length = static_cast<uint8_t>(g_rx_count - 1U);
    g_write_reg[slot] = g_rx_buffer[0];
    g_write_length[slot] = length;
    for (uint8_t i = 0U; i < length; i++) {
        g_write_data[slot][i] = g_rx_buffer[static_cast<uint8_t>(i + 1U)];
    }
    g_write_tail = static_cast<uint8_t>((g_write_tail + 1U) % kWriteQueueLength);
    g_write_count++;
}

void ResetTargetFifos(void)
{
    DL_I2C_flushTargetRXFIFO(BOARD_I2C_INST);
    DL_I2C_flushTargetTXFIFO(BOARD_I2C_INST);
}

void ConfigureI2cPinsWithPullUps(void)
{
    DL_GPIO_initPeripheralInputFunctionFeatures(BOARD_I2C_SDA_IOMUX,
                                                BOARD_I2C_SDA_IOMUX_FUNC,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_PULL_UP,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(BOARD_I2C_SCL_IOMUX,
                                                BOARD_I2C_SCL_IOMUX_FUNC,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_PULL_UP,
                                                DL_GPIO_HYSTERESIS_DISABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(BOARD_I2C_SDA_IOMUX);
    DL_GPIO_enableHiZ(BOARD_I2C_SCL_IOMUX);
}

void ConfigureTargetPeripheral(void)
{
    DL_I2C_disableTarget(BOARD_I2C_INST);
    ConfigureI2cPinsWithPullUps();
    DL_I2C_setTargetAddressingMode(BOARD_I2C_INST,
                                   DL_I2C_TARGET_ADDRESSING_MODE_7_BIT);
    DL_I2C_setTargetOwnAddress(BOARD_I2C_INST, g_i2c_address);
    DL_I2C_enableTargetOwnAddress(BOARD_I2C_INST);
    ResetTargetFifos();
    DL_I2C_enableInterrupt(BOARD_I2C_INST,
                           DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER |
                           DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
                           DL_I2C_INTERRUPT_TARGET_RXFIFO_OVERFLOW |
                           DL_I2C_INTERRUPT_TARGET_TXFIFO_UNDERFLOW |
                           DL_I2C_INTERRUPT_TARGET_ARBITRATION_LOST |
                           DL_I2C_INTERRUPT_TARGET_START |
                           DL_I2C_INTERRUPT_TARGET_STOP |
                           DL_I2C_TARGET_INTERRUPT_OVERFLOW);
    DL_I2C_enableTarget(BOARD_I2C_INST);
}

void RecoverTargetPeripheral(void)
{
    NVIC_DisableIRQ(BOARD_I2C_IRQN);
    DL_I2C_disableTarget(BOARD_I2C_INST);
    DL_I2C_reset(BOARD_I2C_INST);
    DL_I2C_enablePower(BOARD_I2C_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    SYSCFG_DL_I2C_CONTROL_init();

    ResetRxTransaction();
    g_write_head = 0U;
    g_write_tail = 0U;
    g_write_count = 0U;
    ConfigureTargetPeripheral();

    g_error_flags |= kErrorBusStuckRecovery;
    g_recovery_count++;
    g_sda_low_tracking = false;
    NVIC_ClearPendingIRQ(BOARD_I2C_IRQN);
    NVIC_EnableIRQ(BOARD_I2C_IRQN);
}

}  // namespace

void BoardI2cTarget_Init(const protocol::RegisterMap *registers)
{
    g_registers = registers;
    g_selected_reg = 0U;
    ResetRxTransaction();
    g_write_head = 0U;
    g_write_tail = 0U;
    g_write_count = 0U;
    g_dropped_writes = 0U;
    g_error_flags = 0U;
    g_sda_low_since_ms = 0U;
    g_recovery_count = 0U;
    g_sda_low_tracking = false;

    ConfigureTargetPeripheral();
    NVIC_EnableIRQ(BOARD_I2C_IRQN);
}

bool BoardI2cTarget_Process(uint32_t now_ms)
{
    const bool scl_high =
        (DL_GPIO_readPins(BOARD_I2C_SCL_PORT, BOARD_I2C_SCL_PIN) != 0U);
    const bool sda_high =
        (DL_GPIO_readPins(BOARD_I2C_SDA_PORT, BOARD_I2C_SDA_PIN) != 0U);

    if ((!scl_high) || sda_high) {
        g_sda_low_tracking = false;
        return false;
    }

    if (!g_sda_low_tracking) {
        g_sda_low_since_ms = now_ms;
        g_sda_low_tracking = true;
        return false;
    }

    if ((now_ms - g_sda_low_since_ms) < kBusStuckRecoveryMs) {
        return false;
    }

    RecoverTargetPeripheral();
    return true;
}

bool BoardI2cTarget_TakeWrite(BoardI2cTargetWrite *write)
{
    if (write == 0) {
        return false;
    }

    const uint32_t irq_enabled = NVIC_GetEnableIRQ(BOARD_I2C_IRQN);
    NVIC_DisableIRQ(BOARD_I2C_IRQN);

    const bool available = (g_write_count > 0U);
    if (available) {
        const uint8_t slot = g_write_head;
        write->reg = g_write_reg[slot];
        write->length = g_write_length[slot];
        for (uint8_t i = 0U; i < write->length; i++) {
            write->data[i] = g_write_data[slot][i];
        }
        g_write_head =
            static_cast<uint8_t>((g_write_head + 1U) % kWriteQueueLength);
        g_write_count--;
    }

    if (irq_enabled != 0U) {
        NVIC_EnableIRQ(BOARD_I2C_IRQN);
    }

    return available;
}

uint8_t BoardI2cTarget_Address(void)
{
    return g_i2c_address;
}

void BoardI2cTarget_SetAddress(uint8_t address)
{
    g_i2c_address = address;
    DL_I2C_disableTarget(BOARD_I2C_INST);
    DL_I2C_setTargetOwnAddress(BOARD_I2C_INST, address);
    DL_I2C_enableTarget(BOARD_I2C_INST);
}

uint32_t BoardI2cTarget_DroppedWrites(void)
{
    return g_dropped_writes;
}

uint32_t BoardI2cTarget_ErrorFlags(void)
{
    return g_error_flags;
}

uint32_t BoardI2cTarget_RecoveryCount(void)
{
    return g_recovery_count;
}

void BoardI2cTarget_IrqHandler(void)
{
    switch (DL_I2C_getPendingInterrupt(BOARD_I2C_INST)) {
    case DL_I2C_IIDX_TARGET_START:
        DrainRxFifo();
        ResetRxTransaction();
        DL_I2C_flushTargetTXFIFO(BOARD_I2C_INST);
        break;

    case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
    case DL_I2C_IIDX_TARGET_RXFIFO_FULL:
    case DL_I2C_IIDX_TARGET_RX_DONE:
        DrainRxFifo();
        break;

    case DL_I2C_IIDX_TARGET_TXFIFO_TRIGGER:
    case DL_I2C_IIDX_TARGET_TXFIFO_EMPTY:
        FillTxFifo();
        break;

    case DL_I2C_IIDX_TARGET_STOP:
        DrainRxFifo();
        QueueWriteFromRx();
        ResetRxTransaction();
        break;

    case DL_I2C_IIDX_TARGET_TXFIFO_UNDERFLOW:
        g_error_flags |= kErrorTxUnderflow;
        break;

    case DL_I2C_IIDX_TARGET_RXFIFO_OVERFLOW:
        g_error_flags |= kErrorRxOverflow;
        DL_I2C_flushTargetRXFIFO(BOARD_I2C_INST);
        break;

    case DL_I2C_IIDX_TARGET_ARBITRATION_LOST:
        g_error_flags |= kErrorArbitrationLost;
        break;

    case DL_I2C_IIDX_INTERRUPT_OVERFLOW:
        g_error_flags |= kErrorInterruptOverflow;
        break;

    default:
        break;
    }
}

}  // namespace board

extern "C" void I2C_CONTROL_INST_IRQHandler(void)
{
    board::BoardI2cTarget_IrqHandler();
}
