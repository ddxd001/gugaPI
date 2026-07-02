#include "drivers/i2c_slave.h"

#include "ti_msp_dl_config.h"

namespace {

MotorRegisterMap* g_registers = nullptr;
volatile bool g_writeActivity = false;
#if defined(I2C_TARGET_INST)
volatile uint8_t g_selectedRegister = 0;
volatile bool g_haveSelectedRegister = false;
#endif

#if defined(I2C_TARGET_INST)
void fillTxFifo(void)
{
    if (g_registers == nullptr) {
        while (DL_I2C_transmitTargetDataCheck(I2C_TARGET_INST, 0x00) != false) {
        }
        return;
    }

    uint8_t reg = g_selectedRegister;
    while (DL_I2C_transmitTargetDataCheck(I2C_TARGET_INST, g_registers->read(reg)) != false) {
        if (reg < (MotorProtocol::kRegisterCount - 1U)) {
            ++reg;
        }
    }
    g_selectedRegister = reg;
}
#endif

}  // namespace

void I2cSlave_init(MotorRegisterMap* registers, uint8_t address)
{
    g_registers = registers;

#if defined(I2C_TARGET_INST)
    DL_I2C_disableTargetOwnAddress(I2C_TARGET_INST);
    DL_I2C_setTargetOwnAddress(I2C_TARGET_INST, address);
    DL_I2C_enableTargetOwnAddress(I2C_TARGET_INST);

    DL_I2C_flushTargetTXFIFO(I2C_TARGET_INST);
    DL_I2C_flushTargetRXFIFO(I2C_TARGET_INST);
    DL_I2C_enableInterrupt(I2C_TARGET_INST,
        DL_I2C_INTERRUPT_TARGET_START |
        DL_I2C_INTERRUPT_TARGET_STOP |
        DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER |
        DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER);
    NVIC_EnableIRQ(I2C_TARGET_INST_INT_IRQN);
#else
    (void) address;
#endif
}

bool I2cSlave_consumeWriteActivity(void)
{
    const bool activity = g_writeActivity;
    g_writeActivity = false;
    return activity;
}

#if defined(I2C_TARGET_INST)
extern "C" void I2C_TARGET_INST_IRQHandler(void)
{
    switch (DL_I2C_getPendingInterrupt(I2C_TARGET_INST)) {
        case DL_I2C_IIDX_TARGET_START:
            DL_I2C_flushTargetTXFIFO(I2C_TARGET_INST);
            g_haveSelectedRegister = false;
            break;

        case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
            while (DL_I2C_isTargetRXFIFOEmpty(I2C_TARGET_INST) != true) {
                const uint8_t byte = DL_I2C_receiveTargetData(I2C_TARGET_INST);
                if (!g_haveSelectedRegister) {
                    g_selectedRegister = byte;
                    g_haveSelectedRegister = true;
                } else if (g_registers != nullptr) {
                    g_registers->write(g_selectedRegister, byte);
                    g_writeActivity = true;
                    if (g_selectedRegister < (MotorProtocol::kRegisterCount - 1U)) {
                        ++g_selectedRegister;
                    }
                }
            }
            break;

        case DL_I2C_IIDX_TARGET_TXFIFO_TRIGGER:
            fillTxFifo();
            break;

        case DL_I2C_IIDX_TARGET_STOP:
            break;

        default:
            break;
    }
}
#endif
