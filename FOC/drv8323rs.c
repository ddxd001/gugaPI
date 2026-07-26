#include "drv8323rs.h"

#include "foc_config.h"
#include "ti_msp_dl_config.h"

enum {
    DRV_REG_FAULT_STATUS_1 = 0x00,
    DRV_REG_FAULT_STATUS_2 = 0x01,
    DRV_REG_DRIVER_CONTROL = 0x02,
    DRV_REG_GATE_DRIVE_HS  = 0x03,
    DRV_REG_GATE_DRIVE_LS  = 0x04,
    DRV_REG_OCP_CONTROL    = 0x05,
    DRV_REG_CSA_CONTROL    = 0x06,
};

/*
 * Conservative bring-up settings:
 * - 3x PWM, INHx phase PWM and INLx half-bridge enables
 * - 60 mA source / 120 mA sink gate current
 * - 1 us TDRIVE
 * - latched OCP, 100 ns extra dead time, 4 us deglitch, 0.75 V VDS limit
 */
#define DRV_DRIVER_CONTROL_3X_PWM      (0x0020U)
#define DRV_GATE_DRIVE_HS_BRINGUP      FOC_CFG_DRV_GATE_DRIVE_HS
#define DRV_GATE_DRIVE_LS_BRINGUP      FOC_CFG_DRV_GATE_DRIVE_LS
#define DRV_OCP_CONTROL_BRINGUP        FOC_CFG_DRV_OCP_CONTROL
/* VREF/2 bidirectional sensing, 20 V/V gain, 1 V sense-OCP threshold. */
#define DRV_CSA_CONTROL_CURRENT_SENSE  FOC_CFG_DRV_CSA_CONTROL
#define DRV_REGISTER_DATA_MASK         (0x07FFU)

static void delayUs(uint32_t microseconds)
{
    delay_cycles((CPUCLK_FREQ / 1000000U) * microseconds);
}

static uint16_t spiTransfer(uint16_t transmitWord)
{
    uint16_t receiveWord;

    while (!DL_SPI_isRXFIFOEmpty(DRV_SPI_INST)) {
        (void) DL_SPI_receiveData16(DRV_SPI_INST);
    }

    DL_GPIO_clearPins(MOTOR_CTRL_PORT, MOTOR_CTRL_SPI_CS_N_PIN);
    delay_cycles(4);

    DL_SPI_transmitDataBlocking16(DRV_SPI_INST, transmitWord);
    while (DL_SPI_isBusy(DRV_SPI_INST)) {
    }
    receiveWord = DL_SPI_receiveDataBlocking16(DRV_SPI_INST);

    delay_cycles(4);
    DL_GPIO_setPins(MOTOR_CTRL_PORT, MOTOR_CTRL_SPI_CS_N_PIN);

    return receiveWord & DRV_REGISTER_DATA_MASK;
}

static uint16_t writeRegister(uint8_t address, uint16_t data)
{
    uint16_t frame = (((uint16_t) address & 0x0FU) << 11) |
                     (data & DRV_REGISTER_DATA_MASK);
    return spiTransfer(frame);
}

uint16_t DRV8323RS_readRegister(uint8_t address)
{
    uint16_t frame = 0x8000U | (((uint16_t) address & 0x0FU) << 11);
    return spiTransfer(frame);
}

void DRV8323RS_setEnabled(bool enabled)
{
    if (enabled) {
        DL_GPIO_setPins(MOTOR_CTRL_PORT, MOTOR_CTRL_ENABLE_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_CTRL_PORT, MOTOR_CTRL_ENABLE_PIN);
    }
}

void DRV8323RS_setCurrentSenseCalibration(bool enabled)
{
    if (enabled) {
        DL_GPIO_setPins(MOTOR_CTRL_PORT, MOTOR_CTRL_CAL_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_CTRL_PORT, MOTOR_CTRL_CAL_PIN);
    }
}

bool DRV8323RS_isFaultActive(void)
{
    return (DL_GPIO_readPins(DRV_FAULT_PORT, DRV_FAULT_NFAULT_PIN) == 0U);
}

DRV8323RS_FaultStatus DRV8323RS_readFaults(void)
{
    DRV8323RS_FaultStatus status;
    status.faultStatus1 = DRV8323RS_readRegister(DRV_REG_FAULT_STATUS_1);
    status.faultStatus2 = DRV8323RS_readRegister(DRV_REG_FAULT_STATUS_2);
    return status;
}

bool DRV8323RS_clearFaults(void)
{
    (void) writeRegister(
        DRV_REG_DRIVER_CONTROL, DRV_DRIVER_CONTROL_3X_PWM | 0x0001U);
    delayUs(10);
    return !DRV8323RS_isFaultActive();
}

bool DRV8323RS_initialize(void)
{
    bool registerCheck;

    DRV8323RS_setEnabled(false);
    DL_GPIO_clearPins(MOTOR_CTRL_PORT, MOTOR_CTRL_CAL_PIN);
    delayUs(100);

    DRV8323RS_setEnabled(true);
    delayUs(1200); /* tWAKE is 1 ms maximum. */

    (void) writeRegister(DRV_REG_DRIVER_CONTROL, DRV_DRIVER_CONTROL_3X_PWM);
    (void) writeRegister(DRV_REG_GATE_DRIVE_HS, DRV_GATE_DRIVE_HS_BRINGUP);
    (void) writeRegister(DRV_REG_GATE_DRIVE_LS, DRV_GATE_DRIVE_LS_BRINGUP);
    (void) writeRegister(DRV_REG_OCP_CONTROL, DRV_OCP_CONTROL_BRINGUP);
    (void) writeRegister(
        DRV_REG_CSA_CONTROL, DRV_CSA_CONTROL_CURRENT_SENSE);

    (void) DRV8323RS_clearFaults();

    registerCheck =
        (DRV8323RS_readRegister(DRV_REG_DRIVER_CONTROL) ==
            DRV_DRIVER_CONTROL_3X_PWM) &&
        (DRV8323RS_readRegister(DRV_REG_GATE_DRIVE_HS) ==
            DRV_GATE_DRIVE_HS_BRINGUP) &&
        (DRV8323RS_readRegister(DRV_REG_GATE_DRIVE_LS) ==
            DRV_GATE_DRIVE_LS_BRINGUP) &&
        (DRV8323RS_readRegister(DRV_REG_OCP_CONTROL) ==
            DRV_OCP_CONTROL_BRINGUP) &&
        (DRV8323RS_readRegister(DRV_REG_CSA_CONTROL) ==
            DRV_CSA_CONTROL_CURRENT_SENSE);

    return registerCheck && !DRV8323RS_isFaultActive();
}
