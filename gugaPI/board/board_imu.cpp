#include "board/board_imu.h"

#include "board/board_pins.h"

namespace board {
namespace {

static const uint8_t kSpiReadMask = 0x80U;
static const uint8_t kLis3mdlAutoIncrementMask = 0x40U;
static const uint8_t kLis3mdlWhoAmIRegister = 0x0FU;
static const uint32_t kSelectDelayCycles = 32U;
static const uint32_t kWiggleDelayCycles = 64U;

static bool g_imuReady = false;

DL_SPI_FRAME_FORMAT SpiModeToFrameFormat(uint8_t mode)
{
    switch (mode) {
        case 0U:
            return DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0;
        case 1U:
            return DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA1;
        case 2U:
            return DL_SPI_FRAME_FORMAT_MOTO4_POL1_PHA0;
        default:
            return DL_SPI_FRAME_FORMAT_MOTO4_POL1_PHA1;
    }
}

void DelaySmall(void)
{
    for (volatile uint32_t i = 0U; i < kSelectDelayCycles; i++) {
    }
}

void DelayWiggle(void)
{
    for (volatile uint32_t i = 0U; i < kWiggleDelayCycles; i++) {
    }
}

void DeselectAll(void)
{
    DL_GPIO_setPins(BOARD_IMU_ICM45686_CS_PORT,
                    BOARD_IMU_ICM45686_CS_PIN | BOARD_IMU_LIS3MDL_CS_PIN);
}

void SelectIcm45686(void)
{
    DeselectAll();
    DelaySmall();
    DL_GPIO_clearPins(BOARD_IMU_ICM45686_CS_PORT, BOARD_IMU_ICM45686_CS_PIN);
    DelaySmall();
}

void SelectLis3mdl(void)
{
    DeselectAll();
    DelaySmall();
    DL_GPIO_clearPins(BOARD_IMU_LIS3MDL_CS_PORT, BOARD_IMU_LIS3MDL_CS_PIN);
    DelaySmall();
}

drivers::DriverStatus TransferByte(uint8_t tx, uint8_t *rx)
{
    uint32_t timeout = BOARD_IMU_SPI_TIMEOUT_ITERATIONS;
    uint8_t received = 0U;

    if (rx == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    while (!DL_SPI_transmitDataCheck8(BOARD_IMU_SPI_INST, tx)) {
        if (timeout == 0U) {
            return drivers::DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    timeout = BOARD_IMU_SPI_TIMEOUT_ITERATIONS;
    while (!DL_SPI_receiveDataCheck8(BOARD_IMU_SPI_INST, &received)) {
        if (timeout == 0U) {
            return drivers::DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    *rx = received;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus ReadRegisterWithCommand(uint8_t command, uint8_t *value)
{
    uint8_t ignored = 0U;
    drivers::DriverStatus status = TransferByte(command, &ignored);

    if (status != drivers::DRIVER_OK) {
        return status;
    }

    return TransferByte(0x00U, value);
}

void DrainRxFifo(void)
{
    uint8_t ignored = 0U;

    while (DL_SPI_receiveDataCheck8(BOARD_IMU_SPI_INST, &ignored)) {
    }
}

} /* namespace */

drivers::DriverStatus Board_ImuInit(void)
{
    DL_SPI_disable(BOARD_IMU_SPI_INST);
    DL_SPI_setFrameFormat(BOARD_IMU_SPI_INST, SpiModeToFrameFormat(3U));
    DL_SPI_enable(BOARD_IMU_SPI_INST);

    DL_GPIO_setPins(BOARD_IMU_ICM45686_CS_PORT,
                    BOARD_IMU_ICM45686_CS_PIN | BOARD_IMU_LIS3MDL_CS_PIN);
    DL_GPIO_enableOutput(BOARD_IMU_ICM45686_CS_PORT,
                         BOARD_IMU_ICM45686_CS_PIN | BOARD_IMU_LIS3MDL_CS_PIN);
    DeselectAll();
    g_imuReady = true;
    return drivers::DRIVER_OK;
}

bool Board_ImuIsReady(void)
{
    return g_imuReady;
}

drivers::DriverStatus Board_ImuSetChipSelectDebug(bool icm_output_enable,
                                                  bool icm_high,
                                                  bool lis_output_enable,
                                                  bool lis_high)
{
    if (!g_imuReady) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    if (icm_high) {
        DL_GPIO_setPins(BOARD_IMU_ICM45686_CS_PORT, BOARD_IMU_ICM45686_CS_PIN);
    } else {
        DL_GPIO_clearPins(BOARD_IMU_ICM45686_CS_PORT, BOARD_IMU_ICM45686_CS_PIN);
    }

    if (lis_high) {
        DL_GPIO_setPins(BOARD_IMU_LIS3MDL_CS_PORT, BOARD_IMU_LIS3MDL_CS_PIN);
    } else {
        DL_GPIO_clearPins(BOARD_IMU_LIS3MDL_CS_PORT, BOARD_IMU_LIS3MDL_CS_PIN);
    }

    if (icm_output_enable) {
        DL_GPIO_enableOutput(BOARD_IMU_ICM45686_CS_PORT,
                             BOARD_IMU_ICM45686_CS_PIN);
    } else {
        DL_GPIO_disableOutput(BOARD_IMU_ICM45686_CS_PORT,
                              BOARD_IMU_ICM45686_CS_PIN);
    }

    if (lis_output_enable) {
        DL_GPIO_enableOutput(BOARD_IMU_LIS3MDL_CS_PORT,
                             BOARD_IMU_LIS3MDL_CS_PIN);
    } else {
        DL_GPIO_disableOutput(BOARD_IMU_LIS3MDL_CS_PORT,
                              BOARD_IMU_LIS3MDL_CS_PIN);
    }

    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_ImuSetSpiMode(uint8_t mode)
{
    if (!g_imuReady) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (mode > 3U) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    DeselectAll();
    DL_SPI_disable(BOARD_IMU_SPI_INST);
    DL_SPI_setFrameFormat(BOARD_IMU_SPI_INST, SpiModeToFrameFormat(mode));
    DL_SPI_enable(BOARD_IMU_SPI_INST);
    DrainRxFifo();

    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_ImuWiggleSpiPins(uint32_t loops)
{
    if (!g_imuReady) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (loops == 0U) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    DeselectAll();

    DL_GPIO_initDigitalOutput(BOARD_IMU_SPI_SCLK_IOMUX);
    DL_GPIO_initDigitalOutput(BOARD_IMU_SPI_PICO_IOMUX);
    DL_GPIO_initDigitalOutput(BOARD_IMU_SPI_POCI_IOMUX);
    DL_GPIO_enableOutput(BOARD_IMU_SPI_SCLK_PORT,
                         BOARD_IMU_SPI_SCLK_PIN | BOARD_IMU_SPI_PICO_PIN |
                             BOARD_IMU_SPI_POCI_PIN);

    for (uint32_t i = 0U; i < loops; i++) {
        DL_GPIO_setPins(BOARD_IMU_SPI_SCLK_PORT,
                        BOARD_IMU_SPI_SCLK_PIN | BOARD_IMU_SPI_PICO_PIN |
                            BOARD_IMU_SPI_POCI_PIN);
        DelayWiggle();
        DL_GPIO_clearPins(BOARD_IMU_SPI_SCLK_PORT,
                          BOARD_IMU_SPI_SCLK_PIN | BOARD_IMU_SPI_PICO_PIN |
                              BOARD_IMU_SPI_POCI_PIN);
        DelayWiggle();
    }

    DL_GPIO_disableOutput(BOARD_IMU_SPI_POCI_PORT, BOARD_IMU_SPI_POCI_PIN);
    DL_GPIO_initPeripheralOutputFunction(BOARD_IMU_SPI_SCLK_IOMUX,
                                         BOARD_IMU_SPI_SCLK_IOMUX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(BOARD_IMU_SPI_PICO_IOMUX,
                                         BOARD_IMU_SPI_PICO_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(BOARD_IMU_SPI_POCI_IOMUX,
                                        BOARD_IMU_SPI_POCI_IOMUX_FUNC);

    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_ImuSpiBurstIcm(uint32_t bytes, uint8_t value)
{
    if (!g_imuReady) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (bytes == 0U) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    DrainRxFifo();
    SelectIcm45686();

    for (uint32_t i = 0U; i < bytes; i++) {
        uint32_t timeout = BOARD_IMU_SPI_TIMEOUT_ITERATIONS;

        while (!DL_SPI_transmitDataCheck8(BOARD_IMU_SPI_INST, value)) {
            if (timeout == 0U) {
                DeselectAll();
                return drivers::DRIVER_ERROR_TIMEOUT;
            }
            timeout--;
        }

        DrainRxFifo();
    }

    uint32_t timeout = BOARD_IMU_SPI_TIMEOUT_ITERATIONS;
    while (DL_SPI_isBusy(BOARD_IMU_SPI_INST)) {
        if (timeout == 0U) {
            DeselectAll();
            return drivers::DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    DrainRxFifo();
    DeselectAll();
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_ImuGetLineStatus(BoardImuLineStatus *status)
{
    if (status == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    status->icm_cs_high =
        (DL_GPIO_readPins(BOARD_IMU_ICM45686_CS_PORT,
                          BOARD_IMU_ICM45686_CS_PIN) != 0U);
    status->lis_cs_high =
        (DL_GPIO_readPins(BOARD_IMU_LIS3MDL_CS_PORT,
                          BOARD_IMU_LIS3MDL_CS_PIN) != 0U);
    status->icm_cs_latch_high =
        ((BOARD_IMU_ICM45686_CS_PORT->DOUT31_0 &
          BOARD_IMU_ICM45686_CS_PIN) != 0U);
    status->lis_cs_latch_high =
        ((BOARD_IMU_LIS3MDL_CS_PORT->DOUT31_0 &
          BOARD_IMU_LIS3MDL_CS_PIN) != 0U);
    status->icm_cs_output_enabled =
        ((BOARD_IMU_ICM45686_CS_PORT->DOE31_0 &
          BOARD_IMU_ICM45686_CS_PIN) != 0U);
    status->lis_cs_output_enabled =
        ((BOARD_IMU_LIS3MDL_CS_PORT->DOE31_0 &
          BOARD_IMU_LIS3MDL_CS_PIN) != 0U);
    status->icm_int1_high =
        (DL_GPIO_readPins(BOARD_IMU_ICM45686_INT1_PORT,
                          BOARD_IMU_ICM45686_INT1_PIN) != 0U);
    status->icm_int2_fsync_high =
        (DL_GPIO_readPins(BOARD_IMU_ICM45686_INT2_FSYNC_PORT,
                          BOARD_IMU_ICM45686_INT2_FSYNC_PIN) != 0U);
    status->lis_drdy_high =
        (DL_GPIO_readPins(BOARD_IMU_LIS3MDL_DRDY_PORT,
                          BOARD_IMU_LIS3MDL_DRDY_PIN) != 0U);

    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_Lis3mdlReadRegister(uint8_t reg, uint8_t *value)
{
    if (!g_imuReady) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    SelectLis3mdl();
    const drivers::DriverStatus status = ReadRegisterWithCommand(
        (uint8_t) (kSpiReadMask | kLis3mdlAutoIncrementMask | (reg & 0x3FU)),
        value);
    DeselectAll();

    return status;
}

drivers::DriverStatus Board_Lis3mdlReadWhoAmI(uint8_t *value)
{
    return Board_Lis3mdlReadRegister(kLis3mdlWhoAmIRegister, value);
}

drivers::DriverStatus Board_Icm45686ReadRegister(uint8_t reg, uint8_t *value)
{
    if (!g_imuReady) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    SelectIcm45686();
    const drivers::DriverStatus status = ReadRegisterWithCommand(
        (uint8_t) (kSpiReadMask | reg),
        value);
    DeselectAll();

    return status;
}

} /* namespace board */
