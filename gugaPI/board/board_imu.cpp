#include "board/board_imu.h"

#include "board/board_pins.h"

namespace board {
namespace {

static const uint32_t kSelectDelayCycles = 32U;
static const uint32_t kWiggleDelayCycles = 64U;

/* The shared SPI bus can remain available for diagnostics even when the ICM
 * device is absent. Device readiness is tracked separately in g_icmCtx. */
static bool g_imuSpiReady = false;
static uint8_t g_imuSpiMode = 3U;

static const drivers::Icm45686Config kIcmConfig = {
    BOARD_IMU_SPI_INST,
    BOARD_IMU_ICM45686_CS_PORT,
    BOARD_IMU_ICM45686_CS_PIN,
    BOARD_IMU_SPI_TIMEOUT_ITERATIONS,
    drivers::ICM45686_ACCEL_FS_4G,
    drivers::ICM45686_GYRO_FS_1000DPS,
    drivers::ICM45686_ODR_200HZ,
    drivers::ICM45686_ODR_200HZ,
};

static drivers::Icm45686Context g_icmCtx = { &kIcmConfig, false };
static drivers::DriverStatus g_icmInitStatus =
    drivers::DRIVER_ERROR_NOT_INITIALIZED;

static const drivers::Lis3mdlConfig kLisConfig = {
    BOARD_IMU_SPI_INST,
    BOARD_IMU_LIS3MDL_CS_PORT,
    BOARD_IMU_LIS3MDL_CS_PIN,
    BOARD_IMU_ICM45686_CS_PORT,
    BOARD_IMU_ICM45686_CS_PIN,
    BOARD_IMU_SPI_TIMEOUT_ITERATIONS,
    drivers::LIS3MDL_FULL_SCALE_4_G,
    drivers::LIS3MDL_ODR_20_HZ,
    drivers::LIS3MDL_PERFORMANCE_ULTRA_HIGH,
    drivers::LIS3MDL_PERFORMANCE_ULTRA_HIGH,
    drivers::LIS3MDL_MODE_CONTINUOUS,
};

static drivers::Lis3mdlContext g_lisCtx = {
    &kLisConfig,
    false,
    drivers::LIS3MDL_FULL_SCALE_4_G,
    drivers::LIS3MDL_ODR_20_HZ,
    drivers::LIS3MDL_PERFORMANCE_ULTRA_HIGH,
    drivers::LIS3MDL_PERFORMANCE_ULTRA_HIGH,
    drivers::LIS3MDL_MODE_CONTINUOUS,
};
static drivers::DriverStatus g_lisInitStatus =
    drivers::DRIVER_ERROR_NOT_INITIALIZED;

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

void DrainRxFifo(void)
{
    uint8_t ignored = 0U;

    while (DL_SPI_receiveDataCheck8(BOARD_IMU_SPI_INST, &ignored)) {
    }
}

drivers::DriverStatus PrepareDeviceAccess(void)
{
    if (!g_imuSpiReady) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    DeselectAll();
    DL_GPIO_enableOutput(BOARD_IMU_ICM45686_CS_PORT,
                         BOARD_IMU_ICM45686_CS_PIN |
                             BOARD_IMU_LIS3MDL_CS_PIN);
    if (g_imuSpiMode == 3U) {
        return drivers::DRIVER_OK;
    }

    uint32_t timeout = BOARD_IMU_SPI_TIMEOUT_ITERATIONS;
    while (DL_SPI_isBusy(BOARD_IMU_SPI_INST)) {
        if (timeout == 0U) {
            return drivers::DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    DL_SPI_disable(BOARD_IMU_SPI_INST);
    DL_SPI_setFrameFormat(BOARD_IMU_SPI_INST, SpiModeToFrameFormat(3U));
    DL_SPI_enable(BOARD_IMU_SPI_INST);
    DrainRxFifo();
    g_imuSpiMode = 3U;
    return drivers::DRIVER_OK;
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
    g_imuSpiReady = true;
    g_imuSpiMode = 3U;

    /* Initialize both devices independently. A failure never clears shared
     * bus readiness or the other device's context, so either sensor remains
     * available for diagnostics and retry. Keep the ICM error first for
     * compatibility with callers that historically treated Board_ImuInit()
     * as the ICM startup result. */
    g_icmInitStatus = drivers::Icm45686_Init(&g_icmCtx, &kIcmConfig);
    DeselectAll();
    g_lisInitStatus = drivers::Lis3mdl_Init(&g_lisCtx, &kLisConfig);
    DeselectAll();

    return (g_icmInitStatus != drivers::DRIVER_OK) ?
           g_icmInitStatus : g_lisInitStatus;
}

bool Board_ImuIsReady(void)
{
    return g_imuSpiReady;
}

bool Board_ImuSpiIsReady(void)
{
    return Board_ImuIsReady();
}

drivers::DriverStatus Board_ImuSetChipSelectDebug(bool icm_output_enable,
                                                  bool icm_high,
                                                  bool lis_output_enable,
                                                  bool lis_high)
{
    if (!g_imuSpiReady) {
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
    if (!g_imuSpiReady) {
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
    g_imuSpiMode = mode;

    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_ImuWiggleSpiPins(uint32_t loops)
{
    if (!g_imuSpiReady) {
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
    if (!g_imuSpiReady) {
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

drivers::DriverStatus Board_ImuSpiSampleIcm(uint8_t tx,
                                            uint8_t *rx,
                                            uint32_t count)
{
    if (!g_imuSpiReady) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if ((rx == 0) || (count == 0U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    DrainRxFifo();
    SelectIcm45686();

    for (uint32_t i = 0U; i < count; i++) {
        const drivers::DriverStatus status = TransferByte(tx, &rx[i]);
        if (status != drivers::DRIVER_OK) {
            DeselectAll();
            return status;
        }
    }

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
    const drivers::DriverStatus prepare_status = PrepareDeviceAccess();
    if (prepare_status != drivers::DRIVER_OK) {
        return prepare_status;
    }
    return drivers::Lis3mdl_ProbeRegister(&kLisConfig, reg, value);
}

drivers::DriverStatus Board_Lis3mdlReadWhoAmI(uint8_t *value)
{
    const drivers::DriverStatus prepare_status = PrepareDeviceAccess();
    if (prepare_status != drivers::DRIVER_OK) {
        return prepare_status;
    }
    return drivers::Lis3mdl_ProbeWhoAmI(&kLisConfig, value);
}

drivers::DriverStatus Board_Lis3mdlInit(void)
{
    const drivers::DriverStatus prepare_status = PrepareDeviceAccess();
    if (prepare_status != drivers::DRIVER_OK) {
        g_lisInitStatus = prepare_status;
        return g_lisInitStatus;
    }
    g_lisInitStatus = drivers::Lis3mdl_Init(&g_lisCtx, &kLisConfig);
    return g_lisInitStatus;
}

bool Board_Lis3mdlIsReady(void)
{
    return drivers::Lis3mdl_IsReady(&g_lisCtx);
}

drivers::DriverStatus Board_Lis3mdlGetInitStatus(void)
{
    return g_lisInitStatus;
}

drivers::DriverStatus Board_Lis3mdlSetFullScale(uint8_t full_scale)
{
    const drivers::DriverStatus prepare_status = PrepareDeviceAccess();
    if (prepare_status != drivers::DRIVER_OK) {
        return prepare_status;
    }
    return drivers::Lis3mdl_SetFullScale(&g_lisCtx, full_scale);
}

drivers::DriverStatus Board_Lis3mdlSetOutputDataRate(uint8_t output_data_rate)
{
    const drivers::DriverStatus prepare_status = PrepareDeviceAccess();
    if (prepare_status != drivers::DRIVER_OK) {
        return prepare_status;
    }
    return drivers::Lis3mdl_SetOutputDataRate(&g_lisCtx, output_data_rate);
}

drivers::DriverStatus Board_Lis3mdlSetOperatingMode(uint8_t operating_mode)
{
    const drivers::DriverStatus prepare_status = PrepareDeviceAccess();
    if (prepare_status != drivers::DRIVER_OK) {
        return prepare_status;
    }
    return drivers::Lis3mdl_SetOperatingMode(&g_lisCtx, operating_mode);
}

drivers::DriverStatus Board_Lis3mdlIsDataReady(bool *ready)
{
    const drivers::DriverStatus prepare_status = PrepareDeviceAccess();
    if (prepare_status != drivers::DRIVER_OK) {
        return prepare_status;
    }
    return drivers::Lis3mdl_IsDataReady(&g_lisCtx, ready);
}

drivers::DriverStatus Board_Lis3mdlReadRaw(drivers::Lis3mdlRawData *data)
{
    const drivers::DriverStatus prepare_status = PrepareDeviceAccess();
    if (prepare_status != drivers::DRIVER_OK) {
        return prepare_status;
    }
    return drivers::Lis3mdl_ReadRaw(&g_lisCtx, data);
}

uint8_t Board_Lis3mdlGetFullScale(void)
{
    return drivers::Lis3mdl_GetFullScale(&g_lisCtx);
}

uint8_t Board_Lis3mdlGetOutputDataRate(void)
{
    return drivers::Lis3mdl_GetOutputDataRate(&g_lisCtx);
}

uint8_t Board_Lis3mdlGetOperatingMode(void)
{
    return drivers::Lis3mdl_GetOperatingMode(&g_lisCtx);
}

const drivers::Lis3mdlConfig *Board_Lis3mdlGetConfig(void)
{
    return &kLisConfig;
}

drivers::DriverStatus Board_Icm45686Init(void)
{
    const drivers::DriverStatus prepare_status = PrepareDeviceAccess();
    if (prepare_status != drivers::DRIVER_OK) {
        g_icmInitStatus = prepare_status;
        return g_icmInitStatus;
    }
    g_icmInitStatus = drivers::Icm45686_Init(&g_icmCtx, &kIcmConfig);
    return g_icmInitStatus;
}

bool Board_Icm45686IsReady(void)
{
    return drivers::Icm45686_IsReady(&g_icmCtx);
}

drivers::DriverStatus Board_Icm45686GetInitStatus(void)
{
    return g_icmInitStatus;
}

drivers::DriverStatus Board_Icm45686ReadRegister(uint8_t reg, uint8_t *value)
{
    const drivers::DriverStatus prepare_status = PrepareDeviceAccess();
    if (prepare_status != drivers::DRIVER_OK) {
        return prepare_status;
    }
    return drivers::Icm45686_ReadRegister(&g_icmCtx, reg, value);
}

drivers::DriverStatus Board_Icm45686WriteRegister(uint8_t reg, uint8_t value)
{
    const drivers::DriverStatus prepare_status = PrepareDeviceAccess();
    if (prepare_status != drivers::DRIVER_OK) {
        return prepare_status;
    }
    return drivers::Icm45686_WriteRegister(&g_icmCtx, reg, value);
}

drivers::DriverStatus Board_Icm45686ReadBurst(uint8_t reg,
                                              uint8_t *buf,
                                              uint16_t len)
{
    const drivers::DriverStatus prepare_status = PrepareDeviceAccess();
    if (prepare_status != drivers::DRIVER_OK) {
        return prepare_status;
    }
    return drivers::Icm45686_ReadBurst(&g_icmCtx, reg, buf, len);
}

drivers::DriverStatus Board_Icm45686ReadWhoAmI(uint8_t *value)
{
    const drivers::DriverStatus prepare_status = PrepareDeviceAccess();
    if (prepare_status != drivers::DRIVER_OK) {
        return prepare_status;
    }
    return drivers::Icm45686_ReadWhoAmI(&g_icmCtx, value);
}

drivers::DriverStatus Board_Icm45686ReadSensors(drivers::Icm45686SensorData *data)
{
    const drivers::DriverStatus prepare_status = PrepareDeviceAccess();
    if (prepare_status != drivers::DRIVER_OK) {
        return prepare_status;
    }
    return drivers::Icm45686_ReadSensors(&g_icmCtx, data);
}

const drivers::Icm45686Config *Board_Icm45686GetConfig(void)
{
    return &kIcmConfig;
}

} /* namespace board */
