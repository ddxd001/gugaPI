#include "drivers/icm45686/icm45686.h"

namespace drivers {
namespace {

/* Register addresses (ICM-45686, User Bank 0). */
static const uint8_t kRegAccelDataX1 = 0x00U;
static const uint8_t kRegPwrMgmt0    = 0x10U;
static const uint8_t kRegInt1Config0 = 0x16U;
static const uint8_t kRegInt1Config2 = 0x18U;
static const uint8_t kRegAccelConfig0 = 0x1BU;
static const uint8_t kRegGyroConfig0  = 0x1CU;
static const uint8_t kRegWhoAmI      = 0x72U;
static const uint8_t kRegMisc2       = 0x7FU;

static const uint8_t kSpiReadMask    = 0x80U;
static const uint8_t kSoftResetCmd   = 0x02U; /* REG_MISC2 bit1 SOFT_RST */
static const uint8_t kPwrGyroAccelLn = 0x0FU; /* GYRO_MODE=11 ACCEL_MODE=11 */
static const uint32_t kSelectDelayCycles = 32U;
static const uint32_t kResetPollRetries   = 300U;

/* ACCEL_UI_FS_SEL -> full scale in g (±value). */
uint32_t AccelFsG(uint8_t fs)
{
    switch (fs) {
    case ICM45686_ACCEL_FS_32G: return 32U;
    case ICM45686_ACCEL_FS_16G: return 16U;
    case ICM45686_ACCEL_FS_8G:  return 8U;
    case ICM45686_ACCEL_FS_4G:  return 4U;
    case ICM45686_ACCEL_FS_2G:  return 2U;
    default:                    return 0U;
    }
}

/* GYRO_UI_FS_SEL -> full scale in dps (±value). */
uint32_t GyroFsDps(uint8_t fs)
{
    switch (fs) {
    case ICM45686_GYRO_FS_4000DPS: return 4000U;
    case ICM45686_GYRO_FS_2000DPS: return 2000U;
    case ICM45686_GYRO_FS_1000DPS: return 1000U;
    case ICM45686_GYRO_FS_500DPS:  return 500U;
    case ICM45686_GYRO_FS_250DPS:  return 250U;
    case ICM45686_GYRO_FS_125DPS:  return 125U;
    default:                       return 0U;
    }
}

void DelaySmall(void)
{
    for (volatile uint32_t i = 0U; i < kSelectDelayCycles; i++) {
    }
}

void Select(const Icm45686Config *cfg)
{
    DL_GPIO_setPins(cfg->cs_port, cfg->cs_pin);
    DelaySmall();
    DL_GPIO_clearPins(cfg->cs_port, cfg->cs_pin);
    DelaySmall();
}

void Deselect(const Icm45686Config *cfg)
{
    DL_GPIO_setPins(cfg->cs_port, cfg->cs_pin);
}

void DrainRxFifo(SPI_Regs *spi)
{
    uint8_t ignored = 0U;
    while (DL_SPI_receiveDataCheck8(spi, &ignored)) {
    }
}

void WaitIdle(const Icm45686Config *cfg)
{
    uint32_t timeout = cfg->timeout_iterations;
    while (DL_SPI_isBusy(cfg->spi)) {
        if (timeout == 0U) {
            break;
        }
        timeout--;
    }
}

DriverStatus TransferByte(const Icm45686Config *cfg, uint8_t tx, uint8_t *rx)
{
    uint32_t timeout = cfg->timeout_iterations;
    uint8_t received = 0U;

    while (!DL_SPI_transmitDataCheck8(cfg->spi, tx)) {
        if (timeout == 0U) {
            return DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    timeout = cfg->timeout_iterations;
    while (!DL_SPI_receiveDataCheck8(cfg->spi, &received)) {
        if (timeout == 0U) {
            return DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    if (rx != 0) {
        *rx = received;
    }
    return DRIVER_OK;
}

int16_t CombineLe(uint8_t lo, uint8_t hi)
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(lo) |
        (static_cast<uint16_t>(hi) << 8U));
}

/* Write a register, then read it back to confirm the value stuck. The device
 * may reject writes for a short window after soft reset, so retry a few times. */
DriverStatus WriteVerify(Icm45686Context *ctx, uint8_t reg, uint8_t value)
{
    for (uint32_t attempt = 0U; attempt < 5U; attempt++) {
        DriverStatus status = Icm45686_WriteRegister(ctx, reg, value);
        if (status != DRIVER_OK) {
            continue;
        }
        uint8_t readback = 0U;
        status = Icm45686_ReadRegister(ctx, reg, &readback);
        if ((status == DRIVER_OK) && (readback == value)) {
            return DRIVER_OK;
        }
    }
    return DRIVER_ERROR;
}

} /* namespace */

DriverStatus Icm45686_SoftReset(Icm45686Context *ctx)
{
    if ((ctx == 0) || (ctx->config == 0) || (!ctx->initialized)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    DriverStatus status = Icm45686_WriteRegister(ctx, kRegMisc2, kSoftResetCmd);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    /* SOFT_RST self-clears on completion. Wait until REG_MISC2 bit1 clears
     * AND the device responds with the correct WHO_AM_I, so that subsequent
     * config writes are accepted reliably. */
    for (uint32_t i = 0U; i < kResetPollRetries; i++) {
        uint8_t misc2 = 0U;
        uint8_t who = 0U;
        if ((Icm45686_ReadRegister(ctx, kRegMisc2, &misc2) == DRIVER_OK) &&
            ((misc2 & 0x02U) == 0U) &&
            (Icm45686_ReadRegister(ctx, kRegWhoAmI, &who) == DRIVER_OK) &&
            (who == ICM45686_WHO_AM_I_VALUE)) {
            return DRIVER_OK;
        }
    }

    return DRIVER_ERROR_TIMEOUT;
}

DriverStatus Icm45686_ReadRegister(Icm45686Context *ctx,
                                   uint8_t reg,
                                   uint8_t *value)
{
    return Icm45686_ReadBurst(ctx, reg, value, 1U);
}

DriverStatus Icm45686_WriteRegister(Icm45686Context *ctx,
                                    uint8_t reg,
                                    uint8_t value)
{
    if ((ctx == 0) || (ctx->config == 0) || (!ctx->initialized)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    const Icm45686Config *cfg = ctx->config;
    DriverStatus status = DRIVER_OK;

    DrainRxFifo(cfg->spi);
    Select(cfg);
    status = TransferByte(cfg, static_cast<uint8_t>(reg & ~kSpiReadMask), 0);
    if (status == DRIVER_OK) {
        status = TransferByte(cfg, value, 0);
    }
    WaitIdle(cfg);
    Deselect(cfg);

    return status;
}

DriverStatus Icm45686_ReadBurst(Icm45686Context *ctx,
                                uint8_t reg,
                                uint8_t *buf,
                                uint16_t len)
{
    if ((ctx == 0) || (ctx->config == 0) || (!ctx->initialized)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if ((buf == 0) || (len == 0U)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const Icm45686Config *cfg = ctx->config;
    DriverStatus status = DRIVER_OK;

    DrainRxFifo(cfg->spi);
    Select(cfg);

    /* Address byte: MSB=1 for read. Received byte here is don't-care. */
    status = TransferByte(cfg, static_cast<uint8_t>(reg | kSpiReadMask), 0);
    for (uint16_t i = 0U; (i < len) && (status == DRIVER_OK); i++) {
        status = TransferByte(cfg, 0x00U, &buf[i]);
    }

    WaitIdle(cfg);
    Deselect(cfg);
    return status;
}

DriverStatus Icm45686_ReadWhoAmI(Icm45686Context *ctx, uint8_t *value)
{
    if (value == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }
    return Icm45686_ReadRegister(ctx, kRegWhoAmI, value);
}

DriverStatus Icm45686_ReadSensors(Icm45686Context *ctx, Icm45686SensorData *data)
{
    if (data == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    uint8_t buf[14];
    const DriverStatus status =
        Icm45686_ReadBurst(ctx, kRegAccelDataX1, buf, sizeof(buf));
    if (status != DRIVER_OK) {
        return status;
    }

    data->accel_x = CombineLe(buf[0], buf[1]);
    data->accel_y = CombineLe(buf[2], buf[3]);
    data->accel_z = CombineLe(buf[4], buf[5]);
    data->gyro_x  = CombineLe(buf[6], buf[7]);
    data->gyro_y  = CombineLe(buf[8], buf[9]);
    data->gyro_z  = CombineLe(buf[10], buf[11]);
    data->temp    = CombineLe(buf[12], buf[13]);
    return DRIVER_OK;
}

DriverStatus Icm45686_Init(Icm45686Context *ctx, const Icm45686Config *config)
{
    if ((ctx == 0) || (config == 0) || (config->spi == 0) ||
        (config->cs_port == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->config = config;
    ctx->initialized = true;

    DriverStatus status = Icm45686_SoftReset(ctx);
    if (status != DRIVER_OK) {
        ctx->initialized = false;
        return status;
    }

    uint8_t who = 0U;
    status = Icm45686_ReadWhoAmI(ctx, &who);
    if ((status != DRIVER_OK) || (who != ICM45686_WHO_AM_I_VALUE)) {
        ctx->initialized = false;
        return (status != DRIVER_OK) ? status : DRIVER_ERROR;
    }

    /* INT1: push-pull (no external pull-up needed), DRDY routing disabled
     * for now. Settings may only change while all interrupts are disabled. */
    status = Icm45686_WriteRegister(ctx, kRegInt1Config0, 0x00U);
    if (status != DRIVER_OK) {
        ctx->initialized = false;
        return status;
    }
    status = Icm45686_WriteRegister(ctx, kRegInt1Config2, 0x00U);
    if (status != DRIVER_OK) {
        ctx->initialized = false;
        return status;
    }

    /* Full scale + ODR before enabling the sensors. Use verified writes so a
     * device that is not yet ready after soft reset is caught here. */
    const uint8_t accel_cfg =
        static_cast<uint8_t>((config->accel_fs << 4U) | (config->accel_odr & 0x0FU));
    status = WriteVerify(ctx, kRegAccelConfig0, accel_cfg);
    if (status != DRIVER_OK) {
        ctx->initialized = false;
        return status;
    }

    const uint8_t gyro_cfg =
        static_cast<uint8_t>((config->gyro_fs << 4U) | (config->gyro_odr & 0x0FU));
    status = WriteVerify(ctx, kRegGyroConfig0, gyro_cfg);
    if (status != DRIVER_OK) {
        ctx->initialized = false;
        return status;
    }

    /* Enable gyro + accel in Low-Noise mode. */
    status = WriteVerify(ctx, kRegPwrMgmt0, kPwrGyroAccelLn);
    if (status != DRIVER_OK) {
        ctx->initialized = false;
        return status;
    }

    return DRIVER_OK;
}

bool Icm45686_IsReady(const Icm45686Context *ctx)
{
    return (ctx != 0) && ctx->initialized;
}

int32_t Icm45686_AccelMilliG(int16_t raw, uint8_t accel_fs)
{
    const uint32_t fs_g = AccelFsG(accel_fs);
    if (fs_g == 0U) {
        return 0;
    }
    return static_cast<int32_t>(
        (static_cast<int64_t>(raw) * static_cast<int64_t>(fs_g) * 1000LL) /
        32768LL);
}

int32_t Icm45686_GyroMilliDps(int16_t raw, uint8_t gyro_fs)
{
    const uint32_t fs_dps = GyroFsDps(gyro_fs);
    if (fs_dps == 0U) {
        return 0;
    }
    return static_cast<int32_t>(
        (static_cast<int64_t>(raw) * static_cast<int64_t>(fs_dps) * 1000LL) /
        32768LL);
}

int32_t Icm45686_TempCentiC(int16_t raw)
{
    /* T_celsius = raw / 128 + 25 ; return centi-degrees (x100). */
    return static_cast<int32_t>(
        (static_cast<int64_t>(raw) * 100LL) / 128LL + 2500LL);
}

} /* namespace drivers */
