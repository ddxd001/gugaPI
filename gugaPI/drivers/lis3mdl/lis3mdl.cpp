#include "drivers/lis3mdl/lis3mdl.h"

namespace drivers {
namespace {

static const uint8_t kRegWhoAmI  = 0x0FU;
static const uint8_t kRegCtrl1   = 0x20U;
static const uint8_t kRegCtrl2   = 0x21U;
static const uint8_t kRegCtrl3   = 0x22U;
static const uint8_t kRegCtrl4   = 0x23U;
static const uint8_t kRegCtrl5   = 0x24U;
static const uint8_t kRegStatus  = 0x27U;
static const uint8_t kRegOutXL   = 0x28U;

static const uint8_t kSpiReadMask       = 0x80U;
static const uint8_t kSpiMultiByteMask  = 0x40U;
static const uint8_t kRegisterMask      = 0x3FU;
static const uint8_t kCtrl1OmMask       = 0x60U;
static const uint8_t kCtrl1DoMask       = 0x1CU;
static const uint8_t kCtrl1FastOdrMask  = 0x02U;
static const uint8_t kCtrl2FsMask       = 0x60U;
static const uint8_t kCtrl2SoftReset    = 0x04U;
static const uint8_t kCtrl3SimMask      = 0x04U;
static const uint8_t kCtrl3ModeMask     = 0x03U;
static const uint8_t kCtrl4OmzMask      = 0x0CU;
static const uint8_t kCtrl4BleMask      = 0x02U;
static const uint8_t kCtrl5Bdu          = 0x40U;
static const uint8_t kStatusZyxDa       = 0x08U;
static const uint32_t kSelectDelayCycles = 32U;
static const uint32_t kResetPollRetries  = 300U;

bool IsConfigValid(const Lis3mdlConfig *config)
{
    if ((config == 0) || (config->spi == 0) || (config->cs_port == 0) ||
        (config->cs_pin == 0U) || (config->timeout_iterations == 0U) ||
        (config->full_scale > LIS3MDL_FULL_SCALE_16_G) ||
        (config->output_data_rate > LIS3MDL_ODR_80_HZ) ||
        (config->xy_performance > LIS3MDL_PERFORMANCE_ULTRA_HIGH) ||
        (config->z_performance > LIS3MDL_PERFORMANCE_ULTRA_HIGH) ||
        (config->operating_mode > LIS3MDL_MODE_POWER_DOWN)) {
        return false;
    }
    if (((config->peer_cs_port == 0) && (config->peer_cs_pin != 0U)) ||
        ((config->peer_cs_port != 0) && (config->peer_cs_pin == 0U)) ||
        ((config->peer_cs_port == config->cs_port) &&
         (config->peer_cs_pin == config->cs_pin))) {
        return false;
    }
    return true;
}

void DelaySmall(void)
{
    for (volatile uint32_t i = 0U; i < kSelectDelayCycles; i++) {
    }
}

void Deselect(const Lis3mdlConfig *config)
{
    DL_GPIO_setPins(config->cs_port, config->cs_pin);
}

void Select(const Lis3mdlConfig *config)
{
    Deselect(config);
    if (config->peer_cs_port != 0) {
        DL_GPIO_setPins(config->peer_cs_port, config->peer_cs_pin);
    }
    DelaySmall();
    DL_GPIO_clearPins(config->cs_port, config->cs_pin);
    DelaySmall();
}

void DrainRxFifo(SPI_Regs *spi)
{
    uint8_t ignored = 0U;
    while (DL_SPI_receiveDataCheck8(spi, &ignored)) {
    }
}

DriverStatus WaitIdle(const Lis3mdlConfig *config)
{
    uint32_t timeout = config->timeout_iterations;
    while (DL_SPI_isBusy(config->spi)) {
        if (timeout == 0U) {
            return DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }
    return DRIVER_OK;
}

DriverStatus TransferByte(const Lis3mdlConfig *config,
                          uint8_t tx,
                          uint8_t *rx)
{
    uint32_t timeout = config->timeout_iterations;
    uint8_t received = 0U;

    while (!DL_SPI_transmitDataCheck8(config->spi, tx)) {
        if (timeout == 0U) {
            return DRIVER_ERROR_TIMEOUT;
        }
        timeout--;
    }

    timeout = config->timeout_iterations;
    while (!DL_SPI_receiveDataCheck8(config->spi, &received)) {
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

DriverStatus ReadBurstWithConfig(const Lis3mdlConfig *config,
                                 uint8_t reg,
                                 uint8_t *buf,
                                 uint16_t len)
{
    if ((!IsConfigValid(config)) || (buf == 0) || (len == 0U) ||
        (reg > kRegisterMask) ||
        ((static_cast<uint32_t>(reg) + static_cast<uint32_t>(len) - 1U) >
         kRegisterMask)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    DriverStatus status = WaitIdle(config);
    if (status != DRIVER_OK) {
        return status;
    }

    DrainRxFifo(config->spi);
    Select(config);

    uint8_t command = static_cast<uint8_t>(kSpiReadMask | reg);
    if (len > 1U) {
        command = static_cast<uint8_t>(command | kSpiMultiByteMask);
    }
    status = TransferByte(config, command, 0);
    for (uint16_t i = 0U; (i < len) && (status == DRIVER_OK); i++) {
        status = TransferByte(config, 0x00U, &buf[i]);
    }

    const DriverStatus idle_status = WaitIdle(config);
    Deselect(config);
    return (status != DRIVER_OK) ? status : idle_status;
}

DriverStatus WriteRegisterWithConfig(const Lis3mdlConfig *config,
                                     uint8_t reg,
                                     uint8_t value)
{
    if ((!IsConfigValid(config)) || (reg > kRegisterMask)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    DriverStatus status = WaitIdle(config);
    if (status != DRIVER_OK) {
        return status;
    }

    DrainRxFifo(config->spi);
    Select(config);
    status = TransferByte(config,
                          static_cast<uint8_t>(reg & kRegisterMask),
                          0);
    if (status == DRIVER_OK) {
        status = TransferByte(config, value, 0);
    }

    const DriverStatus idle_status = WaitIdle(config);
    Deselect(config);
    return (status != DRIVER_OK) ? status : idle_status;
}

DriverStatus WriteMaskedVerify(Lis3mdlContext *ctx,
                               uint8_t reg,
                               uint8_t mask,
                               uint8_t value)
{
    uint8_t current = 0U;
    DriverStatus status = Lis3mdl_ReadRegister(ctx, reg, &current);
    if (status != DRIVER_OK) {
        return status;
    }

    const uint8_t updated = static_cast<uint8_t>(
        (current & static_cast<uint8_t>(~mask)) | (value & mask));
    status = Lis3mdl_WriteRegister(ctx, reg, updated);
    if (status != DRIVER_OK) {
        return status;
    }

    uint8_t readback = 0U;
    status = Lis3mdl_ReadRegister(ctx, reg, &readback);
    if (status != DRIVER_OK) {
        return status;
    }
    return ((readback & mask) == (updated & mask)) ? DRIVER_OK : DRIVER_ERROR;
}

int16_t CombineLe(uint8_t lo, uint8_t hi)
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(lo) |
        (static_cast<uint16_t>(hi) << 8U));
}

uint32_t SensitivityLsbPerGauss(uint8_t full_scale)
{
    switch (full_scale) {
        case LIS3MDL_FULL_SCALE_4_G:  return 6842U;
        case LIS3MDL_FULL_SCALE_8_G:  return 3421U;
        case LIS3MDL_FULL_SCALE_12_G: return 2281U;
        case LIS3MDL_FULL_SCALE_16_G: return 1711U;
        default:                     return 0U;
    }
}

} /* namespace */

DriverStatus Lis3mdl_ReadBurst(Lis3mdlContext *ctx,
                               uint8_t reg,
                               uint8_t *buf,
                               uint16_t len)
{
    if ((ctx == 0) || (ctx->config == 0) || (!ctx->initialized)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    return ReadBurstWithConfig(ctx->config, reg, buf, len);
}

DriverStatus Lis3mdl_ReadRegister(Lis3mdlContext *ctx,
                                  uint8_t reg,
                                  uint8_t *value)
{
    return Lis3mdl_ReadBurst(ctx, reg, value, 1U);
}

DriverStatus Lis3mdl_WriteRegister(Lis3mdlContext *ctx,
                                   uint8_t reg,
                                   uint8_t value)
{
    if ((ctx == 0) || (ctx->config == 0) || (!ctx->initialized)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    return WriteRegisterWithConfig(ctx->config, reg, value);
}

DriverStatus Lis3mdl_ProbeWhoAmI(const Lis3mdlConfig *config, uint8_t *value)
{
    return Lis3mdl_ProbeRegister(config, kRegWhoAmI, value);
}

DriverStatus Lis3mdl_ProbeRegister(const Lis3mdlConfig *config,
                                   uint8_t reg,
                                   uint8_t *value)
{
    if (value == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }
    return ReadBurstWithConfig(config, reg, value, 1U);
}

DriverStatus Lis3mdl_ReadWhoAmI(Lis3mdlContext *ctx, uint8_t *value)
{
    if (value == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }
    return Lis3mdl_ReadRegister(ctx, kRegWhoAmI, value);
}

DriverStatus Lis3mdl_SoftReset(Lis3mdlContext *ctx)
{
    if ((ctx == 0) || (ctx->config == 0) || (!ctx->initialized)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    DriverStatus status = Lis3mdl_WriteRegister(ctx,
                                                kRegCtrl2,
                                                kCtrl2SoftReset);
    if (status != DRIVER_OK) {
        return status;
    }

    DriverStatus last_status = DRIVER_ERROR_TIMEOUT;
    bool received_response = false;
    for (uint32_t i = 0U; i < kResetPollRetries; i++) {
        uint8_t ctrl2 = 0U;
        status = Lis3mdl_ReadRegister(ctx, kRegCtrl2, &ctrl2);
        if (status == DRIVER_OK) {
            received_response = true;
            if ((ctrl2 & kCtrl2SoftReset) == 0U) {
                return DRIVER_OK;
            }
        } else {
            last_status = status;
        }
    }
    return received_response ? DRIVER_ERROR_TIMEOUT : last_status;
}

DriverStatus Lis3mdl_SetFullScale(Lis3mdlContext *ctx, uint8_t full_scale)
{
    if ((ctx == 0) || (ctx->config == 0) || (!ctx->initialized)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (full_scale > LIS3MDL_FULL_SCALE_16_G) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const DriverStatus status = WriteMaskedVerify(
        ctx,
        kRegCtrl2,
        kCtrl2FsMask,
        static_cast<uint8_t>(full_scale << 5U));
    if (status == DRIVER_OK) {
        ctx->full_scale = full_scale;
    }
    return status;
}

DriverStatus Lis3mdl_SetOutputDataRate(Lis3mdlContext *ctx,
                                      uint8_t output_data_rate)
{
    if ((ctx == 0) || (ctx->config == 0) || (!ctx->initialized)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (output_data_rate > LIS3MDL_ODR_80_HZ) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const uint8_t mask = static_cast<uint8_t>(kCtrl1DoMask |
                                               kCtrl1FastOdrMask);
    const DriverStatus status = WriteMaskedVerify(
        ctx,
        kRegCtrl1,
        mask,
        static_cast<uint8_t>(output_data_rate << 2U));
    if (status == DRIVER_OK) {
        ctx->output_data_rate = output_data_rate;
    }
    return status;
}

DriverStatus Lis3mdl_SetPerformance(Lis3mdlContext *ctx,
                                    uint8_t xy_performance,
                                    uint8_t z_performance)
{
    if ((ctx == 0) || (ctx->config == 0) || (!ctx->initialized)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if ((xy_performance > LIS3MDL_PERFORMANCE_ULTRA_HIGH) ||
        (z_performance > LIS3MDL_PERFORMANCE_ULTRA_HIGH)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    DriverStatus status = WriteMaskedVerify(
        ctx,
        kRegCtrl1,
        kCtrl1OmMask,
        static_cast<uint8_t>(xy_performance << 5U));
    if (status != DRIVER_OK) {
        return status;
    }
    ctx->xy_performance = xy_performance;
    status = WriteMaskedVerify(ctx,
                               kRegCtrl4,
                               kCtrl4OmzMask,
                               static_cast<uint8_t>(z_performance << 2U));
    if (status == DRIVER_OK) {
        ctx->z_performance = z_performance;
    }
    return status;
}

DriverStatus Lis3mdl_SetOperatingMode(Lis3mdlContext *ctx,
                                      uint8_t operating_mode)
{
    if ((ctx == 0) || (ctx->config == 0) || (!ctx->initialized)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (operating_mode > LIS3MDL_MODE_POWER_DOWN) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const uint8_t mask = static_cast<uint8_t>(kCtrl3ModeMask | kCtrl3SimMask);
    const DriverStatus status = WriteMaskedVerify(ctx,
                                                  kRegCtrl3,
                                                  mask,
                                                  operating_mode);
    if (status == DRIVER_OK) {
        ctx->operating_mode = operating_mode;
    }
    return status;
}

DriverStatus Lis3mdl_IsDataReady(Lis3mdlContext *ctx, bool *ready)
{
    if (ready == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    uint8_t status_reg = 0U;
    const DriverStatus status = Lis3mdl_ReadRegister(ctx,
                                                     kRegStatus,
                                                     &status_reg);
    if (status == DRIVER_OK) {
        *ready = ((status_reg & kStatusZyxDa) != 0U);
    }
    return status;
}

DriverStatus Lis3mdl_ReadRaw(Lis3mdlContext *ctx, Lis3mdlRawData *data)
{
    if (data == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    uint8_t raw[6];
    const DriverStatus status = Lis3mdl_ReadBurst(ctx,
                                                  kRegOutXL,
                                                  raw,
                                                  sizeof(raw));
    if (status != DRIVER_OK) {
        return status;
    }

    data->x = CombineLe(raw[0], raw[1]);
    data->y = CombineLe(raw[2], raw[3]);
    data->z = CombineLe(raw[4], raw[5]);
    return DRIVER_OK;
}

DriverStatus Lis3mdl_Init(Lis3mdlContext *ctx, const Lis3mdlConfig *config)
{
    if ((ctx == 0) || (!IsConfigValid(config))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->config = config;
    ctx->initialized = true;
    ctx->full_scale = config->full_scale;
    ctx->output_data_rate = config->output_data_rate;
    ctx->xy_performance = config->xy_performance;
    ctx->z_performance = config->z_performance;
    ctx->operating_mode = config->operating_mode;

    DriverStatus status = Lis3mdl_SoftReset(ctx);
    if (status != DRIVER_OK) {
        ctx->initialized = false;
        return status;
    }

    uint8_t who = 0U;
    status = Lis3mdl_ReadWhoAmI(ctx, &who);
    if ((status != DRIVER_OK) || (who != LIS3MDL_WHO_AM_I_VALUE)) {
        ctx->initialized = false;
        return (status != DRIVER_OK) ? status : DRIVER_ERROR;
    }

    /* BDU prevents a sample from changing between its low/high-byte reads.
     * BLE=0 selects little-endian output and SIM=0 selects four-wire SPI. */
    status = WriteMaskedVerify(ctx, kRegCtrl5, kCtrl5Bdu, kCtrl5Bdu);
    if (status == DRIVER_OK) {
        status = WriteMaskedVerify(ctx, kRegCtrl4, kCtrl4BleMask, 0U);
    }
    if (status == DRIVER_OK) {
        status = Lis3mdl_SetPerformance(ctx,
                                        config->xy_performance,
                                        config->z_performance);
    }
    if (status == DRIVER_OK) {
        status = Lis3mdl_SetFullScale(ctx, config->full_scale);
    }
    if (status == DRIVER_OK) {
        status = Lis3mdl_SetOutputDataRate(ctx, config->output_data_rate);
    }
    if (status == DRIVER_OK) {
        status = Lis3mdl_SetOperatingMode(ctx, config->operating_mode);
    }
    if (status != DRIVER_OK) {
        ctx->initialized = false;
    }
    return status;
}

bool Lis3mdl_IsReady(const Lis3mdlContext *ctx)
{
    return (ctx != 0) && ctx->initialized;
}

uint8_t Lis3mdl_GetFullScale(const Lis3mdlContext *ctx)
{
    return (ctx != 0) ? ctx->full_scale : 0U;
}

uint8_t Lis3mdl_GetOutputDataRate(const Lis3mdlContext *ctx)
{
    return (ctx != 0) ? ctx->output_data_rate : 0U;
}

uint8_t Lis3mdl_GetOperatingMode(const Lis3mdlContext *ctx)
{
    return (ctx != 0) ? ctx->operating_mode : 0U;
}

int32_t Lis3mdl_RawToMilliGauss(int16_t raw, uint8_t full_scale)
{
    const uint32_t sensitivity = SensitivityLsbPerGauss(full_scale);
    if (sensitivity == 0U) {
        return 0;
    }
    return static_cast<int32_t>(
        (static_cast<int64_t>(raw) * 1000LL) /
        static_cast<int64_t>(sensitivity));
}

} /* namespace drivers */

