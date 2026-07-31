#include "drivers/fm24cl64b/fm24cl64b.h"

namespace drivers {
namespace {

static const uint16_t kAddressBytes = 2U;
/* A 256-byte burst keeps the longest blocking operation bounded while allowing
 * the complete HConfig record to use one I2C transaction. The controller
 * refills its 8-byte FIFO while a transfer is active, so the FIFO depth is not
 * a transaction-size limit. */
static const uint16_t kMaxBurstPayloadBytes = 256U;
static_assert(kMaxBurstPayloadBytes <= I2C_CONTROLLER_MAX_TRANSFER_BYTES,
              "FRAM read burst exceeds I2C controller transfer limit");
static_assert((kAddressBytes + kMaxBurstPayloadBytes) <=
                  I2C_CONTROLLER_MAX_TRANSFER_BYTES,
              "FRAM write burst exceeds I2C controller transfer limit");
static const uint16_t kMaxWritePayloadBytes = kMaxBurstPayloadBytes;
static const uint16_t kMaxReadPayloadBytes = kMaxBurstPayloadBytes;

bool IsRangeValid(uint16_t address, uint16_t length)
{
    if (length == 0U) {
        return true;
    }
    if (address >= FM24CL64B_SIZE_BYTES) {
        return false;
    }
    return length <= (FM24CL64B_SIZE_BYTES - address);
}

bool IsConfigValid(const Fm24cl64bConfig *config)
{
    return (config != 0) &&
           I2cController_IsConfigValid(config->bus) &&
           I2cController_IsAddressValid(config->i2c_address);
}

DriverStatus CheckContext(Fm24cl64bContext *ctx)
{
    return ((ctx != 0) && ctx->initialized && IsConfigValid(ctx->config)) ?
        DRIVER_OK : DRIVER_ERROR_NOT_INITIALIZED;
}

void FillAddress(uint16_t address, uint8_t buffer[kAddressBytes])
{
    buffer[0] = static_cast<uint8_t>((address >> 8U) & 0xFFU);
    buffer[1] = static_cast<uint8_t>(address & 0xFFU);
}

uint16_t MinU16(uint16_t left, uint16_t right)
{
    return (left < right) ? left : right;
}

DriverStatus WriteChunk(Fm24cl64bContext *ctx,
                        uint16_t address,
                        const uint8_t *data,
                        uint16_t length)
{
    uint8_t address_bytes[kAddressBytes];
    FillAddress(address, address_bytes);
    return I2cController_Write(ctx->config->bus,
                               ctx->config->i2c_address,
                               address_bytes,
                               kAddressBytes,
                               data,
                               length);
}

DriverStatus ReadChunk(Fm24cl64bContext *ctx,
                       uint16_t address,
                       uint8_t *data,
                       uint16_t length)
{
    uint8_t address_bytes[kAddressBytes];
    FillAddress(address, address_bytes);
    return I2cController_WriteRead(ctx->config->bus,
                                   ctx->config->i2c_address,
                                   address_bytes,
                                   kAddressBytes,
                                   data,
                                   length);
}

} /* namespace */

DriverStatus Fm24cl64b_Init(Fm24cl64bContext *ctx,
                            const Fm24cl64bConfig *config)
{
    if ((ctx == 0) || (!IsConfigValid(config))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->config = config;
    ctx->initialized = false;

    /* FM24CL64B has no identity register. A non-destructive byte read proves
     * that the configured target acknowledges before it is reported ready. */
    ctx->initialized = true;
    uint8_t probe = 0U;
    const DriverStatus status = Fm24cl64b_ReadByte(ctx, 0U, &probe);
    if (status != DRIVER_OK) {
        ctx->initialized = false;
    }
    return status;
}

DriverStatus Fm24cl64b_Read(Fm24cl64bContext *ctx,
                            uint16_t address,
                            uint8_t *data,
                            uint16_t length)
{
    DriverStatus status = CheckContext(ctx);
    if (status != DRIVER_OK) {
        return status;
    }
    if ((data == 0) || (!IsRangeValid(address, length))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    uint16_t offset = 0U;
    while (offset < length) {
        const uint16_t chunk = MinU16(
            static_cast<uint16_t>(length - offset),
            kMaxReadPayloadBytes);
        status = ReadChunk(ctx,
                           static_cast<uint16_t>(address + offset),
                           &data[offset],
                           chunk);
        if (status != DRIVER_OK) {
            return status;
        }
        offset = static_cast<uint16_t>(offset + chunk);
    }
    return DRIVER_OK;
}

DriverStatus Fm24cl64b_Write(Fm24cl64bContext *ctx,
                             uint16_t address,
                             const uint8_t *data,
                             uint16_t length)
{
    DriverStatus status = CheckContext(ctx);
    if (status != DRIVER_OK) {
        return status;
    }
    if ((data == 0) || (!IsRangeValid(address, length))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    uint16_t offset = 0U;
    while (offset < length) {
        const uint16_t chunk = MinU16(
            static_cast<uint16_t>(length - offset),
            kMaxWritePayloadBytes);
        status = WriteChunk(ctx,
                            static_cast<uint16_t>(address + offset),
                            &data[offset],
                            chunk);
        if (status != DRIVER_OK) {
            return status;
        }
        offset = static_cast<uint16_t>(offset + chunk);
    }
    return DRIVER_OK;
}

DriverStatus Fm24cl64b_ReadByte(Fm24cl64bContext *ctx,
                                uint16_t address,
                                uint8_t *data)
{
    return Fm24cl64b_Read(ctx, address, data, 1U);
}

DriverStatus Fm24cl64b_WriteByte(Fm24cl64bContext *ctx,
                                 uint16_t address,
                                 uint8_t data)
{
    return Fm24cl64b_Write(ctx, address, &data, 1U);
}

DriverStatus Fm24cl64b_SelfTest(Fm24cl64bContext *ctx,
                                uint16_t test_address)
{
    uint8_t original[FM24CL64B_SELF_TEST_LENGTH];
    const uint8_t pattern[FM24CL64B_SELF_TEST_LENGTH] = {
        0xA5U, 0x5AU, 0x00U, 0xFFU, 0x12U, 0x34U, 0x56U, 0x78U
    };
    uint8_t readback[FM24CL64B_SELF_TEST_LENGTH];

    DriverStatus status = Fm24cl64b_Read(ctx,
                                         test_address,
                                         original,
                                         FM24CL64B_SELF_TEST_LENGTH);
    if (status != DRIVER_OK) {
        return status;
    }

    status = Fm24cl64b_Write(ctx,
                             test_address,
                             pattern,
                             FM24CL64B_SELF_TEST_LENGTH);
    if (status == DRIVER_OK) {
        status = Fm24cl64b_Read(ctx,
                                test_address,
                                readback,
                                FM24CL64B_SELF_TEST_LENGTH);
    }
    if (status == DRIVER_OK) {
        for (uint16_t i = 0U; i < FM24CL64B_SELF_TEST_LENGTH; i++) {
            if (readback[i] != pattern[i]) {
                status = DRIVER_ERROR;
                break;
            }
        }
    }

    const DriverStatus restore_status = Fm24cl64b_Write(
        ctx,
        test_address,
        original,
        FM24CL64B_SELF_TEST_LENGTH);
    return (status == DRIVER_OK) ? restore_status : status;
}

DriverStatus Fm24cl64b_RecoverBus(Fm24cl64bContext *ctx)
{
    const DriverStatus status = CheckContext(ctx);
    return (status == DRIVER_OK) ?
        I2cController_RecoverBus(ctx->config->bus) : status;
}

DriverStatus Fm24cl64b_GetBusStatus(Fm24cl64bContext *ctx,
                                    Fm24cl64bBusStatus *status)
{
    const DriverStatus context_status = CheckContext(ctx);
    if (context_status != DRIVER_OK) {
        return context_status;
    }
    return I2cController_GetBusStatus(ctx->config->bus, status);
}

bool Fm24cl64b_IsReady(const Fm24cl64bContext *ctx)
{
    return (ctx != 0) && ctx->initialized && IsConfigValid(ctx->config);
}

} /* namespace drivers */
