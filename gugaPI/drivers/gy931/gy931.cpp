#include "drivers/gy931/gy931.h"

namespace drivers {
namespace {

static const uint8_t kGy931SampleStartReg = GY931_REG_AX;
static const uint8_t kGy931SampleWords = 12U;
static const uint16_t kGy931UnlockKey = 0xB588U;

bool IsAddressValid(uint8_t address)
{
    return (address >= 0x08U) && (address <= 0x77U);
}

bool IsConfigValid(const Gy931Config *config)
{
    return (config != 0) && (config->i2c != 0) &&
           SoftI2c_IsReady(config->i2c) &&
           IsAddressValid(config->i2c_address);
}

bool IsContextReady(const Gy931Context *ctx)
{
    return (ctx != 0) && ctx->initialized && IsConfigValid(ctx->config);
}

int16_t DecodeInt16Le(const uint8_t *data)
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8U));
}

DriverStatus ReadWords(const Gy931Config *config,
                       uint8_t start_reg,
                       int16_t *words,
                       uint8_t word_count)
{
    uint8_t data[GY931_MAX_READ_WORDS * 2U];

    if ((!IsConfigValid(config)) || (words == 0) || (word_count == 0U) ||
        (word_count > GY931_MAX_READ_WORDS)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const uint16_t byte_count = static_cast<uint16_t>(word_count * 2U);
    DriverStatus status = SoftI2c_ReadReg8(config->i2c,
                                           config->i2c_address,
                                           start_reg,
                                           data,
                                           byte_count);
    if (status != DRIVER_OK) {
        return status;
    }

    for (uint8_t i = 0U; i < word_count; i++) {
        words[i] = DecodeInt16Le(&data[i * 2U]);
    }

    return DRIVER_OK;
}

DriverStatus WriteWord(const Gy931Config *config,
                       uint8_t reg,
                       uint16_t value)
{
    if (!IsConfigValid(config)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const uint8_t data[2] = {
        static_cast<uint8_t>(value & 0xFFU),
        static_cast<uint8_t>((value >> 8U) & 0xFFU)
    };
    return SoftI2c_WriteReg8(config->i2c,
                             config->i2c_address,
                             reg,
                             data,
                             static_cast<uint16_t>(sizeof(data)));
}

void FillScaledSample(Gy931Sample *sample)
{
    for (uint8_t i = 0U; i < 3U; i++) {
        sample->acc_mg[i] = Gy931_AccRawToMg(sample->acc_raw[i]);
        sample->gyro_mdps[i] = Gy931_GyroRawToMdps(sample->gyro_raw[i]);
        sample->angle_mdeg[i] = Gy931_AngleRawToMdeg(sample->angle_raw[i]);
    }
}

} /* namespace */

DriverStatus Gy931_Init(Gy931Context *ctx, const Gy931Config *config)
{
    if ((ctx == 0) || (!IsConfigValid(config))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    ctx->config = config;
    ctx->initialized = false;

    const DriverStatus status = SoftI2c_ProbeAddress(config->i2c,
                                                     config->i2c_address);
    if (status != DRIVER_OK) {
        return status;
    }

    ctx->initialized = true;
    return DRIVER_OK;
}

DriverStatus Gy931_Probe(Gy931Context *ctx)
{
    if ((ctx == 0) || (!IsConfigValid(ctx->config))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    return SoftI2c_ProbeAddress(ctx->config->i2c,
                                ctx->config->i2c_address);
}

DriverStatus Gy931_ReadAngles(Gy931Context *ctx, Gy931Angles *angles)
{
    int16_t words[3];

    if ((!IsContextReady(ctx)) || (angles == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const DriverStatus status = ReadWords(ctx->config,
                                          GY931_REG_ROLL,
                                          words,
                                          3U);
    if (status != DRIVER_OK) {
        return status;
    }

    angles->roll_raw = words[0];
    angles->pitch_raw = words[1];
    angles->yaw_raw = words[2];
    angles->roll_mdeg = Gy931_AngleRawToMdeg(words[0]);
    angles->pitch_mdeg = Gy931_AngleRawToMdeg(words[1]);
    angles->yaw_mdeg = Gy931_AngleRawToMdeg(words[2]);

    return DRIVER_OK;
}

DriverStatus Gy931_ReadSample(Gy931Context *ctx, Gy931Sample *sample)
{
    int16_t words[kGy931SampleWords];

    if ((!IsContextReady(ctx)) || (sample == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const DriverStatus status = ReadWords(ctx->config,
                                          kGy931SampleStartReg,
                                          words,
                                          kGy931SampleWords);
    if (status != DRIVER_OK) {
        return status;
    }

    for (uint8_t i = 0U; i < 3U; i++) {
        sample->acc_raw[i] = words[i];
        sample->gyro_raw[i] = words[3U + i];
        sample->mag_raw[i] = words[6U + i];
        sample->angle_raw[i] = words[9U + i];
    }
    FillScaledSample(sample);

    return DRIVER_OK;
}

DriverStatus Gy931_ReadRawRegisters(Gy931Context *ctx,
                                    uint8_t start_reg,
                                    int16_t *words,
                                    uint8_t word_count)
{
    if (!IsContextReady(ctx)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }

    return ReadWords(ctx->config, start_reg, words, word_count);
}

DriverStatus Gy931_ReadAlgorithm(Gy931Context *ctx,
                                 Gy931Algorithm *algorithm)
{
    int16_t value = 0;

    if ((!IsContextReady(ctx)) || (algorithm == 0)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    const DriverStatus status = ReadWords(ctx->config,
                                          GY931_REG_AXIS6,
                                          &value,
                                          1U);
    if (status != DRIVER_OK) {
        return status;
    }
    if ((value != static_cast<int16_t>(GY931_ALGORITHM_9_AXIS)) &&
        (value != static_cast<int16_t>(GY931_ALGORITHM_6_AXIS))) {
        return DRIVER_ERROR;
    }

    *algorithm = static_cast<Gy931Algorithm>(value);
    return DRIVER_OK;
}

DriverStatus Gy931_SetAlgorithmTemporary(Gy931Context *ctx,
                                         Gy931Algorithm algorithm)
{
    if ((!IsContextReady(ctx)) ||
        ((algorithm != GY931_ALGORITHM_9_AXIS) &&
         (algorithm != GY931_ALGORITHM_6_AXIS))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    DriverStatus status = WriteWord(ctx->config,
                                    GY931_REG_KEY,
                                    kGy931UnlockKey);
    if (status != DRIVER_OK) {
        return status;
    }
    status = WriteWord(ctx->config,
                       GY931_REG_AXIS6,
                       static_cast<uint16_t>(algorithm));
    if (status != DRIVER_OK) {
        return status;
    }

    Gy931Algorithm readback = GY931_ALGORITHM_9_AXIS;
    status = Gy931_ReadAlgorithm(ctx, &readback);
    if (status != DRIVER_OK) {
        return status;
    }
    return (readback == algorithm) ? DRIVER_OK : DRIVER_ERROR;
}

bool Gy931_IsReady(const Gy931Context *ctx)
{
    return IsContextReady(ctx);
}

int32_t Gy931_AngleRawToMdeg(int16_t raw)
{
    return static_cast<int32_t>(
        (static_cast<int64_t>(raw) * 180000LL) / 32768LL);
}

int32_t Gy931_AccRawToMg(int16_t raw)
{
    return static_cast<int32_t>(
        (static_cast<int64_t>(raw) * 16000LL) / 32768LL);
}

int32_t Gy931_GyroRawToMdps(int16_t raw)
{
    return static_cast<int32_t>(
        (static_cast<int64_t>(raw) * 2000000LL) / 32768LL);
}

} /* namespace drivers */
