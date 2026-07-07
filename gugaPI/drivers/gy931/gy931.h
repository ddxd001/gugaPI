#ifndef DRIVERS_GY931_GY931_H_
#define DRIVERS_GY931_GY931_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "drivers/soft_i2c/soft_i2c.h"

namespace drivers {

static const uint8_t GY931_DEFAULT_I2C_ADDRESS = 0x50U;
static const uint8_t GY931_REG_AX = 0x34U;
static const uint8_t GY931_REG_GX = 0x37U;
static const uint8_t GY931_REG_HX = 0x3AU;
static const uint8_t GY931_REG_ROLL = 0x3DU;
static const uint8_t GY931_REG_TEMP = 0x40U;
static const uint8_t GY931_REG_CHIP_ID_L = 0x8DU;
static const uint8_t GY931_MAX_READ_WORDS = 16U;

struct Gy931Config {
    SoftI2cContext *i2c;
    uint8_t i2c_address;
};

struct Gy931Context {
    const Gy931Config *config;
    bool initialized;
};

struct Gy931Angles {
    int16_t roll_raw;
    int16_t pitch_raw;
    int16_t yaw_raw;
    int32_t roll_mdeg;
    int32_t pitch_mdeg;
    int32_t yaw_mdeg;
};

struct Gy931Sample {
    int16_t acc_raw[3];
    int16_t gyro_raw[3];
    int16_t mag_raw[3];
    int16_t angle_raw[3];
    int32_t acc_mg[3];
    int32_t gyro_mdps[3];
    int32_t angle_mdeg[3];
};

DriverStatus Gy931_Init(Gy931Context *ctx, const Gy931Config *config);
DriverStatus Gy931_Probe(Gy931Context *ctx);
DriverStatus Gy931_ReadAngles(Gy931Context *ctx, Gy931Angles *angles);
DriverStatus Gy931_ReadSample(Gy931Context *ctx, Gy931Sample *sample);
DriverStatus Gy931_ReadRawRegisters(Gy931Context *ctx,
                                    uint8_t start_reg,
                                    int16_t *words,
                                    uint8_t word_count);
bool Gy931_IsReady(const Gy931Context *ctx);
int32_t Gy931_AngleRawToMdeg(int16_t raw);
int32_t Gy931_AccRawToMg(int16_t raw);
int32_t Gy931_GyroRawToMdps(int16_t raw);

} /* namespace drivers */

#endif /* DRIVERS_GY931_GY931_H_ */
