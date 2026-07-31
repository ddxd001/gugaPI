#ifndef DRIVERS_ICM45686_ICM45686_H_
#define DRIVERS_ICM45686_ICM45686_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

/* ICM-45686 device identity (WHO_AM_I @ 0x72). */
static const uint8_t ICM45686_WHO_AM_I_VALUE = 0xE9U;

/* Accelerometer full-scale select (ACCEL_UI_FS_SEL, ACCEL_CONFIG0[6:4]). */
enum Icm45686AccelFs : uint8_t {
    ICM45686_ACCEL_FS_32G = 0x00U,
    ICM45686_ACCEL_FS_16G = 0x01U,
    ICM45686_ACCEL_FS_8G  = 0x02U,
    ICM45686_ACCEL_FS_4G  = 0x03U,
    ICM45686_ACCEL_FS_2G  = 0x04U,
};

/* Gyroscope full-scale select (GYRO_UI_FS_SEL, GYRO_CONFIG0[7:4]). */
enum Icm45686GyroFs : uint8_t {
    ICM45686_GYRO_FS_4000DPS = 0x00U,
    ICM45686_GYRO_FS_2000DPS = 0x01U,
    ICM45686_GYRO_FS_1000DPS = 0x02U,
    ICM45686_GYRO_FS_500DPS  = 0x03U,
    ICM45686_GYRO_FS_250DPS  = 0x04U,
    ICM45686_GYRO_FS_125DPS  = 0x05U,
};

/* Output data rate (ACCEL_ODR / GYRO_ODR, low nibble of CONFIG0). */
enum Icm45686Odr : uint8_t {
    ICM45686_ODR_800HZ  = 0x06U,
    ICM45686_ODR_400HZ  = 0x07U,
    ICM45686_ODR_200HZ  = 0x08U,
    ICM45686_ODR_100HZ  = 0x09U,
    ICM45686_ODR_50HZ   = 0x0AU,
    ICM45686_ODR_25HZ   = 0x0BU,
    ICM45686_ODR_12_5HZ = 0x0CU,
};

struct Icm45686Config {
    SPI_Regs *spi;
    GPIO_Regs *cs_port;
    uint32_t cs_pin;
    uint32_t timeout_iterations; /* per-byte SPI poll budget */
    uint8_t accel_fs;            /* Icm45686AccelFs */
    uint8_t gyro_fs;             /* Icm45686GyroFs */
    uint8_t accel_odr;           /* Icm45686Odr */
    uint8_t gyro_odr;            /* Icm45686Odr */
};

struct Icm45686Context {
    const Icm45686Config *config;
    bool initialized;
};

struct Icm45686SensorData {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
};

DriverStatus Icm45686_Init(Icm45686Context *ctx, const Icm45686Config *config);
bool Icm45686_IsReady(const Icm45686Context *ctx);
DriverStatus Icm45686_SoftReset(Icm45686Context *ctx);
DriverStatus Icm45686_ReadWhoAmI(Icm45686Context *ctx, uint8_t *value);
DriverStatus Icm45686_ReadRegister(Icm45686Context *ctx,
                                   uint8_t reg,
                                   uint8_t *value);
DriverStatus Icm45686_WriteRegister(Icm45686Context *ctx,
                                    uint8_t reg,
                                    uint8_t value);
DriverStatus Icm45686_ReadBurst(Icm45686Context *ctx,
                                uint8_t reg,
                                uint8_t *buf,
                                uint16_t len);
DriverStatus Icm45686_ReadSensors(Icm45686Context *ctx,
                                  Icm45686SensorData *data);

/* Hardware-independent scaling helpers. */
int32_t Icm45686_AccelMilliG(int16_t raw, uint8_t accel_fs);
int32_t Icm45686_GyroMilliDps(int16_t raw, uint8_t gyro_fs);
int32_t Icm45686_TempCentiC(int16_t raw);

} /* namespace drivers */

#endif /* DRIVERS_ICM45686_ICM45686_H_ */
