#ifndef DRIVERS_LIS3MDL_LIS3MDL_H_
#define DRIVERS_LIS3MDL_LIS3MDL_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

static const uint8_t LIS3MDL_WHO_AM_I_VALUE = 0x3DU;

/* CTRL_REG2 FS[1:0]. */
enum Lis3mdlFullScale : uint8_t {
    LIS3MDL_FULL_SCALE_4_G  = 0U,
    LIS3MDL_FULL_SCALE_8_G  = 1U,
    LIS3MDL_FULL_SCALE_12_G = 2U,
    LIS3MDL_FULL_SCALE_16_G = 3U,
};

/* CTRL_REG1 DO[2:0]. FAST_ODR remains disabled. */
enum Lis3mdlOutputDataRate : uint8_t {
    LIS3MDL_ODR_0_625_HZ = 0U,
    LIS3MDL_ODR_1_25_HZ  = 1U,
    LIS3MDL_ODR_2_5_HZ   = 2U,
    LIS3MDL_ODR_5_HZ     = 3U,
    LIS3MDL_ODR_10_HZ    = 4U,
    LIS3MDL_ODR_20_HZ    = 5U,
    LIS3MDL_ODR_40_HZ    = 6U,
    LIS3MDL_ODR_80_HZ    = 7U,
};

/* CTRL_REG1 OM[1:0] and CTRL_REG4 OMZ[1:0]. */
enum Lis3mdlPerformance : uint8_t {
    LIS3MDL_PERFORMANCE_LOW_POWER  = 0U,
    LIS3MDL_PERFORMANCE_MEDIUM     = 1U,
    LIS3MDL_PERFORMANCE_HIGH       = 2U,
    LIS3MDL_PERFORMANCE_ULTRA_HIGH = 3U,
};

/* CTRL_REG3 MD[1:0]. Values 2 and 3 both select power-down. */
enum Lis3mdlOperatingMode : uint8_t {
    LIS3MDL_MODE_CONTINUOUS = 0U,
    LIS3MDL_MODE_SINGLE     = 1U,
    LIS3MDL_MODE_POWER_DOWN = 2U,
};

struct Lis3mdlConfig {
    SPI_Regs *spi;
    GPIO_Regs *cs_port;
    uint32_t cs_pin;
    /* Optional peer CS on a shared SPI bus. It is forced high before this
     * device is selected. Set both fields to zero on a dedicated bus. */
    GPIO_Regs *peer_cs_port;
    uint32_t peer_cs_pin;
    uint32_t timeout_iterations; /* per-byte and bus-idle poll budget */
    uint8_t full_scale;           /* Lis3mdlFullScale */
    uint8_t output_data_rate;     /* Lis3mdlOutputDataRate */
    uint8_t xy_performance;       /* Lis3mdlPerformance */
    uint8_t z_performance;        /* Lis3mdlPerformance */
    uint8_t operating_mode;       /* Lis3mdlOperatingMode */
};

struct Lis3mdlContext {
    const Lis3mdlConfig *config;
    bool initialized;
    uint8_t full_scale;
    uint8_t output_data_rate;
    uint8_t xy_performance;
    uint8_t z_performance;
    uint8_t operating_mode;
};

struct Lis3mdlRawData {
    int16_t x;
    int16_t y;
    int16_t z;
};

DriverStatus Lis3mdl_Init(Lis3mdlContext *ctx, const Lis3mdlConfig *config);
bool Lis3mdl_IsReady(const Lis3mdlContext *ctx);
DriverStatus Lis3mdl_SoftReset(Lis3mdlContext *ctx);

/* Probe is intentionally usable before Init() so an absent or misidentified
 * device can still be diagnosed from the shell. */
DriverStatus Lis3mdl_ProbeWhoAmI(const Lis3mdlConfig *config, uint8_t *value);
DriverStatus Lis3mdl_ProbeRegister(const Lis3mdlConfig *config,
                                   uint8_t reg,
                                   uint8_t *value);
DriverStatus Lis3mdl_ReadWhoAmI(Lis3mdlContext *ctx, uint8_t *value);
DriverStatus Lis3mdl_ReadRegister(Lis3mdlContext *ctx,
                                  uint8_t reg,
                                  uint8_t *value);
DriverStatus Lis3mdl_WriteRegister(Lis3mdlContext *ctx,
                                   uint8_t reg,
                                   uint8_t value);
DriverStatus Lis3mdl_ReadBurst(Lis3mdlContext *ctx,
                               uint8_t reg,
                               uint8_t *buf,
                               uint16_t len);

DriverStatus Lis3mdl_SetFullScale(Lis3mdlContext *ctx, uint8_t full_scale);
DriverStatus Lis3mdl_SetOutputDataRate(Lis3mdlContext *ctx,
                                      uint8_t output_data_rate);
DriverStatus Lis3mdl_SetPerformance(Lis3mdlContext *ctx,
                                    uint8_t xy_performance,
                                    uint8_t z_performance);
DriverStatus Lis3mdl_SetOperatingMode(Lis3mdlContext *ctx,
                                      uint8_t operating_mode);
DriverStatus Lis3mdl_IsDataReady(Lis3mdlContext *ctx, bool *ready);
DriverStatus Lis3mdl_ReadRaw(Lis3mdlContext *ctx, Lis3mdlRawData *data);

uint8_t Lis3mdl_GetFullScale(const Lis3mdlContext *ctx);
uint8_t Lis3mdl_GetOutputDataRate(const Lis3mdlContext *ctx);
uint8_t Lis3mdl_GetOperatingMode(const Lis3mdlContext *ctx);

/* Convert one raw axis to milli-gauss using the data-sheet sensitivity for
 * the selected full scale. Invalid full-scale values return zero. */
int32_t Lis3mdl_RawToMilliGauss(int16_t raw, uint8_t full_scale);

} /* namespace drivers */

#endif /* DRIVERS_LIS3MDL_LIS3MDL_H_ */

