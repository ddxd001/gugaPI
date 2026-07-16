#include "app/app_imu.h"

#include "app/config_store.h"
#include "board/board_imu.h"
#include "drivers/common/driver_status.h"
#include "drivers/icm45686/icm45686.h"
#include "services/time.h"

namespace app {
namespace {

AppImuData g_data = { { 0, 0, 0 }, { 0, 0, 0 }, 0, 0, 0, 0, false };
uint32_t g_lastUpdateUs = 0U;
int64_t g_yawRemainder = 0LL;

static const uint32_t kMaxYawIntegrationIntervalUs = 100000U;
static const int64_t kMicrosPerSecond = 1000000LL;

/* CORDIC vectoring-mode atan2. Returns atan2(y, x) in milli-degrees,
 * range [-180000, 180000]. No floating point / libm dependency. */
int32_t Atan2MilliDeg(int32_t y, int32_t x)
{
    if ((x == 0) && (y == 0)) {
        return 0;
    }

    /* atan(2^-i) in micro-degrees (degrees * 1e6), i = 0..23. */
    static const int32_t kAtan[24] = {
        45000000, 26565051, 14036243, 7125016, 3576334, 1789911,
        895174, 447614, 223811, 111906, 55953, 27976,
        13988, 6994, 3497, 1748, 874, 437, 218, 109, 55, 27, 14, 7
    };

    const int64_t scale = 1LL << 20;
    int64_t xi = static_cast<int64_t>(x) * scale;
    int64_t yi = static_cast<int64_t>(y) * scale;
    int32_t z = 0;

    for (int32_t i = 0; i < 24; i++) {
        int64_t dx;
        int64_t dy;
        if (yi >= 0) {
            dx = xi + (yi >> i);
            dy = yi - (xi >> i);
            z += kAtan[static_cast<uint32_t>(i)];
        } else {
            dx = xi - (yi >> i);
            dy = yi + (xi >> i);
            z -= kAtan[static_cast<uint32_t>(i)];
        }
        xi = dx;
        yi = dy;
    }

    return z / 1000; /* micro-degrees -> milli-degrees */
}

int32_t Normalize360(int32_t angle_mdeg)
{
    while (angle_mdeg < 0) {
        angle_mdeg += 360000;
    }
    while (angle_mdeg >= 360000) {
        angle_mdeg -= 360000;
    }
    return angle_mdeg;
}

int32_t NormalizeSigned180(int32_t angle_mdeg)
{
    while (angle_mdeg > 180000) {
        angle_mdeg -= 360000;
    }
    while (angle_mdeg <= -180000) {
        angle_mdeg += 360000;
    }
    return angle_mdeg;
}

void UpdateYaw(uint32_t now_us)
{
    if (g_lastUpdateUs == 0U) {
        g_lastUpdateUs = now_us;
        return;
    }

    const uint32_t elapsed_us = now_us - g_lastUpdateUs;
    g_lastUpdateUs = now_us;
    if ((elapsed_us == 0U) ||
        (elapsed_us > kMaxYawIntegrationIntervalUs)) {
        g_yawRemainder = 0LL;
        return;
    }

    const int64_t scaled_delta =
        static_cast<int64_t>(g_data.gyro_mdps[2]) * elapsed_us +
        g_yawRemainder;
    const int32_t delta_mdeg =
        static_cast<int32_t>(scaled_delta / kMicrosPerSecond);
    g_yawRemainder = scaled_delta % kMicrosPerSecond;
    g_data.yaw_mdeg = NormalizeSigned180(g_data.yaw_mdeg + delta_mdeg);
}

} /* namespace */

void App_ImuInit(void)
{
    g_data.yaw_mdeg = 0;
    g_lastUpdateUs = 0U;
    g_yawRemainder = 0LL;
    g_data.valid = false;
}

void App_ImuUpdate(void)
{
    if (!board::Board_Icm45686IsReady()) {
        return;
    }

    drivers::Icm45686SensorData raw = {};
    const drivers::DriverStatus status =
        board::Board_Icm45686ReadSensors(&raw);
    if (status != drivers::DRIVER_OK) {
        return;
    }

    const drivers::Icm45686Config *cfg = board::Board_Icm45686GetConfig();
    const uint8_t accel_fs = (cfg != 0) ? cfg->accel_fs : drivers::ICM45686_ACCEL_FS_4G;
    const uint8_t gyro_fs = (cfg != 0) ? cfg->gyro_fs : drivers::ICM45686_GYRO_FS_1000DPS;

    const int16_t accel_raw[3] = { raw.accel_x, raw.accel_y, raw.accel_z };
    const int16_t gyro_raw[3] = { raw.gyro_x, raw.gyro_y, raw.gyro_z };

    const ConfigStoreParams *params = ConfigStore_Get();

    for (uint8_t i = 0U; i < 3U; i++) {
        int32_t accel_mg = drivers::Icm45686_AccelMilliG(accel_raw[i], accel_fs);
        int32_t gyro_mdps = drivers::Icm45686_GyroMilliDps(gyro_raw[i], gyro_fs);
        if (params != 0) {
            const int32_t accel_bias[3] = {
                params->imu_accel_bias_x_mg,
                params->imu_accel_bias_y_mg,
                params->imu_accel_bias_z_mg,
            };
            const int32_t gyro_bias[3] = {
                params->imu_gyro_bias_x_mdps,
                params->imu_gyro_bias_y_mdps,
                params->imu_gyro_bias_z_mdps,
            };
            accel_mg -= accel_bias[i];
            gyro_mdps -= gyro_bias[i];
        }
        g_data.accel_mg[i] = accel_mg;
        g_data.gyro_mdps[i] = gyro_mdps;
    }

    g_data.temp_centi_c = drivers::Icm45686_TempCentiC(raw.temp);

    /* Tilt angles from the accelerometer (gravity reference), in degrees.
     * Pitch = atan2(-ax, az), Roll = atan2(ay, az); normalized to 0..360. */
    g_data.pitch_mdeg =
        Normalize360(Atan2MilliDeg(-g_data.accel_mg[0], g_data.accel_mg[2]));
    g_data.roll_mdeg =
        Normalize360(Atan2MilliDeg(g_data.accel_mg[1], g_data.accel_mg[2]));
    UpdateYaw(services::Time_Micros());
    g_data.valid = true;
}

const AppImuData *App_ImuGetData(void)
{
    return &g_data;
}

} /* namespace app */
