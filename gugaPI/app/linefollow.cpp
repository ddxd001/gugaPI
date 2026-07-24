#include "app/linefollow.h"

#include "app/app_grayscale.h"
#include "app/chassis.h"
#include "drivers/common/driver_status.h"
#include "services/fault.h"
#include "services/time.h"

namespace app {
namespace {

/* Line polarity: +1 = line is LOW adc (black surface, weak reflectance), so
 * blackness = (max - value)/(max-min). Set -1 if the line reads HIGH adc. */
static const int32_t kLinePolarity = 1;
/* Channel-order sign: +1 = channel 0 on one side; flip to -1 if the robot
 * steers the wrong way during bench testing. */
static const int32_t kLineSign = 1;

static const uint32_t kCalDurationMs = 2000U;
static const int32_t kMinLineWeight = 1000;   /* >=1 channel fully black */
static const int32_t kScale = 1000000;        /* correction = error * kp / kScale */
static const uint8_t kChannelCount = 8U;

LFState g_state;

int32_t ClampInt32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

void ResetCalData(void)
{
    for (uint8_t i = 0U; i < kChannelCount; i++) {
        g_state.cal_min[i] = 0xFFFFU;
        g_state.cal_max[i] = 0U;
        g_state.threshold[i] = 0U;
    }
    g_state.calibrated = false;
}

void SafetyStop(services::FaultCode code)
{
    g_state.mode = LF_IDLE;
    (void) Chassis_Stop();
    if (code != services::FAULT_NONE) {
        services::Fault_Set(code);
    }
}

/* Blackness of one channel, 0..1000 (1000 = fully on the line). Uses the
 * calibrated min/max for that channel; clamps if adc drifts outside. */
int32_t ChannelBlackness(uint8_t i, uint16_t value)
{
    const uint16_t lo = g_state.cal_min[i];
    const uint16_t hi = g_state.cal_max[i];
    if (hi <= lo) {
        return 0;   /* uncalibrated / flat channel */
    }
    const int32_t range = static_cast<int32_t>(hi) - static_cast<int32_t>(lo);
    int32_t b;
    if (kLinePolarity > 0) {
        /* line = low adc: blacker when value is smaller */
        b = (static_cast<int32_t>(hi) - static_cast<int32_t>(value)) * 1000 /
            range;
    } else {
        b = (static_cast<int32_t>(value) - static_cast<int32_t>(lo)) * 1000 /
            range;
    }
    return ClampInt32(b, 0, 1000);
}

} /* namespace */

void LF_Init(void)
{
    g_state.mode = LF_IDLE;
    g_state.calibrated = false;
    g_state.error_mpos = 0;
    g_state.correction_rpm = 0;
    g_state.lost = true;
    g_state.base_rpm = 0;
    g_state.follow_start_ms = 0U;
    g_state.follow_duration_ms = 0U;
    g_state.cal_start_ms = 0U;
    g_state.lost_since_ms = 0U;
    g_state.kp = 10000;            /* 1 channel error (~1000 mpos) -> 10 RPM */
    g_state.max_correction_rpm = 30;
    g_state.lost_timeout_ms = 500U;
    g_state.last_status = drivers::DRIVER_OK;
    ResetCalData();
}

drivers::DriverStatus LF_CalibrateStart(void)
{
    if (g_state.mode != LF_IDLE) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    ResetCalData();
    g_state.mode = LF_CAL;
    g_state.cal_start_ms = services::Time_Millis();
    return drivers::DRIVER_OK;
}

drivers::DriverStatus LF_Start(int32_t base_rpm, uint32_t duration_ms)
{
    if (g_state.mode != LF_IDLE) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (!g_state.calibrated) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (services::Fault_HasFault()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    g_state.mode = LF_FOLLOW;
    g_state.base_rpm = base_rpm;
    g_state.follow_start_ms = services::Time_Millis();
    g_state.follow_duration_ms = duration_ms;
    g_state.correction_rpm = 0;
    g_state.lost_since_ms = 0U;
    g_state.lost = false;
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus LF_Stop(void)
{
    g_state.mode = LF_IDLE;
    const drivers::DriverStatus s = Chassis_Stop();
    g_state.last_status = s;
    return s;
}

void LF_Update(void)
{
    if (g_state.mode == LF_IDLE) {
        return;
    }

    const uint32_t now = services::Time_Millis();

    if (services::Fault_HasFault()) {
        SafetyStop(services::FAULT_NONE);
        return;
    }

    const AppGrayscaleData *g = App_GrayscaleGetData();
    if ((g == 0) || (!g->valid)) {
        SafetyStop(services::FAULT_SENSOR_LOST);
        return;
    }

    if (g_state.mode == LF_CAL) {
        for (uint8_t i = 0U; i < kChannelCount; i++) {
            const uint16_t v = g->raw[i];
            if (v < g_state.cal_min[i]) {
                g_state.cal_min[i] = v;
            }
            if (v > g_state.cal_max[i]) {
                g_state.cal_max[i] = v;
            }
        }
        if ((now - g_state.cal_start_ms) >= kCalDurationMs) {
            for (uint8_t i = 0U; i < kChannelCount; i++) {
                g_state.threshold[i] =
                    static_cast<uint16_t>(
                        (static_cast<uint32_t>(g_state.cal_min[i]) +
                         static_cast<uint32_t>(g_state.cal_max[i])) /
                        2U);
            }
            g_state.calibrated = true;
            g_state.mode = LF_IDLE;
        }
        return;
    }

    /* LF_FOLLOW */
    if ((now - g_state.follow_start_ms) >= g_state.follow_duration_ms) {
        (void) LF_Stop();
        return;
    }

    int32_t total = 0;
    int32_t weighted = 0;
    for (uint8_t i = 0U; i < kChannelCount; i++) {
        const int32_t b = ChannelBlackness(i, g->raw[i]);
        weighted += static_cast<int32_t>(i) * b;
        total += b;
    }

    int32_t correction;
    if (total < kMinLineWeight) {
        g_state.lost = true;
        g_state.error_mpos = 0;
        if (g_state.lost_since_ms == 0U) {
            g_state.lost_since_ms = now;
        }
        if ((now - g_state.lost_since_ms) > g_state.lost_timeout_ms) {
            (void) LF_Stop();
            return;
        }
        /* retain last steering while searching */
        correction = g_state.correction_rpm;
    } else {
        g_state.lost = false;
        g_state.lost_since_ms = 0U;
        const int32_t pos_milli = (total != 0) ? (weighted * 1000) / total : 3500;
        g_state.error_mpos = (pos_milli - 3500) * kLineSign;
        correction = g_state.error_mpos * g_state.kp / kScale;
        correction = ClampInt32(correction,
                                 -g_state.max_correction_rpm,
                                 g_state.max_correction_rpm);
        g_state.correction_rpm = correction;
    }

    const int32_t left = g_state.base_rpm - correction;
    const int32_t right = g_state.base_rpm + correction;
    const drivers::DriverStatus s = Chassis_SetWheelRpm(left, right);
    g_state.last_status = s;
    if (s != drivers::DRIVER_OK) {
        SafetyStop(services::FAULT_NONE);
    }
}

const LFState *LF_GetState(void)
{
    return &g_state;
}

bool LF_IsLineDetected(void)
{
    if (!g_state.calibrated) {
        return false;
    }
    const AppGrayscaleData *g = App_GrayscaleGetData();
    if ((g == 0) || (!g->valid)) {
        return false;
    }
    int32_t total = 0;
    for (uint8_t i = 0U; i < kChannelCount; i++) {
        total += ChannelBlackness(i, g->raw[i]);
    }
    return (total >= kMinLineWeight);
}

void LF_SetKp(int32_t kp)
{
    if (kp >= 0) {
        g_state.kp = kp;
    }
}

void LF_SetMaxCorrection(int32_t max_correction_rpm)
{
    if (max_correction_rpm >= 0) {
        g_state.max_correction_rpm = max_correction_rpm;
    }
}

void LF_SetLostTimeout(uint32_t timeout_ms)
{
    g_state.lost_timeout_ms = timeout_ms;
}

} /* namespace app */
