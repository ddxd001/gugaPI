#include "app/ball_balance.h"

#include <limits.h>

#include "app/app_imu.h"
#include "app/ball_vision.h"
#include "app/chassis.h"
#include "app/config_store.h"
#include "app/dm_g6220_controller.h"
#include "services/time.h"

namespace app {
namespace {

static const int16_t kMaximumTarget0p1mm = 1000;
static const int16_t kEndpointLimit0p1mm = 1150;
static const uint32_t kControlPeriodMs = 10U;
static const int32_t kIntegralLimit0p1mmMs = 1000000;

const BallBalanceParams kDefaultParams = {
    10, 3, 0, 0,
    8000, 3000, 30000,
    100, 100, 200U,
    { -8000, -4000, 0, 4000, 8000 },
    { -1000, -500, 0, 500, 1000 }
};

BallBalanceParams g_params = kDefaultParams;
BallBalanceState g_state = {};

int32_t Abs32(int32_t value)
{
    if (value >= 0) {
        return value;
    }
    return (value == INT32_MIN) ? INT32_MAX : -value;
}

int32_t Clamp32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

int32_t StepToward(int32_t current, int32_t target, int32_t step)
{
    if (current < target) {
        return ((target - current) <= step) ? target : current + step;
    }
    if (current > target) {
        return ((current - target) <= step) ? target : current - step;
    }
    return current;
}

int32_t SignedPitchMdeg(void)
{
    const AppImuData *imu = App_ImuGetData();
    if ((imu == 0) || (!imu->valid)) {
        return 0;
    }
    int32_t pitch = imu->pitch_mdeg;
    if (pitch > 180000) {
        pitch -= 360000;
    }
    return pitch;
}

bool ParamsValid(const BallBalanceParams *params)
{
    if ((params == 0) || (params->kp_mdeg_per_0p1mm < 0) ||
        (params->kp_mdeg_per_0p1mm > 1000) ||
        (params->kd_mdeg_per_0p1mm_s < 0) ||
        (params->kd_mdeg_per_0p1mm_s > 1000) ||
        (params->ki_mdeg_per_0p1mm_s < 0) ||
        (params->ki_mdeg_per_0p1mm_s > 1000) ||
        (params->pitch_gain_permille < -2000) ||
        (params->pitch_gain_permille > 2000) ||
        (params->maximum_angle_mdeg < 100) ||
        (params->maximum_angle_mdeg > 15000) ||
        (params->degraded_angle_mdeg < 100) ||
        (params->degraded_angle_mdeg > params->maximum_angle_mdeg) ||
        (params->angle_slew_mdeg_s < 100) ||
        (params->angle_slew_mdeg_s > 180000) ||
        (params->position_tolerance_0p1mm < 1) ||
        (params->position_tolerance_0p1mm > 1000) ||
        (params->velocity_tolerance_0p1mm_s < 1) ||
        (params->settle_ms < 10U) || (params->settle_ms > 5000U)) {
        return false;
    }
    const bool dm_increasing =
        params->dm_position_mrad[1] > params->dm_position_mrad[0];
    if (params->dm_position_mrad[1] ==
        params->dm_position_mrad[0]) {
        return false;
    }
    for (uint8_t i = 1U; i < 5U; i++) {
        if ((params->beam_angle_mdeg[i] <=
             params->beam_angle_mdeg[i - 1U]) ||
            (params->beam_angle_mdeg[i] < -15000) ||
            (params->beam_angle_mdeg[i] > 15000) ||
            (params->dm_position_mrad[i] < -12500) ||
            (params->dm_position_mrad[i] > 12500)) {
            return false;
        }
        if (dm_increasing) {
            if (params->dm_position_mrad[i] <=
                params->dm_position_mrad[i - 1U]) {
                return false;
            }
        } else if (params->dm_position_mrad[i] >=
                   params->dm_position_mrad[i - 1U]) {
            return false;
        }
    }
    if ((params->beam_angle_mdeg[0] < -15000) ||
        (params->beam_angle_mdeg[0] > 15000) ||
        (params->dm_position_mrad[0] < -12500) ||
        (params->dm_position_mrad[0] > 12500)) {
        return false;
    }
    return true;
}

bool LoadConfiguredParams(BallBalanceParams *params)
{
    const ConfigStoreParams *config = ConfigStore_Get();
    if ((config == 0) || (params == 0)) {
        return false;
    }
    params->kp_mdeg_per_0p1mm =
        config->ball_kp_mdeg_per_0p1mm;
    params->kd_mdeg_per_0p1mm_s =
        config->ball_kd_mdeg_per_0p1mm_s;
    params->ki_mdeg_per_0p1mm_s =
        config->ball_ki_mdeg_per_0p1mm_s;
    params->pitch_gain_permille =
        config->ball_pitch_gain_permille;
    params->maximum_angle_mdeg =
        config->ball_max_angle_mdeg;
    params->degraded_angle_mdeg =
        config->ball_degraded_angle_mdeg;
    params->angle_slew_mdeg_s =
        config->ball_angle_slew_mdeg_s;
    params->position_tolerance_0p1mm =
        config->ball_position_tolerance_0p1mm;
    params->velocity_tolerance_0p1mm_s =
        config->ball_velocity_tolerance_0p1mm_s;
    params->settle_ms = config->ball_settle_ms;
    for (uint8_t i = 0U; i < 5U; i++) {
        params->beam_angle_mdeg[i] =
            config->ball_map_angle_mdeg[i];
        params->dm_position_mrad[i] =
            config->ball_map_dm_mrad[i];
    }
    return ParamsValid(params);
}

void UpdateEstimator(const drivers::BallVisionFrame &sample,
                     uint32_t now_ms)
{
    const uint32_t sample_ms = sample.received_ms - sample.source_delay_ms;
    if ((!g_state.has_sample) ||
        (sample.sequence != g_state.last_sample_sequence)) {
        if (g_state.has_sample) {
            const uint32_t elapsed = sample_ms - g_state.last_sample_ms;
            if ((elapsed != 0U) && (elapsed <= 200U)) {
                const int32_t raw_velocity = static_cast<int32_t>(
                    (static_cast<int64_t>(
                        sample.position_0p1mm -
                        g_state.estimated_position_0p1mm) * 1000LL) /
                    elapsed);
                g_state.estimated_velocity_0p1mm_s =
                    (g_state.estimated_velocity_0p1mm_s * 3 +
                     raw_velocity) / 4;
            }
        } else {
            g_state.estimated_velocity_0p1mm_s = 0;
        }
        g_state.estimated_position_0p1mm = sample.position_0p1mm;
        g_state.last_sample_ms = sample_ms;
        g_state.last_sample_sequence = sample.sequence;
        g_state.has_sample = true;
    }

    const uint32_t prediction_ms = now_ms - g_state.last_sample_ms;
    if (prediction_ms <= 120U) {
        g_state.estimated_position_0p1mm =
            sample.position_0p1mm +
            static_cast<int32_t>(
                (static_cast<int64_t>(
                    g_state.estimated_velocity_0p1mm_s) *
                 prediction_ms) / 1000LL);
    }
}

void Fail(BallBalanceResult result)
{
    g_state.mode = BALL_BALANCE_FAILED;
    g_state.result = result;
    g_state.last_status = drivers::DRIVER_ERROR;
    g_state.beam_target_mdeg = 0;
    g_state.dm_target_mrad = BallBalance_MapBeamToDm(&g_params, 0);
    (void) DmG6220Controller_ExternalSetPosition(
        DM_EXTERNAL_OWNER_BALL_BALANCE,
        g_state.dm_target_mrad);
    (void) Chassis_Stop();
}

drivers::DriverStatus Start(BallBalanceMode mode,
                            int16_t target_0p1mm,
                            uint32_t timeout_ms)
{
    if ((target_0p1mm < -kMaximumTarget0p1mm) ||
        (target_0p1mm > kMaximumTarget0p1mm) ||
        ((mode != BALL_BALANCE_HOLD) && (mode != BALL_BALANCE_MOVE))) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    BallBalanceParams configured;
    if (!LoadConfiguredParams(&configured)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    const uint32_t now_ms = services::Time_Millis();
    const BallVisionData *vision = BallVision_GetData();
    if ((vision == 0) || !BallVision_IsUsable(now_ms) ||
        (Abs32(vision->ball_frame.position_0p1mm) >
         kEndpointLimit0p1mm)) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    const drivers::DriverStatus acquire =
        DmG6220Controller_ExternalAcquire(
            DM_EXTERNAL_OWNER_BALL_BALANCE);
    if (acquire != drivers::DRIVER_OK) {
        return acquire;
    }

    g_params = configured;
    g_state = {};
    g_state.initialized = true;
    g_state.mode = mode;
    g_state.result = (mode == BALL_BALANCE_HOLD)
        ? BALL_BALANCE_RESULT_SUCCESS : BALL_BALANCE_RESULT_RUNNING;
    g_state.last_status = drivers::DRIVER_OK;
    g_state.target_position_0p1mm = target_0p1mm;
    g_state.start_ms = now_ms;
    g_state.timeout_ms = timeout_ms;
    g_state.last_update_ms = now_ms;
    UpdateEstimator(vision->ball_frame, now_ms);
    return drivers::DRIVER_OK;
}

} /* namespace */

void BallBalance_Init(void)
{
    if (!LoadConfiguredParams(&g_params)) {
        g_params = kDefaultParams;
    }
    g_state = {};
    g_state.initialized = true;
    g_state.mode = BALL_BALANCE_IDLE;
    g_state.result = BALL_BALANCE_RESULT_IDLE;
    g_state.last_status = drivers::DRIVER_OK;
}

drivers::DriverStatus BallBalance_StartHold(int16_t target_0p1mm)
{
    return Start(BALL_BALANCE_HOLD, target_0p1mm, 0U);
}

drivers::DriverStatus BallBalance_StartMove(int16_t target_0p1mm,
                                            uint32_t timeout_ms)
{
    if ((timeout_ms < 50U) || (timeout_ms > 30000U)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    return Start(BALL_BALANCE_MOVE, target_0p1mm, timeout_ms);
}

drivers::DriverStatus BallBalance_Stop(bool disable)
{
    drivers::DriverStatus status = drivers::DRIVER_OK;
    if (BallBalance_IsActive() ||
        (g_state.mode == BALL_BALANCE_FAILED)) {
        status = DmG6220Controller_ExternalRelease(
            DM_EXTERNAL_OWNER_BALL_BALANCE, disable);
    }
    g_state.mode = BALL_BALANCE_IDLE;
    g_state.result = BALL_BALANCE_RESULT_IDLE;
    g_state.integral_error_0p1mm_ms = 0;
    g_state.settling = false;
    g_state.last_status = status;
    return status;
}

void BallBalance_EmergencyStop(void)
{
    if (BallBalance_IsActive() ||
        (g_state.mode == BALL_BALANCE_FAILED)) {
        (void) DmG6220Controller_ExternalRelease(
            DM_EXTERNAL_OWNER_BALL_BALANCE, true);
    }
    g_state.mode = BALL_BALANCE_IDLE;
    g_state.result = BALL_BALANCE_RESULT_IDLE;
}

void BallBalance_Update(void)
{
    if (!BallBalance_IsActive()) {
        return;
    }
    const uint32_t now_ms = services::Time_Millis();
    const BallVisionData *vision = BallVision_GetData();
    if ((vision == 0) || !BallVision_IsUsable(now_ms)) {
        Fail(BALL_BALANCE_RESULT_VISION_LOST);
        return;
    }
    if (Abs32(vision->ball_frame.position_0p1mm) >
        kEndpointLimit0p1mm) {
        Fail(BALL_BALANCE_RESULT_ENDPOINT);
        return;
    }

    UpdateEstimator(vision->ball_frame, now_ms);
    uint32_t elapsed = now_ms - g_state.last_update_ms;
    if (elapsed == 0U) {
        elapsed = kControlPeriodMs;
    } else if (elapsed > 100U) {
        elapsed = 100U;
    }
    g_state.last_update_ms = now_ms;
    g_state.position_error_0p1mm =
        g_state.target_position_0p1mm -
        g_state.estimated_position_0p1mm;
    const int32_t abs_error = Abs32(g_state.position_error_0p1mm);
    if (abs_error > g_state.maximum_abs_error_0p1mm) {
        g_state.maximum_abs_error_0p1mm = abs_error;
    }

    const bool degraded = vision->ball_age_ms > 60U;
    if (!degraded) {
        const int64_t integral =
            static_cast<int64_t>(g_state.integral_error_0p1mm_ms) +
            static_cast<int64_t>(g_state.position_error_0p1mm) *
            elapsed;
        g_state.integral_error_0p1mm_ms = Clamp32(
            static_cast<int32_t>(integral),
            -kIntegralLimit0p1mmMs,
            kIntegralLimit0p1mmMs);
    }

    int64_t angle =
        static_cast<int64_t>(g_params.kp_mdeg_per_0p1mm) *
            g_state.position_error_0p1mm -
        static_cast<int64_t>(g_params.kd_mdeg_per_0p1mm_s) *
            g_state.estimated_velocity_0p1mm_s +
        (static_cast<int64_t>(g_params.ki_mdeg_per_0p1mm_s) *
         g_state.integral_error_0p1mm_ms) / 1000LL -
        (static_cast<int64_t>(g_params.pitch_gain_permille) *
         SignedPitchMdeg()) / 1000LL;
    const int32_t angle_limit = degraded
        ? g_params.degraded_angle_mdeg : g_params.maximum_angle_mdeg;
    angle = Clamp32(static_cast<int32_t>(angle),
                    -angle_limit, angle_limit);
    int32_t slew_step = static_cast<int32_t>(
        (static_cast<int64_t>(g_params.angle_slew_mdeg_s) *
         elapsed) / 1000LL);
    if (slew_step < 1) {
        slew_step = 1;
    }
    g_state.beam_target_mdeg = StepToward(
        g_state.beam_target_mdeg,
        static_cast<int32_t>(angle),
        slew_step);
    g_state.dm_target_mrad =
        BallBalance_MapBeamToDm(&g_params, g_state.beam_target_mdeg);
    const drivers::DriverStatus status =
        DmG6220Controller_ExternalSetPosition(
            DM_EXTERNAL_OWNER_BALL_BALANCE,
            g_state.dm_target_mrad);
    g_state.last_status = status;
    if (status != drivers::DRIVER_OK) {
        Fail(BALL_BALANCE_RESULT_DM_ERROR);
        return;
    }

    if (g_state.mode != BALL_BALANCE_MOVE) {
        return;
    }
    if ((now_ms - g_state.start_ms) > g_state.timeout_ms) {
        Fail(BALL_BALANCE_RESULT_TIMEOUT);
        return;
    }
    const bool settled =
        (abs_error <= g_params.position_tolerance_0p1mm) &&
        (Abs32(g_state.estimated_velocity_0p1mm_s) <=
         g_params.velocity_tolerance_0p1mm_s);
    if (!settled) {
        g_state.settling = false;
        g_state.settle_start_ms = 0U;
    } else if (!g_state.settling) {
        g_state.settling = true;
        g_state.settle_start_ms = now_ms;
    } else if ((now_ms - g_state.settle_start_ms) >=
               g_params.settle_ms) {
        g_state.mode = BALL_BALANCE_HOLD;
        g_state.result = BALL_BALANCE_RESULT_SUCCESS;
        g_state.settling = false;
    }
}

bool BallBalance_IsActive(void)
{
    return (g_state.mode == BALL_BALANCE_HOLD) ||
           (g_state.mode == BALL_BALANCE_MOVE);
}

const BallBalanceState *BallBalance_GetState(void)
{
    return &g_state;
}

const BallBalanceParams *BallBalance_GetParams(void)
{
    return &g_params;
}

drivers::DriverStatus BallBalance_SetParams(const BallBalanceParams *params)
{
    if (!ParamsValid(params)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (BallBalance_IsActive()) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    g_params = *params;
    return drivers::DRIVER_OK;
}

int32_t BallBalance_MapBeamToDm(const BallBalanceParams *params,
                                int32_t beam_angle_mdeg)
{
    if (params == 0) {
        return 0;
    }
    if (beam_angle_mdeg <= params->beam_angle_mdeg[0]) {
        return params->dm_position_mrad[0];
    }
    for (uint8_t i = 1U; i < 5U; i++) {
        if (beam_angle_mdeg <= params->beam_angle_mdeg[i]) {
            const int32_t x0 = params->beam_angle_mdeg[i - 1U];
            const int32_t x1 = params->beam_angle_mdeg[i];
            const int32_t y0 = params->dm_position_mrad[i - 1U];
            const int32_t y1 = params->dm_position_mrad[i];
            return y0 + static_cast<int32_t>(
                (static_cast<int64_t>(beam_angle_mdeg - x0) *
                 (y1 - y0)) / (x1 - x0));
        }
    }
    return params->dm_position_mrad[4];
}

const char *BallBalance_ModeText(BallBalanceMode mode)
{
    switch (mode) {
    case BALL_BALANCE_IDLE: return "idle";
    case BALL_BALANCE_HOLD: return "hold";
    case BALL_BALANCE_MOVE: return "move";
    case BALL_BALANCE_FAILED: return "failed";
    default: return "unknown";
    }
}

const char *BallBalance_ResultText(BallBalanceResult result)
{
    switch (result) {
    case BALL_BALANCE_RESULT_IDLE: return "idle";
    case BALL_BALANCE_RESULT_RUNNING: return "running";
    case BALL_BALANCE_RESULT_SUCCESS: return "success";
    case BALL_BALANCE_RESULT_TIMEOUT: return "timeout";
    case BALL_BALANCE_RESULT_VISION_LOST: return "vision_lost";
    case BALL_BALANCE_RESULT_ENDPOINT: return "endpoint";
    case BALL_BALANCE_RESULT_DM_ERROR: return "dm_error";
    default: return "unknown";
    }
}

} /* namespace app */
