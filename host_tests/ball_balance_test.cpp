#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "app/app_imu.h"
#include "app/ball_balance.h"
#include "app/ball_vision.h"
#include "app/config_store.h"
#include "app/dm_g6220_controller.h"

namespace {

uint32_t g_now_ms = 0U;
app::BallVisionData g_vision = {};
app::AppImuData g_imu = {};
bool g_vision_usable = true;
bool g_dm_acquired = false;
int32_t g_dm_target = 0;
uint32_t g_chassis_stops = 0U;
app::ConfigStoreParams g_config = {};

void SetDefaultBallConfig()
{
    g_config.ball_kp_mdeg_per_0p1mm = 10;
    g_config.ball_kd_mdeg_per_0p1mm_s = 3;
    g_config.ball_ki_mdeg_per_0p1mm_s = 0;
    g_config.ball_pitch_gain_permille = 0;
    g_config.ball_max_angle_mdeg = 8000;
    g_config.ball_degraded_angle_mdeg = 3000;
    g_config.ball_angle_slew_mdeg_s = 30000;
    g_config.ball_position_tolerance_0p1mm = 100;
    g_config.ball_velocity_tolerance_0p1mm_s = 100;
    g_config.ball_settle_ms = 200U;
    const int16_t angles[5] = { -8000, -4000, 0, 4000, 8000 };
    const int16_t positions[5] = { -1000, -500, 0, 500, 1000 };
    for (uint8_t i = 0U; i < 5U; i++) {
        g_config.ball_map_angle_mdeg[i] = angles[i];
        g_config.ball_map_dm_mrad[i] = positions[i];
    }
}

void SetBall(int16_t position)
{
    g_vision.ball_frame.sequence++;
    g_vision.ball_frame.flags = 7U;
    g_vision.ball_frame.position_0p1mm = position;
    g_vision.ball_frame.confidence = 1000U;
    g_vision.ball_frame.source_delay_ms = 0U;
    g_vision.ball_frame.received_ms = g_now_ms;
    g_vision.has_ball_sample = true;
    g_vision.ball_age_ms = 0U;
}

} /* namespace */

namespace services {

uint32_t Time_Millis(void)
{
    return g_now_ms;
}

} /* namespace services */

namespace app {

const ConfigStoreParams *ConfigStore_Get(void)
{
    return &g_config;
}

const BallVisionData *BallVision_GetData(void)
{
    return &g_vision;
}

bool BallVision_IsUsable(uint32_t)
{
    return g_vision_usable;
}

const AppImuData *App_ImuGetData(void)
{
    return &g_imu;
}

drivers::DriverStatus DmG6220Controller_ExternalAcquire(
    DmG6220ExternalOwner owner)
{
    assert(owner == DM_EXTERNAL_OWNER_BALL_BALANCE);
    if (g_dm_acquired) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    g_dm_acquired = true;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus DmG6220Controller_ExternalSetPosition(
    DmG6220ExternalOwner owner,
    int32_t target_mrad)
{
    if ((!g_dm_acquired) || (owner != DM_EXTERNAL_OWNER_BALL_BALANCE)) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    g_dm_target = target_mrad;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus DmG6220Controller_ExternalRelease(
    DmG6220ExternalOwner owner,
    bool)
{
    if ((!g_dm_acquired) || (owner != DM_EXTERNAL_OWNER_BALL_BALANCE)) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    g_dm_acquired = false;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Chassis_Stop(void)
{
    g_chassis_stops++;
    return drivers::DRIVER_OK;
}

} /* namespace app */

int main(void)
{
    using namespace app;

    SetDefaultBallConfig();
    BallBalance_Init();
    const BallBalanceParams *params = BallBalance_GetParams();
    assert(BallBalance_MapBeamToDm(params, -8000) == -1000);
    assert(BallBalance_MapBeamToDm(params, -2000) == -250);
    assert(BallBalance_MapBeamToDm(params, 2000) == 250);
    assert(BallBalance_MapBeamToDm(params, 9000) == 1000);

    SetBall(500);
    assert(BallBalance_StartMove(0, 5000U) == drivers::DRIVER_OK);
    assert(g_dm_acquired);
    g_now_ms += 10U;
    BallBalance_Update();
    const BallBalanceState *state = BallBalance_GetState();
    assert(state->mode == BALL_BALANCE_MOVE);
    assert(state->beam_target_mdeg < 0);
    assert(g_dm_target < 0);
    g_config.ball_kp_mdeg_per_0p1mm = 20;
    assert(BallBalance_GetParams()->kp_mdeg_per_0p1mm == 10);

    /* Feed a smooth approach and then enough zero-velocity samples to meet
     * the 200 ms settle qualification. */
    for (int16_t position = 450; position >= 0; position -= 50) {
        g_now_ms += 10U;
        SetBall(position);
        BallBalance_Update();
    }
    for (uint8_t i = 0U; i < 80U; i++) {
        g_now_ms += 10U;
        SetBall(0);
        BallBalance_Update();
    }
    assert(state->mode == BALL_BALANCE_HOLD);
    assert(state->result == BALL_BALANCE_RESULT_SUCCESS);
    assert(BallBalance_Stop(false) == drivers::DRIVER_OK);
    assert(!g_dm_acquired);

    SetBall(0);
    assert(BallBalance_StartHold(0) == drivers::DRIVER_OK);
    g_now_ms += 10U;
    SetBall(1151);
    BallBalance_Update();
    assert(state->mode == BALL_BALANCE_FAILED);
    assert(state->result == BALL_BALANCE_RESULT_ENDPOINT);
    assert(g_chassis_stops == 1U);
    assert(BallBalance_Stop(true) == drivers::DRIVER_OK);

    SetBall(0);
    g_vision_usable = true;
    assert(BallBalance_StartHold(0) == drivers::DRIVER_OK);
    g_vision_usable = false;
    BallBalance_Update();
    assert(state->result == BALL_BALANCE_RESULT_VISION_LOST);
    assert(g_chassis_stops == 2U);

    puts("ball balance ok");
    return 0;
}
