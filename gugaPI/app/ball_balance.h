#ifndef APP_BALL_BALANCE_H_
#define APP_BALL_BALANCE_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace app {

enum BallBalanceMode : uint8_t {
    BALL_BALANCE_IDLE = 0U,
    BALL_BALANCE_HOLD,
    BALL_BALANCE_MOVE,
    BALL_BALANCE_FAILED
};

enum BallBalanceResult : uint8_t {
    BALL_BALANCE_RESULT_IDLE = 0U,
    BALL_BALANCE_RESULT_RUNNING,
    BALL_BALANCE_RESULT_SUCCESS,
    BALL_BALANCE_RESULT_TIMEOUT,
    BALL_BALANCE_RESULT_VISION_LOST,
    BALL_BALANCE_RESULT_ENDPOINT,
    BALL_BALANCE_RESULT_DM_ERROR
};

struct BallBalanceParams {
    int16_t kp_mdeg_per_0p1mm;
    int16_t kd_mdeg_per_0p1mm_s;
    int16_t ki_mdeg_per_0p1mm_s;
    int16_t pitch_gain_permille;
    int16_t maximum_angle_mdeg;
    int16_t degraded_angle_mdeg;
    int32_t angle_slew_mdeg_s;
    int16_t position_tolerance_0p1mm;
    int16_t velocity_tolerance_0p1mm_s;
    uint16_t settle_ms;
    int16_t beam_angle_mdeg[5];
    int16_t dm_position_mrad[5];
};

struct BallBalanceState {
    bool initialized;
    BallBalanceMode mode;
    BallBalanceResult result;
    drivers::DriverStatus last_status;
    int16_t target_position_0p1mm;
    int32_t estimated_position_0p1mm;
    int32_t estimated_velocity_0p1mm_s;
    int32_t position_error_0p1mm;
    int32_t beam_target_mdeg;
    int32_t dm_target_mrad;
    int32_t integral_error_0p1mm_ms;
    int32_t maximum_abs_error_0p1mm;
    uint32_t start_ms;
    uint32_t timeout_ms;
    uint32_t settle_start_ms;
    uint32_t last_update_ms;
    uint32_t last_sample_ms;
    uint8_t last_sample_sequence;
    bool has_sample;
    bool settling;
};

void BallBalance_Init(void);
drivers::DriverStatus BallBalance_StartHold(int16_t target_0p1mm);
drivers::DriverStatus BallBalance_StartMove(int16_t target_0p1mm,
                                            uint32_t timeout_ms);
drivers::DriverStatus BallBalance_Stop(bool disable);
void BallBalance_EmergencyStop(void);
void BallBalance_Update(void);
bool BallBalance_IsActive(void);
const BallBalanceState *BallBalance_GetState(void);
const BallBalanceParams *BallBalance_GetParams(void);
drivers::DriverStatus BallBalance_SetParams(const BallBalanceParams *params);
int32_t BallBalance_MapBeamToDm(const BallBalanceParams *params,
                                int32_t beam_angle_mdeg);
const char *BallBalance_ModeText(BallBalanceMode mode);
const char *BallBalance_ResultText(BallBalanceResult result);

} /* namespace app */

#endif /* APP_BALL_BALANCE_H_ */
