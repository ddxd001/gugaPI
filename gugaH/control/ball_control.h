#ifndef GUGAH_CONTROL_BALL_CONTROL_H_
#define GUGAH_CONTROL_BALL_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "config/h_config.h"
#include "control/control_types.h"

namespace gugah {

enum BallMode : uint8_t {
    BALL_IDLE = 0U,
    BALL_HOLD,
    BALL_MOVE,
    BALL_LEVEL,
    BALL_FAILED
};

enum BallResult : uint8_t {
    BALL_RESULT_IDLE = 0U,
    BALL_RESULT_RUNNING,
    BALL_RESULT_SUCCESS,
    BALL_RESULT_TIMEOUT,
    BALL_RESULT_VISION_LOST,
    BALL_RESULT_ENDPOINT,
    BALL_RESULT_DM_STALE,
    BALL_RESULT_IMU_STALE
};

struct BallInput {
    uint32_t now_ms;
    VisionFeedback vision;
    ImuFeedback imu;
    DmFeedback dm;
    int32_t chassis_accel_mm_s2;
};

struct BallOutput {
    bool command_valid;
    bool stop_chassis;
    int32_t dm_target_mrad;
};

struct BallState {
    BallMode mode;
    BallResult result;
    int16_t target_position_0p1mm;
    int32_t measured_position_0p1mm;
    int32_t estimated_position_0p1mm;
    int32_t estimated_velocity_0p1mm_s;
    int32_t position_error_0p1mm;
    int32_t target_velocity_0p1mm_s;
    int32_t velocity_error_0p1mm_s;
    int32_t desired_acceleration_0p1mm_s2;
    int32_t model_acceleration_0p1mm_s2;
    int32_t rail_compensation_mdeg;
    int32_t observer_position_0p1mm;
    int32_t observer_velocity_0p1mm_s;
    int32_t measured_velocity_0p1mm_s;
    int32_t observer_position_q8;
    int32_t observer_velocity_q8;
    int32_t stiction_compensation_mdeg;
    int32_t integral_error_0p1mm_ms;
    int32_t pid_p_mdeg;
    int32_t pid_i_mdeg;
    int32_t pid_d_mdeg;
    int32_t pid_correction_mdeg;
    int32_t chassis_feedforward_mdeg;
    int32_t beam_target_mdeg;
    int32_t dm_target_mrad;
    int32_t maximum_abs_error_0p1mm;
    uint32_t start_ms;
    uint32_t timeout_ms;
    uint32_t last_update_ms;
    uint32_t last_sample_ms;
    uint32_t estimator_update_ms;
    uint32_t settle_start_ms;
    uint32_t stationary_start_ms;
    int16_t position_history_0p1mm[5];
    uint32_t sample_history_ms[5];
    uint8_t last_sequence;
    uint8_t sample_history_count;
    bool has_sample;
};

void Ball_Init(BallState *state);
bool Ball_StartHold(BallState *state,
                    int16_t target_0p1mm,
                    const BallInput *input,
                    const HConfig *config);
bool Ball_StartMove(BallState *state,
                    int16_t target_0p1mm,
                    uint32_t timeout_ms,
                    const BallInput *input,
                    const HConfig *config);
void Ball_Stop(BallState *state);
void Ball_Level(BallState *state);
BallOutput Ball_Update(BallState *state,
                       const BallInput *input,
                       const HConfig *config);
bool Ball_IsActive(const BallState *state);
int32_t Ball_MapBeamToDm(const HConfig *config, int32_t beam_angle_mdeg);
int32_t Ball_HoldAngleMdeg(const HConfig *config,
                          int32_t position_0p1mm);
int16_t Ball_CalibrateCameraPosition0p1mm(int16_t camera_position_0p1mm);

} /* namespace gugah */

#endif /* GUGAH_CONTROL_BALL_CONTROL_H_ */
