#ifndef GUGAH_APP_H_APP_H_
#define GUGAH_APP_H_APP_H_

#include <stdbool.h>
#include <stdint.h>

#include "config/h_config.h"
#include "control/ball_control.h"
#include "control/course_control.h"

namespace gugah {

enum HProblem : uint8_t {
    H_PROBLEM_2 = 2U,
    H_PROBLEM_3 = 3U,
    H_PROBLEM_4 = 4U,
    H_PROBLEM_5 = 5U,
    H_PROBLEM_6 = 6U
};

enum HRunState : uint8_t {
    H_STATE_BOOT = 0U,
    H_STATE_READY,
    H_STATE_PREFLIGHT,
    H_STATE_RUNNING,
    H_STATE_PASS,
    H_STATE_FAIL,
    H_STATE_ABORTED
};

enum HFailure : uint8_t {
    H_FAILURE_NONE = 0U,
    H_FAILURE_PREFLIGHT,
    H_FAILURE_HARDWARE,
    H_FAILURE_CHASSIS,
    H_FAILURE_VISION,
    H_FAILURE_DM,
    H_FAILURE_IMU,
    H_FAILURE_COURSE,
    H_FAILURE_BALL,
    H_FAILURE_TIMEOUT
};

struct HButtonEvents {
    bool b1_short;
    bool b1_long;
    bool b2_decrement;
    bool b3_increment;
    bool any_pressed;
};

struct HAppInput {
    uint32_t now_ms;
    HButtonEvents buttons;
    bool hardware_fault;
    bool chassis_fault;
    const drivers::GrayscaleProcessedData *line;
    bool line_frame_new;
    ChassisFeedback chassis;
    VisionFeedback vision;
    ImuFeedback imu;
    DmFeedback dm;
};

struct HAppOutput {
    MotionCommand motion;
    BallOutput ball;
    bool timer_running;
    bool result_changed;
};

struct HAppState {
    HRunState run_state;
    HProblem selected_problem;
    HFailure failure;
    CourseState course;
    BallState ball;
    int16_t h6_target_0p1mm;
    int16_t active_ball_target_0p1mm;
    int32_t chassis_accel_mm_s2;
    int16_t last_average_rpm;
    uint32_t last_accel_ms;
    uint32_t last_chassis_feedback_ms;
    uint32_t run_start_ms;
    uint32_t result_time_ms;
    int32_t maximum_ball_error_0p1mm;
    uint8_t h3_stage;
};

void HApp_Init(HAppState *state);
bool HApp_Start(HAppState *state,
                const HAppInput *input,
                const HConfig *config);
HAppOutput HApp_Abort(HAppState *state);
HAppOutput HApp_Update(HAppState *state,
                       const HAppInput *input,
                       const HConfig *config);
BallOutput HApp_UpdateBall2ms(HAppState *state,
                              const HAppInput *input,
                              const HConfig *config);
bool HApp_IsRunning(const HAppState *state);
const char *HApp_StateText(HRunState state);
const char *HApp_FailureText(HFailure failure);

} /* namespace gugah */

#endif /* GUGAH_APP_H_APP_H_ */
