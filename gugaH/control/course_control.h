#ifndef GUGAH_CONTROL_COURSE_CONTROL_H_
#define GUGAH_CONTROL_COURSE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "config/h_config.h"
#include "control/control_types.h"
#include "control/line_control.h"

namespace gugah {

enum CourseKind : uint8_t {
    COURSE_H2 = 2U,
    COURSE_H4 = 4U,
    COURSE_H5 = 5U,
    COURSE_H6 = 6U
};

enum CoursePhase : uint8_t {
    COURSE_IDLE = 0U,
    COURSE_CRUISE,
    COURSE_APPROACH,
    COURSE_FINAL_POSITION,
    COURSE_STOPPING,
    COURSE_COMPLETE,
    COURSE_FAILED,
    COURSE_ABORTED
};

enum CourseFailure : uint8_t {
    COURSE_FAILURE_NONE = 0U,
    COURSE_FAILURE_INVALID_INPUT,
    COURSE_FAILURE_LINE_LOST,
    COURSE_FAILURE_CHASSIS_STALE,
    COURSE_FAILURE_IMU_STALE,
    COURSE_FAILURE_FINISH_NOT_FOUND,
    COURSE_FAILURE_TIMEOUT
};

struct CourseInput {
    uint32_t now_ms;
    const drivers::GrayscaleProcessedData *line;
    const ChassisFeedback *chassis;
    bool line_frame_new;
    const ImuFeedback *imu;
};

struct CourseState {
    CourseKind kind;
    CoursePhase phase;
    CourseFailure failure;
    LineControlState line_control;
    int32_t start_left_count;
    int32_t start_right_count;
    int32_t distance_mm;
    int32_t final_left_count;
    int32_t final_right_count;
    int32_t h4_target_yaw_mdeg;
    int32_t h4_yaw_error_mdeg;
    int32_t h4_commanded_accel_mm_s2;
    int32_t h5_commanded_accel_mm_s2;
    int32_t h5_speed_limit_millirpm;
    int16_t h4_heading_correction_rpm;
    int16_t h5_road_ramp_rpm_s;
    uint32_t start_ms;
    uint32_t pass_ms;
    uint32_t h4_brake_start_ms;
    uint32_t h5_brake_start_ms;
    uint32_t settle_start_ms;
    uint32_t last_line_frame_ms;
    uint32_t h5_speed_limit_update_ms;
    uint8_t finish_confirm_frames;
    uint16_t h4_brake_start_rpm;
    uint16_t h5_brake_start_rpm;
    bool passed_b_or_a;
    bool h4_braking;
    bool h5_braking;
    bool h4_heading_locked;
    bool h5_curve_mode;
};

void Course_Init(CourseState *state);
bool Course_Start(CourseState *state,
                  CourseKind kind,
                  const ChassisFeedback *chassis,
                  uint32_t now_ms,
                  const HConfig *config);
MotionCommand Course_Update(CourseState *state,
                            const CourseInput *input,
                            const HConfig *config);
MotionCommand Course_Abort(CourseState *state);
bool Course_IsRunning(const CourseState *state);

} /* namespace gugah */

#endif /* GUGAH_CONTROL_COURSE_CONTROL_H_ */
