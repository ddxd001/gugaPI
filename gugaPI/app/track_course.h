#ifndef APP_TRACK_COURSE_H_
#define APP_TRACK_COURSE_H_

#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace app {

enum TrackCoursePhase : uint8_t {
    TRACK_COURSE_PHASE_IDLE = 0U,
    TRACK_COURSE_PHASE_CRUISE,
    TRACK_COURSE_PHASE_APPROACH,
    TRACK_COURSE_PHASE_STOPPED
};

enum TrackCourseResult : uint8_t {
    TRACK_COURSE_RESULT_IDLE = 0U,
    TRACK_COURSE_RESULT_RUNNING,
    TRACK_COURSE_RESULT_SUCCESS,
    TRACK_COURSE_RESULT_FAILED,
    TRACK_COURSE_RESULT_CANCELLED
};

enum TrackCourseCompletion : uint8_t {
    TRACK_COURSE_COMPLETION_NONE = 0U,
    TRACK_COURSE_COMPLETION_LINE,
    TRACK_COURSE_COMPLETION_ENCODER
};

enum TrackCourseFailure : uint8_t {
    TRACK_COURSE_FAILURE_NONE = 0U,
    TRACK_COURSE_FAILURE_PRECONDITION,
    TRACK_COURSE_FAILURE_SENSOR_STALE,
    TRACK_COURSE_FAILURE_ENCODER_STALE,
    TRACK_COURSE_FAILURE_LINEFOLLOW,
    TRACK_COURSE_FAILURE_RECOVERY_TIMEOUT,
    TRACK_COURSE_FAILURE_TIMEOUT,
    TRACK_COURSE_FAILURE_STOP
};

struct TrackCourseState {
    TrackCoursePhase phase;
    TrackCourseResult result;
    TrackCourseCompletion completion;
    TrackCourseFailure failure;
    int32_t left_distance_mm;
    int32_t right_distance_mm;
    int32_t average_distance_mm;
    int32_t start_left_count;
    int32_t start_right_count;
    uint16_t cruise_rpm;
    uint16_t approach_rpm;
    uint16_t lap_distance_mm;
    uint16_t approach_start_mm;
    uint16_t finish_gate_mm;
    uint8_t finish_mask;
    uint8_t finish_count;
    uint32_t last_grayscale_sequence;
    uint32_t start_ms;
    uint32_t stop_ms;
    uint32_t stop_sequence;
    drivers::DriverStatus last_status;
};

void TrackCourse_Init(void);
drivers::DriverStatus TrackCourse_CheckStartReady(
    uint32_t cruise_rpm,
    uint32_t approach_rpm,
    uint32_t lap_distance_mm);
drivers::DriverStatus TrackCourse_Start(uint32_t cruise_rpm,
                                        uint32_t approach_rpm,
                                        uint32_t lap_distance_mm);
drivers::DriverStatus TrackCourse_Cancel(void);
void TrackCourse_Update(void);
const TrackCourseState *TrackCourse_GetState(void);
const char *TrackCourse_PhaseText(TrackCoursePhase phase);
const char *TrackCourse_ResultText(TrackCourseResult result);
const char *TrackCourse_CompletionText(TrackCourseCompletion completion);
const char *TrackCourse_FailureText(TrackCourseFailure failure);

} /* namespace app */

#endif /* APP_TRACK_COURSE_H_ */
