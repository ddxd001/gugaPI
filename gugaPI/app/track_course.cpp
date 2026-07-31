#include "app/track_course.h"

#include <limits.h>

#include "app/app_grayscale.h"
#include "app/chassis.h"
#include "app/line_sensor.h"
#include "app/linefollow.h"
#include "drivers/grayscale/grayscale_processing.h"
#include "services/fault.h"
#include "services/time.h"

namespace app {
namespace {

static const uint32_t kChassisMaximumAgeMs = 100U;
static const uint32_t kCourseTimeoutMs = 30000U;
static const uint32_t kRecoveryMaximumMs = 500U;
static const uint16_t kApproachMarginMm = 900U;
static const uint16_t kFinishGateMarginMm = 1200U;
static const uint8_t kFinishCenterMask = 0x7EU; /* physical CH1..CH6 */
static const uint8_t kFinishMinimumChannels = 5U;
static const uint8_t kFinishConfirmFrames = 2U;

TrackCourseState g_state = {};

int32_t EncoderDeltaToMillimeters(int32_t delta_counts,
                                  uint32_t wheel_radius_um,
                                  uint32_t counts_per_rev)
{
    if ((wheel_radius_um == 0U) || (counts_per_rev == 0U)) {
        return 0;
    }
    static const int64_t kPiNumerator = 3141593LL;
    static const int64_t kPiDenominator = 1000000LL;
    const int64_t numerator =
        static_cast<int64_t>(delta_counts) * 2LL * kPiNumerator *
        static_cast<int64_t>(wheel_radius_um);
    const int64_t denominator =
        kPiDenominator * static_cast<int64_t>(counts_per_rev) * 1000LL;
    if (numerator >= 0) {
        return static_cast<int32_t>(
            (numerator + denominator / 2LL) / denominator);
    }
    return static_cast<int32_t>(
        (numerator - denominator / 2LL) / denominator);
}

bool ChassisFeedbackFresh(const ChassisState *chassis, uint32_t now)
{
    return (chassis != 0) && chassis->initialized &&
           (chassis->feedback_sequence != 0U) &&
           (chassis->last_feedback_status == drivers::DRIVER_OK) &&
           ((now - chassis->last_feedback_ms) <= kChassisMaximumAgeMs) &&
           (chassis->config.wheel_radius_um != 0U) &&
           (chassis->config.left_counts_per_rev != 0U) &&
           (chassis->config.right_counts_per_rev != 0U);
}

uint8_t CountBits(uint8_t value)
{
    uint8_t count = 0U;
    while (value != 0U) {
        count = static_cast<uint8_t>(count + (value & 1U));
        value = static_cast<uint8_t>(value >> 1U);
    }
    return count;
}

uint8_t LongestRun(uint8_t value)
{
    uint8_t longest = 0U;
    uint8_t current = 0U;
    for (uint8_t i = 0U; i < 8U; i++) {
        if ((value & static_cast<uint8_t>(1U << i)) != 0U) {
            current++;
            if (current > longest) {
                longest = current;
            }
        } else {
            current = 0U;
        }
    }
    return longest;
}

void RecordStop(void)
{
    g_state.stop_ms = services::Time_Millis();
    if (g_state.stop_sequence != UINT32_MAX) {
        g_state.stop_sequence++;
    }
}

void Fail(TrackCourseFailure failure, drivers::DriverStatus status)
{
    const drivers::DriverStatus stop_status = LF_Stop();
    g_state.phase = TRACK_COURSE_PHASE_STOPPED;
    g_state.result = TRACK_COURSE_RESULT_FAILED;
    g_state.completion = TRACK_COURSE_COMPLETION_NONE;
    g_state.failure = (stop_status == drivers::DRIVER_OK)
        ? failure : TRACK_COURSE_FAILURE_STOP;
    g_state.last_status = (stop_status == drivers::DRIVER_OK)
        ? status : stop_status;
    RecordStop();
}

void Complete(TrackCourseCompletion completion)
{
    const drivers::DriverStatus stop_status = LF_Stop();
    g_state.phase = TRACK_COURSE_PHASE_STOPPED;
    g_state.completion = completion;
    if (stop_status == drivers::DRIVER_OK) {
        g_state.result = TRACK_COURSE_RESULT_SUCCESS;
        g_state.failure = TRACK_COURSE_FAILURE_NONE;
        g_state.last_status = drivers::DRIVER_OK;
    } else {
        g_state.result = TRACK_COURSE_RESULT_FAILED;
        g_state.failure = TRACK_COURSE_FAILURE_STOP;
        g_state.last_status = stop_status;
    }
    RecordStop();
}

void UpdateDistance(const ChassisState *chassis)
{
    g_state.left_distance_mm = EncoderDeltaToMillimeters(
        chassis->left.encoder_count - g_state.start_left_count,
        chassis->config.wheel_radius_um,
        chassis->config.left_counts_per_rev);
    g_state.right_distance_mm = EncoderDeltaToMillimeters(
        chassis->right.encoder_count - g_state.start_right_count,
        chassis->config.wheel_radius_um,
        chassis->config.right_counts_per_rev);
    g_state.average_distance_mm = static_cast<int32_t>(
        (static_cast<int64_t>(g_state.left_distance_mm) +
         static_cast<int64_t>(g_state.right_distance_mm)) / 2LL);
}

bool UpdateFinishEvidence(const AppGrayscaleData *gray)
{
    if ((gray == 0) || !gray->valid || !gray->processed_valid ||
        (gray->processing_status != drivers::DRIVER_OK) ||
        (gray->sequence == g_state.last_grayscale_sequence)) {
        return false;
    }
    g_state.last_grayscale_sequence = gray->sequence;
    g_state.finish_mask =
        static_cast<uint8_t>(gray->active_mask & kFinishCenterMask);

    const bool finish =
        (CountBits(g_state.finish_mask) >= kFinishMinimumChannels) &&
        (LongestRun(g_state.finish_mask) >= kFinishMinimumChannels);
    if (finish) {
        if (g_state.finish_count < UINT8_MAX) {
            g_state.finish_count++;
        }
    } else {
        g_state.finish_count = 0U;
    }
    return g_state.finish_count >= kFinishConfirmFrames;
}

} /* namespace */

void TrackCourse_Init(void)
{
    g_state = {};
    g_state.phase = TRACK_COURSE_PHASE_IDLE;
    g_state.result = TRACK_COURSE_RESULT_IDLE;
    g_state.last_status = drivers::DRIVER_OK;
}

drivers::DriverStatus TrackCourse_CheckStartReady(
    uint32_t cruise_rpm,
    uint32_t approach_rpm,
    uint32_t lap_distance_mm)
{
    const uint32_t now = services::Time_Millis();
    const ChassisState *chassis = Chassis_GetState();
    const LineSensorSnapshot *line = LineSensor_GetSnapshot();
    const AppGrayscaleData *gray = App_GrayscaleGetData();
    if ((cruise_rpm == 0U) || (approach_rpm == 0U) ||
        (approach_rpm > cruise_rpm) || (lap_distance_mm < 3000U) ||
        (lap_distance_mm > 8000U) || (chassis == 0) ||
        (cruise_rpm > chassis->config.max_wheel_rpm)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (services::Fault_HasFault() ||
        !LineSensor_IsReadyForMotion() ||
        !ChassisFeedbackFresh(chassis, now) || (line == 0) ||
        !line->valid || !line->fresh || !line->calibrated ||
        !line->line_detected || !line->position_valid ||
        (line->track_state != drivers::GRAYSCALE_TRACK_VALID) ||
        (line->channel_anomaly_mask != 0U) || (gray == 0) ||
        !gray->valid || !gray->processed_valid) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    return drivers::DRIVER_OK;
}

drivers::DriverStatus TrackCourse_Start(uint32_t cruise_rpm,
                                        uint32_t approach_rpm,
                                        uint32_t lap_distance_mm)
{
    if (g_state.result == TRACK_COURSE_RESULT_RUNNING) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    const drivers::DriverStatus readiness = TrackCourse_CheckStartReady(
        cruise_rpm, approach_rpm, lap_distance_mm);
    if (readiness != drivers::DRIVER_OK) {
        g_state.result = TRACK_COURSE_RESULT_FAILED;
        g_state.failure = TRACK_COURSE_FAILURE_PRECONDITION;
        g_state.last_status = readiness;
        return g_state.last_status;
    }

    const uint32_t now = services::Time_Millis();
    const ChassisState *chassis = Chassis_GetState();
    const AppGrayscaleData *gray = App_GrayscaleGetData();
    const uint32_t prior_stop_sequence = g_state.stop_sequence;
    g_state = {};
    g_state.stop_sequence = prior_stop_sequence;
    g_state.phase = TRACK_COURSE_PHASE_CRUISE;
    g_state.result = TRACK_COURSE_RESULT_RUNNING;
    g_state.cruise_rpm = static_cast<uint16_t>(cruise_rpm);
    g_state.approach_rpm = static_cast<uint16_t>(approach_rpm);
    g_state.lap_distance_mm = static_cast<uint16_t>(lap_distance_mm);
    g_state.approach_start_mm = static_cast<uint16_t>(
        lap_distance_mm - kApproachMarginMm);
    g_state.finish_gate_mm = static_cast<uint16_t>(
        lap_distance_mm - kFinishGateMarginMm);
    g_state.start_left_count = chassis->left.encoder_count;
    g_state.start_right_count = chassis->right.encoder_count;
    g_state.last_grayscale_sequence = gray->sequence;
    g_state.start_ms = now;
    g_state.last_status = LF_Start(static_cast<int32_t>(cruise_rpm), 0U);
    if (g_state.last_status != drivers::DRIVER_OK) {
        g_state.phase = TRACK_COURSE_PHASE_STOPPED;
        g_state.result = TRACK_COURSE_RESULT_FAILED;
        g_state.failure = TRACK_COURSE_FAILURE_PRECONDITION;
        return g_state.last_status;
    }
    return drivers::DRIVER_OK;
}

drivers::DriverStatus TrackCourse_Cancel(void)
{
    if (g_state.result != TRACK_COURSE_RESULT_RUNNING) {
        return drivers::DRIVER_OK;
    }
    const drivers::DriverStatus status = LF_Stop();
    g_state.phase = TRACK_COURSE_PHASE_STOPPED;
    g_state.result = TRACK_COURSE_RESULT_CANCELLED;
    g_state.completion = TRACK_COURSE_COMPLETION_NONE;
    g_state.failure = TRACK_COURSE_FAILURE_NONE;
    g_state.last_status = status;
    RecordStop();
    return status;
}

void TrackCourse_Update(void)
{
    if (g_state.result != TRACK_COURSE_RESULT_RUNNING) {
        return;
    }
    const uint32_t now = services::Time_Millis();
    const ChassisState *chassis = Chassis_GetState();
    const LineSensorSnapshot *line = LineSensor_GetSnapshot();
    const LFState *linefollow = LF_GetState();
    const AppGrayscaleData *gray = App_GrayscaleGetData();

    if (services::Fault_HasFault()) {
        Fail(TRACK_COURSE_FAILURE_LINEFOLLOW,
             drivers::DRIVER_ERROR_NOT_INITIALIZED);
        return;
    }
    if ((line == 0) || !line->valid || !line->fresh ||
        (line->channel_anomaly_mask != 0U) || (gray == 0) ||
        !gray->valid || !gray->processed_valid) {
        Fail(TRACK_COURSE_FAILURE_SENSOR_STALE,
             drivers::DRIVER_ERROR_NOT_INITIALIZED);
        return;
    }
    if (!ChassisFeedbackFresh(chassis, now)) {
        Fail(TRACK_COURSE_FAILURE_ENCODER_STALE,
             drivers::DRIVER_ERROR_NOT_INITIALIZED);
        return;
    }
    if ((linefollow == 0) || (linefollow->mode != LF_FOLLOW)) {
        Fail(TRACK_COURSE_FAILURE_LINEFOLLOW, drivers::DRIVER_ERROR);
        return;
    }
    if ((linefollow->recovery_mode != LF_RECOVERY_NONE) &&
        (linefollow->recovery_elapsed_ms > kRecoveryMaximumMs)) {
        Fail(TRACK_COURSE_FAILURE_RECOVERY_TIMEOUT,
             drivers::DRIVER_ERROR_TIMEOUT);
        return;
    }
    if ((now - g_state.start_ms) > kCourseTimeoutMs) {
        Fail(TRACK_COURSE_FAILURE_TIMEOUT, drivers::DRIVER_ERROR_TIMEOUT);
        return;
    }

    UpdateDistance(chassis);

    /* Temporary task-0 policy: the encoder lap distance is the only normal
     * completion trigger.  Check it before marker diagnostics or the
     * approach-speed handoff so reaching the configured distance stops in
     * this same 2 ms controller update. */
    if (g_state.average_distance_mm >= g_state.lap_distance_mm) {
        Complete(TRACK_COURSE_COMPLETION_ENCODER);
        return;
    }

    const bool finish_gate_open =
        g_state.average_distance_mm >= g_state.finish_gate_mm;
    if (finish_gate_open) {
        /* Keep the center-marker evidence visible in course status for
         * diagnostics, but do not let it stop the temporary encoder-only
         * task. */
        (void)UpdateFinishEvidence(gray);
    }
    if (!finish_gate_open) {
        g_state.finish_count = 0U;
    }

    if ((g_state.phase == TRACK_COURSE_PHASE_CRUISE) &&
        (g_state.average_distance_mm >= g_state.approach_start_mm)) {
        const drivers::DriverStatus status =
            LF_ContinueForMotionHandoff(g_state.approach_rpm, 0U);
        if (status != drivers::DRIVER_OK) {
            Fail(TRACK_COURSE_FAILURE_LINEFOLLOW, status);
            return;
        }
        g_state.phase = TRACK_COURSE_PHASE_APPROACH;
    }

}

const TrackCourseState *TrackCourse_GetState(void)
{
    return &g_state;
}

const char *TrackCourse_PhaseText(TrackCoursePhase phase)
{
    switch (phase) {
    case TRACK_COURSE_PHASE_CRUISE: return "cruise";
    case TRACK_COURSE_PHASE_APPROACH: return "approach";
    case TRACK_COURSE_PHASE_STOPPED: return "stopped";
    case TRACK_COURSE_PHASE_IDLE:
    default: return "idle";
    }
}

const char *TrackCourse_ResultText(TrackCourseResult result)
{
    switch (result) {
    case TRACK_COURSE_RESULT_RUNNING: return "running";
    case TRACK_COURSE_RESULT_SUCCESS: return "success";
    case TRACK_COURSE_RESULT_FAILED: return "failed";
    case TRACK_COURSE_RESULT_CANCELLED: return "cancelled";
    case TRACK_COURSE_RESULT_IDLE:
    default: return "idle";
    }
}

const char *TrackCourse_CompletionText(TrackCourseCompletion completion)
{
    switch (completion) {
    case TRACK_COURSE_COMPLETION_LINE: return "line";
    case TRACK_COURSE_COMPLETION_ENCODER: return "encoder";
    case TRACK_COURSE_COMPLETION_NONE:
    default: return "none";
    }
}

const char *TrackCourse_FailureText(TrackCourseFailure failure)
{
    switch (failure) {
    case TRACK_COURSE_FAILURE_PRECONDITION: return "precondition";
    case TRACK_COURSE_FAILURE_SENSOR_STALE: return "sensor-stale";
    case TRACK_COURSE_FAILURE_ENCODER_STALE: return "encoder-stale";
    case TRACK_COURSE_FAILURE_LINEFOLLOW: return "linefollow";
    case TRACK_COURSE_FAILURE_RECOVERY_TIMEOUT: return "recovery-timeout";
    case TRACK_COURSE_FAILURE_TIMEOUT: return "timeout";
    case TRACK_COURSE_FAILURE_STOP: return "stop";
    case TRACK_COURSE_FAILURE_NONE:
    default: return "none";
    }
}

} /* namespace app */
