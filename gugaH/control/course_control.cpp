#include "control/course_control.h"

namespace gugah {
namespace {

static const uint8_t kFinishCenterMask = 0x7EU;
static const uint8_t kFinishMinimumChannels = 5U;
static const uint8_t kFinishConfirmFrames = 2U;
static const uint32_t kChassisMaximumAgeMs = 100U;

uint8_t CountBits(uint8_t value)
{
    uint8_t count = 0U;
    while (value != 0U) {
        count = static_cast<uint8_t>(count + (value & 1U));
        value = static_cast<uint8_t>(value >> 1U);
    }
    return count;
}

int32_t CountsToMillimeters(int32_t counts,
                            uint32_t wheel_radius_um,
                            uint32_t counts_per_rev)
{
    if ((wheel_radius_um == 0U) || (counts_per_rev == 0U)) {
        return 0;
    }
    const int64_t numerator =
        static_cast<int64_t>(counts) * 2LL * 3141593LL *
        static_cast<int64_t>(wheel_radius_um);
    const int64_t denominator =
        1000000LL * static_cast<int64_t>(counts_per_rev) * 1000LL;
    return static_cast<int32_t>(
        (numerator >= 0)
            ? ((numerator + denominator / 2LL) / denominator)
            : ((numerator - denominator / 2LL) / denominator));
}

int32_t MillimetersToCounts(int32_t millimeters,
                            uint32_t wheel_radius_um,
                            uint32_t counts_per_rev)
{
    if ((wheel_radius_um == 0U) || (counts_per_rev == 0U)) {
        return 0;
    }
    const int64_t numerator =
        static_cast<int64_t>(millimeters) *
        static_cast<int64_t>(counts_per_rev) * 1000000000LL;
    const int64_t denominator =
        2LL * 3141593LL * static_cast<int64_t>(wheel_radius_um);
    return static_cast<int32_t>(
        (numerator >= 0)
            ? ((numerator + denominator / 2LL) / denominator)
            : ((numerator - denominator / 2LL) / denominator));
}

bool ChassisFresh(const ChassisFeedback *chassis, uint32_t now_ms)
{
    return (chassis != 0) && chassis->valid &&
           ((now_ms - chassis->received_ms) <= kChassisMaximumAgeMs);
}

bool FinishLineDetected(const drivers::GrayscaleProcessedData *line)
{
    if (line == 0) {
        return false;
    }
    const uint8_t mask =
        static_cast<uint8_t>(line->active_mask & kFinishCenterMask);
    return (line->track_state == drivers::GRAYSCALE_TRACK_WIDE) &&
           (CountBits(mask) >= kFinishMinimumChannels);
}

MotionCommand StopCommand(void)
{
    MotionCommand command = {};
    command.mode = MOTION_COMMAND_STOP;
    return command;
}

void Fail(CourseState *state, CourseFailure failure)
{
    state->phase = COURSE_FAILED;
    state->failure = failure;
}

uint32_t TimeoutFor(CourseKind kind)
{
    if (kind == COURSE_H2) {
        return 20000U;
    }
    if (kind == COURSE_H4) {
        return 8000U;
    }
    return 30000U;
}

uint16_t FinishGateFor(CourseKind kind, const HConfig *config)
{
    if (kind == COURSE_H5) {
        return config->h5_finish_gate_mm;
    }
    if (kind == COURSE_H6) {
        return config->h6_finish_gate_mm;
    }
    return config->finish_gate_mm;
}

uint16_t CruiseFor(CourseKind kind, const HConfig *config)
{
    if (kind == COURSE_H4) {
        return config->h4_cruise_rpm;
    }
    if (kind == COURSE_H5) {
        return config->h5_cruise_rpm;
    }
    if (kind == COURSE_H6) {
        return config->h6_cruise_rpm;
    }
    return config->cruise_rpm;
}

uint16_t ApproachFor(CourseKind kind, const HConfig *config)
{
    if (kind == COURSE_H5) {
        return config->h5_approach_rpm;
    }
    if (kind == COURSE_H6) {
        return config->h6_approach_rpm;
    }
    return config->approach_rpm;
}

uint16_t ApproachStartFor(CourseKind kind, const HConfig *config)
{
    if (kind == COURSE_H5) {
        return config->h5_approach_start_mm;
    }
    if (kind == COURSE_H6) {
        return config->h6_approach_start_mm;
    }
    return config->approach_start_mm;
}

} /* namespace */

void Course_Init(CourseState *state)
{
    if (state == 0) {
        return;
    }
    *state = {};
    state->phase = COURSE_IDLE;
    LineControl_Init(&state->line_control);
}

bool Course_Start(CourseState *state,
                  CourseKind kind,
                  const ChassisFeedback *chassis,
                  uint32_t now_ms,
                  const HConfig *config)
{
    if ((state == 0) || (config == 0) ||
        !ChassisFresh(chassis, now_ms) ||
        ((kind != COURSE_H2) && (kind != COURSE_H4) &&
         (kind != COURSE_H5) && (kind != COURSE_H6))) {
        if (state != 0) {
            Fail(state, COURSE_FAILURE_INVALID_INPUT);
        }
        return false;
    }
    Course_Init(state);
    state->kind = kind;
    state->phase = COURSE_CRUISE;
    state->start_left_count = chassis->left_encoder_count;
    state->start_right_count = chassis->right_encoder_count;
    state->start_ms = now_ms;
    state->last_line_frame_ms = now_ms;
    return true;
}

MotionCommand Course_Update(CourseState *state,
                            const CourseInput *input,
                            const HConfig *config)
{
    MotionCommand command = {};
    if ((state == 0) || (input == 0) || (config == 0)) {
        command.mode = MOTION_COMMAND_STOP;
        return command;
    }
    if (!Course_IsRunning(state)) {
        if ((state->phase == COURSE_COMPLETE) ||
            (state->phase == COURSE_FAILED) ||
            (state->phase == COURSE_ABORTED)) {
            command.mode = MOTION_COMMAND_STOP;
        }
        return command;
    }
    if (!ChassisFresh(input->chassis, input->now_ms)) {
        Fail(state, COURSE_FAILURE_CHASSIS_STALE);
        return StopCommand();
    }
    const bool timed_task_pending =
        (state->kind != COURSE_H4) || !state->passed_b_or_a;
    if (timed_task_pending &&
        ((input->now_ms - state->start_ms) > TimeoutFor(state->kind))) {
        Fail(state, COURSE_FAILURE_TIMEOUT);
        return StopCommand();
    }

    const int32_t left_mm = CountsToMillimeters(
        input->chassis->left_encoder_count - state->start_left_count,
        config->wheel_radius_um,
        config->left_counts_per_rev);
    const int32_t right_mm = CountsToMillimeters(
        input->chassis->right_encoder_count - state->start_right_count,
        config->wheel_radius_um,
        config->right_counts_per_rev);
    state->distance_mm = static_cast<int32_t>(
        (static_cast<int64_t>(left_mm) + right_mm) / 2LL);

    if (state->phase == COURSE_FINAL_POSITION) {
        command.mode = MOTION_COMMAND_POSITION;
        command.left_position_count = state->final_left_count;
        command.right_position_count = state->final_right_count;
        const bool position_close =
            ((input->chassis->left_encoder_count -
              state->final_left_count) < 30) &&
            ((input->chassis->left_encoder_count -
              state->final_left_count) > -30) &&
            ((input->chassis->right_encoder_count -
              state->final_right_count) < 30) &&
            ((input->chassis->right_encoder_count -
              state->final_right_count) > -30);
        const bool stopped =
            (input->chassis->left_rpm <= 2) &&
            (input->chassis->left_rpm >= -2) &&
            (input->chassis->right_rpm <= 2) &&
            (input->chassis->right_rpm >= -2);
        if (!position_close || !stopped) {
            state->settle_start_ms = 0U;
        } else if (state->settle_start_ms == 0U) {
            state->settle_start_ms = input->now_ms;
        } else if ((input->now_ms - state->settle_start_ms) >= 200U) {
            state->phase = COURSE_COMPLETE;
            state->pass_ms = input->now_ms;
            return StopCommand();
        }
        return command;
    }

    if (state->phase == COURSE_STOPPING) {
        command = StopCommand();
        if ((input->chassis->left_rpm <= 2) &&
            (input->chassis->left_rpm >= -2) &&
            (input->chassis->right_rpm <= 2) &&
            (input->chassis->right_rpm >= -2)) {
            state->phase = COURSE_COMPLETE;
        }
        return command;
    }

    /*
     * Distance, timeout and terminal-stop checks stay responsive at the
     * 2 ms application rate.  A cruise/approach control step is released
     * only after all eight grayscale channels form a new processed frame.
     */
    if (state->kind == COURSE_H4) {
        if (!state->passed_b_or_a &&
            (state->distance_mm >= config->h4_b_distance_mm)) {
            state->passed_b_or_a = true;
            state->pass_ms = input->now_ms;
        }
        if (state->distance_mm >= config->h4_stop_distance_mm) {
            state->phase = COURSE_STOPPING;
            return StopCommand();
        }
    } else if ((state->distance_mm >
                static_cast<int32_t>(config->lap_distance_mm) + 300) &&
               (state->finish_confirm_frames == 0U)) {
        Fail(state, COURSE_FAILURE_FINISH_NOT_FOUND);
        return StopCommand();
    }

    if (!input->line_frame_new) {
        if ((input->now_ms - state->last_line_frame_ms) >
            config->line_lost_grace_ms) {
            Fail(state, COURSE_FAILURE_LINE_LOST);
            return StopCommand();
        }
        return command;
    }
    state->last_line_frame_ms = input->now_ms;

    const bool gate_open = (state->kind == COURSE_H4)
        ? (state->distance_mm >= config->h4_b_distance_mm)
        : (state->distance_mm >=
           FinishGateFor(state->kind, config));
    if (gate_open && FinishLineDetected(input->line)) {
        if (state->finish_confirm_frames < 0xFFU) {
            state->finish_confirm_frames++;
        }
    } else {
        state->finish_confirm_frames = 0U;
    }

    if ((state->kind == COURSE_H2) &&
        (state->finish_confirm_frames >= kFinishConfirmFrames)) {
        state->final_left_count =
            input->chassis->left_encoder_count +
            MillimetersToCounts(config->sensor_to_reference_mm,
                                 config->wheel_radius_um,
                                 config->left_counts_per_rev);
        state->final_right_count =
            input->chassis->right_encoder_count +
            MillimetersToCounts(config->sensor_to_reference_mm,
                                 config->wheel_radius_um,
                                 config->right_counts_per_rev);
        state->phase = COURSE_FINAL_POSITION;
        command.mode = MOTION_COMMAND_POSITION;
        command.left_position_count = state->final_left_count;
        command.right_position_count = state->final_right_count;
        return command;
    }

    if (((state->kind == COURSE_H5) || (state->kind == COURSE_H6)) &&
        (state->finish_confirm_frames >= kFinishConfirmFrames)) {
        state->passed_b_or_a = true;
        state->pass_ms = input->now_ms;
        state->phase = COURSE_STOPPING;
        return StopCommand();
    }

    int16_t base_rpm = static_cast<int16_t>(
        CruiseFor(state->kind, config));
    if ((state->kind != COURSE_H4) &&
        (state->distance_mm >=
         ApproachStartFor(state->kind, config))) {
        state->phase = COURSE_APPROACH;
        base_rpm = static_cast<int16_t>(
            ApproachFor(state->kind, config));
    }
    if (!LineControl_Update(&state->line_control,
                            input->line,
                            base_rpm,
                            input->now_ms,
                            config)) {
        Fail(state, COURSE_FAILURE_LINE_LOST);
        return StopCommand();
    }
    command.mode = MOTION_COMMAND_SPEED;
    command.left_rpm = state->line_control.left_rpm;
    command.right_rpm = state->line_control.right_rpm;
    return command;
}

MotionCommand Course_Abort(CourseState *state)
{
    if (state != 0) {
        state->phase = COURSE_ABORTED;
    }
    return StopCommand();
}

bool Course_IsRunning(const CourseState *state)
{
    if (state == 0) {
        return false;
    }
    return (state->phase == COURSE_CRUISE) ||
           (state->phase == COURSE_APPROACH) ||
           (state->phase == COURSE_FINAL_POSITION) ||
           (state->phase == COURSE_STOPPING);
}

} /* namespace gugah */
