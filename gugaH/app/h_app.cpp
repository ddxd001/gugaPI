#include "app/h_app.h"

namespace gugah {
namespace {

static const int16_t kH6Minimum0p1mm = -1000;
static const int16_t kH6Maximum0p1mm = 1000;
static const int16_t kH3CenterTarget0p1mm = 0;
static const int16_t kH3PositiveTarget0p1mm = 500;
static const int16_t kH3NegativeTarget0p1mm = -500;
/* The first two transfers have strategy deadlines, not failure deadlines:
 * leave +50 after at most 1.5 s and leave O after at most 1 s.  A retarget keeps
 * the estimator and beam reference continuous.  Only the final -50 transfer
 * retains the long diagnostic timeout; PASS then holds -50 indefinitely. */
static const uint32_t kH3PositiveWindowMs = 1500U;
static const uint32_t kH3CenterWindowMs = 1000U;
static const uint32_t kH3TransferTimeoutMs = 30000U;

bool LineReady(const HAppInput *input)
{
    return (input->line != 0) && input->line->line_detected &&
           input->line->position_valid &&
           (input->line->track_state ==
            drivers::GRAYSCALE_TRACK_VALID) &&
           (input->line->calibration_fault_mask == 0U);
}

BallInput MakeBallInput(const HAppState *state, const HAppInput *input)
{
    BallInput ball_input = {};
    ball_input.now_ms = input->now_ms;
    ball_input.vision = input->vision;
    ball_input.imu = input->imu;
    ball_input.dm = input->dm;
    if ((state->selected_problem == H_PROBLEM_4) &&
        (state->course.kind == COURSE_H4)) {
        ball_input.chassis_accel_mm_s2 =
            state->course.h4_commanded_accel_mm_s2;
    } else if ((state->selected_problem == H_PROBLEM_5) &&
               (state->course.kind == COURSE_H5)) {
        ball_input.chassis_accel_mm_s2 =
            state->course.h5_commanded_accel_mm_s2;
    } else {
        ball_input.chassis_accel_mm_s2 =
            state->chassis_accel_mm_s2;
    }
    return ball_input;
}

void UpdateChassisAcceleration(HAppState *state,
                               const HAppInput *input,
                               const HConfig *config)
{
    if (!input->chassis.valid) {
        return;
    }
    if (input->chassis.received_ms ==
        state->last_chassis_feedback_ms) {
        return;
    }
    const int16_t average_rpm = static_cast<int16_t>(
        (static_cast<int32_t>(input->chassis.left_rpm) +
         input->chassis.right_rpm) /
        2);
    const uint32_t elapsed_ms = input->chassis.received_ms -
        state->last_chassis_feedback_ms;
    if ((state->last_accel_ms != 0U) &&
        (elapsed_ms >= 5U) && (elapsed_ms <= 100U)) {
        const int32_t delta_rpm =
            static_cast<int32_t>(average_rpm) -
            state->last_average_rpm;
        const int64_t circumference_um =
            2LL * 3141593LL * config->wheel_radius_um / 1000000LL;
        const int32_t raw_accel = static_cast<int32_t>(
            (static_cast<int64_t>(delta_rpm) * circumference_um *
             1000LL) /
            (60000LL * elapsed_ms));
        state->chassis_accel_mm_s2 =
            (state->chassis_accel_mm_s2 * 3 + raw_accel) / 4;
    }
    state->last_average_rpm = average_rpm;
    state->last_accel_ms = input->chassis.received_ms;
    state->last_chassis_feedback_ms = input->chassis.received_ms;
}

HFailure BallPreflightFailure(const HAppState *state,
                              const HAppInput *input)
{
    BallInput ball_input = MakeBallInput(state, input);
    if (!ball_input.vision.ball_usable) {
        return H_FAILURE_VISION;
    }
    if (!ball_input.imu.valid ||
        ((input->now_ms - ball_input.imu.received_ms) > 100U)) {
        return H_FAILURE_IMU;
    }
    if (!ball_input.dm.valid || (ball_input.dm.state != 1U) ||
        ((input->now_ms - ball_input.dm.received_ms) > 100U)) {
        return H_FAILURE_DM;
    }
    return H_FAILURE_NONE;
}

HFailure BallRuntimeFailure(const BallState *ball)
{
    if (ball == 0) {
        return H_FAILURE_BALL;
    }
    switch (ball->result) {
    case BALL_RESULT_TIMEOUT: return H_FAILURE_TIMEOUT;
    case BALL_RESULT_VISION_LOST: return H_FAILURE_VISION;
    case BALL_RESULT_DM_STALE: return H_FAILURE_DM;
    case BALL_RESULT_IMU_STALE: return H_FAILURE_IMU;
    default: return H_FAILURE_BALL;
    }
}

void SetFailure(HAppState *state, HFailure failure, uint32_t now_ms)
{
    state->run_state = H_STATE_FAIL;
    state->failure = failure;
    state->result_time_ms = now_ms - state->run_start_ms;
    (void)Course_Abort(&state->course);
    Ball_Level(&state->ball);
}

void SetPass(HAppState *state, uint32_t elapsed_ms)
{
    state->run_state = H_STATE_PASS;
    state->failure = H_FAILURE_NONE;
    state->result_time_ms = elapsed_ms;
}

bool StartRunning(HAppState *state,
                  const HAppInput *input,
                  const HConfig *config)
{
    const bool needs_course =
        (state->selected_problem != H_PROBLEM_3) &&
        (state->selected_problem != H_PROBLEM_CAL_ZERO) &&
        (state->selected_problem != H_PROBLEM_CAL_H2_LOOP);
    const bool needs_ball =
        (state->selected_problem != H_PROBLEM_2) &&
        (state->selected_problem != H_PROBLEM_CAL_H2_LOOP);
    const bool needs_line =
        (state->selected_problem == H_PROBLEM_2) ||
        (state->selected_problem == H_PROBLEM_5) ||
        (state->selected_problem == H_PROBLEM_6);
    HFailure preflight_failure = H_FAILURE_NONE;
    if (input->hardware_fault) {
        preflight_failure = H_FAILURE_HARDWARE;
    } else if (needs_course && input->chassis_fault) {
        preflight_failure = H_FAILURE_CHASSIS;
    } else if (needs_course && !input->chassis.valid) {
        preflight_failure = H_FAILURE_PREFLIGHT;
    } else if (needs_line && !LineReady(input)) {
        preflight_failure = H_FAILURE_PREFLIGHT;
    } else if (needs_ball) {
        preflight_failure = BallPreflightFailure(state, input);
    }
    if (preflight_failure != H_FAILURE_NONE) {
        state->run_state = H_STATE_FAIL;
        state->failure = preflight_failure;
        state->result_time_ms = 0U;
        return false;
    }

    BallInput ball_input = MakeBallInput(state, input);
    if (state->selected_problem == H_PROBLEM_3) {
        /* READY already holds O.  Start the scored sequence directly at
         * +50 mm; the strategy deadline below prevents this leg from
         * consuming the rest of the run if static friction stops it short. */
        state->active_ball_target_0p1mm = kH3PositiveTarget0p1mm;
        state->h3_stage = 0U;
        if (!Ball_StartMove(&state->ball,
                            kH3PositiveTarget0p1mm,
                            kH3TransferTimeoutMs,
                            &ball_input,
                            config)) {
            state->run_state = H_STATE_FAIL;
            state->failure = H_FAILURE_PREFLIGHT;
            return false;
        }
    } else if (needs_ball) {
        state->active_ball_target_0p1mm =
            (state->selected_problem == H_PROBLEM_6)
                ? state->h6_target_0p1mm : 0;
        if (!Ball_StartHold(&state->ball,
                            state->active_ball_target_0p1mm,
                            &ball_input,
                            config)) {
            state->run_state = H_STATE_FAIL;
            state->failure = H_FAILURE_PREFLIGHT;
            return false;
        }
    }

    if (needs_course) {
        if (!Course_Start(
                &state->course,
                static_cast<CourseKind>(state->selected_problem),
                &input->chassis,
                input->now_ms,
                config)) {
            Ball_Level(&state->ball);
            state->run_state = H_STATE_FAIL;
            state->failure = H_FAILURE_PREFLIGHT;
            return false;
        }
    }
    state->run_start_ms = input->now_ms;
    state->result_time_ms = 0U;
    state->maximum_ball_error_0p1mm = 0;
    state->run_state = H_STATE_RUNNING;
    return true;
}

void HandleReadyButtons(HAppState *state, const HButtonEvents *buttons)
{
    if (buttons->b1_short) {
        uint8_t problem =
            static_cast<uint8_t>(state->selected_problem) + 1U;
        if (problem > static_cast<uint8_t>(H_PROBLEM_CAL_H2_LOOP)) {
            problem = static_cast<uint8_t>(H_PROBLEM_2);
        }
        state->selected_problem = static_cast<HProblem>(problem);
    }
    if (state->selected_problem == H_PROBLEM_6) {
        if (buttons->b2_decrement &&
            (state->h6_target_0p1mm > kH6Minimum0p1mm)) {
            state->h6_target_0p1mm =
                static_cast<int16_t>(
                    state->h6_target_0p1mm - 10);
        }
        if (buttons->b3_increment &&
            (state->h6_target_0p1mm < kH6Maximum0p1mm)) {
            state->h6_target_0p1mm =
                static_cast<int16_t>(
                    state->h6_target_0p1mm + 10);
        }
    }
}

} /* namespace */

void HApp_Init(HAppState *state)
{
    if (state == 0) {
        return;
    }
    *state = {};
    state->run_state = H_STATE_READY;
    state->selected_problem = H_PROBLEM_2;
    Course_Init(&state->course);
    Ball_Init(&state->ball);
}

bool HApp_Start(HAppState *state,
                const HAppInput *input,
                const HConfig *config)
{
    if ((state == 0) || (input == 0) || (config == 0) ||
        (state->run_state != H_STATE_READY)) {
        return false;
    }
    state->run_state = H_STATE_PREFLIGHT;
    return StartRunning(state, input, config);
}

HAppOutput HApp_Abort(HAppState *state)
{
    HAppOutput output = {};
    if (state == 0) {
        output.motion.mode = MOTION_COMMAND_STOP;
        return output;
    }
    output.motion = Course_Abort(&state->course);
    Ball_Level(&state->ball);
    state->run_state = H_STATE_ABORTED;
    state->failure = H_FAILURE_NONE;
    output.ball.command_valid = true;
    output.result_changed = true;
    return output;
}

HAppOutput HApp_Update(HAppState *state,
                       const HAppInput *input,
                       const HConfig *config)
{
    HAppOutput output = {};
    if ((state == 0) || (input == 0) || (config == 0)) {
        output.motion.mode = MOTION_COMMAND_STOP;
        return output;
    }
    UpdateChassisAcceleration(state, input, config);

    if (state->run_state == H_STATE_READY) {
        HandleReadyButtons(state, &input->buttons);
        if (input->buttons.b1_long) {
            (void)HApp_Start(state, input, config);
            output.result_changed = true;
        }
        return output;
    }
    if ((state->run_state == H_STATE_PASS) ||
        (state->run_state == H_STATE_FAIL) ||
        (state->run_state == H_STATE_ABORTED)) {
        output.motion.mode = MOTION_COMMAND_STOP;
        if (input->buttons.b1_short) {
            const HProblem selected = state->selected_problem;
            const int16_t h6_target = state->h6_target_0p1mm;
            HApp_Init(state);
            state->selected_problem = selected;
            state->h6_target_0p1mm = h6_target;
            output.result_changed = true;
        }
        return output;
    }
    if (!HApp_IsRunning(state)) {
        return output;
    }
    output.timer_running = true;
    const bool calibrating_zero =
        state->selected_problem == H_PROBLEM_CAL_ZERO;
    const bool calibrating_h2_loop =
        state->selected_problem == H_PROBLEM_CAL_H2_LOOP;
    const bool calibrating =
        calibrating_zero || calibrating_h2_loop;
    if (calibrating_zero && input->buttons.b1_pressed) {
        Ball_Level(&state->ball);
        state->run_state = H_STATE_READY;
        state->failure = H_FAILURE_NONE;
        state->result_time_ms = 0U;
        output.motion.mode = MOTION_COMMAND_STOP;
        output.timer_running = false;
        output.result_changed = true;
        output.save_config = true;
        return output;
    }
    if (calibrating_h2_loop && input->buttons.b1_short) {
        state->run_state = H_STATE_READY;
        state->failure = H_FAILURE_NONE;
        state->result_time_ms = 0U;
        output.motion.mode = MOTION_COMMAND_STOP;
        output.timer_running = false;
        output.result_changed = true;
        output.save_config = true;
        return output;
    }
    if (!calibrating && input->buttons.any_pressed) {
        return HApp_Abort(state);
    }
    if (input->hardware_fault) {
        SetFailure(state, H_FAILURE_HARDWARE, input->now_ms);
        output.motion.mode = MOTION_COMMAND_STOP;
        output.result_changed = true;
        return output;
    }
    if ((state->selected_problem != H_PROBLEM_3) &&
        !calibrating &&
        input->chassis_fault) {
        SetFailure(state, H_FAILURE_CHASSIS, input->now_ms);
        output.motion.mode = MOTION_COMMAND_STOP;
        output.result_changed = true;
        return output;
    }

    BallInput ball_input = MakeBallInput(state, input);
    if ((state->selected_problem != H_PROBLEM_2) &&
        !calibrating_h2_loop) {
        if (state->ball.maximum_abs_error_0p1mm >
            state->maximum_ball_error_0p1mm) {
            state->maximum_ball_error_0p1mm =
                state->ball.maximum_abs_error_0p1mm;
        }
        if ((state->ball.mode == BALL_FAILED) ||
            output.ball.stop_chassis) {
            SetFailure(state,
                       BallRuntimeFailure(&state->ball),
                       input->now_ms);
            output.motion.mode = MOTION_COMMAND_STOP;
            output.result_changed = true;
            return output;
        }
    }

    if (state->selected_problem == H_PROBLEM_3) {
        const uint32_t stage_elapsed_ms =
            input->now_ms - state->ball.start_ms;
        if ((state->h3_stage == 0U) &&
            ((state->ball.result == BALL_RESULT_SUCCESS) ||
             (stage_elapsed_ms >= kH3PositiveWindowMs))) {
            if (!Ball_StartMove(&state->ball,
                                kH3CenterTarget0p1mm,
                                kH3TransferTimeoutMs,
                                &ball_input,
                                config)) {
                SetFailure(state, H_FAILURE_BALL, input->now_ms);
                output.result_changed = true;
            } else {
                state->h3_stage = 1U;
                state->active_ball_target_0p1mm =
                    kH3CenterTarget0p1mm;
                output.buzzer_pulse = true;
            }
        } else if ((state->h3_stage == 1U) &&
                   ((state->ball.result == BALL_RESULT_SUCCESS) ||
                    (stage_elapsed_ms >= kH3CenterWindowMs))) {
            if (!Ball_StartMove(&state->ball,
                                kH3NegativeTarget0p1mm,
                                kH3TransferTimeoutMs,
                                &ball_input,
                                config)) {
                SetFailure(state, H_FAILURE_BALL, input->now_ms);
                output.result_changed = true;
            } else {
                state->h3_stage = 2U;
                state->active_ball_target_0p1mm =
                    kH3NegativeTarget0p1mm;
            }
        } else if ((state->h3_stage == 2U) &&
                   (state->ball.result == BALL_RESULT_SUCCESS)) {
            SetPass(state, input->now_ms - state->run_start_ms);
            output.result_changed = true;
            output.buzzer_pulse = true;
        } else if (state->h3_stage > 2U) {
            SetFailure(state, H_FAILURE_BALL, input->now_ms);
            output.result_changed = true;
        }
        output.motion.mode = MOTION_COMMAND_NONE;
        return output;
    }

    if (calibrating_zero) {
        if (input->buttons.b2_decrement !=
            input->buttons.b3_increment) {
            output.ball_zero_delta_0p1mm =
                input->buttons.b2_decrement ? -10 : 10;
        }
        output.motion.mode = MOTION_COMMAND_NONE;
        return output;
    }

    if (calibrating_h2_loop) {
        if (input->buttons.b2_decrement !=
            input->buttons.b3_increment) {
            output.h2_loop_delta_mm =
                input->buttons.b2_decrement ? -10 : 10;
        }
        output.motion.mode = MOTION_COMMAND_NONE;
        return output;
    }

    CourseInput course_input = {};
    course_input.now_ms = input->now_ms;
    course_input.line = input->line;
    course_input.chassis = &input->chassis;
    course_input.line_frame_new = input->line_frame_new;
    course_input.imu = &input->imu;
    output.motion =
        Course_Update(&state->course, &course_input, config);
    if (state->course.phase == COURSE_FAILED) {
        HFailure failure = H_FAILURE_COURSE;
        if (state->course.failure == COURSE_FAILURE_CHASSIS_STALE) {
            failure = H_FAILURE_CHASSIS;
        } else if (state->course.failure == COURSE_FAILURE_IMU_STALE) {
            failure = H_FAILURE_IMU;
        } else if (state->course.failure == COURSE_FAILURE_TIMEOUT) {
            failure = H_FAILURE_TIMEOUT;
        }
        SetFailure(state, failure, input->now_ms);
        output.motion.mode = MOTION_COMMAND_STOP;
        output.result_changed = true;
    } else if (state->course.phase == COURSE_COMPLETE) {
        uint32_t elapsed = input->now_ms - state->run_start_ms;
        if (state->course.pass_ms >= state->run_start_ms) {
            elapsed = state->course.pass_ms - state->run_start_ms;
        }
        SetPass(state, elapsed);
        output.result_changed = true;
    }
    return output;
}

BallOutput HApp_UpdateBall2ms(HAppState *state,
                              const HAppInput *input,
                              const HConfig *config)
{
    BallOutput output = {};
    if ((state == 0) || (input == 0) || (config == 0)) {
        output.stop_chassis = true;
        return output;
    }
    const bool terminal =
        (state->run_state == H_STATE_PASS) ||
        (state->run_state == H_STATE_FAIL) ||
        (state->run_state == H_STATE_ABORTED);
    const bool task_ball =
        (state->run_state == H_STATE_RUNNING) &&
        (state->selected_problem != H_PROBLEM_2) &&
        (state->selected_problem != H_PROBLEM_CAL_H2_LOOP);
    if (!terminal && !task_ball) {
        return output;
    }
    BallInput ball_input = MakeBallInput(state, input);
    const bool recoverable_pass_hold =
        (state->run_state == H_STATE_PASS) &&
        (state->selected_problem != H_PROBLEM_2) &&
        (state->ball.mode == BALL_FAILED) &&
        ((state->ball.result == BALL_RESULT_VISION_LOST) ||
         (state->ball.result == BALL_RESULT_DM_STALE) ||
         (state->ball.result == BALL_RESULT_IMU_STALE));
    if (recoverable_pass_hold &&
        Ball_StartHold(&state->ball,
                       state->active_ball_target_0p1mm,
                       &ball_input,
                       config)) {
        /* A completed task must resume its final hold after a transient
         * sensor/actuator gap. */
        return Ball_Update(&state->ball, &ball_input, config);
    }
    return Ball_Update(&state->ball, &ball_input, config);
}

bool HApp_IsRunning(const HAppState *state)
{
    return (state != 0) &&
           ((state->run_state == H_STATE_PREFLIGHT) ||
            (state->run_state == H_STATE_RUNNING));
}

const char *HApp_StateText(HRunState state)
{
    switch (state) {
    case H_STATE_BOOT: return "BOOT";
    case H_STATE_READY: return "READY";
    case H_STATE_PREFLIGHT: return "CHECK";
    case H_STATE_RUNNING: return "RUN";
    case H_STATE_PASS: return "PASS";
    case H_STATE_FAIL: return "FAIL";
    case H_STATE_ABORTED: return "ABORT";
    default: return "?";
    }
}

const char *HApp_FailureText(HFailure failure)
{
    switch (failure) {
    case H_FAILURE_PREFLIGHT: return "preflight";
    case H_FAILURE_HARDWARE: return "hardware";
    case H_FAILURE_CHASSIS: return "chassis";
    case H_FAILURE_VISION: return "vision";
    case H_FAILURE_DM: return "dm";
    case H_FAILURE_IMU: return "imu";
    case H_FAILURE_COURSE: return "course";
    case H_FAILURE_BALL: return "ball";
    case H_FAILURE_TIMEOUT: return "timeout";
    case H_FAILURE_NONE:
    default: return "none";
    }
}

} /* namespace gugah */
