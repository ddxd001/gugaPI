#include "control/course_control.h"

namespace gugah {
namespace {

static const uint32_t kChassisMaximumAgeMs = 100U;
static const uint32_t kImuMaximumAgeMs = 100U;
static const int32_t kMaximumHeadingErrorMdeg = 90000;
static const int32_t kH2FinalPositionDistanceMm = 200;
static const int16_t kH2FinalMinimumRpm = 15;
static const int32_t kH5OdometryFinishOffsetMm = 50;
/* Curve entry is deliberately gentler than launch/final stopping.  Finish
 * the 95->63 RPM transition early enough to give the 0.8 s ball model time
 * to settle before steering curvature reaches its full value. */
static const uint16_t kH5CurveEntryRampRpmS = 30U;
static const int32_t kH5CurvePreparationMarginMm = 180;
static const int32_t kCourseLineTransitionMm = 250;
static const uint16_t kStraightDerivativeFilterPermille = 200U;
static const uint16_t kCurveDerivativeFilterPermille = 1000U;

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

bool ChassisFresh(const ChassisFeedback *chassis, uint32_t now_ms)
{
    return (chassis != 0) && chassis->valid &&
           ((now_ms - chassis->received_ms) <= kChassisMaximumAgeMs);
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
    if ((kind == COURSE_H4) || (kind == COURSE_H7)) {
        return 8000U;
    }
    return 30000U;
}

bool UsesH5ChassisStrategy(CourseKind kind)
{
    return (kind == COURSE_H5) || (kind == COURSE_H6);
}

bool UsesH4ChassisStrategy(CourseKind kind)
{
    return (kind == COURSE_H4) || (kind == COURSE_H7);
}

uint16_t CruiseFor(CourseKind kind, const HConfig *config)
{
    if (UsesH4ChassisStrategy(kind)) {
        return config->h4_cruise_rpm;
    }
    if (UsesH5ChassisStrategy(kind)) {
        return config->h5_cruise_rpm;
    }
    return config->cruise_rpm;
}

bool ImuFresh(const ImuFeedback *imu, uint32_t now_ms)
{
    return (imu != 0) && imu->valid &&
           ((now_ms - imu->received_ms) <= kImuMaximumAgeMs);
}

int32_t Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
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

int32_t ShortestAngleDiff(int32_t target_mdeg, int32_t actual_mdeg)
{
    int32_t error = target_mdeg - actual_mdeg;
    while (error > 180000) {
        error -= 360000;
    }
    while (error < -180000) {
        error += 360000;
    }
    return error;
}

MotionCommand H4StraightCommand(CourseState *state,
                                int16_t base_rpm,
                                const ImuFeedback *imu,
                                const HConfig *config)
{
    MotionCommand command = {};
    command.mode = MOTION_COMMAND_SPEED;
    if (!state->h4_heading_locked) {
        state->h4_target_yaw_mdeg = imu->yaw_mdeg;
        state->h4_heading_locked = true;
    }
    const int32_t error = ShortestAngleDiff(
        state->h4_target_yaw_mdeg, imu->yaw_mdeg);
    state->h4_yaw_error_mdeg = error;
    int32_t correction = static_cast<int32_t>(
        (static_cast<int64_t>(error) * config->h4_heading_kp) /
        1000000LL);
    correction = Clamp32(
        correction,
        -config->h4_heading_max_correction_rpm,
        config->h4_heading_max_correction_rpm);
    /* Keep both wheels forward during the low-speed launch and B crossing. */
    const int32_t low_speed_limit = base_rpm / 2;
    correction = Clamp32(correction,
                         -low_speed_limit,
                         low_speed_limit);
    state->h4_heading_correction_rpm =
        static_cast<int16_t>(correction);
    command.left_rpm = static_cast<int16_t>(base_rpm - correction);
    command.right_rpm = static_cast<int16_t>(base_rpm + correction);
    return command;
}

int16_t H4RequestedRpm(const CourseState *state,
                       uint32_t now_ms,
                       const HConfig *config)
{
    if (state->h4_braking) {
        const uint32_t braking_ms = now_ms - state->h4_brake_start_ms;
        const uint32_t reduction = static_cast<uint32_t>(
            (static_cast<uint64_t>(braking_ms) *
             config->h4_stop_ramp_rpm_s) / 1000ULL);
        uint32_t rpm = (reduction >= state->h4_brake_start_rpm)
            ? 0U
            : static_cast<uint32_t>(
                state->h4_brake_start_rpm - reduction);
        /* Never stop just before the scoring point because of odometry or
         * ramp error.  Cross B slowly, then finish the same ramp to zero. */
        static const uint32_t kMinimumBCrossingRpm = 20U;
        if (!state->passed_b_or_a) {
            if (rpm < kMinimumBCrossingRpm) {
                rpm = kMinimumBCrossingRpm;
            }
        } else if (rpm <= kMinimumBCrossingRpm) {
            /* The pre-B clamp must not become a 20->0 RPM step at B.
             * Release the final 20 RPM with the same gentle stop ramp. */
            const uint32_t post_b_reduction = static_cast<uint32_t>(
                (static_cast<uint64_t>(now_ms - state->pass_ms) *
                 config->h4_stop_ramp_rpm_s) / 1000ULL);
            rpm = (post_b_reduction >= kMinimumBCrossingRpm)
                ? 0U : (kMinimumBCrossingRpm - post_b_reduction);
        }
        return static_cast<int16_t>(rpm);
    }
    const uint32_t elapsed_ms = now_ms - state->start_ms;
    uint32_t rpm = static_cast<uint32_t>(
        (static_cast<uint64_t>(elapsed_ms) *
         config->h4_launch_ramp_rpm_s) / 1000ULL);
    if (rpm > config->h4_cruise_rpm) {
        rpm = config->h4_cruise_rpm;
    }
    return static_cast<int16_t>(rpm);
}

int32_t RpmRateToAccelerationMmS2(uint16_t rpm_per_second,
                                  const HConfig *config)
{
    const int64_t circumference_um =
        2LL * 3141593LL * config->wheel_radius_um / 1000000LL;
    return static_cast<int32_t>(
        (static_cast<int64_t>(rpm_per_second) * circumference_um) /
        60000LL);
}

int32_t H4CommandedAccelerationMmS2(const CourseState *state,
                                    uint32_t now_ms,
                                    const HConfig *config)
{
    const int16_t rpm = H4RequestedRpm(state, now_ms, config);
    if (!state->h4_braking) {
        return (rpm < static_cast<int16_t>(config->h4_cruise_rpm))
            ? RpmRateToAccelerationMmS2(
                config->h4_launch_ramp_rpm_s, config)
            : 0;
    }
    if (state->passed_b_or_a) {
        return (rpm > 0)
            ? -RpmRateToAccelerationMmS2(
                config->h4_stop_ramp_rpm_s, config)
            : 0;
    }
    return (rpm > 20)
        ? -RpmRateToAccelerationMmS2(
            config->h4_stop_ramp_rpm_s, config)
        : 0;
}

int16_t H5RequestedRpm(const CourseState *state,
                       uint32_t now_ms,
                       const HConfig *config)
{
    if (state->h5_braking) {
        const uint32_t reduction = static_cast<uint32_t>(
            (static_cast<uint64_t>(now_ms - state->h5_brake_start_ms) *
             config->h5_stop_ramp_rpm_s) / 1000ULL);
        uint32_t rpm = (reduction >= state->h5_brake_start_rpm)
            ? 0U : (state->h5_brake_start_rpm - reduction);
        static const uint32_t kMinimumACrossingRpm = 20U;
        if (!state->passed_b_or_a) {
            if (rpm < kMinimumACrossingRpm) {
                rpm = kMinimumACrossingRpm;
            }
        } else if (rpm <= kMinimumACrossingRpm) {
            const uint32_t ramp_to_crossing_ms =
                (state->h5_brake_start_rpm > kMinimumACrossingRpm)
                ? static_cast<uint32_t>(
                    ((static_cast<uint64_t>(
                          state->h5_brake_start_rpm -
                          kMinimumACrossingRpm) * 1000ULL) +
                     config->h5_stop_ramp_rpm_s - 1U) /
                    config->h5_stop_ramp_rpm_s)
                : 0U;
            const uint32_t natural_crossing_ms =
                state->h5_brake_start_ms + ramp_to_crossing_ms;
            const uint32_t final_ramp_start_ms =
                (state->pass_ms > natural_crossing_ms)
                ? state->pass_ms : natural_crossing_ms;
            const uint32_t post_a_reduction = static_cast<uint32_t>(
                (static_cast<uint64_t>(
                     now_ms - final_ramp_start_ms) *
                 config->h5_stop_ramp_rpm_s) / 1000ULL);
            rpm = (post_a_reduction >= kMinimumACrossingRpm)
                ? 0U : (kMinimumACrossingRpm - post_a_reduction);
        }
        return static_cast<int16_t>(rpm);
    }
    uint32_t rpm = static_cast<uint32_t>(
        (static_cast<uint64_t>(now_ms - state->start_ms) *
         config->h5_launch_ramp_rpm_s) / 1000ULL);
    if (rpm > config->h5_cruise_rpm) {
        rpm = config->h5_cruise_rpm;
    }
    return static_cast<int16_t>(rpm);
}

int32_t H5CurveBrakeLeadMm(const HConfig *config)
{
    if (config->h5_cruise_rpm <= config->h5_approach_rpm) {
        return 0;
    }
    const int64_t cruise_squared =
        static_cast<int64_t>(config->h5_cruise_rpm) *
        config->h5_cruise_rpm;
    const int64_t curve_squared =
        static_cast<int64_t>(config->h5_approach_rpm) *
        config->h5_approach_rpm;
    const int64_t numerator = 2LL * 3141593LL *
        config->wheel_radius_um *
        (cruise_squared - curve_squared);
    const int64_t denominator = 1000000000LL * 120LL *
        kH5CurveEntryRampRpmS;
    return static_cast<int32_t>(
        (numerator + denominator / 2LL) / denominator) +
        kH5CurvePreparationMarginMm;
}

bool H5DistanceRequestsCurve(int32_t distance_mm,
                             const HConfig *config)
{
    const int32_t straight_mm = config->h4_b_distance_mm;
    const int32_t curved_total_mm =
        static_cast<int32_t>(config->lap_distance_mm) -
        2 * straight_mm;
    if (curved_total_mm <= 0) {
        return false;
    }
    const int32_t semicircle_mm = curved_total_mm / 2;
    const int32_t brake_lead_mm = H5CurveBrakeLeadMm(config);
    const int32_t first_start_mm =
        (brake_lead_mm < straight_mm)
            ? (straight_mm - brake_lead_mm) : 0;
    const int32_t first_end_mm = straight_mm + semicircle_mm;
    const int32_t second_start_mm =
        first_end_mm + straight_mm - brake_lead_mm;
    return ((distance_mm >= first_start_mm) &&
            (distance_mm < first_end_mm)) ||
           ((distance_mm >= second_start_mm) &&
            (distance_mm < config->lap_distance_mm));
}

uint16_t RampFactorPermille(int32_t distance_mm,
                            int32_t start_mm,
                            int32_t end_mm,
                            bool increasing)
{
    if (distance_mm <= start_mm) {
        return increasing ? 0U : 1000U;
    }
    if (distance_mm >= end_mm) {
        return increasing ? 1000U : 0U;
    }
    const int32_t span_mm = end_mm - start_mm;
    const uint16_t rising = static_cast<uint16_t>(
        ((distance_mm - start_mm) * 1000) / span_mm);
    return increasing ? rising
                      : static_cast<uint16_t>(1000U - rising);
}

uint16_t CourseLineCurveFactorPermille(int32_t distance_mm,
                                       const HConfig *config)
{
    const int32_t straight_mm = config->h4_b_distance_mm;
    const int32_t curved_total_mm =
        static_cast<int32_t>(config->lap_distance_mm) -
        2 * straight_mm;
    if (curved_total_mm <= 0) {
        return 0U;
    }
    const int32_t semicircle_mm = curved_total_mm / 2;
    const int32_t first_curve_start_mm = straight_mm;
    const int32_t first_curve_end_mm =
        first_curve_start_mm + semicircle_mm;
    const int32_t second_curve_start_mm =
        first_curve_end_mm + straight_mm;

    if (distance_mm <
        first_curve_start_mm - kCourseLineTransitionMm) {
        return 0U;
    }
    if (distance_mm < first_curve_start_mm) {
        return RampFactorPermille(
            distance_mm,
            first_curve_start_mm - kCourseLineTransitionMm,
            first_curve_start_mm,
            true);
    }
    if (distance_mm < first_curve_end_mm) {
        return 1000U;
    }
    if (distance_mm < first_curve_end_mm + kCourseLineTransitionMm) {
        return RampFactorPermille(
            distance_mm,
            first_curve_end_mm,
            first_curve_end_mm + kCourseLineTransitionMm,
            false);
    }
    if (distance_mm <
        second_curve_start_mm - kCourseLineTransitionMm) {
        return 0U;
    }
    if (distance_mm < second_curve_start_mm) {
        return RampFactorPermille(
            distance_mm,
            second_curve_start_mm - kCourseLineTransitionMm,
            second_curve_start_mm,
            true);
    }
    return 1000U;
}

int32_t Interpolate(int32_t straight_value,
                    int32_t curve_value,
                    uint16_t curve_factor_permille)
{
    return straight_value + static_cast<int32_t>(
        (static_cast<int64_t>(curve_value - straight_value) *
         curve_factor_permille) / 1000LL);
}

LineControlTuning CourseLineTuning(const HConfig *config,
                                   uint16_t curve_factor_permille)
{
    LineControlTuning tuning = {};
    tuning.kp_milli = Interpolate(
        config->course_straight_line_kp_milli,
        config->line_kp_milli,
        curve_factor_permille);
    tuning.kd_milli = Interpolate(
        config->course_straight_line_kd_milli,
        config->line_kd_milli,
        curve_factor_permille);
    tuning.max_correction_rpm = static_cast<int16_t>(Interpolate(
        config->course_straight_line_max_correction_rpm,
        config->line_max_correction_rpm,
        curve_factor_permille));
    tuning.correction_slew_rpm_s = static_cast<uint16_t>(Interpolate(
        config->course_straight_line_slew_rpm_s,
        config->line_correction_slew_rpm_s,
        curve_factor_permille));
    tuning.derivative_filter_permille = static_cast<uint16_t>(Interpolate(
        kStraightDerivativeFilterPermille,
        kCurveDerivativeFilterPermille,
        curve_factor_permille));
    return tuning;
}

void H5UpdateRoadSpeedLimit(CourseState *state,
                            uint32_t now_ms,
                            const HConfig *config)
{
    state->h5_curve_mode =
        H5DistanceRequestsCurve(state->distance_mm, config);

    uint32_t elapsed_ms = now_ms - state->h5_speed_limit_update_ms;
    state->h5_speed_limit_update_ms = now_ms;
    if (elapsed_ms > 100U) {
        elapsed_ms = 100U;
    }
    const int32_t target_millirpm = static_cast<int32_t>(
        state->h5_curve_mode
            ? config->h5_approach_rpm : config->h5_cruise_rpm) * 1000;
    if (state->h5_speed_limit_millirpm == target_millirpm) {
        state->h5_road_ramp_rpm_s = 0;
        return;
    }
    const bool accelerating =
        state->h5_speed_limit_millirpm < target_millirpm;
    const int16_t rate_rpm_s = static_cast<int16_t>(
        accelerating ? config->h5_launch_ramp_rpm_s
                     : kH5CurveEntryRampRpmS);
    const int32_t maximum_delta = static_cast<int32_t>(
        static_cast<uint32_t>(rate_rpm_s) * elapsed_ms);
    const int32_t remaining = accelerating
        ? (target_millirpm - state->h5_speed_limit_millirpm)
        : (state->h5_speed_limit_millirpm - target_millirpm);
    const int32_t delta =
        (remaining < maximum_delta) ? remaining : maximum_delta;
    state->h5_speed_limit_millirpm += accelerating ? delta : -delta;
    state->h5_road_ramp_rpm_s = static_cast<int16_t>(
        accelerating ? rate_rpm_s : -rate_rpm_s);
}

int16_t H5RoadLimitedRpm(const CourseState *state, int16_t profile_rpm)
{
    const int32_t road_limit_rpm = state->h5_speed_limit_millirpm / 1000;
    return static_cast<int16_t>(
        (road_limit_rpm < profile_rpm) ? road_limit_rpm : profile_rpm);
}

int32_t H5CommandedAccelerationMmS2(const CourseState *state,
                                    uint32_t now_ms,
                                    const HConfig *config)
{
    const int16_t rpm = H5RequestedRpm(state, now_ms, config);
    if (!state->h5_braking) {
        if ((state->h5_speed_limit_millirpm <
             static_cast<int32_t>(rpm) * 1000) &&
            (state->h5_road_ramp_rpm_s != 0)) {
            const int32_t magnitude = RpmRateToAccelerationMmS2(
                static_cast<uint16_t>(Abs32(
                    state->h5_road_ramp_rpm_s)), config);
            return (state->h5_road_ramp_rpm_s > 0)
                ? magnitude : -magnitude;
        }
        return (rpm < static_cast<int16_t>(config->h5_cruise_rpm))
            ? RpmRateToAccelerationMmS2(
                config->h5_launch_ramp_rpm_s, config)
            : 0;
    }
    if (state->passed_b_or_a) {
        return (rpm > 0)
            ? -RpmRateToAccelerationMmS2(
                config->h5_stop_ramp_rpm_s, config)
            : 0;
    }
    return (rpm > 20)
        ? -RpmRateToAccelerationMmS2(
            config->h5_stop_ramp_rpm_s, config)
        : 0;
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
         (kind != COURSE_H5) && (kind != COURSE_H6) &&
         (kind != COURSE_H7))) {
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
    state->h5_speed_limit_millirpm =
        static_cast<int32_t>(config->h5_cruise_rpm) * 1000;
    state->h5_speed_limit_update_ms = now_ms;
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
    const bool score_not_frozen = !state->passed_b_or_a;
    const bool timed_task_pending = (state->kind == COURSE_H2)
        ? false
        : ((UsesH4ChassisStrategy(state->kind) ||
            UsesH5ChassisStrategy(state->kind))
            ? score_not_frozen : true);
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
        if (UsesH4ChassisStrategy(state->kind)) {
            state->h4_commanded_accel_mm_s2 =
                -RpmRateToAccelerationMmS2(
                    config->h4_stop_ramp_rpm_s, config);
        } else if (UsesH5ChassisStrategy(state->kind)) {
            state->h5_commanded_accel_mm_s2 =
                -RpmRateToAccelerationMmS2(
                    config->h5_stop_ramp_rpm_s, config);
        }
        if ((input->chassis->left_rpm <= 2) &&
            (input->chassis->left_rpm >= -2) &&
            (input->chassis->right_rpm <= 2) &&
            (input->chassis->right_rpm >= -2)) {
            state->phase = COURSE_COMPLETE;
            state->h4_commanded_accel_mm_s2 = 0;
            state->h5_commanded_accel_mm_s2 = 0;
        }
        return command;
    }

    /* Distance, timeout and terminal-stop checks stay responsive at the
     * 2 ms application rate.  H4/H7 return through the IMU-yaw straight-drive
     * path below; only H2/H5/H6 wait for complete grayscale frames. */
    if (UsesH4ChassisStrategy(state->kind)) {
        if (!state->h4_braking &&
            (state->distance_mm >= config->h4_brake_distance_mm)) {
            state->h4_brake_start_rpm = static_cast<uint16_t>(
                H4RequestedRpm(state, input->now_ms, config));
            state->h4_brake_start_ms = input->now_ms;
            state->h4_braking = true;
        }
        if (!state->passed_b_or_a &&
            (state->distance_mm >= config->h4_b_distance_mm)) {
            state->passed_b_or_a = true;
            state->pass_ms = input->now_ms;
        }
        if (state->distance_mm >= config->h4_stop_distance_mm) {
            state->h4_commanded_accel_mm_s2 =
                -RpmRateToAccelerationMmS2(
                    config->h4_stop_ramp_rpm_s, config);
            state->phase = COURSE_STOPPING;
            return StopCommand();
        }
        if (state->passed_b_or_a &&
            (H4RequestedRpm(state, input->now_ms, config) == 0)) {
            state->h4_commanded_accel_mm_s2 =
                -RpmRateToAccelerationMmS2(
                    config->h4_stop_ramp_rpm_s, config);
            state->phase = COURSE_STOPPING;
            return StopCommand();
        }
        if (!ImuFresh(input->imu, input->now_ms)) {
            Fail(state, COURSE_FAILURE_IMU_STALE);
            return StopCommand();
        }
        const int16_t base_rpm =
            H4RequestedRpm(state, input->now_ms, config);
        state->h4_commanded_accel_mm_s2 =
            H4CommandedAccelerationMmS2(
                state, input->now_ms, config);
        const MotionCommand straight = H4StraightCommand(
            state, base_rpm, input->imu, config);
        if (Abs32(state->h4_yaw_error_mdeg) >
            kMaximumHeadingErrorMdeg) {
            Fail(state, COURSE_FAILURE_IMU_STALE);
            return StopCommand();
        }
        return straight;
    } else if (UsesH5ChassisStrategy(state->kind) &&
               state->passed_b_or_a) {
        const int16_t base_rpm =
            H5RequestedRpm(state, input->now_ms, config);
        if (base_rpm == 0) {
            state->h5_commanded_accel_mm_s2 =
                -RpmRateToAccelerationMmS2(
                    config->h5_stop_ramp_rpm_s, config);
            state->phase = COURSE_STOPPING;
            return StopCommand();
        }
        state->h5_commanded_accel_mm_s2 =
            -RpmRateToAccelerationMmS2(
                config->h5_stop_ramp_rpm_s, config);
        command.mode = MOTION_COMMAND_SPEED;
        command.left_rpm = base_rpm;
        command.right_rpm = base_rpm;
        return command;
    } else if (UsesH5ChassisStrategy(state->kind)) {
        if (!state->h5_braking &&
            (state->distance_mm >= config->h5_brake_distance_mm)) {
            state->h5_brake_start_rpm = static_cast<uint16_t>(
                H5RoadLimitedRpm(
                    state,
                    H5RequestedRpm(state, input->now_ms, config)));
            state->h5_brake_start_ms = input->now_ms;
            state->h5_braking = true;
        }
        state->h5_commanded_accel_mm_s2 =
            H5CommandedAccelerationMmS2(
                state, input->now_ms, config);
        if (state->distance_mm >=
            static_cast<int32_t>(config->lap_distance_mm) +
                kH5OdometryFinishOffsetMm) {
            /* H5 and H6 deliberately share the same odometry-only chassis
             * completion.  Gray data remains a steering input, but the
             * physical A marker does not participate in either finish. */
            state->passed_b_or_a = true;
            state->pass_ms = input->now_ms;
            state->h5_commanded_accel_mm_s2 =
                -RpmRateToAccelerationMmS2(
                    config->h5_stop_ramp_rpm_s, config);
        }
    } else if ((state->kind == COURSE_H2) &&
               (state->distance_mm >=
            static_cast<int32_t>(config->lap_distance_mm) +
                    config->h2_loop_offset_mm)) {
        /* H2 completion uses average odometry because the inner and outer
         * wheels necessarily travel different distances around a lap. */
        state->phase = COURSE_COMPLETE;
        state->pass_ms = input->now_ms;
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

    /* H2/H5/H6 no longer use the wide A marker as a finish decision. */
    state->finish_confirm_frames = 0U;

    int16_t base_rpm = static_cast<int16_t>(
        CruiseFor(state->kind, config));
    const bool segmented_line_control =
        UsesH5ChassisStrategy(state->kind);
    if (segmented_line_control) {
        state->line_curve_factor_permille =
            CourseLineCurveFactorPermille(
                state->distance_mm, config);
    }
    if (UsesH5ChassisStrategy(state->kind)) {
        const int16_t profile_rpm =
            H5RequestedRpm(state, input->now_ms, config);
        if (!state->h5_braking) {
            H5UpdateRoadSpeedLimit(state, input->now_ms, config);
        }
        base_rpm = state->h5_braking
            ? profile_rpm : H5RoadLimitedRpm(state, profile_rpm);
        state->h5_commanded_accel_mm_s2 =
            H5CommandedAccelerationMmS2(
                state, input->now_ms, config);
    } else if (state->kind == COURSE_H2) {
        const int32_t target_distance_mm =
            static_cast<int32_t>(config->lap_distance_mm) +
            config->h2_loop_offset_mm;
        const int32_t remaining_mm =
            target_distance_mm - state->distance_mm;
        if (remaining_mm <= kH2FinalPositionDistanceMm) {
            state->phase = COURSE_APPROACH;
            int32_t maximum_rpm = config->approach_rpm;
            if (maximum_rpm < kH2FinalMinimumRpm) {
                maximum_rpm = kH2FinalMinimumRpm;
            }
            const int32_t bounded_remaining = Clamp32(
                remaining_mm, 0, kH2FinalPositionDistanceMm);
            base_rpm = static_cast<int16_t>(
                kH2FinalMinimumRpm +
                ((maximum_rpm - kH2FinalMinimumRpm) *
                 bounded_remaining) /
                    kH2FinalPositionDistanceMm);
        }
    }
    if (!LineControl_IsTrackUsable(input->line) &&
        (input->line != 0) &&
        (input->line->calibration_fault_mask == 0U) &&
        (input->line->track_state !=
         drivers::GRAYSCALE_TRACK_SENSOR_FAULT)) {
        /* Every line-following task treats a valid ADC frame with no usable
         * track as recoverable.  Preserve the current speed-profile mean and
         * search along a forward-right arc until the track returns.  A
         * stalled ADC scan, calibration fault or sensor fault remains a
         * hardware failure and follows the normal safe-stop path. */
        if (!LineControl_SearchRight(&state->line_control,
                                     base_rpm,
                                     input->now_ms)) {
            Fail(state, COURSE_FAILURE_LINE_LOST);
            return StopCommand();
        }
        command.mode = MOTION_COMMAND_SPEED;
        command.left_rpm = state->line_control.left_rpm;
        command.right_rpm = state->line_control.right_rpm;
        return command;
    }
    bool line_control_ok = false;
    if (segmented_line_control) {
        const LineControlTuning tuning = CourseLineTuning(
            config, state->line_curve_factor_permille);
        line_control_ok = LineControl_UpdateTuned(
            &state->line_control,
            input->line,
            base_rpm,
            input->now_ms,
            config,
            &tuning);
    } else {
        line_control_ok = LineControl_Update(
            &state->line_control,
            input->line,
            base_rpm,
            input->now_ms,
            config);
    }
    if (!line_control_ok) {
        Fail(state, COURSE_FAILURE_LINE_LOST);
        return StopCommand();
    }
    command.mode = MOTION_COMMAND_SPEED;
    command.left_rpm = state->line_control.left_rpm;
    command.right_rpm = state->line_control.right_rpm;
    if (UsesH5ChassisStrategy(state->kind) ||
        ((state->kind == COURSE_H2) &&
         (state->phase == COURSE_APPROACH))) {
        const int32_t correction_limit = base_rpm / 2;
        const int32_t correction = Clamp32(
            state->line_control.last_correction_rpm,
            -correction_limit,
            correction_limit);
        command.left_rpm = static_cast<int16_t>(base_rpm - correction);
        command.right_rpm = static_cast<int16_t>(base_rpm + correction);
        state->line_control.left_rpm = command.left_rpm;
        state->line_control.right_rpm = command.right_rpm;
    }
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
