#include "control/ball_control.h"

#include <limits.h>

namespace gugah {
namespace {

static const int16_t kMaximumTarget0p1mm = 1000;
static const int16_t kEndpointLimit0p1mm = 1150;
static const int32_t kMaximumEstimatedVelocity0p1mmS = 8000;
/* g * rad/mdeg in 0.1 mm/s^2, scaled by 1000. */
static const int32_t kGravity0p1mmS2PerMdegMilli = 1712;
static const int32_t kFastBrakeSlewMdegS = 60000;
/*
 * Only a velocity measured from consecutive camera frames may declare that
 * the ball is rolling.  Using the model-predicted velocity here made the
 * controller remove its breakaway angle before the stationary ball had
 * actually moved.  The 8 mm/s threshold also rejects bench position jitter.
 */
static const int32_t kRollingDetectionVelocity0p1mmS = 80;
static const int32_t kStictionRampMdegS = 3000;
static const int32_t kStictionReleaseMdegS = 60000;
static const uint32_t kStationaryConfirmMs = 400U;
static const int32_t kPdCorrectionLimitMdeg = 4500;
static const uint8_t kVelocityHistoryCount = 5U;
static const int32_t kEstimatorScale = 256;
/* Temporary bench comparison: keep the calibrated table in FRAM/Shell, but
 * bypass it in both the observer model and the commanded beam angle. */
static const bool kEnableHoldAngleCompensation = false;

int32_t Abs32(int32_t value)
{
    if (value >= 0) {
        return value;
    }
    return (value == INT32_MIN) ? INT32_MAX : -value;
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

int32_t StepToward(int32_t current, int32_t target, int32_t step)
{
    if (current < target) {
        const int32_t remaining = target - current;
        return current + ((remaining < step) ? remaining : step);
    }
    if (current > target) {
        const int32_t remaining = current - target;
        return current - ((remaining < step) ? remaining : step);
    }
    return current;
}

bool InputReady(const BallInput *input)
{
    return (input != 0) && input->vision.ball_usable &&
           input->imu.valid && input->dm.valid &&
           (input->dm.state == 1U) &&
           (input->vision.ball_age_ms <= 60U) &&
           ((input->now_ms - input->imu.received_ms) <= 100U) &&
           ((input->now_ms - input->dm.received_ms) <= 100U) &&
           (Abs32(input->vision.frame.position_0p1mm) <=
            kEndpointLimit0p1mm);
}

int32_t MapDmToBeam(const HConfig *config, int32_t dm_position_mrad)
{
    const bool ascending =
        config->dm_position_mrad[4] >= config->dm_position_mrad[0];
    if ((ascending &&
         (dm_position_mrad <= config->dm_position_mrad[0])) ||
        (!ascending &&
         (dm_position_mrad >= config->dm_position_mrad[0]))) {
        return config->beam_angle_mdeg[0];
    }
    for (uint8_t i = 1U; i < 5U; i++) {
        const int32_t dm0 = config->dm_position_mrad[i - 1U];
        const int32_t dm1 = config->dm_position_mrad[i];
        const bool in_segment = ascending
            ? (dm_position_mrad <= dm1)
            : (dm_position_mrad >= dm1);
        if (in_segment) {
            if (dm1 == dm0) {
                return config->beam_angle_mdeg[i - 1U];
            }
            const int32_t beam0 = config->beam_angle_mdeg[i - 1U];
            const int32_t beam1 = config->beam_angle_mdeg[i];
            return beam0 + static_cast<int32_t>(
                (static_cast<int64_t>(dm_position_mrad - dm0) *
                 (beam1 - beam0)) /
                (dm1 - dm0));
        }
    }
    return config->beam_angle_mdeg[4];
}

int16_t CalibrateCameraPosition0p1mm(int16_t camera_position_0p1mm)
{
    /* 2026-08-01 bench calibration.  Both axes use 0.1 mm.  The camera
     * breakpoints are deliberately non-uniform; interpolate to the measured
     * physical position, and linearly extrapolate the end segments so the
     * endpoint safety check cannot be hidden by clamping. */
    static const int16_t camera_points[11] = {
        -1000, -853, -664, -500, -300, -89,
        130, 367, 581, 808, 1000
    };
    static const int16_t physical_points[11] = {
        -1000, -800, -600, -400, -200, 0,
        200, 400, 600, 800, 1000
    };
    uint8_t upper = 1U;
    if (camera_position_0p1mm >= camera_points[10]) {
        upper = 10U;
    } else if (camera_position_0p1mm > camera_points[0]) {
        while ((upper < 10U) &&
               (camera_position_0p1mm > camera_points[upper])) {
            upper++;
        }
    }
    const int32_t x0 = camera_points[upper - 1U];
    const int32_t x1 = camera_points[upper];
    const int32_t y0 = physical_points[upper - 1U];
    const int32_t y1 = physical_points[upper];
    const int32_t calibrated = y0 + static_cast<int32_t>(
        (static_cast<int64_t>(camera_position_0p1mm - x0) *
         (y1 - y0)) / (x1 - x0));
    return static_cast<int16_t>(Clamp32(calibrated, INT16_MIN, INT16_MAX));
}

int16_t ConfiguredPosition(const BallInput *input, const HConfig *config)
{
    int16_t value = CalibrateCameraPosition0p1mm(
        input->vision.frame.position_0p1mm);
    if (config->vision_position_invert != 0U) {
        value = static_cast<int16_t>(-value);
    }
    return value;
}

int32_t BeamAcceleration0p1mmS2(int32_t beam_angle_mdeg,
                                const HConfig *config)
{
    return static_cast<int32_t>(
        (static_cast<int64_t>(beam_angle_mdeg) *
         kGravity0p1mmS2PerMdegMilli *
         config->ball_model_roll_gain_permille) /
        1000000LL);
}

int32_t HoldAngleMdeg(int32_t position_0p1mm,
                      const HConfig *config)
{
    if (!kEnableHoldAngleCompensation) {
        return 0;
    }
    if (position_0p1mm <= config->ball_hold_position_0p1mm[0]) {
        return config->ball_hold_angle_mdeg[0];
    }
    for (uint8_t i = 1U; i < 5U; i++) {
        if (position_0p1mm <= config->ball_hold_position_0p1mm[i]) {
            const int32_t x0 = config->ball_hold_position_0p1mm[i - 1U];
            const int32_t x1 = config->ball_hold_position_0p1mm[i];
            const int32_t y0 = config->ball_hold_angle_mdeg[i - 1U];
            const int32_t y1 = config->ball_hold_angle_mdeg[i];
            return y0 + static_cast<int32_t>(
                (static_cast<int64_t>(position_0p1mm - x0) *
                 (y1 - y0)) / (x1 - x0));
        }
    }
    return config->ball_hold_angle_mdeg[4];
}

int32_t ModelAcceleration0p1mmS2(const BallState *state,
                                 const BallInput *input,
                                 const HConfig *config)
{
    const int32_t actual_beam_mdeg =
        MapDmToBeam(config, input->dm.position_mrad);
    const int32_t global_beam_mdeg = actual_beam_mdeg +
        static_cast<int32_t>(
            (static_cast<int64_t>(config->ball_pitch_gain_permille) *
             input->imu.pitch_mdeg) /
            1000LL);
    /* The table gives the global angle that produces zero acceleration at
     * this position.  Subtract it before applying the rolling model. */
    int32_t effective_beam_mdeg = global_beam_mdeg -
        HoldAngleMdeg(state->observer_position_0p1mm, config);
    const int32_t breakaway_mdeg =
        config->ball_kp_mdeg_per_0p1mm;
    const int32_t rolling_friction_mdeg =
        config->ball_accel_ff_mdeg_per_mm_s2;
    if (Abs32(state->observer_velocity_0p1mm_s) <=
        kRollingDetectionVelocity0p1mmS) {
        if (Abs32(effective_beam_mdeg) <= breakaway_mdeg) {
            effective_beam_mdeg = 0;
        } else if (effective_beam_mdeg > 0) {
            effective_beam_mdeg -= rolling_friction_mdeg;
        } else {
            effective_beam_mdeg += rolling_friction_mdeg;
        }
    } else if (state->observer_velocity_0p1mm_s > 0) {
        effective_beam_mdeg -= rolling_friction_mdeg;
    } else {
        effective_beam_mdeg += rolling_friction_mdeg;
    }
    const int32_t chassis_component = static_cast<int32_t>(
        (static_cast<int64_t>(input->chassis_accel_mm_s2) * 10LL *
         config->ball_model_roll_gain_permille) /
        1000LL);
    return BeamAcceleration0p1mmS2(effective_beam_mdeg, config) -
        chassis_component;
}

int32_t ChassisFeedforwardMdeg(const BallInput *input,
                              const HConfig *config)
{
    const int32_t chassis_component = static_cast<int32_t>(
        (static_cast<int64_t>(input->chassis_accel_mm_s2) * 10LL *
         config->ball_model_roll_gain_permille) /
        1000LL);
    const int64_t denominator =
        static_cast<int64_t>(kGravity0p1mmS2PerMdegMilli) *
        config->ball_model_roll_gain_permille;
    return static_cast<int32_t>(
        (static_cast<int64_t>(chassis_component) * 1000000LL) /
        denominator);
}

void UpdateEstimator(BallState *state,
                     const BallInput *input,
                     const HConfig *config)
{
    const int16_t position = ConfiguredPosition(input, config);
    const uint32_t received_ms = input->vision.frame.received_ms;
    uint32_t sample_ms =
        (received_ms >= input->vision.frame.source_delay_ms)
        ? (received_ms - input->vision.frame.source_delay_ms)
        : received_ms;

    if (!state->has_sample) {
        state->observer_position_q8 =
            static_cast<int32_t>(position) * kEstimatorScale;
        state->observer_velocity_q8 = 0;
        state->observer_position_0p1mm = position;
        state->observer_velocity_0p1mm_s = 0;
        state->estimated_position_0p1mm = position;
        state->estimated_velocity_0p1mm_s = 0;
        state->measured_position_0p1mm = position;
        state->position_history_0p1mm[0] = position;
        state->sample_history_ms[0] = sample_ms;
        state->sample_history_count = 1U;
        state->last_sample_ms = sample_ms;
        state->last_sequence = input->vision.frame.sequence;
        state->estimator_update_ms = input->now_ms;
        state->has_sample = true;
        state->model_acceleration_0p1mm_s2 =
            ModelAcceleration0p1mmS2(state, input, config);
        return;
    }

    state->model_acceleration_0p1mm_s2 =
        ModelAcceleration0p1mmS2(state, input, config);
    uint32_t prediction_ms =
        input->now_ms - state->estimator_update_ms;
    if (prediction_ms > 100U) {
        prediction_ms = 100U;
    }
    state->observer_position_q8 += static_cast<int32_t>(
        (static_cast<int64_t>(state->observer_velocity_q8) *
         prediction_ms) / 1000LL) + static_cast<int32_t>(
        (static_cast<int64_t>(state->model_acceleration_0p1mm_s2) *
         prediction_ms * prediction_ms * kEstimatorScale) /
        2000000LL);
    state->observer_velocity_q8 = Clamp32(
        state->observer_velocity_q8 + static_cast<int32_t>(
            (static_cast<int64_t>(
                 state->model_acceleration_0p1mm_s2) *
             prediction_ms * kEstimatorScale) / 1000LL),
        -kMaximumEstimatedVelocity0p1mmS * kEstimatorScale,
        kMaximumEstimatedVelocity0p1mmS * kEstimatorScale);
    state->estimator_update_ms = input->now_ms;

    if (input->vision.frame.sequence != state->last_sequence) {
        if ((sample_ms <= state->last_sample_ms) ||
            ((sample_ms - state->last_sample_ms) > 100U)) {
            sample_ms = received_ms;
            if (sample_ms <= state->last_sample_ms) {
                sample_ms = state->last_sample_ms + 1U;
            }
        }
        const uint32_t sample_interval_ms =
            sample_ms - state->last_sample_ms;
        if (state->sample_history_count < kVelocityHistoryCount) {
            const uint8_t index = state->sample_history_count;
            state->position_history_0p1mm[index] = position;
            state->sample_history_ms[index] = sample_ms;
            state->sample_history_count++;
        } else {
            for (uint8_t i = 0U;
                 i < (kVelocityHistoryCount - 1U); i++) {
                state->position_history_0p1mm[i] =
                    state->position_history_0p1mm[i + 1U];
                state->sample_history_ms[i] =
                    state->sample_history_ms[i + 1U];
            }
            state->position_history_0p1mm[
                kVelocityHistoryCount - 1U] = position;
            state->sample_history_ms[
                kVelocityHistoryCount - 1U] = sample_ms;
        }

        int32_t measured_velocity = state->observer_velocity_0p1mm_s;
        if (state->sample_history_count >= 2U) {
            const uint8_t newest =
                static_cast<uint8_t>(state->sample_history_count - 1U);
            if (state->sample_history_count ==
                kVelocityHistoryCount) {
                /* Five-point quadratic Savitzky-Golay derivative at the
                 * newest endpoint.  Unlike a straight-line regression, it
                 * does not report the velocity from 40 ms in the past. */
                const uint32_t span_ms =
                    state->sample_history_ms[newest] -
                    state->sample_history_ms[0];
                const uint32_t average_period_ms = span_ms / 4U;
                if (average_period_ms != 0U) {
                    static const int8_t weights[5] = {
                        26, -27, -40, -13, 54
                    };
                    int64_t weighted_position = 0;
                    for (uint8_t i = 0U; i < 5U; i++) {
                        weighted_position +=
                            static_cast<int32_t>(weights[i]) *
                            state->position_history_0p1mm[i];
                    }
                    measured_velocity = Clamp32(
                        static_cast<int32_t>(
                            (weighted_position * 1000LL) /
                            (70LL * average_period_ms)),
                        -kMaximumEstimatedVelocity0p1mmS,
                        kMaximumEstimatedVelocity0p1mmS);
                }
            } else if (state->sample_history_count >= 3U) {
                /* Quadratic backward derivative while the five-frame
                 * window is warming up. */
                const uint8_t previous = newest - 1U;
                const uint8_t oldest = newest - 2U;
                const uint32_t span_ms =
                    state->sample_history_ms[newest] -
                    state->sample_history_ms[oldest];
                if (span_ms != 0U) {
                    const int32_t numerator =
                        3 * state->position_history_0p1mm[newest] -
                        4 * state->position_history_0p1mm[previous] +
                        state->position_history_0p1mm[oldest];
                    measured_velocity = Clamp32(
                        static_cast<int32_t>(
                            (static_cast<int64_t>(numerator) * 1000LL) /
                            span_ms),
                        -kMaximumEstimatedVelocity0p1mmS,
                        kMaximumEstimatedVelocity0p1mmS);
                }
            } else {
                const uint32_t elapsed =
                    state->sample_history_ms[newest] -
                    state->sample_history_ms[0];
                if (elapsed != 0U) {
                    measured_velocity = Clamp32(
                        static_cast<int32_t>(
                            (static_cast<int64_t>(
                                 state->position_history_0p1mm[newest] -
                                 state->position_history_0p1mm[0]) *
                             1000LL) / elapsed),
                        -kMaximumEstimatedVelocity0p1mmS,
                        kMaximumEstimatedVelocity0p1mmS);
                }
            }
        }

        uint32_t measurement_age_ms = input->now_ms - sample_ms;
        if (measurement_age_ms > 120U) {
            measurement_age_ms = 120U;
        }
        /* Project the delayed position measurement to the current control
         * instant with the model observer.  The camera derivative above is
         * deliberately not used here: it is retained only as a diagnostic
         * and as evidence that the ball has really broken static friction. */
        const int32_t measured_now_q8 =
            static_cast<int32_t>(position) * kEstimatorScale +
            static_cast<int32_t>(
                (static_cast<int64_t>(state->observer_velocity_q8) *
                 measurement_age_ms) / 1000LL) -
            static_cast<int32_t>(
                (static_cast<int64_t>(
                     state->model_acceleration_0p1mm_s2) *
                 measurement_age_ms * measurement_age_ms *
                 kEstimatorScale) / 2000000LL);
        const int32_t position_residual_q8 =
            measured_now_q8 - state->observer_position_q8;
        state->observer_position_q8 += static_cast<int32_t>(
            (static_cast<int64_t>(position_residual_q8) *
             config->ball_observer_alpha_permille) / 1000LL);
        state->observer_velocity_q8 = Clamp32(
            state->observer_velocity_q8 + static_cast<int32_t>(
                (static_cast<int64_t>(position_residual_q8) *
                 config->ball_observer_beta_permille) /
                sample_interval_ms),
            -kMaximumEstimatedVelocity0p1mmS * kEstimatorScale,
            kMaximumEstimatedVelocity0p1mmS * kEstimatorScale);
        state->measured_velocity_0p1mm_s = measured_velocity;
        state->measured_position_0p1mm = position;
        state->last_sample_ms = sample_ms;
        state->last_sequence = input->vision.frame.sequence;
    }
    state->observer_position_0p1mm = static_cast<int32_t>(
        state->observer_position_q8 / kEstimatorScale);
    state->observer_velocity_0p1mm_s = Clamp32(
        static_cast<int32_t>(
            state->observer_velocity_q8 / kEstimatorScale),
        -kMaximumEstimatedVelocity0p1mmS,
        kMaximumEstimatedVelocity0p1mmS);
    state->estimated_position_0p1mm = state->observer_position_0p1mm;
    state->estimated_velocity_0p1mm_s = state->observer_velocity_0p1mm_s;
}

bool Start(BallState *state,
           BallMode mode,
           int16_t target_0p1mm,
           uint32_t timeout_ms,
           const BallInput *input,
           const HConfig *config)
{
    if ((state == 0) || (config == 0) || !InputReady(input) ||
        (target_0p1mm < -kMaximumTarget0p1mm) ||
        (target_0p1mm > kMaximumTarget0p1mm)) {
        return false;
    }
    /*
     * gugaPI keeps one continuous BallBalance owner.  Retargeting an
     * already active controller must therefore preserve its estimator and
     * beam reference; clearing them here creates a large H3 stage jump.
     */
    const bool retarget = Ball_IsActive(state);
    if (!retarget) {
        *state = {};
        state->beam_target_mdeg =
            MapDmToBeam(config, input->dm.position_mrad);
        state->dm_target_mrad = input->dm.position_mrad;
    }
    state->mode = mode;
    state->result = (mode == BALL_HOLD)
        ? BALL_RESULT_SUCCESS : BALL_RESULT_RUNNING;
    state->target_position_0p1mm = target_0p1mm;
    state->start_ms = input->now_ms;
    state->last_update_ms = input->now_ms;
    state->timeout_ms = timeout_ms;
    state->settle_start_ms = 0U;
    state->integral_error_0p1mm_ms = 0;
    UpdateEstimator(state, input, config);
    return true;
}

BallOutput Fail(BallState *state,
                BallResult result,
                const HConfig *config)
{
    state->mode = BALL_FAILED;
    state->result = result;
    state->beam_target_mdeg = 0;
    state->dm_target_mrad = Ball_MapBeamToDm(config, 0);
    BallOutput output = {};
    output.command_valid = true;
    output.stop_chassis = true;
    output.dm_target_mrad = state->dm_target_mrad;
    return output;
}

} /* namespace */

void Ball_Init(BallState *state)
{
    if (state == 0) {
        return;
    }
    *state = {};
    state->mode = BALL_IDLE;
    state->result = BALL_RESULT_IDLE;
}

bool Ball_StartHold(BallState *state,
                    int16_t target_0p1mm,
                    const BallInput *input,
                    const HConfig *config)
{
    return Start(state, BALL_HOLD, target_0p1mm, 0U, input, config);
}

bool Ball_StartMove(BallState *state,
                    int16_t target_0p1mm,
                    uint32_t timeout_ms,
                    const BallInput *input,
                    const HConfig *config)
{
    if ((timeout_ms < 50U) || (timeout_ms > 30000U)) {
        return false;
    }
    return Start(state,
                 BALL_MOVE,
                 target_0p1mm,
                 timeout_ms,
                 input,
                 config);
}

void Ball_Stop(BallState *state)
{
    if (state != 0) {
        Ball_Init(state);
    }
}

void Ball_Level(BallState *state)
{
    if (state == 0) {
        return;
    }
    state->mode = BALL_LEVEL;
    state->result = BALL_RESULT_IDLE;
    state->target_position_0p1mm = 0;
    state->integral_error_0p1mm_ms = 0;
}

BallOutput Ball_Update(BallState *state,
                       const BallInput *input,
                       const HConfig *config)
{
    BallOutput output = {};
    if ((state == 0) || (input == 0) || (config == 0)) {
        output.stop_chassis = true;
        return output;
    }
    if (state->mode == BALL_IDLE) {
        if (input->vision.ball_usable) {
            UpdateEstimator(state, input, config);
        }
        return output;
    }
    if ((state->mode == BALL_LEVEL) || (state->mode == BALL_FAILED)) {
        state->beam_target_mdeg = StepToward(
            state->beam_target_mdeg, 0, 150);
        state->dm_target_mrad =
            Ball_MapBeamToDm(config, state->beam_target_mdeg);
        output.command_valid = input->dm.valid;
        output.dm_target_mrad = state->dm_target_mrad;
        output.stop_chassis = (state->mode == BALL_FAILED);
        return output;
    }
    if (!input->vision.ball_usable ||
        (input->vision.ball_age_ms > 120U)) {
        return Fail(state, BALL_RESULT_VISION_LOST, config);
    }
    if ((input->now_ms - input->dm.received_ms) > 100U ||
        !input->dm.valid || (input->dm.state != 1U)) {
        return Fail(state, BALL_RESULT_DM_STALE, config);
    }
    if ((input->now_ms - input->imu.received_ms) > 100U ||
        !input->imu.valid) {
        return Fail(state, BALL_RESULT_IMU_STALE, config);
    }
    if (Abs32(ConfiguredPosition(input, config)) >
        kEndpointLimit0p1mm) {
        return Fail(state, BALL_RESULT_ENDPOINT, config);
    }

    UpdateEstimator(state, input, config);
    uint32_t elapsed_ms = input->now_ms - state->last_update_ms;
    if (elapsed_ms == 0U) {
        elapsed_ms = 10U;
    } else if (elapsed_ms > 100U) {
        elapsed_ms = 100U;
    }
    state->last_update_ms = input->now_ms;
    state->position_error_0p1mm =
        state->target_position_0p1mm -
        state->estimated_position_0p1mm;
    state->rail_compensation_mdeg = HoldAngleMdeg(
        state->estimated_position_0p1mm, config);
    const int32_t abs_error = Abs32(state->position_error_0p1mm);
    if (abs_error > state->maximum_abs_error_0p1mm) {
        state->maximum_abs_error_0p1mm = abs_error;
    }

    const bool degraded = input->vision.ball_age_ms > 60U;
    state->target_velocity_0p1mm_s = 0;
    state->velocity_error_0p1mm_s =
        -state->estimated_velocity_0p1mm_s;
    /* The ball-beam plant already contains the integrations from beam angle
     * to velocity and position.  Keep the outer loop strictly PD: position
     * error requests slope and the model-observed velocity supplies damping. */
    state->integral_error_0p1mm_ms = 0;
    state->pid_p_mdeg = static_cast<int32_t>(
        (static_cast<int64_t>(state->position_error_0p1mm) *
         config->ball_pid_kp_mdeg_per_mm) / 10LL);
    state->pid_i_mdeg = 0;
    state->pid_d_mdeg = static_cast<int32_t>(
        (-static_cast<int64_t>(state->estimated_velocity_0p1mm_s) *
         config->ball_pid_kd_mdeg_per_mm_s) / 10LL);
    state->pid_correction_mdeg = Clamp32(
        state->pid_p_mdeg + state->pid_d_mdeg,
        -kPdCorrectionLimitMdeg,
        kPdCorrectionLimitMdeg);
    state->desired_acceleration_0p1mm_s2 =
        BeamAcceleration0p1mmS2(state->pid_correction_mdeg, config);
    const int32_t limit = degraded
        ? config->ball_degraded_angle_mdeg
        : config->ball_max_angle_mdeg;
    int32_t stiction_target_mdeg = 0;
    const bool ball_rolling =
        Abs32(state->measured_velocity_0p1mm_s) >
        kRollingDetectionVelocity0p1mmS;
    if (ball_rolling ||
        (abs_error <= config->ball_position_tolerance_0p1mm)) {
        state->stationary_start_ms = 0U;
    } else if (state->stationary_start_ms == 0U) {
        state->stationary_start_ms = input->now_ms;
    }
    const bool breakaway_armed =
        (state->stationary_start_ms != 0U) &&
        ((input->now_ms - state->stationary_start_ms) >=
         kStationaryConfirmMs);
    /* Do not inject a large breakaway pulse inside the accepted +/-10 mm
     * band.  Small continuous PD corrections remain active there. */
    if (!ball_rolling && breakaway_armed &&
        (abs_error > config->ball_position_tolerance_0p1mm)) {
        if (state->position_error_0p1mm > 0) {
            stiction_target_mdeg =
                config->ball_kp_mdeg_per_0p1mm;
        } else if (state->position_error_0p1mm < 0) {
            stiction_target_mdeg =
                -config->ball_kp_mdeg_per_0p1mm;
        }
    }
    const int32_t stiction_rate =
        (ball_rolling || !breakaway_armed)
        ? kStictionReleaseMdegS
        : kStictionRampMdegS;
    int32_t stiction_step = static_cast<int32_t>(
        (static_cast<int64_t>(stiction_rate) * elapsed_ms) /
        1000LL);
    if (stiction_step < 1) {
        stiction_step = 1;
    }
    state->stiction_compensation_mdeg = StepToward(
        state->stiction_compensation_mdeg,
        stiction_target_mdeg,
        stiction_step);
    int32_t friction_feedforward_mdeg = 0;
    if (state->estimated_velocity_0p1mm_s >
        kRollingDetectionVelocity0p1mmS) {
        friction_feedforward_mdeg =
            config->ball_accel_ff_mdeg_per_mm_s2;
    } else if (state->estimated_velocity_0p1mm_s <
               -kRollingDetectionVelocity0p1mmS) {
        friction_feedforward_mdeg =
            -config->ball_accel_ff_mdeg_per_mm_s2;
    }
    const int32_t pitch_compensation_mdeg = static_cast<int32_t>(
        (static_cast<int64_t>(config->ball_pitch_gain_permille) *
         input->imu.pitch_mdeg) / 1000LL);
    const int32_t angle = Clamp32(
        state->rail_compensation_mdeg +
        state->pid_correction_mdeg +
        ChassisFeedforwardMdeg(input, config) -
        pitch_compensation_mdeg +
        friction_feedforward_mdeg +
        state->stiction_compensation_mdeg,
        -limit,
        limit);
    const bool fast_brake =
        Abs32(state->estimated_velocity_0p1mm_s) >
        config->ball_velocity_tolerance_0p1mm_s;
    const int32_t slew_rate = fast_brake
        ? kFastBrakeSlewMdegS
        : config->ball_angle_slew_mdeg_s;
    int32_t slew_step = static_cast<int32_t>(
        (static_cast<int64_t>(
             slew_rate) *
         elapsed_ms) /
        1000LL);
    if (slew_step < 1) {
        slew_step = 1;
    }
    state->beam_target_mdeg = StepToward(
        state->beam_target_mdeg,
        angle,
        slew_step);
    state->dm_target_mrad =
        Ball_MapBeamToDm(config, state->beam_target_mdeg);
    output.command_valid = true;
    output.dm_target_mrad = state->dm_target_mrad;

    if (state->mode != BALL_MOVE) {
        return output;
    }
    if ((input->now_ms - state->start_ms) > state->timeout_ms) {
        return Fail(state, BALL_RESULT_TIMEOUT, config);
    }
    const bool settled =
        (abs_error <= config->ball_position_tolerance_0p1mm) &&
        (Abs32(state->estimated_velocity_0p1mm_s) <=
         config->ball_velocity_tolerance_0p1mm_s);
    if (!settled) {
        state->settle_start_ms = 0U;
    } else if (state->settle_start_ms == 0U) {
        state->settle_start_ms = input->now_ms;
    } else if ((input->now_ms - state->settle_start_ms) >=
               config->ball_settle_ms) {
        state->mode = BALL_HOLD;
        state->result = BALL_RESULT_SUCCESS;
    }
    return output;
}

bool Ball_IsActive(const BallState *state)
{
    return (state != 0) &&
           ((state->mode == BALL_HOLD) ||
            (state->mode == BALL_MOVE));
}

int32_t Ball_HoldAngleMdeg(const HConfig *config,
                          int32_t position_0p1mm)
{
    return (config == 0) ? 0 : HoldAngleMdeg(position_0p1mm, config);
}

int16_t Ball_CalibrateCameraPosition0p1mm(int16_t camera_position_0p1mm)
{
    return CalibrateCameraPosition0p1mm(camera_position_0p1mm);
}

int32_t Ball_MapBeamToDm(const HConfig *config, int32_t beam_angle_mdeg)
{
    if (config == 0) {
        return 0;
    }
    if (beam_angle_mdeg <= config->beam_angle_mdeg[0]) {
        return config->dm_position_mrad[0];
    }
    for (uint8_t i = 1U; i < 5U; i++) {
        if (beam_angle_mdeg <= config->beam_angle_mdeg[i]) {
            const int32_t x0 = config->beam_angle_mdeg[i - 1U];
            const int32_t x1 = config->beam_angle_mdeg[i];
            const int32_t y0 = config->dm_position_mrad[i - 1U];
            const int32_t y1 = config->dm_position_mrad[i];
            return y0 + static_cast<int32_t>(
                (static_cast<int64_t>(beam_angle_mdeg - x0) *
                 (y1 - y0)) /
                (x1 - x0));
        }
    }
    return config->dm_position_mrad[4];
}

} /* namespace gugah */
