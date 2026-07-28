#include "app/local_motor_controller.h"

#include <limits.h>

#include "board/board_local_motor.h"
#include "config/feature_config.h"
#include "drivers/drv8876/drv8876.h"

namespace app {

#if FEATURE_ENABLE_LOCAL_MOTOR
namespace {

static const uint32_t kControlNominalPeriodMs = 10U;
static const uint32_t kControlMaximumGapMs = 50U;
static const uint32_t kCommandLeaseTimeoutMs = 300U;
/* Persistent PID gains retain the old MotorDriver convention: derivative
 * and integral terms are normalized to a 100 ms reference interval. */
static const uint32_t kPidReferencePeriodMs = 100U;
static const int32_t kPidScale = 16;

struct WheelControlState {
    int64_t integral_q4_ms;
    int32_t previous_error_rpm;
    uint32_t ramp_remainder;
    int8_t previous_direction;
};

LocalMotorState g_state = {};
#if !FEATURE_LOCAL_MOTOR_RIGHT_ONLY
WheelControlState g_leftControl = {};
#endif
WheelControlState g_rightControl = {};

int32_t AbsInt32(int32_t value)
{
    if (value == INT32_MIN) {
        return INT32_MAX;
    }
    return (value < 0) ? -value : value;
}

int32_t SaturateInt32(int64_t value)
{
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return static_cast<int32_t>(value);
}

int32_t SignedRoundedDivide(int64_t numerator, int64_t denominator)
{
    if (denominator <= 0) {
        return 0;
    }
    if (numerator >= 0) {
        numerator += denominator / 2;
    } else {
        numerator -= denominator / 2;
    }
    return SaturateInt32(numerator / denominator);
}

bool ConfigIsValid(const LocalMotorConfig *config)
{
    return (config != 0) &&
           (config->left_counts_per_rev != 0U) &&
           (config->right_counts_per_rev != 0U) &&
           (config->max_wheel_rpm != 0U) &&
           (config->max_duty_percent <= 100U) &&
           (config->min_duty_percent <= config->max_duty_percent) &&
           ((config->output_invert_flags & 0xFCU) == 0U) &&
           ((config->encoder_invert_flags & 0xFCU) == 0U);
}

bool ConfigsEqual(const LocalMotorConfig &left,
                  const LocalMotorConfig &right)
{
    return (left.left_counts_per_rev == right.left_counts_per_rev) &&
           (left.right_counts_per_rev == right.right_counts_per_rev) &&
           (left.max_wheel_rpm == right.max_wheel_rpm) &&
           (left.accel_rpm_per_s == right.accel_rpm_per_s) &&
           (left.decel_rpm_per_s == right.decel_rpm_per_s) &&
           (left.kp_q4_4 == right.kp_q4_4) &&
           (left.ki_q4_4 == right.ki_q4_4) &&
           (left.kd_q4_4 == right.kd_q4_4) &&
           (left.max_duty_percent == right.max_duty_percent) &&
           (left.min_duty_percent == right.min_duty_percent) &&
           (left.output_invert_flags == right.output_invert_flags) &&
           (left.encoder_invert_flags == right.encoder_invert_flags);
}

void ResetWheelControl(WheelControlState *control)
{
    control->integral_q4_ms = 0;
    control->previous_error_rpm = 0;
    control->ramp_remainder = 0U;
    control->previous_direction = 0;
}

void ResetControl(void)
{
#if !FEATURE_LOCAL_MOTOR_RIGHT_ONLY
    ResetWheelControl(&g_leftControl);
#endif
    ResetWheelControl(&g_rightControl);
    g_state.left.control_target_rpm = 0;
    g_state.right.control_target_rpm = 0;
    g_state.left.duty_q8 = 0U;
    g_state.right.duty_q8 = 0U;
}

int32_t AdvanceRampedTarget(int32_t current,
                            int32_t requested,
                            uint16_t accel_rate,
                            uint16_t decel_rate,
                            uint32_t elapsed_ms,
                            uint32_t *remainder)
{
    if (remainder == 0) {
        return requested;
    }
    if (current == requested) {
        *remainder = 0U;
        return requested;
    }

    int32_t stage_target = requested;
    if (((current > 0) && (requested < 0)) ||
        ((current < 0) && (requested > 0))) {
        stage_target = 0;
    }

    const bool accelerating =
        AbsInt32(stage_target) > AbsInt32(current);
    const uint16_t rate = accelerating ? accel_rate : decel_rate;
    if (rate == 0U) {
        *remainder = 0U;
        return stage_target;
    }

    const uint64_t scaled =
        static_cast<uint64_t>(rate) * elapsed_ms + *remainder;
    const uint32_t step = static_cast<uint32_t>(scaled / 1000U);
    *remainder = static_cast<uint32_t>(scaled % 1000U);
    if (step == 0U) {
        return current;
    }

    const int64_t difference =
        static_cast<int64_t>(stage_target) - current;
    const uint64_t distance = (difference < 0) ?
        static_cast<uint64_t>(-difference) : static_cast<uint64_t>(difference);
    if (step >= distance) {
        *remainder = 0U;
        return stage_target;
    }
    return (difference > 0) ?
        static_cast<int32_t>(current + static_cast<int32_t>(step)) :
        static_cast<int32_t>(current - static_cast<int32_t>(step));
}

int32_t ApplySign(int32_t value, bool invert)
{
    return invert ? SaturateInt32(-static_cast<int64_t>(value)) : value;
}

drivers::DriverStatus RefreshFeedbackOne(
    board::LocalMotorWheel wheel,
    LocalMotorWheelState *state,
    uint32_t counts_per_rev,
    bool encoder_inverted)
{
    board::LocalMotorEncoderSnapshot encoder = {};
    const drivers::DriverStatus status =
        board::Board_LocalMotorGetEncoder(wheel, &encoder);
    if (status != drivers::DRIVER_OK) {
        return status;
    }

    state->encoder_count = ApplySign(encoder.count, encoder_inverted);
    state->encoder_counts_per_second =
        ApplySign(encoder.counts_per_second, encoder_inverted);
    state->actual_rpm = SignedRoundedDivide(
        static_cast<int64_t>(state->encoder_counts_per_second) * 60LL,
        counts_per_rev);
    state->encoder_state = encoder.state;
    state->awake = board::Board_LocalMotorIsAwake(wheel);
    return drivers::DRIVER_OK;
}

uint16_t CalculateDutyQ8(WheelControlState *control,
                         int32_t control_target_rpm,
                         int32_t actual_rpm,
                         uint32_t elapsed_ms,
                         const LocalMotorConfig &config)
{
    const int8_t direction = (control_target_rpm < 0) ? -1 : 1;
    if (control->previous_direction != direction) {
        control->integral_q4_ms = 0;
        control->previous_error_rpm = 0;
        control->previous_direction = direction;
    }

    const int32_t measured_aligned =
        (direction < 0) ? ApplySign(actual_rpm, true) : actual_rpm;
    const int32_t error_rpm = SaturateInt32(
        static_cast<int64_t>(AbsInt32(control_target_rpm)) -
        static_cast<int64_t>(measured_aligned));
    uint32_t pid_elapsed_ms = elapsed_ms;
    if (pid_elapsed_ms == 0U) {
        pid_elapsed_ms = kControlNominalPeriodMs;
    } else if (pid_elapsed_ms > kControlMaximumGapMs) {
        pid_elapsed_ms = kControlMaximumGapMs;
    }

    const int64_t derivative_rpm =
        ((static_cast<int64_t>(error_rpm) -
          static_cast<int64_t>(control->previous_error_rpm)) *
         kPidReferencePeriodMs) / pid_elapsed_ms;
    control->previous_error_rpm = error_rpm;

    const int64_t max_q4 =
        static_cast<int64_t>(config.max_duty_percent) * kPidScale;
    const int64_t max_integral_q4_ms =
        max_q4 * kPidReferencePeriodMs;
    int64_t proposed_integral =
        control->integral_q4_ms +
        static_cast<int64_t>(config.ki_q4_4) * error_rpm * pid_elapsed_ms;
    if (proposed_integral > max_integral_q4_ms) {
        proposed_integral = max_integral_q4_ms;
    } else if (proposed_integral < -max_integral_q4_ms) {
        proposed_integral = -max_integral_q4_ms;
    }

    const int64_t proportional =
        static_cast<int64_t>(config.kp_q4_4) * error_rpm;
    const int64_t derivative =
        static_cast<int64_t>(config.kd_q4_4) * derivative_rpm;
    const int64_t proposed_output =
        proportional +
        (proposed_integral / kPidReferencePeriodMs) + derivative;
    const bool saturating_high =
        (proposed_output > max_q4) && (error_rpm > 0);
    const bool saturating_low =
        (proposed_output < 0) && (error_rpm < 0);
    if (!saturating_high && !saturating_low) {
        control->integral_q4_ms = proposed_integral;
    }

    int64_t output_q4 =
        proportional +
        (control->integral_q4_ms / kPidReferencePeriodMs) + derivative;
    if (output_q4 <= 0) {
        return 0U;
    }
    const int64_t min_q4 =
        static_cast<int64_t>(config.min_duty_percent) * kPidScale;
    if (output_q4 < min_q4) {
        output_q4 = min_q4;
    }
    if (output_q4 > max_q4) {
        output_q4 = max_q4;
    }
    return static_cast<uint16_t>(
        (output_q4 * drivers::DRV8876_DUTY_Q8_SCALE) / kPidScale);
}

drivers::DriverStatus ApplyWheel(board::LocalMotorWheel wheel,
                                 LocalMotorWheelState *state,
                                 WheelControlState *control,
                                 bool output_inverted,
                                 uint32_t elapsed_ms,
                                 uint32_t now_ms)
{
    state->control_target_rpm = AdvanceRampedTarget(
        state->control_target_rpm,
        state->target_rpm,
        g_state.config.accel_rpm_per_s,
        g_state.config.decel_rpm_per_s,
        elapsed_ms,
        &control->ramp_remainder);

    if (state->control_target_rpm == 0) {
        ResetWheelControl(control);
        state->duty_q8 = 0U;
        return board::Board_LocalMotorSleep(wheel);
    }

    state->duty_q8 = CalculateDutyQ8(control,
                                     state->control_target_rpm,
                                     state->actual_rpm,
                                     elapsed_ms,
                                     g_state.config);
    const bool reverse =
        (state->control_target_rpm < 0) != output_inverted;
    const drivers::DriverStatus status = board::Board_LocalMotorRun(
        wheel, reverse, state->duty_q8, now_ms);
    return (status == drivers::DRIVER_ERROR_BUSY) ?
        drivers::DRIVER_OK : status;
}

drivers::DriverStatus EmergencyStop(drivers::DriverStatus reason)
{
    const drivers::DriverStatus stop_status =
        board::Board_LocalMotorSleepAll();
    g_state.active = false;
    g_state.left.target_rpm = 0;
    g_state.right.target_rpm = 0;
    ResetControl();
    g_state.last_status = (stop_status == drivers::DRIVER_OK) ?
        reason : stop_status;
    return g_state.last_status;
}

} /* namespace */

drivers::DriverStatus LocalMotorController_Init(const LocalMotorConfig *config,
                                                uint32_t now_ms)
{
    if (!ConfigIsValid(config)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (!board::Board_LocalMotorIsReady()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    g_state = {};
    g_state.config = *config;
    g_state.last_control_ms = now_ms;
    g_state.last_lease_ms = now_ms;
    ResetControl();
    const drivers::DriverStatus status = board::Board_LocalMotorSleepAll();
    g_state.initialized = (status == drivers::DRIVER_OK);
    g_state.last_status = status;
    return status;
}

drivers::DriverStatus LocalMotorController_Configure(
    const LocalMotorConfig *config)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (!ConfigIsValid(config)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (g_state.active && !ConfigsEqual(g_state.config, *config)) {
        /* Applying CPR, PID or polarity changes to a live bridge can reverse
         * PH without a controlled zero crossing. The caller will stop, then
         * the next service pass can apply the new configuration. */
        return drivers::DRIVER_ERROR_BUSY;
    }
    g_state.config = *config;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus LocalMotorController_SetTargets(int32_t left_rpm,
                                                      int32_t right_rpm,
                                                      uint32_t now_ms)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (g_state.watchdog_latched) {
        return g_state.last_status;
    }
#if FEATURE_LOCAL_MOTOR_RIGHT_ONLY
    /* Never reinterpret a differential-drive command as a right-motor
     * command. The caller must explicitly request left=0 for this bench-only
     * hardware profile. */
    if (left_rpm != 0) {
        return drivers::DRIVER_ERROR_UNSUPPORTED;
    }
#endif
    if ((AbsInt32(left_rpm) > g_state.config.max_wheel_rpm) ||
        (AbsInt32(right_rpm) > g_state.config.max_wheel_rpm)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if ((left_rpm == 0) && (right_rpm == 0)) {
        return LocalMotorController_Stop();
    }

    g_state.left.target_rpm = left_rpm;
    g_state.right.target_rpm = right_rpm;
    g_state.active = true;
    g_state.last_lease_ms = now_ms;
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus LocalMotorController_RefreshLease(uint32_t now_ms)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (g_state.active) {
        g_state.last_lease_ms = now_ms;
    }
    return g_state.watchdog_latched ?
        g_state.last_status : drivers::DRIVER_OK;
}

drivers::DriverStatus LocalMotorController_Stop(void)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    g_state.active = false;
    g_state.left.target_rpm = 0;
    g_state.right.target_rpm = 0;
    ResetControl();
    const drivers::DriverStatus status = board::Board_LocalMotorSleepAll();
    /* A successful Stop is the explicit re-arm boundary after a latched
     * timeout. It never restores the command which caused the fault. */
    g_state.watchdog_latched = (status != drivers::DRIVER_OK);
    g_state.last_status = status;
    return status;
}

drivers::DriverStatus LocalMotorController_Update(uint32_t now_ms)
{
    if (!g_state.initialized) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    if (g_state.watchdog_latched) {
        (void) board::Board_LocalMotorSleepAll();
        return g_state.last_status;
    }

    uint32_t elapsed_ms = now_ms - g_state.last_control_ms;
    if (g_state.active && (elapsed_ms > kControlMaximumGapMs)) {
        g_state.watchdog_latched = true;
        return EmergencyStop(drivers::DRIVER_ERROR_TIMEOUT);
    }
    if (g_state.active &&
        ((now_ms - g_state.last_lease_ms) > kCommandLeaseTimeoutMs)) {
        g_state.watchdog_latched = true;
        return EmergencyStop(drivers::DRIVER_ERROR_TIMEOUT);
    }
    if (elapsed_ms == 0U) {
        elapsed_ms = kControlNominalPeriodMs;
    }
    g_state.last_control_ms = now_ms;

    drivers::DriverStatus status =
        board::Board_LocalMotorProcessEncoders(now_ms);
    if (status != drivers::DRIVER_OK) {
        return EmergencyStop(status);
    }

    const bool right_encoder_inverted =
        (g_state.config.encoder_invert_flags & 0x01U) != 0U;
#if FEATURE_LOCAL_MOTOR_RIGHT_ONLY
    status = RefreshFeedbackOne(board::LOCAL_MOTOR_RIGHT,
                                &g_state.right,
                                g_state.config.right_counts_per_rev,
                                right_encoder_inverted);
#else
    const bool left_encoder_inverted =
        (g_state.config.encoder_invert_flags & 0x02U) != 0U;
    status = RefreshFeedbackOne(board::LOCAL_MOTOR_LEFT,
                                &g_state.left,
                                g_state.config.left_counts_per_rev,
                                left_encoder_inverted);
    if (status == drivers::DRIVER_OK) {
        status = RefreshFeedbackOne(board::LOCAL_MOTOR_RIGHT,
                                    &g_state.right,
                                    g_state.config.right_counts_per_rev,
                                    right_encoder_inverted);
    }
#endif
    if (status != drivers::DRIVER_OK) {
        return EmergencyStop(status);
    }

    if (g_state.active) {
        const bool right_output_inverted =
            (g_state.config.output_invert_flags & 0x01U) != 0U;
#if FEATURE_LOCAL_MOTOR_RIGHT_ONLY
        status = ApplyWheel(board::LOCAL_MOTOR_RIGHT,
                            &g_state.right,
                            &g_rightControl,
                            right_output_inverted,
                            elapsed_ms,
                            now_ms);
#else
        const bool left_output_inverted =
            (g_state.config.output_invert_flags & 0x02U) != 0U;
        status = ApplyWheel(board::LOCAL_MOTOR_LEFT,
                            &g_state.left,
                            &g_leftControl,
                            left_output_inverted,
                            elapsed_ms,
                            now_ms);
        if (status == drivers::DRIVER_OK) {
            status = ApplyWheel(board::LOCAL_MOTOR_RIGHT,
                                &g_state.right,
                                &g_rightControl,
                                right_output_inverted,
                                elapsed_ms,
                                now_ms);
        }
#endif
        if (status != drivers::DRIVER_OK) {
            return EmergencyStop(status);
        }
    }

#if FEATURE_LOCAL_MOTOR_RIGHT_ONLY
    g_state.left = {};
#else
    g_state.left.awake =
        board::Board_LocalMotorIsAwake(board::LOCAL_MOTOR_LEFT);
#endif
    g_state.right.awake =
        board::Board_LocalMotorIsAwake(board::LOCAL_MOTOR_RIGHT);
    if (g_state.update_sequence != UINT32_MAX) {
        g_state.update_sequence++;
    }
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

const LocalMotorState *LocalMotorController_GetState(void)
{
    return &g_state;
}

#else

drivers::DriverStatus LocalMotorController_Init(const LocalMotorConfig *,
                                                uint32_t)
{
    return drivers::DRIVER_ERROR_UNSUPPORTED;
}

drivers::DriverStatus LocalMotorController_Configure(const LocalMotorConfig *)
{
    return drivers::DRIVER_ERROR_UNSUPPORTED;
}

drivers::DriverStatus LocalMotorController_SetTargets(int32_t,
                                                      int32_t,
                                                      uint32_t)
{
    return drivers::DRIVER_ERROR_UNSUPPORTED;
}

drivers::DriverStatus LocalMotorController_RefreshLease(uint32_t)
{
    return drivers::DRIVER_ERROR_UNSUPPORTED;
}

drivers::DriverStatus LocalMotorController_Stop(void)
{
    return drivers::DRIVER_ERROR_UNSUPPORTED;
}

drivers::DriverStatus LocalMotorController_Update(uint32_t)
{
    return drivers::DRIVER_ERROR_UNSUPPORTED;
}

const LocalMotorState *LocalMotorController_GetState(void)
{
    static const LocalMotorState state = {};
    return &state;
}

#endif

} /* namespace app */
