#ifndef APP_HEADING_ARC_MATH_H_
#define APP_HEADING_ARC_MATH_H_

#include <stdbool.h>
#include <stdint.h>

namespace app {
namespace heading_arc {

struct WheelCommand {
    int32_t left_rpm;
    int32_t right_rpm;
};

inline int32_t AbsInt32(int32_t value)
{
    if (value < 0) {
        return (value == INT32_MIN) ? INT32_MAX : -value;
    }
    return value;
}

inline int32_t ClampInt64ToInt32(int64_t value)
{
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return static_cast<int32_t>(value);
}

/* Predict the remaining yaw error after the configured actuation horizon.
 * The fixed margin is applied only while angular velocity is carrying the
 * chassis toward the target. A prediction that crosses zero deliberately
 * becomes a small reverse correction so the arc can damp overshoot without
 * commanding both wheels to stop. */
inline int32_t PredictErrorMdeg(int32_t error_mdeg,
                                int32_t gyro_rate_mdps,
                                uint32_t brake_ms,
                                int32_t brake_margin_mdeg)
{
    const int32_t direction = (error_mdeg > 0) ? 1 :
                              ((error_mdeg < 0) ? -1 : 0);
    const int64_t predicted_motion =
        (static_cast<int64_t>(gyro_rate_mdps) * brake_ms) / 1000LL;
    int64_t predicted = static_cast<int64_t>(error_mdeg) -
                        predicted_motion;
    if ((direction != 0) &&
        ((static_cast<int64_t>(gyro_rate_mdps) * direction) > 0LL)) {
        predicted -= static_cast<int64_t>(direction) *
                     brake_margin_mdeg;
    }
    return ClampInt64ToInt32(predicted);
}

inline int32_t BrakeAngleMdeg(int32_t error_mdeg,
                              int32_t gyro_rate_mdps,
                              uint32_t brake_ms,
                              int32_t brake_margin_mdeg)
{
    const int32_t direction = (error_mdeg >= 0) ? 1 : -1;
    const int64_t toward_rate =
        static_cast<int64_t>(gyro_rate_mdps) * direction;
    if (toward_rate <= 0LL) {
        return 0;
    }
    const int64_t angle =
        (toward_rate * brake_ms) / 1000LL +
        brake_margin_mdeg;
    return ClampInt64ToInt32(angle);
}

inline int32_t ComputeCorrectionRpm(int32_t error_mdeg,
                                    int32_t predicted_error_mdeg,
                                    int32_t kp,
                                    int32_t minimum_rpm,
                                    int32_t maximum_rpm,
                                    int32_t wheel_headroom_rpm,
                                    int32_t tolerance_mdeg)
{
    static const int32_t kGainScale = 1000000;
    int32_t correction = ClampInt64ToInt32(
        (static_cast<int64_t>(predicted_error_mdeg) * kp) /
        kGainScale);

    const bool same_direction =
        ((error_mdeg > 0) && (predicted_error_mdeg > 0)) ||
        ((error_mdeg < 0) && (predicted_error_mdeg < 0));
    if (same_direction &&
        (AbsInt32(error_mdeg) > tolerance_mdeg) &&
        (AbsInt32(predicted_error_mdeg) > tolerance_mdeg) &&
        (AbsInt32(correction) < minimum_rpm)) {
        correction = (predicted_error_mdeg > 0)
            ? minimum_rpm
            : -minimum_rpm;
    }

    int32_t limit = maximum_rpm;
    if (limit > wheel_headroom_rpm) {
        limit = wheel_headroom_rpm;
    }
    if (limit < 0) {
        limit = 0;
    }
    if (correction > limit) {
        correction = limit;
    } else if (correction < -limit) {
        correction = -limit;
    }
    return correction;
}

inline WheelCommand MakeWheelCommand(int32_t base_rpm,
                                     int32_t correction_rpm)
{
    WheelCommand command = {
        base_rpm - correction_rpm,
        base_rpm + correction_rpm
    };
    return command;
}

/* Automatic road corners keep the outer-wheel command produced by the
 * existing base+correction law, but give the inner wheel an independent
 * reverse endpoint. The interpolation is continuous at zero correction and
 * reaches -inner_reverse_max_rpm at maximum correction. Calculations are
 * performed in the direction of travel so signed reverse motion is handled
 * without changing the yaw-controller convention. */
inline WheelCommand MakeAsymmetricWheelCommand(
    int32_t base_rpm,
    int32_t correction_rpm,
    int32_t maximum_correction_rpm,
    int32_t outer_max_rpm,
    int32_t inner_reverse_max_rpm,
    int32_t wheel_limit_rpm)
{
    const int32_t travel_direction = (base_rpm < 0) ? -1 : 1;
    const int32_t base_magnitude = AbsInt32(base_rpm);
    int32_t correction_magnitude = AbsInt32(correction_rpm);
    if (correction_magnitude > maximum_correction_rpm) {
        correction_magnitude = maximum_correction_rpm;
    }

    int32_t wheel_limit = wheel_limit_rpm;
    if (wheel_limit < 0) {
        wheel_limit = 0;
    }
    int32_t outer_limit = outer_max_rpm;
    if (outer_limit < base_magnitude) {
        outer_limit = base_magnitude;
    }
    if (outer_limit > wheel_limit) {
        outer_limit = wheel_limit;
    }
    int32_t reverse_limit = inner_reverse_max_rpm;
    if (reverse_limit < 0) {
        reverse_limit = 0;
    }
    if (reverse_limit > wheel_limit) {
        reverse_limit = wheel_limit;
    }

    if ((correction_magnitude == 0) ||
        (maximum_correction_rpm <= 0) ||
        (base_magnitude > wheel_limit)) {
        const int32_t limited_base =
            (base_magnitude > wheel_limit) ? wheel_limit : base_magnitude;
        const int32_t command = limited_base * travel_direction;
        const WheelCommand straight = { command, command };
        return straight;
    }

    int32_t outer_motion_rpm = ClampInt64ToInt32(
        static_cast<int64_t>(base_magnitude) + correction_magnitude);
    if (outer_motion_rpm > outer_limit) {
        outer_motion_rpm = outer_limit;
    }
    const int64_t inner_span =
        static_cast<int64_t>(base_magnitude) + reverse_limit;
    const int32_t inner_motion_rpm = ClampInt64ToInt32(
        static_cast<int64_t>(base_magnitude) -
        (inner_span * correction_magnitude) /
            maximum_correction_rpm);

    /* correction*travel_direction > 0 means the right wheel is the outer
     * wheel in the direction of travel. Compare signs instead of multiplying
     * to avoid signed overflow. */
    const bool right_is_outer =
        ((correction_rpm > 0) && (travel_direction > 0)) ||
        ((correction_rpm < 0) && (travel_direction < 0));
    const int32_t outer_command = outer_motion_rpm * travel_direction;
    const int32_t inner_command = inner_motion_rpm * travel_direction;
    const WheelCommand command = right_is_outer
        ? WheelCommand{ inner_command, outer_command }
        : WheelCommand{ outer_command, inner_command };
    return command;
}

inline bool IsAtTarget(int32_t error_mdeg,
                       int32_t gyro_rate_mdps,
                       int32_t tolerance_mdeg,
                       int32_t settle_rate_mdps)
{
    return (AbsInt32(error_mdeg) <= tolerance_mdeg) &&
           (AbsInt32(gyro_rate_mdps) <= settle_rate_mdps);
}

inline bool IsRollingDistanceReached(int32_t remaining_um,
                                     int32_t travel_direction)
{
    return (travel_direction != 0) &&
           ((static_cast<int64_t>(remaining_um) * travel_direction) <= 0LL);
}

} /* namespace heading_arc */
} /* namespace app */

#endif /* APP_HEADING_ARC_MATH_H_ */
