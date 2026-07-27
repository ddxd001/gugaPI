#ifndef APP_HEADING_LOCK_MATH_H_
#define APP_HEADING_LOCK_MATH_H_

#include <stdint.h>

namespace app {
namespace heading_lock {

inline int32_t Abs(int32_t value)
{
    if (value == INT32_MIN) {
        return INT32_MAX;
    }
    return (value < 0) ? -value : value;
}

/* Shortest signed angle from current to target in milli-degrees. The input
 * yaw produced by AppImu is already bounded, so int32 subtraction is safe. */
inline int32_t ShortestAngleDiff(int32_t target_mdeg, int32_t current_mdeg)
{
    int32_t difference = target_mdeg - current_mdeg;
    while (difference > 180000) {
        difference -= 360000;
    }
    while (difference < -180000) {
        difference += 360000;
    }
    return difference;
}

inline bool ShouldWake(int32_t error_mdeg, int32_t wake_mdeg)
{
    return Abs(error_mdeg) >= wake_mdeg;
}

inline bool ReadyToSettle(int32_t error_mdeg,
                          int32_t gyro_rate_mdps,
                          int32_t settle_mdeg,
                          int32_t settle_rate_mdps)
{
    return (Abs(error_mdeg) <= settle_mdeg) &&
           (Abs(gyro_rate_mdps) <= settle_rate_mdps);
}

inline bool DurationReached(uint32_t now_ms,
                            uint32_t start_ms,
                            uint32_t duration_ms)
{
    return static_cast<uint32_t>(now_ms - start_ms) >= duration_ms;
}

/* Returns the signed yaw correction before the board-specific yaw sign is
 * applied. A minimum RPM is enforced only while the command is driving in
 * the same direction as the remaining error; an opposing D term remains
 * free to brake without being amplified into a minimum-speed reversal. */
inline int32_t ComputeCorrectionRpm(int32_t error_mdeg,
                                    int32_t gyro_rate_mdps,
                                    int32_t kp,
                                    int32_t kd,
                                    int32_t minimum_rpm,
                                    int32_t maximum_rpm)
{
    static const int64_t kScale = 1000000LL;
    int64_t correction =
        (static_cast<int64_t>(error_mdeg) * kp) / kScale;
    correction -=
        (static_cast<int64_t>(gyro_rate_mdps) * kd) / kScale;

    if (correction > maximum_rpm) {
        correction = maximum_rpm;
    } else if (correction < -maximum_rpm) {
        correction = -maximum_rpm;
    }

    const bool same_direction =
        ((correction > 0) && (error_mdeg > 0)) ||
        ((correction < 0) && (error_mdeg < 0));
    if (same_direction &&
        (Abs(static_cast<int32_t>(correction)) < minimum_rpm)) {
        correction = (correction > 0) ? minimum_rpm : -minimum_rpm;
    }
    return static_cast<int32_t>(correction);
}

} /* namespace heading_lock */
} /* namespace app */

#endif /* APP_HEADING_LOCK_MATH_H_ */
