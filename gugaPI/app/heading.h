#ifndef APP_HEADING_H_
#define APP_HEADING_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace app {

enum HeadingMode {
    HEADING_IDLE = 0,
    HEADING_HOLD,
    HEADING_TURN,
    HEADING_DISTANCE
};

struct HeadingState {
    HeadingMode mode;
    int32_t target_yaw_mdeg;   /* HOLD: locked yaw; TURN: wrapped target */
    int32_t base_rpm;          /* HOLD forward base; TURN = 0 */
    int32_t correction_rpm;   /* last differential correction (HOLD) / turn speed (TURN) */
    int32_t error_mdeg;        /* last shortest-angle error */
    bool at_target;            /* TURN within tolerance */
    uint32_t at_target_since_ms;
    uint32_t turn_start_ms;
    int32_t target_distance_mm;
    int32_t traveled_distance_mm;
    int32_t remaining_distance_mm;
    int32_t distance_max_rpm;
    int32_t start_left_encoder_count;
    int32_t start_right_encoder_count;
    uint32_t distance_start_ms;
    uint32_t distance_timeout_ms;
    uint32_t last_feedback_sequence;
    uint32_t last_run_ms;      /* scheduler-cadence / freshness watchdog */
    drivers::DriverStatus last_status;
};

/* IMU (ICM-45686 gyro yaw) heading closed-loop. Hold locks the current yaw
 * and drives straight with differential correction; Turn rotates to a
 * relative +/-N deg target (shortest path, |delta| <= 180) with segmented
 * deceleration. Distance drives a signed millimeter target using both wheel
 * encoders while the same yaw loop keeps the initial heading. All safety
 * failures stop the chassis and latch a fault. */
void Heading_Init(void);
drivers::DriverStatus Heading_HoldStart(int32_t base_rpm);
drivers::DriverStatus Heading_TurnStart(int32_t delta_deg);
/* distance_mm: positive forward, negative reverse. max_rpm must be positive.
 * timeout_ms=0 derives a bounded timeout from distance and speed. */
drivers::DriverStatus Heading_DistanceStart(int32_t distance_mm,
                                            int32_t max_rpm,
                                            uint32_t timeout_ms);
drivers::DriverStatus Heading_Stop(void);
void Heading_Update(void);
const HeadingState *Heading_GetState(void);

} /* namespace app */

#endif /* APP_HEADING_H_ */
