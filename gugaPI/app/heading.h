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
    HEADING_DISTANCE,
    HEADING_LOCK,
    HEADING_ARC_TURN
};

enum DistanceProfilePhase {
    DISTANCE_PHASE_IDLE = 0,
    DISTANCE_PHASE_LEGACY,
    DISTANCE_PHASE_ACCEL,
    DISTANCE_PHASE_CRUISE,
    DISTANCE_PHASE_BRAKE,
    DISTANCE_PHASE_CREEP,
    DISTANCE_PHASE_SETTLE
};

enum HeadingTurnPhase {
    HEADING_TURN_PHASE_IDLE = 0,
    HEADING_TURN_PHASE_DRIVE,
    HEADING_TURN_PHASE_BRAKE,
    HEADING_TURN_PHASE_SETTLE
};

enum HeadingLockPhase {
    HEADING_LOCK_PHASE_IDLE = 0,
    HEADING_LOCK_PHASE_LOCKED,
    HEADING_LOCK_PHASE_RECOVER,
    HEADING_LOCK_PHASE_SETTLE,
    HEADING_LOCK_PHASE_FAILED
};

struct HeadingState {
    HeadingMode mode;
    int32_t target_yaw_mdeg;   /* HOLD: locked yaw; TURN: wrapped target */
    int32_t base_rpm;          /* HOLD/DISTANCE/ARC forward base; TURN = 0 */
    int32_t correction_rpm;   /* last differential correction / turn speed */
    int32_t arc_left_command_rpm;  /* last asymmetric ARC wheel target */
    int32_t arc_right_command_rpm;
    int32_t error_mdeg;        /* last shortest-angle error */
    bool at_target;            /* TURN within tolerance */
    uint32_t at_target_since_ms;
    uint32_t turn_start_ms;
    HeadingTurnPhase turn_phase;
    int32_t turn_rate_mdps;        /* latest gyro Z rate used by TURN */
    int32_t turn_brake_angle_mdeg; /* predicted coast angle + margin */
    HeadingLockPhase lock_phase;
    int32_t lock_rate_mdps;
    uint32_t lock_recover_start_ms;
    uint32_t lock_settle_start_ms;
    uint32_t lock_recover_elapsed_ms;
    drivers::DriverStatus lock_result;
    int32_t target_distance_mm;
    int32_t traveled_distance_mm;
    int32_t remaining_distance_mm;
    int32_t distance_max_rpm;
    int32_t start_left_encoder_count;
    int32_t start_right_encoder_count;
    int32_t left_target_delta_counts;
    int32_t right_target_delta_counts;
    int32_t profile_command_rpm;
    int32_t brake_distance_mm;
    DistanceProfilePhase distance_phase;
    uint8_t distance_settle_cycles;
    bool distance_rolling_handoff;
    uint32_t distance_start_ms;
    uint32_t distance_timeout_ms;
    uint32_t profile_last_update_ms;
    uint32_t profile_ramp_remainder;
    int8_t profile_ramp_direction;
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
/* Captures the current relative ICM-45686 yaw while stationary. The lock
 * remains armed until Heading_Stop(), waking the wheels only after an
 * external disturbance exceeds the configured deadband. */
drivers::DriverStatus Heading_LockStart(void);
drivers::DriverStatus Heading_TurnStart(int32_t delta_deg);
/* Moving relative turn for automatic road handling. The signed base RPM is
 * preserved throughout the arc and subsequent target-heading reacquisition;
 * only a fault, timeout, cancellation or explicit stop commands zero speed. */
drivers::DriverStatus Heading_ArcTurnStart(int32_t delta_deg,
                                           int32_t base_rpm);
/* distance_mm: positive forward, negative reverse. max_rpm must be positive.
 * timeout_ms=0 derives a bounded timeout from distance and speed. */
drivers::DriverStatus Heading_DistanceStart(int32_t distance_mm,
                                            int32_t max_rpm,
                                            uint32_t timeout_ms);
/* Distance start for a moving controller handoff. initial_rpm seeds the
 * speed profile so the first distance update does not restart from zero. */
drivers::DriverStatus Heading_DistanceStartWithInitialRpm(
    int32_t distance_mm,
    int32_t max_rpm,
    uint32_t timeout_ms,
    int32_t initial_rpm);
/* Road-corner variant: hold the supplied moving speed and initial yaw without
 * endpoint braking. Reaching the encoder target releases Heading while the
 * last wheel target remains active for the following arc controller. */
drivers::DriverStatus Heading_DistanceStartForRollingHandoff(
    int32_t distance_mm,
    int32_t max_rpm,
    uint32_t timeout_ms,
    int32_t initial_rpm);
/* Release a completed arc to another moving controller without sending a
 * chassis stop command. Valid only after HEADING_ARC_TURN reaches target. */
drivers::DriverStatus Heading_ReleaseForMotionHandoff(void);
/* Release a running HEADING_HOLD search directly to LF_FOLLOW. The caller
 * must start LF first so one closed-loop owner is always active. */
drivers::DriverStatus Heading_HoldReleaseForMotionHandoff(void);
drivers::DriverStatus Heading_Stop(void);
void Heading_Update(void);
const HeadingState *Heading_GetState(void);

} /* namespace app */

#endif /* APP_HEADING_H_ */
