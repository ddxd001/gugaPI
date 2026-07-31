#ifndef APP_LINEFOLLOW_H_
#define APP_LINEFOLLOW_H_

#include <stdbool.h>
#include <stdint.h>

#include "app/grayscale_road.h"
#include "drivers/common/driver_status.h"
#include "drivers/grayscale/grayscale_processing.h"

namespace app {

/* Steering is calculated at this reference speed, then scaled with the
 * requested forward speed so the same line error produces approximately the
 * same chassis curvature. Geometry and filter constants remain fixed; gain,
 * correction ceiling, final steering ratio, and correction slew are runtime
 * tuning parameters. */
enum LFControllerConstant {
    LF_REFERENCE_RPM = 40,
    LF_DEFAULT_MAX_STEERING_PERMILLE = 400,
    LF_MIN_MAX_STEERING_PERMILLE = 100,
    LF_MAX_MAX_STEERING_PERMILLE = 1000,
    LF_DEFAULT_ERROR_DEADBAND_MPOS = 20,
    LF_MAX_ERROR_DEADBAND_MPOS = 500,
    LF_DERIVATIVE_FILTER_TAU_MS = 40,
    LF_DEFAULT_CORRECTION_SLEW_PERMILLE_PER_SECOND = 25000,
    /* Retain the historical diagnostic constant for source compatibility.
     * ADC8 geometry loss now holds the last wheel command until a valid line
     * is reacquired; hardware/stale/anomaly failures still stop immediately. */
    LF_INVALID_TRACK_STOP_FRAMES = 6,
    LF_RECOVERY_CONFIRM_FRAMES = 3,
    LF_STRONG_CONFIDENCE_MIN = 300
};

enum LFMode {
    LF_IDLE = 0,
    LF_CAL,      /* unified grayscale sweep calibration */
    LF_FOLLOW    /* following the line */
};

enum LFRecoveryMode : uint8_t {
    LF_RECOVERY_NONE = 0U,
    LF_RECOVERY_HOLD,
    LF_RECOVERY_DECEL
};

struct LFState {
    LFMode mode;
    bool calibrated;
    int32_t error_mpos;        /* -3500..+3500 (0 = centered) */
    int32_t correction_rpm;    /* last differential correction */
    bool lost;                 /* line not detected this update */
    int32_t base_rpm;
    uint32_t follow_start_ms;
    uint32_t follow_duration_ms;
    uint32_t last_sequence;
    uint32_t last_frame_ms;
    uint32_t processed_frame_count;
    int32_t last_error_mpos;
    int32_t derivative_mpos_per_s;
    GrayscaleRoadType road_type;
    bool position_valid;
    uint8_t selected_mask;
    uint16_t position_confidence;
    drivers::GrayscalePositionSource position_source;
    drivers::GrayscaleTrackState track_state;
    uint8_t weak_tracking_frames;
    uint8_t invalid_frames;
    LFRecoveryMode recovery_mode;
    uint32_t recovery_start_ms;
    uint32_t recovery_elapsed_ms;
    uint8_t recovery_confirm_frames;
    int32_t recovery_base_rpm;
    int32_t recovery_correction_rpm;
    int16_t last_strong_position_mpos;
    /* Tunable parameters are loaded from ConfigStore and remain runtime-settable.
     * max_correction_rpm is the ceiling at LF_REFERENCE_RPM; the controller
     * scales it with the requested speed before applying the configurable
     * steering-ratio safety limit. */
    int32_t kp;
    int32_t kd;
    int32_t max_correction_rpm;
    uint16_t max_steering_permille;
    uint16_t deadband_mpos;
    uint32_t correction_slew_permille_per_second;
    drivers::DriverStatus last_status;
};

/* 8-channel grayscale line follower. Calibrate (sweep sensor over line +
 * floor for 2 s), then LF_Start drives following the line for a duration.
 * Fault/stale/sensor anomaly stops immediately. A weak but valid ADC8 position
 * remains in closed-loop control. Invalid ADC8 geometry holds the exact last
 * wheel command until three consecutive valid positions reacquire the line. */
void LF_Init(void);
void LF_ReloadConfig(void);
drivers::DriverStatus LF_CalibrateStart(void);
drivers::DriverStatus LF_Start(int32_t base_rpm, uint32_t duration_ms);
drivers::DriverStatus LF_Stop(void);
/* Retarget an already-running follower without issuing a stop command. This is
 * used when two road-navigation actions hand motion ownership directly from
 * one node to the next. */
drivers::DriverStatus LF_ContinueForMotionHandoff(
    int32_t base_rpm,
    uint32_t duration_ms);
/* Transfer motion ownership without writing a zero-speed command. The caller
 * must immediately start another closed-loop motion and stop on failure. */
drivers::DriverStatus LF_ReleaseForMotionHandoff(void);
void LF_Update(void);
const LFState *LF_GetState(void);

/* True if the grayscale currently provides a fresh, explicitly valid,
 * anomaly-free tracking position, or ADC8 follow is actively holding its last
 * command while searching. Callable any time; used by the action interpreter
 * so a temporary ADC8 geometry loss does not abort the follow action. */
bool LF_IsLineDetected(void);

/* Runtime parameter setters. */
void LF_SetKp(int32_t kp);
void LF_SetKd(int32_t kd);
void LF_SetMaxCorrection(int32_t max_correction_rpm);
void LF_SetMaxSteeringRatio(uint32_t permille);
void LF_SetDeadband(uint32_t deadband_mpos);
void LF_SetCorrectionSlew(uint32_t permille_per_second);

} /* namespace app */

#endif /* APP_LINEFOLLOW_H_ */
