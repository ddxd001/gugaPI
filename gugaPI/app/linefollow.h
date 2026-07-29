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
 * correction ceiling, and correction slew are runtime tuning parameters. */
enum LFControllerConstant {
    LF_REFERENCE_RPM = 40,
    LF_MAX_STEERING_PERMILLE = 400,
    LF_ERROR_DEADBAND_MPOS = 50,
    LF_DERIVATIVE_FILTER_TAU_MS = 40,
    LF_DEFAULT_CORRECTION_SLEW_PERMILLE_PER_SECOND = 25000,
    /* A complete grayscale position frame is about 7 ms. Geometry-only
     * invalid states therefore receive about 42 ms to recover as the line
     * crosses a gap between adjacent sensors. Hardware/stale/anomaly faults
     * still stop immediately in LF_Update(). */
    LF_INVALID_TRACK_STOP_FRAMES = 6
};

enum LFMode {
    LF_IDLE = 0,
    LF_CAL,      /* unified grayscale sweep calibration */
    LF_FOLLOW    /* following the line */
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
    uint32_t lost_since_ms;
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
    /* Tunable parameters are loaded from ConfigStore and remain runtime-settable.
     * max_correction_rpm is the ceiling at LF_REFERENCE_RPM; the controller
     * scales it with the requested speed before applying the steering-ratio
     * safety limit. Lost-line timing is retained for configuration/API
     * compatibility. Brief geometry gaps are tolerated for six complete
     * grayscale frames (about 42 ms). */
    int32_t kp;
    int32_t kd;
    int32_t max_correction_rpm;
    uint32_t correction_slew_permille_per_second;
    uint32_t lost_hold_ms;
    uint32_t lost_timeout_ms;
    drivers::DriverStatus last_status;
};

/* 8-channel grayscale line follower. Calibrate (sweep sensor over line +
 * floor for 2 s), then LF_Start drives following the line for a duration.
 * Fault/stale/sensor anomaly stops immediately. Lost, multiple, or wide line
 * geometry stops after six consecutive complete frames, without search. */
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
 * anomaly-free tracking position. Callable any time (does not
 * require LF_FOLLOW); used by the action interpreter's LINE_DETECTED /
 * LINE_LOST conditions so action completion matches the stop policy. */
bool LF_IsLineDetected(void);

/* Runtime parameter setters. ConfigStore keeps the existing public surface;
 * lost timing setters are compatibility-only while stop-on-invalid is active. */
void LF_SetKp(int32_t kp);
void LF_SetKd(int32_t kd);
void LF_SetMaxCorrection(int32_t max_correction_rpm);
void LF_SetCorrectionSlew(uint32_t permille_per_second);
void LF_SetLostHold(uint32_t hold_ms);
void LF_SetLostTimeout(uint32_t timeout_ms);

} /* namespace app */

#endif /* APP_LINEFOLLOW_H_ */
