#ifndef APP_LINEFOLLOW_H_
#define APP_LINEFOLLOW_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace app {

enum LFMode {
    LF_IDLE = 0,
    LF_CAL,      /* calibrating: recording per-channel min/max */
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
    uint32_t cal_start_ms;
    uint32_t lost_since_ms;
    /* per-channel calibration (RAM; recalibrate each session) */
    uint16_t cal_min[8];
    uint16_t cal_max[8];
    uint16_t threshold[8];
    /* tunable params (RAM, shell-set; persist to ConfigStore later) */
    int32_t kp;
    int32_t max_correction_rpm;
    uint32_t lost_timeout_ms;
    drivers::DriverStatus last_status;
};

/* 8-channel grayscale line follower. Calibrate (sweep sensor over line +
 * floor for 2 s), then LF_Start drives following the line for a duration.
 * Lost line -> retain last steering for lost_timeout_ms then stop. Safety:
 * fault / grayscale invalid / uncalibrated / lost-timeout -> stop. */
void LF_Init(void);
drivers::DriverStatus LF_CalibrateStart(void);
drivers::DriverStatus LF_Start(int32_t base_rpm, uint32_t duration_ms);
drivers::DriverStatus LF_Stop(void);
void LF_Update(void);
const LFState *LF_GetState(void);

/* Runtime param setters (RAM). */
void LF_SetKp(int32_t kp);
void LF_SetMaxCorrection(int32_t max_correction_rpm);
void LF_SetLostTimeout(uint32_t timeout_ms);

} /* namespace app */

#endif /* APP_LINEFOLLOW_H_ */
