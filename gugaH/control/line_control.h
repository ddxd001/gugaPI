#ifndef GUGAH_CONTROL_LINE_CONTROL_H_
#define GUGAH_CONTROL_LINE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "config/h_config.h"
#include "drivers/grayscale/grayscale_processing.h"

namespace gugah {

struct LineControlTuning {
    int32_t kp_milli;
    int32_t kd_milli;
    int16_t max_correction_rpm;
    uint16_t correction_slew_rpm_s;
    /* 1000 follows the raw derivative; smaller values reject frame jitter. */
    uint16_t derivative_filter_permille;
};

struct LineControlState {
    bool initialized;
    bool line_valid;
    bool failed;
    int16_t left_rpm;
    int16_t right_rpm;
    int32_t last_position;
    int32_t raw_derivative_per_s;
    int32_t filtered_derivative_per_s;
    int32_t last_correction_rpm;
    LineControlTuning applied_tuning;
    uint32_t last_update_ms;
    uint32_t invalid_since_ms;
};

void LineControl_Init(LineControlState *state);
bool LineControl_IsTrackUsable(
    const drivers::GrayscaleProcessedData *line);
bool LineControl_SearchRight(LineControlState *state,
                             int16_t forward_rpm,
                             uint32_t now_ms);
bool LineControl_Update(LineControlState *state,
                        const drivers::GrayscaleProcessedData *line,
                        int16_t base_rpm,
                        uint32_t now_ms,
                        const HConfig *config);
bool LineControl_UpdateTuned(LineControlState *state,
                             const drivers::GrayscaleProcessedData *line,
                             int16_t base_rpm,
                             uint32_t now_ms,
                             const HConfig *config,
                             const LineControlTuning *tuning);

} /* namespace gugah */

#endif /* GUGAH_CONTROL_LINE_CONTROL_H_ */
