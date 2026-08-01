#ifndef GUGAH_CONTROL_LINE_CONTROL_H_
#define GUGAH_CONTROL_LINE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "config/h_config.h"
#include "drivers/grayscale/grayscale_processing.h"

namespace gugah {

struct LineControlState {
    bool initialized;
    bool line_valid;
    bool failed;
    int16_t left_rpm;
    int16_t right_rpm;
    int32_t last_position;
    int32_t last_correction_rpm;
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

} /* namespace gugah */

#endif /* GUGAH_CONTROL_LINE_CONTROL_H_ */
