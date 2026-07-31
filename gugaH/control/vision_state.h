#ifndef GUGAH_CONTROL_VISION_STATE_H_
#define GUGAH_CONTROL_VISION_STATE_H_

#include <stdint.h>

#include "control/control_types.h"

namespace gugah {

struct VisionState {
    drivers::BallVisionFrame latest_frame;
    drivers::BallVisionFrame ball_frame;
    bool latest_valid;
    bool ball_valid;
};

void VisionState_Init(VisionState *state);
void VisionState_Accept(VisionState *state,
                        const drivers::BallVisionFrame *frame,
                        uint16_t minimum_confidence);
VisionFeedback VisionState_Get(const VisionState *state,
                               uint32_t now_ms,
                               uint16_t minimum_confidence);

} /* namespace gugah */

#endif /* GUGAH_CONTROL_VISION_STATE_H_ */
