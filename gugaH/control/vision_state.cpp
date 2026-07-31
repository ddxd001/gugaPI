#include "control/vision_state.h"

namespace gugah {

void VisionState_Init(VisionState *state)
{
    if (state != 0) {
        *state = {};
    }
}

void VisionState_Accept(VisionState *state,
                        const drivers::BallVisionFrame *frame,
                        uint16_t minimum_confidence)
{
    if ((state == 0) || (frame == 0)) {
        return;
    }
    state->latest_frame = *frame;
    state->latest_valid = true;
    const uint8_t required =
        drivers::BALL_VISION_FLAG_BALL_FOUND |
        drivers::BALL_VISION_FLAG_CALIBRATION_VALID |
        drivers::BALL_VISION_FLAG_CAMERA_OK;
    if (((frame->flags & required) == required) &&
        (frame->confidence >= minimum_confidence)) {
        state->ball_frame = *frame;
        state->ball_valid = true;
    }
}

VisionFeedback VisionState_Get(const VisionState *state,
                               uint32_t now_ms,
                               uint16_t minimum_confidence)
{
    VisionFeedback result = {};
    if ((state == 0) || !state->latest_valid) {
        return result;
    }
    result.frame = state->ball_frame;
    result.frame_age_ms =
        now_ms - state->latest_frame.received_ms;
    result.ball_age_ms = state->ball_valid
        ? (now_ms - state->ball_frame.received_ms +
           state->ball_frame.source_delay_ms)
        : 0xFFFFFFFFUL;
    result.communication_online = result.frame_age_ms <= 120U;
    result.ball_usable = result.communication_online &&
        state->ball_valid && (result.ball_age_ms <= 120U) &&
        (state->ball_frame.confidence >= minimum_confidence);
    return result;
}

} /* namespace gugah */
