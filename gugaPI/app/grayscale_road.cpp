#include "app/grayscale_road.h"

namespace app {

void GrayscaleRoad_Init(GrayscaleRoadClassifierState *state)
{
    if (state == 0) {
        return;
    }
    state->road = GRAYSCALE_ROAD_UNKNOWN;
    state->candidate = GRAYSCALE_ROAD_UNKNOWN;
    state->candidate_count = 0U;
    state->last_sequence = 0U;
}

GrayscaleRoadType GrayscaleRoad_Classify(
    uint8_t active_mask,
    const GrayscaleRoadClassifierConfig *config)
{
    if ((config == 0) || (config->left_mask == 0U) ||
        (config->center_mask == 0U) || (config->right_mask == 0U)) {
        return GRAYSCALE_ROAD_UNKNOWN;
    }
    if (active_mask == 0U) {
        return GRAYSCALE_ROAD_LOST;
    }

    const bool left =
        (active_mask & config->left_mask) == config->left_mask;
    const bool center = (active_mask & config->center_mask) != 0U;
    const bool right =
        (active_mask & config->right_mask) == config->right_mask;

    if (left && center && right) {
        return GRAYSCALE_ROAD_CROSS;
    }
    if (left && right) {
        return GRAYSCALE_ROAD_T;
    }
    if (left && center) {
        return GRAYSCALE_ROAD_LEFT_BRANCH;
    }
    if (right && center) {
        return GRAYSCALE_ROAD_RIGHT_BRANCH;
    }
    if (center) {
        return GRAYSCALE_ROAD_STRAIGHT;
    }
    if (left) {
        return GRAYSCALE_ROAD_LEFT_BRANCH;
    }
    if (right) {
        return GRAYSCALE_ROAD_RIGHT_BRANCH;
    }
    return GRAYSCALE_ROAD_UNKNOWN;
}

GrayscaleRoadType GrayscaleRoad_Update(
    GrayscaleRoadClassifierState *state,
    const GrayscaleRoadClassifierConfig *config,
    uint8_t active_mask,
    uint32_t sequence)
{
    if ((state == 0) || (config == 0) || (config->confirm_frames == 0U)) {
        return GRAYSCALE_ROAD_UNKNOWN;
    }
    if (sequence == state->last_sequence) {
        return state->road;
    }
    state->last_sequence = sequence;

    const GrayscaleRoadType observed =
        GrayscaleRoad_Classify(active_mask, config);
    if (observed == state->road) {
        state->candidate = observed;
        state->candidate_count = 0U;
        return state->road;
    }

    if (observed != state->candidate) {
        state->candidate = observed;
        state->candidate_count = 1U;
    } else if (state->candidate_count < config->confirm_frames) {
        state->candidate_count++;
    }

    if (state->candidate_count >= config->confirm_frames) {
        state->road = state->candidate;
        state->candidate_count = 0U;
    }
    return state->road;
}

const char *GrayscaleRoad_TypeText(GrayscaleRoadType road)
{
    switch (road) {
    case GRAYSCALE_ROAD_LOST:
        return "lost";
    case GRAYSCALE_ROAD_STRAIGHT:
        return "straight";
    case GRAYSCALE_ROAD_LEFT_BRANCH:
        return "left_branch";
    case GRAYSCALE_ROAD_RIGHT_BRANCH:
        return "right_branch";
    case GRAYSCALE_ROAD_T:
        return "t";
    case GRAYSCALE_ROAD_CROSS:
        return "cross";
    case GRAYSCALE_ROAD_UNKNOWN:
    default:
        return "unknown";
    }
}

} /* namespace app */
