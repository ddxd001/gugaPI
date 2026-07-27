#include "app/grayscale_road.h"

namespace app {
namespace {

uint8_t ConfigValueOrDefault(uint8_t value, uint8_t fallback)
{
    return (value == 0U) ? fallback : value;
}

uint8_t CountBits(uint8_t value)
{
    uint8_t count = 0U;
    while (value != 0U) {
        count = static_cast<uint8_t>(count + (value & 1U));
        value = static_cast<uint8_t>(value >> 1U);
    }
    return count;
}

uint8_t ObservePaths(uint8_t active_mask,
                     const GrayscaleRoadClassifierConfig *config)
{
    uint8_t paths = 0U;
    if ((active_mask & config->left_mask) == config->left_mask) {
        paths |= GRAYSCALE_ROAD_PATH_LEFT;
    }
    if ((active_mask & config->center_mask) != 0U) {
        paths |= GRAYSCALE_ROAD_PATH_FORWARD;
    }
    if ((active_mask & config->right_mask) == config->right_mask) {
        paths |= GRAYSCALE_ROAD_PATH_RIGHT;
    }
    return paths;
}

bool HasSidePath(uint8_t paths)
{
    return (paths & (GRAYSCALE_ROAD_PATH_LEFT |
                     GRAYSCALE_ROAD_PATH_RIGHT)) != 0U;
}

GrayscaleRoadType TypeFromPaths(uint8_t paths)
{
    const bool left = (paths & GRAYSCALE_ROAD_PATH_LEFT) != 0U;
    const bool forward = (paths & GRAYSCALE_ROAD_PATH_FORWARD) != 0U;
    const bool right = (paths & GRAYSCALE_ROAD_PATH_RIGHT) != 0U;

    if (left && forward && right) {
        return GRAYSCALE_ROAD_CROSS;
    }
    if (left && right) {
        return GRAYSCALE_ROAD_T;
    }
    if (left && forward) {
        return GRAYSCALE_ROAD_LEFT_BRANCH;
    }
    if (right && forward) {
        return GRAYSCALE_ROAD_RIGHT_BRANCH;
    }
    if (left) {
        return GRAYSCALE_ROAD_LEFT_CORNER;
    }
    if (right) {
        return GRAYSCALE_ROAD_RIGHT_CORNER;
    }
    if (forward) {
        return GRAYSCALE_ROAD_STRAIGHT;
    }
    return GRAYSCALE_ROAD_LOST;
}

void ResetObservation(GrayscaleRoadClassifierState *state)
{
    state->candidate = GRAYSCALE_ROAD_UNKNOWN;
    state->candidate_count = 0U;
    state->observed_paths = 0U;
    state->observe_frames = 0U;
    state->left_evidence_frames = 0U;
    state->forward_evidence_frames = 0U;
    state->right_evidence_frames = 0U;
    state->center_absent_frames = 0U;
    state->exit_frames = 0U;
    state->entry_mask = 0U;
    state->peak_mask = 0U;
    state->peak_active_count = 0U;
    state->first_frame_sequence = 0U;
}

void BeginObservation(GrayscaleRoadClassifierState *state,
                      uint8_t active_mask,
                      uint32_t sequence)
{
    ResetObservation(state);
    state->phase = GRAYSCALE_ROAD_PHASE_OBSERVING;
    state->entry_mask = active_mask;
    state->peak_mask = active_mask;
    state->peak_active_count = CountBits(active_mask);
    state->first_frame_sequence = sequence;
    state->observed_paths = 0U;
}

void UpdatePathEvidence(GrayscaleRoadClassifierState *state,
                        uint8_t paths,
                        uint8_t confirm_frames)
{
    if ((paths & GRAYSCALE_ROAD_PATH_LEFT) != 0U) {
        if (state->left_evidence_frames < UINT8_MAX) {
            state->left_evidence_frames++;
        }
    }
    if ((paths & GRAYSCALE_ROAD_PATH_FORWARD) != 0U) {
        if (state->forward_evidence_frames < UINT8_MAX) {
            state->forward_evidence_frames++;
        }
    }
    if ((paths & GRAYSCALE_ROAD_PATH_RIGHT) != 0U) {
        if (state->right_evidence_frames < UINT8_MAX) {
            state->right_evidence_frames++;
        }
    }

    if (state->left_evidence_frames >= confirm_frames) {
        state->observed_paths |= GRAYSCALE_ROAD_PATH_LEFT;
    }
    if (state->forward_evidence_frames >= confirm_frames) {
        state->observed_paths |= GRAYSCALE_ROAD_PATH_FORWARD;
    }
    if (state->right_evidence_frames >= confirm_frames) {
        state->observed_paths |= GRAYSCALE_ROAD_PATH_RIGHT;
    }
}

uint16_t CalculateConfidence(const GrayscaleRoadClassifierState *state,
                             GrayscaleRoadType type)
{
    uint16_t confidence = 650U;
    const uint16_t frame_bonus =
        static_cast<uint16_t>(state->observe_frames) * 20U;
    confidence = static_cast<uint16_t>(confidence +
        ((frame_bonus > 250U) ? 250U : frame_bonus));
    if ((type == GRAYSCALE_ROAD_LEFT_CORNER) ||
        (type == GRAYSCALE_ROAD_RIGHT_CORNER) ||
        (type == GRAYSCALE_ROAD_T)) {
        confidence = static_cast<uint16_t>(confidence + 100U);
    }
    return (confidence > 1000U) ? 1000U : confidence;
}

void PublishEvent(GrayscaleRoadClassifierState *state,
                  GrayscaleRoadType type,
                  uint8_t exit_mask,
                  uint32_t sequence)
{
    state->event_sequence++;
    if (state->event_sequence == 0U) {
        state->event_sequence = 1U;
    }
    state->road = type;
    state->phase = GRAYSCALE_ROAD_PHASE_LATCHED;
    state->rearm_count = 0U;
    state->last_event.valid = true;
    state->last_event.sequence = state->event_sequence;
    state->last_event.type = type;
    state->last_event.observed_paths = state->observed_paths;
    state->last_event.confidence = CalculateConfidence(state, type);
    state->last_event.entry_mask = state->entry_mask;
    state->last_event.peak_mask = state->peak_mask;
    state->last_event.exit_mask = exit_mask;
    state->last_event.first_frame_sequence = state->first_frame_sequence;
    state->last_event.last_frame_sequence = sequence;
}

GrayscaleRoadType ResolveObservedType(
    const GrayscaleRoadClassifierState *state,
    bool forward_now)
{
    uint8_t paths = state->observed_paths;
    if (forward_now) {
        paths |= GRAYSCALE_ROAD_PATH_FORWARD;
    } else {
        paths &= static_cast<uint8_t>(~GRAYSCALE_ROAD_PATH_FORWARD);
    }
    return TypeFromPaths(paths);
}

} /* namespace */

void GrayscaleRoad_Init(GrayscaleRoadClassifierState *state)
{
    if (state == 0) {
        return;
    }
    *state = {};
    state->road = GRAYSCALE_ROAD_UNKNOWN;
    state->candidate = GRAYSCALE_ROAD_UNKNOWN;
    state->phase = GRAYSCALE_ROAD_PHASE_NORMAL;
    state->last_event.type = GRAYSCALE_ROAD_UNKNOWN;
}

GrayscaleRoadType GrayscaleRoad_Classify(
    uint8_t active_mask,
    const GrayscaleRoadClassifierConfig *config)
{
    if ((config == 0) || (config->left_mask == 0U) ||
        (config->center_mask == 0U) || (config->right_mask == 0U)) {
        return GRAYSCALE_ROAD_UNKNOWN;
    }
    return TypeFromPaths(ObservePaths(active_mask, config));
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

    const uint8_t confirm_frames =
        ConfigValueOrDefault(config->confirm_frames, 2U);
    const uint8_t exit_confirm_frames =
        ConfigValueOrDefault(config->exit_confirm_frames, 3U);
    const uint8_t rearm_frames =
        ConfigValueOrDefault(config->rearm_frames, 6U);
    const uint8_t max_observe_frames =
        ConfigValueOrDefault(config->max_observe_frames, 24U);
    const uint8_t paths = ObservePaths(active_mask, config);
    const bool side_now = HasSidePath(paths);
    const bool forward_now =
        (paths & GRAYSCALE_ROAD_PATH_FORWARD) != 0U;

    if (state->phase == GRAYSCALE_ROAD_PHASE_LATCHED) {
        if ((!side_now) && forward_now) {
            if (state->rearm_count < rearm_frames) {
                state->rearm_count++;
            }
            if (state->rearm_count >= rearm_frames) {
                state->phase = GRAYSCALE_ROAD_PHASE_NORMAL;
                state->road = GRAYSCALE_ROAD_STRAIGHT;
                state->rearm_count = 0U;
                ResetObservation(state);
            }
        } else {
            state->rearm_count = 0U;
        }
        return state->road;
    }

    if (state->phase == GRAYSCALE_ROAD_PHASE_NORMAL) {
        if (!side_now) {
            state->road = forward_now
                ? GRAYSCALE_ROAD_STRAIGHT
                : ((active_mask == 0U)
                    ? GRAYSCALE_ROAD_LOST
                    : GRAYSCALE_ROAD_UNKNOWN);
            return state->road;
        }
        BeginObservation(state, active_mask, sequence);
    }

    if (state->observe_frames < UINT8_MAX) {
        state->observe_frames++;
    }
    UpdatePathEvidence(state, paths, confirm_frames);
    const uint8_t active_count = CountBits(active_mask);
    if (active_count > state->peak_active_count) {
        state->peak_active_count = active_count;
        state->peak_mask = active_mask;
    }

    const GrayscaleRoadType observed = TypeFromPaths(paths);
    if (observed == state->candidate) {
        if (state->candidate_count < UINT8_MAX) {
            state->candidate_count++;
        }
    } else {
        state->candidate = observed;
        state->candidate_count = 1U;
    }
    state->road = observed;

    if (side_now && !forward_now) {
        if (state->center_absent_frames < UINT8_MAX) {
            state->center_absent_frames++;
        }
    } else {
        state->center_absent_frames = 0U;
    }

    /* A corner or a T junction has no forward continuation. Confirm it
     * promptly so the motion layer can stop/turn before leaving the road. */
    if (state->center_absent_frames >= confirm_frames) {
        const GrayscaleRoadType no_forward_type =
            ResolveObservedType(state, false);
        if ((no_forward_type == GRAYSCALE_ROAD_LEFT_CORNER) ||
            (no_forward_type == GRAYSCALE_ROAD_RIGHT_CORNER) ||
            (no_forward_type == GRAYSCALE_ROAD_T)) {
            PublishEvent(state, no_forward_type, active_mask, sequence);
            return state->road;
        }
    }

    if (!side_now) {
        if (state->exit_frames < UINT8_MAX) {
            state->exit_frames++;
        }
    } else {
        state->exit_frames = 0U;
    }

    /* Branches and crossings are finalized only after their side evidence
     * has passed while the forward line remains. This keeps the entry of a
     * 90-degree corner from being emitted prematurely as a branch. */
    if (state->exit_frames >= exit_confirm_frames) {
        if (!HasSidePath(state->observed_paths)) {
            state->phase = GRAYSCALE_ROAD_PHASE_NORMAL;
            state->road = forward_now
                ? GRAYSCALE_ROAD_STRAIGHT
                : GRAYSCALE_ROAD_LOST;
            ResetObservation(state);
            return state->road;
        }
        const GrayscaleRoadType resolved =
            ResolveObservedType(state, forward_now);
        PublishEvent(state, resolved, active_mask, sequence);
        return state->road;
    }

    if (state->observe_frames >= max_observe_frames) {
        const GrayscaleRoadType resolved =
            ResolveObservedType(state, forward_now);
        PublishEvent(state, resolved, active_mask, sequence);
    }
    return state->road;
}

const GrayscaleRoadEvent *GrayscaleRoad_GetLastEvent(
    const GrayscaleRoadClassifierState *state)
{
    return (state == 0) ? 0 : &state->last_event;
}

void GrayscaleRoad_ClearLastEvent(GrayscaleRoadClassifierState *state)
{
    if (state != 0) {
        state->last_event.valid = false;
    }
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
    case GRAYSCALE_ROAD_LEFT_CORNER:
        return "left_corner";
    case GRAYSCALE_ROAD_RIGHT_CORNER:
        return "right_corner";
    case GRAYSCALE_ROAD_UNKNOWN:
    default:
        return "unknown";
    }
}

const char *GrayscaleRoad_PhaseText(GrayscaleRoadPhase phase)
{
    switch (phase) {
    case GRAYSCALE_ROAD_PHASE_NORMAL:
        return "normal";
    case GRAYSCALE_ROAD_PHASE_OBSERVING:
        return "observing";
    case GRAYSCALE_ROAD_PHASE_LATCHED:
        return "latched";
    default:
        return "unknown";
    }
}

} /* namespace app */
