#ifndef APP_GRAYSCALE_ROAD_H_
#define APP_GRAYSCALE_ROAD_H_

#include <stdint.h>

namespace app {

enum GrayscaleRoadType : uint8_t {
    GRAYSCALE_ROAD_UNKNOWN = 0U,
    GRAYSCALE_ROAD_LOST,
    GRAYSCALE_ROAD_STRAIGHT,
    GRAYSCALE_ROAD_LEFT_BRANCH,
    GRAYSCALE_ROAD_RIGHT_BRANCH,
    GRAYSCALE_ROAD_T,
    GRAYSCALE_ROAD_CROSS,
    GRAYSCALE_ROAD_LEFT_CORNER,
    GRAYSCALE_ROAD_RIGHT_CORNER
};

enum GrayscaleRoadPath : uint8_t {
    GRAYSCALE_ROAD_PATH_LEFT = 0x01U,
    GRAYSCALE_ROAD_PATH_FORWARD = 0x02U,
    GRAYSCALE_ROAD_PATH_RIGHT = 0x04U
};

enum GrayscaleRoadPhase : uint8_t {
    GRAYSCALE_ROAD_PHASE_NORMAL = 0U,
    GRAYSCALE_ROAD_PHASE_OBSERVING,
    GRAYSCALE_ROAD_PHASE_LATCHED
};

struct GrayscaleRoadClassifierConfig {
    uint8_t left_mask;
    uint8_t center_mask;
    uint8_t right_mask;
    uint8_t confirm_frames;
    uint8_t exit_confirm_frames;
    uint8_t rearm_frames;
    uint8_t max_observe_frames;
};

struct GrayscaleRoadEvent {
    bool valid;
    uint32_t sequence;
    GrayscaleRoadType type;
    uint8_t observed_paths;
    uint16_t confidence;
    uint8_t entry_mask;
    uint8_t peak_mask;
    uint8_t exit_mask;
    uint32_t first_frame_sequence;
    uint32_t last_frame_sequence;
};

struct GrayscaleRoadClassifierState {
    GrayscaleRoadType road;
    GrayscaleRoadType candidate;
    uint8_t candidate_count;
    uint32_t last_sequence;
    GrayscaleRoadPhase phase;
    uint8_t observed_paths;
    uint8_t observe_frames;
    uint8_t left_evidence_frames;
    uint8_t forward_evidence_frames;
    uint8_t right_evidence_frames;
    uint8_t center_absent_frames;
    uint8_t exit_frames;
    uint8_t rearm_count;
    uint8_t entry_mask;
    uint8_t peak_mask;
    uint8_t peak_active_count;
    uint32_t first_frame_sequence;
    uint32_t event_sequence;
    GrayscaleRoadEvent last_event;
};

void GrayscaleRoad_Init(GrayscaleRoadClassifierState *state);
GrayscaleRoadType GrayscaleRoad_Classify(
    uint8_t active_mask,
    const GrayscaleRoadClassifierConfig *config);
GrayscaleRoadType GrayscaleRoad_Update(
    GrayscaleRoadClassifierState *state,
    const GrayscaleRoadClassifierConfig *config,
    uint8_t active_mask,
    uint32_t sequence);
const GrayscaleRoadEvent *GrayscaleRoad_GetLastEvent(
    const GrayscaleRoadClassifierState *state);
void GrayscaleRoad_ClearLastEvent(GrayscaleRoadClassifierState *state);
const char *GrayscaleRoad_TypeText(GrayscaleRoadType road);
const char *GrayscaleRoad_PhaseText(GrayscaleRoadPhase phase);

} /* namespace app */

#endif /* APP_GRAYSCALE_ROAD_H_ */
