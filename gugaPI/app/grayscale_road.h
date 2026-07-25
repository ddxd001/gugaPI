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
    GRAYSCALE_ROAD_CROSS
};

struct GrayscaleRoadClassifierConfig {
    uint8_t left_mask;
    uint8_t center_mask;
    uint8_t right_mask;
    uint8_t confirm_frames;
};

struct GrayscaleRoadClassifierState {
    GrayscaleRoadType road;
    GrayscaleRoadType candidate;
    uint8_t candidate_count;
    uint32_t last_sequence;
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
const char *GrayscaleRoad_TypeText(GrayscaleRoadType road);

} /* namespace app */

#endif /* APP_GRAYSCALE_ROAD_H_ */
