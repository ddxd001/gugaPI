#ifndef APP_ROAD_EVENT_CONTROLLER_H_
#define APP_ROAD_EVENT_CONTROLLER_H_

#include <stdint.h>

#include "app/grayscale_road.h"
#include "drivers/common/driver_status.h"

namespace app {

enum RoadControlMode : uint8_t {
    ROAD_CONTROL_DETECT_ONLY = 0U,
    ROAD_CONTROL_AUTO_CORNERS
};

enum RoadControlPhase : uint8_t {
    ROAD_CONTROL_PHASE_IDLE = 0U,
    ROAD_CONTROL_PHASE_ALIGNING,
    ROAD_CONTROL_PHASE_TURNING,
    ROAD_CONTROL_PHASE_REACQUIRE,
    ROAD_CONTROL_PHASE_STOPPED
};

/* The policy type is deliberately separate from road geometry. Future route
 * code can map any event to an ActionRunner/SeqStore sequence without changing
 * the detector. This iteration only uses DEFAULT, TURN_LEFT, TURN_RIGHT, STOP. */
enum RoadEventPolicy : uint8_t {
    ROAD_POLICY_DEFAULT = 0U,
    ROAD_POLICY_TURN_LEFT,
    ROAD_POLICY_TURN_RIGHT,
    ROAD_POLICY_STOP,
    ROAD_POLICY_ACTION_SEQUENCE
};

struct RoadControlConfig {
    int16_t left_turn_deg;
    int16_t right_turn_deg;
    int16_t align_distance_mm;
    uint16_t align_rpm;
    uint16_t reacquire_timeout_ms;
};

struct RoadControlState {
    RoadControlMode mode;
    RoadControlPhase phase;
    RoadControlConfig config;
    uint32_t handled_event_sequence;
    GrayscaleRoadType handled_event_type;
    RoadEventPolicy last_policy;
    int32_t saved_base_rpm;
    int32_t road_base_rpm;
    uint32_t saved_duration_ms;
    uint32_t phase_start_ms;
    uint32_t reacquire_last_sequence;
    uint8_t reacquire_valid_frames;
    drivers::DriverStatus last_status;
};

void RoadEventController_Init(void);
void RoadEventController_Update(void);
drivers::DriverStatus RoadEventController_SetMode(RoadControlMode mode);
drivers::DriverStatus RoadEventController_SetTurnConfig(
    int32_t left_turn_deg,
    int32_t right_turn_deg,
    int32_t align_distance_mm,
    uint32_t align_rpm,
    uint32_t reacquire_timeout_ms);
drivers::DriverStatus RoadEventController_SetAlignConfig(
    int32_t align_distance_mm,
    uint32_t align_rpm);
drivers::DriverStatus RoadEventController_Cancel(void);
void RoadEventController_ClearEvent(void);
const RoadControlState *RoadEventController_GetState(void);
RoadEventPolicy RoadEventController_GetPolicy(GrayscaleRoadType type);
const char *RoadEventController_ModeText(RoadControlMode mode);
const char *RoadEventController_PhaseText(RoadControlPhase phase);
const char *RoadEventController_PolicyText(RoadEventPolicy policy);

} /* namespace app */

#endif /* APP_ROAD_EVENT_CONTROLLER_H_ */
