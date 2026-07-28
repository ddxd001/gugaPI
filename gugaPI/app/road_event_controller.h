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
    ROAD_CONTROL_PHASE_ARMING,
    ROAD_CONTROL_PHASE_FOLLOWING,
    ROAD_CONTROL_PHASE_ALIGNING,
    ROAD_CONTROL_PHASE_TURNING,
    ROAD_CONTROL_PHASE_PIVOT_TURNING,
    ROAD_CONTROL_PHASE_REACQUIRE,
    ROAD_CONTROL_PHASE_PIVOT_REACQUIRE,
    ROAD_CONTROL_PHASE_STOPPED
};

enum RoadRoute : uint8_t {
    ROAD_ROUTE_LEFT = 0U,
    ROAD_ROUTE_STRAIGHT,
    ROAD_ROUTE_RIGHT,
    ROAD_ROUTE_UTURN_LEFT_ARC,
    ROAD_ROUTE_UTURN_RIGHT_ARC,
    ROAD_ROUTE_UTURN_LEFT_PIVOT,
    ROAD_ROUTE_UTURN_RIGHT_PIVOT,
    ROAD_ROUTE_COUNT
};

enum RoadRouteResult : uint8_t {
    ROAD_ROUTE_RESULT_IDLE = 0U,
    ROAD_ROUTE_RESULT_RUNNING,
    ROAD_ROUTE_RESULT_SUCCESS,
    ROAD_ROUTE_RESULT_UNAVAILABLE,
    ROAD_ROUTE_RESULT_REACQUIRE_FAILED,
    ROAD_ROUTE_RESULT_CANCELLED,
    ROAD_ROUTE_RESULT_CONTROL_ERROR,
    ROAD_ROUTE_RESULT_TIMEOUT,
    ROAD_ROUTE_RESULT_FAULT
};

/* Policy remains separate from road geometry. Explicit ActionRunner route
 * requests use ACTION_SEQUENCE while the global automatic mode keeps the
 * existing default/corner/stop mapping. */
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
    RoadRoute route;
    RoadRouteResult route_result;
    bool route_active;
    uint32_t route_start_ms;
    uint32_t route_timeout_ms;
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
/* Run one explicit route request. A currently active LF_FOLLOW controller is
 * adopted without stopping; otherwise the request starts line following. */
drivers::DriverStatus RoadEventController_StartRoute(
    RoadRoute route,
    int32_t rpm,
    uint32_t timeout_ms);
drivers::DriverStatus RoadEventController_Cancel(void);
bool RoadEventController_IsRouteActive(void);
void RoadEventController_ClearEvent(void);
const RoadControlState *RoadEventController_GetState(void);
RoadEventPolicy RoadEventController_GetPolicy(GrayscaleRoadType type);
const char *RoadEventController_ModeText(RoadControlMode mode);
const char *RoadEventController_PhaseText(RoadControlPhase phase);
const char *RoadEventController_PolicyText(RoadEventPolicy policy);
const char *RoadEventController_RouteText(RoadRoute route);
const char *RoadEventController_RouteResultText(RoadRouteResult result);

} /* namespace app */

#endif /* APP_ROAD_EVENT_CONTROLLER_H_ */
