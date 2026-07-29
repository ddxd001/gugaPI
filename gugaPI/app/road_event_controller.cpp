#include "app/road_event_controller.h"

#include "app/app_grayscale.h"
#include "app/chassis.h"
#include "app/config_store.h"
#include "app/heading.h"
#include "app/linefollow.h"
#include "app/line_sensor.h"
#include "services/fault.h"
#include "services/time.h"

namespace app {
namespace {

static const int16_t kDefaultLeftTurnDeg = 90;
static const int16_t kDefaultRightTurnDeg = -90;
static const int16_t kDefaultAlignDistanceMm = 0;
static const uint16_t kDefaultAlignRpm = 30U;
static const uint16_t kDefaultReacquireTimeoutMs = 800U;
/* Start validating the outgoing branch before the IMU reaches the final
 * heading. Two consecutive fresh frames collected inside this final window
 * let control return to LF_FOLLOW in the same update that reaches target. */
static const int32_t kPreReacquireErrorMdeg = 20000;
static const int32_t kMaximumTurnDeg = 180;
static const int32_t kMaximumAlignDistanceMm = 300;
static const uint32_t kMaximumAlignRpm = 300U;
static const uint32_t kMaximumReacquireTimeoutMs = 5000U;

RoadControlState g_state = {};
int32_t g_pendingTurnDeg = 0;
bool g_pendingPivot = false;

int32_t AbsoluteInt32(int32_t value)
{
    return (value < 0) ? -value : value;
}

bool IsTrackingPositionReady(const AppGrayscaleData *data)
{
    const uint32_t now = services::Time_Millis();
    return (data != 0) && data->valid && data->processed_valid &&
           ((now - data->last_update_ms) <= 50U) &&
           data->line_detected && data->position_valid &&
           (data->track_state == drivers::GRAYSCALE_TRACK_VALID) &&
           (data->channel_anomaly_mask == 0U);
}

bool IsRouteLeft(RoadRoute route)
{
    return (route == ROAD_ROUTE_LEFT) ||
           (route == ROAD_ROUTE_UTURN_LEFT_ARC) ||
           (route == ROAD_ROUTE_UTURN_LEFT_PIVOT);
}

bool IsRouteUturn(RoadRoute route)
{
    return (route == ROAD_ROUTE_UTURN_LEFT_ARC) ||
           (route == ROAD_ROUTE_UTURN_RIGHT_ARC) ||
           (route == ROAD_ROUTE_UTURN_LEFT_PIVOT) ||
           (route == ROAD_ROUTE_UTURN_RIGHT_PIVOT);
}

bool IsRoutePivot(RoadRoute route)
{
    return (route == ROAD_ROUTE_UTURN_LEFT_PIVOT) ||
           (route == ROAD_ROUTE_UTURN_RIGHT_PIVOT);
}

uint8_t DesiredPath(RoadRoute route)
{
    if (route == ROAD_ROUTE_STRAIGHT) {
        return GRAYSCALE_ROAD_PATH_FORWARD;
    }
    return IsRouteLeft(route)
        ? GRAYSCALE_ROAD_PATH_LEFT
        : GRAYSCALE_ROAD_PATH_RIGHT;
}

bool EventHasRoute(const AppGrayscaleData *data, RoadRoute route)
{
    if (data == 0) {
        return false;
    }
    return (data->road_observed_paths & DesiredPath(route)) != 0U;
}

void ResetReacquire(const AppGrayscaleData *data)
{
    g_state.reacquire_last_sequence =
        (data == 0) ? 0U : data->sequence;
    g_state.reacquire_valid_frames = 0U;
}

void FinishStopped(
    drivers::DriverStatus status,
    RoadRouteResult route_result = ROAD_ROUTE_RESULT_CONTROL_ERROR)
{
    (void) Heading_Stop();
    (void) LF_Stop();
    (void) Chassis_Stop();
    g_state.phase = ROAD_CONTROL_PHASE_STOPPED;
    g_state.phase_start_ms = services::Time_Millis();
    g_state.road_base_rpm = 0;
    g_state.reacquire_last_sequence = 0U;
    g_state.reacquire_valid_frames = 0U;
    if (g_state.route_active) {
        g_state.route_active = false;
        g_state.route_result = route_result;
    }
    g_state.last_status = status;
}

void ObserveReacquireFrame(const AppGrayscaleData *data, bool enabled)
{
    if (!enabled) {
        g_state.reacquire_valid_frames = 0U;
        g_state.reacquire_last_sequence =
            (data == 0) ? 0U : data->sequence;
        return;
    }
    if ((data == 0) ||
        (data->sequence == g_state.reacquire_last_sequence)) {
        return;
    }

    g_state.reacquire_last_sequence = data->sequence;
    if (IsTrackingPositionReady(data)) {
        if (g_state.reacquire_valid_frames < UINT8_MAX) {
            g_state.reacquire_valid_frames++;
        }
    } else {
        g_state.reacquire_valid_frames = 0U;
    }
}

bool TryFinishLineHandoff(
    const AppGrayscaleData *data,
    bool pivot_handoff)
{
    if ((data == 0) || (g_state.reacquire_valid_frames < 2U)) {
        return false;
    }

    drivers::DriverStatus status = LF_Start(
        g_state.saved_base_rpm,
        g_state.saved_duration_ms);
    if (status == drivers::DRIVER_OK) {
        status = pivot_handoff
            ? Heading_HoldReleaseForMotionHandoff()
            : Heading_ReleaseForMotionHandoff();
        if (status == drivers::DRIVER_OK) {
            g_state.handled_event_sequence = data->road_event_sequence;
            g_state.phase = ROAD_CONTROL_PHASE_IDLE;
            g_state.road_base_rpm = 0;
            g_state.reacquire_valid_frames = 0U;
            if (g_state.route_active) {
                g_state.route_active = false;
                g_state.route_result = ROAD_ROUTE_RESULT_SUCCESS;
            }
            g_state.last_status = drivers::DRIVER_OK;
            return true;
        }
        FinishStopped(status, ROAD_ROUTE_RESULT_CONTROL_ERROR);
        return true;
    }
    if (status != drivers::DRIVER_ERROR_NOT_INITIALIZED) {
        FinishStopped(status, ROAD_ROUTE_RESULT_CONTROL_ERROR);
        return true;
    }
    return false;
}

drivers::DriverStatus StartTurn(void)
{
    if (g_pendingPivot) {
        const drivers::DriverStatus stop_status = Heading_Stop();
        if (stop_status != drivers::DRIVER_OK) {
            FinishStopped(stop_status, ROAD_ROUTE_RESULT_CONTROL_ERROR);
            return stop_status;
        }
        const drivers::DriverStatus pivot_status =
            Heading_TurnStart(g_pendingTurnDeg);
        if (pivot_status == drivers::DRIVER_OK) {
            g_state.phase = ROAD_CONTROL_PHASE_PIVOT_TURNING;
            g_state.phase_start_ms = services::Time_Millis();
        } else {
            FinishStopped(pivot_status, ROAD_ROUTE_RESULT_CONTROL_ERROR);
        }
        g_state.last_status = pivot_status;
        return pivot_status;
    }

    const drivers::DriverStatus status = Heading_ArcTurnStart(
        g_pendingTurnDeg,
        g_state.road_base_rpm);
    if (status == drivers::DRIVER_OK) {
        g_state.phase = ROAD_CONTROL_PHASE_TURNING;
        g_state.phase_start_ms = services::Time_Millis();
    } else {
        FinishStopped(status, ROAD_ROUTE_RESULT_CONTROL_ERROR);
    }
    g_state.last_status = status;
    return status;
}

void SyncPersistentConfig(void)
{
    const ConfigStoreParams *params = ConfigStore_Get();
    if (params != 0) {
        g_state.config.align_distance_mm =
            static_cast<int16_t>(params->road_align_distance_mm);
        g_state.config.align_rpm = params->road_align_rpm;
    }
}

drivers::DriverStatus StartManeuver(int32_t angle_deg, bool pivot)
{
    const LFState *lf = LF_GetState();
    if ((lf == 0) || (lf->mode != LF_FOLLOW)) {
        const drivers::DriverStatus status =
            drivers::DRIVER_ERROR_NOT_INITIALIZED;
        FinishStopped(status, ROAD_ROUTE_RESULT_CONTROL_ERROR);
        return status;
    }

    const uint32_t now = services::Time_Millis();
    ResetReacquire(App_GrayscaleGetData());
    g_pendingTurnDeg = angle_deg;
    g_pendingPivot = pivot;
    g_state.saved_base_rpm = lf->base_rpm;
    if (lf->follow_duration_ms == 0U) {
        g_state.saved_duration_ms = 0U;
    } else {
        const uint32_t elapsed = now - lf->follow_start_ms;
        g_state.saved_duration_ms = (elapsed < lf->follow_duration_ms)
            ? (lf->follow_duration_ms - elapsed)
            : 1U;
    }

    int32_t align_rpm = static_cast<int32_t>(g_state.config.align_rpm);
    const int32_t base_magnitude = AbsoluteInt32(g_state.saved_base_rpm);
    if (align_rpm > base_magnitude) {
        align_rpm = base_magnitude;
    }
    if (align_rpm == 0) {
        align_rpm = 1;
    }
    const int32_t travel_direction =
        (g_state.saved_base_rpm < 0) ? -1 : 1;
    g_state.road_base_rpm = align_rpm * travel_direction;

    drivers::DriverStatus status = LF_ReleaseForMotionHandoff();
    if (status != drivers::DRIVER_OK) {
        FinishStopped(status, ROAD_ROUTE_RESULT_CONTROL_ERROR);
        return status;
    }

    if (g_state.config.align_distance_mm == 0) {
        return StartTurn();
    }

    status = Heading_DistanceStartForRollingHandoff(
        g_state.config.align_distance_mm * travel_direction,
        align_rpm,
        0U,
        align_rpm);
    if (status == drivers::DRIVER_OK) {
        g_state.phase = ROAD_CONTROL_PHASE_ALIGNING;
        g_state.phase_start_ms = now;
    } else {
        FinishStopped(status, ROAD_ROUTE_RESULT_CONTROL_ERROR);
    }
    g_state.last_status = status;
    return status;
}

drivers::DriverStatus StartCorner(GrayscaleRoadType type)
{
    const int32_t angle = (type == GRAYSCALE_ROAD_LEFT_CORNER)
        ? g_state.config.left_turn_deg
        : g_state.config.right_turn_deg;
    return StartManeuver(angle, false);
}

drivers::DriverStatus StartRequestedManeuver(
    const AppGrayscaleData *data)
{
    g_state.handled_event_sequence =
        (data == 0) ? g_state.handled_event_sequence
                    : data->road_event_sequence;
    g_state.handled_event_type =
        (data == 0) ? GRAYSCALE_ROAD_UNKNOWN
                    : data->road_event_type;
    g_state.last_policy = ROAD_POLICY_ACTION_SEQUENCE;

    int32_t angle = IsRouteLeft(g_state.route)
        ? g_state.config.left_turn_deg
        : g_state.config.right_turn_deg;
    if (IsRouteUturn(g_state.route)) {
        angle = IsRouteLeft(g_state.route) ? 180 : -180;
    }
    return StartManeuver(angle, IsRoutePivot(g_state.route));
}

void FinishStraightRoute(const AppGrayscaleData *data)
{
    g_state.handled_event_sequence = data->road_event_sequence;
    g_state.handled_event_type = data->road_event_type;
    g_state.last_policy = ROAD_POLICY_ACTION_SEQUENCE;
    g_state.phase = ROAD_CONTROL_PHASE_IDLE;
    g_state.route_active = false;
    g_state.route_result = ROAD_ROUTE_RESULT_SUCCESS;
    g_state.last_status = drivers::DRIVER_OK;
}

void HandleRouteEvent(const AppGrayscaleData *data)
{
    if (!EventHasRoute(data, g_state.route)) {
        g_state.handled_event_sequence = data->road_event_sequence;
        g_state.handled_event_type = data->road_event_type;
        g_state.last_policy = ROAD_POLICY_ACTION_SEQUENCE;
        FinishStopped(drivers::DRIVER_ERROR_NOT_INITIALIZED,
                      ROAD_ROUTE_RESULT_UNAVAILABLE);
        return;
    }
    if (g_state.route == ROAD_ROUTE_STRAIGHT) {
        FinishStraightRoute(data);
        return;
    }
    (void) StartRequestedManeuver(data);
}

void HandleNewEvent(const AppGrayscaleData *data)
{
    g_state.handled_event_sequence = data->road_event_sequence;
    g_state.handled_event_type = data->road_event_type;
    g_state.last_policy = RoadEventController_GetPolicy(data->road_event_type);

    switch (g_state.last_policy) {
    case ROAD_POLICY_TURN_LEFT:
    case ROAD_POLICY_TURN_RIGHT:
        if (g_state.mode == ROAD_CONTROL_AUTO_CORNERS) {
            (void) StartCorner(data->road_event_type);
        }
        break;
    case ROAD_POLICY_STOP:
        if (LF_GetState()->mode == LF_FOLLOW) {
            FinishStopped(drivers::DRIVER_OK);
        }
        break;
    case ROAD_POLICY_DEFAULT:
    case ROAD_POLICY_ACTION_SEQUENCE:
    default:
        break;
    }
}

} /* namespace */

void RoadEventController_Init(void)
{
    g_state = {};
    g_state.mode = ROAD_CONTROL_DETECT_ONLY;
    g_state.phase = ROAD_CONTROL_PHASE_IDLE;
    g_state.config.left_turn_deg = kDefaultLeftTurnDeg;
    g_state.config.right_turn_deg = kDefaultRightTurnDeg;
    g_state.config.align_distance_mm = kDefaultAlignDistanceMm;
    g_state.config.align_rpm = kDefaultAlignRpm;
    g_state.config.reacquire_timeout_ms = kDefaultReacquireTimeoutMs;
    SyncPersistentConfig();
    g_state.handled_event_type = GRAYSCALE_ROAD_UNKNOWN;
    g_state.last_policy = ROAD_POLICY_DEFAULT;
    g_state.route = ROAD_ROUTE_STRAIGHT;
    g_state.route_result = ROAD_ROUTE_RESULT_IDLE;
    g_state.route_active = false;
    g_state.last_status = drivers::DRIVER_OK;
    g_pendingTurnDeg = 0;
    g_pendingPivot = false;
}

void RoadEventController_Update(void)
{
    const uint32_t now = services::Time_Millis();
    SyncPersistentConfig();
    if (services::Fault_HasFault()) {
        if (g_state.route_active ||
            ((g_state.phase != ROAD_CONTROL_PHASE_IDLE) &&
             (g_state.phase != ROAD_CONTROL_PHASE_STOPPED))) {
            FinishStopped(drivers::DRIVER_ERROR_NOT_INITIALIZED,
                          ROAD_ROUTE_RESULT_FAULT);
        }
        return;
    }

    if (g_state.route_active &&
        ((now - g_state.route_start_ms) >= g_state.route_timeout_ms)) {
        FinishStopped(drivers::DRIVER_ERROR_TIMEOUT,
                      ROAD_ROUTE_RESULT_TIMEOUT);
        return;
    }

    const AppGrayscaleData *data = App_GrayscaleGetData();

    if (g_state.phase == ROAD_CONTROL_PHASE_ARMING) {
        if (LF_GetState()->mode != LF_FOLLOW) {
            FinishStopped(drivers::DRIVER_ERROR_NOT_INITIALIZED,
                          ROAD_ROUTE_RESULT_CONTROL_ERROR);
            return;
        }
        if ((data != 0) &&
            (data->road_phase == GRAYSCALE_ROAD_PHASE_NORMAL)) {
            g_state.handled_event_sequence =
                data->road_event_sequence;
            g_state.phase = ROAD_CONTROL_PHASE_FOLLOWING;
            g_state.phase_start_ms = now;
        }
        return;
    }

    if (g_state.phase == ROAD_CONTROL_PHASE_FOLLOWING) {
        if (LF_GetState()->mode != LF_FOLLOW) {
            FinishStopped(drivers::DRIVER_ERROR_NOT_INITIALIZED,
                          ROAD_ROUTE_RESULT_CONTROL_ERROR);
            return;
        }
        if ((data != 0) &&
            (data->road_phase == GRAYSCALE_ROAD_PHASE_OBSERVING) &&
            (g_state.route != ROAD_ROUTE_STRAIGHT) &&
            EventHasRoute(data, g_state.route)) {
            (void) StartRequestedManeuver(data);
            return;
        }
        if ((data != 0) && (data->road_event_sequence != 0U) &&
            (data->road_event_sequence !=
             g_state.handled_event_sequence)) {
            HandleRouteEvent(data);
        }
        return;
    }

    if (g_state.phase == ROAD_CONTROL_PHASE_ALIGNING) {
        const HeadingState *heading = Heading_GetState();
        if (heading->mode == HEADING_IDLE) {
            if (heading->last_status != drivers::DRIVER_OK) {
                FinishStopped(heading->last_status,
                              ROAD_ROUTE_RESULT_CONTROL_ERROR);
            } else {
                (void) StartTurn();
            }
        } else if (heading->mode != HEADING_DISTANCE) {
            FinishStopped(drivers::DRIVER_ERROR_NOT_INITIALIZED,
                          ROAD_ROUTE_RESULT_CONTROL_ERROR);
        }
        return;
    }

    if (g_state.phase == ROAD_CONTROL_PHASE_TURNING) {
        const HeadingState *heading = Heading_GetState();
        if (heading->mode == HEADING_ARC_TURN) {
            const AppGrayscaleData *data = App_GrayscaleGetData();
            ObserveReacquireFrame(
                data,
                AbsoluteInt32(heading->error_mdeg) <=
                    kPreReacquireErrorMdeg);
            if (!heading->at_target) {
                return;
            }
            if (TryFinishLineHandoff(data, false)) {
                return;
            }
            g_state.phase = ROAD_CONTROL_PHASE_REACQUIRE;
            g_state.phase_start_ms = now;
        } else if (heading->mode != HEADING_ARC_TURN) {
            const drivers::DriverStatus status =
                (heading->last_status == drivers::DRIVER_OK)
                    ? drivers::DRIVER_ERROR_NOT_INITIALIZED
                    : heading->last_status;
            FinishStopped(status, ROAD_ROUTE_RESULT_CONTROL_ERROR);
        }
        return;
    }

    if (g_state.phase == ROAD_CONTROL_PHASE_PIVOT_TURNING) {
        const HeadingState *heading = Heading_GetState();
        if (heading->mode == HEADING_TURN) {
            return;
        }
        if ((heading->mode != HEADING_IDLE) ||
            (heading->last_status != drivers::DRIVER_OK)) {
            const drivers::DriverStatus status =
                (heading->last_status == drivers::DRIVER_OK)
                    ? drivers::DRIVER_ERROR_NOT_INITIALIZED
                    : heading->last_status;
            FinishStopped(status, ROAD_ROUTE_RESULT_CONTROL_ERROR);
            return;
        }

        ResetReacquire(data);
        const drivers::DriverStatus status =
            Heading_HoldStart(AbsoluteInt32(g_state.road_base_rpm));
        if (status == drivers::DRIVER_OK) {
            g_state.phase = ROAD_CONTROL_PHASE_PIVOT_REACQUIRE;
            g_state.phase_start_ms = now;
            g_state.last_status = drivers::DRIVER_OK;
        } else {
            FinishStopped(status, ROAD_ROUTE_RESULT_CONTROL_ERROR);
        }
        return;
    }

    if (g_state.phase == ROAD_CONTROL_PHASE_REACQUIRE) {
        const HeadingState *heading = Heading_GetState();
        if (heading->mode != HEADING_ARC_TURN) {
            const drivers::DriverStatus status =
                (heading->last_status == drivers::DRIVER_OK)
                    ? drivers::DRIVER_ERROR_NOT_INITIALIZED
                    : heading->last_status;
            FinishStopped(status, ROAD_ROUTE_RESULT_CONTROL_ERROR);
            return;
        }

        ObserveReacquireFrame(data, heading->at_target);
        if (TryFinishLineHandoff(data, false)) {
            return;
        }
        if ((now - g_state.phase_start_ms) >=
            g_state.config.reacquire_timeout_ms) {
            FinishStopped(drivers::DRIVER_ERROR_TIMEOUT,
                          ROAD_ROUTE_RESULT_REACQUIRE_FAILED);
        }
        return;
    }

    if (g_state.phase == ROAD_CONTROL_PHASE_PIVOT_REACQUIRE) {
        const HeadingState *heading = Heading_GetState();
        if (heading->mode != HEADING_HOLD) {
            const drivers::DriverStatus status =
                (heading->last_status == drivers::DRIVER_OK)
                    ? drivers::DRIVER_ERROR_NOT_INITIALIZED
                    : heading->last_status;
            FinishStopped(status, ROAD_ROUTE_RESULT_CONTROL_ERROR);
            return;
        }

        ObserveReacquireFrame(data, true);
        if (TryFinishLineHandoff(data, true)) {
            return;
        }
        if ((now - g_state.phase_start_ms) >=
            g_state.config.reacquire_timeout_ms) {
            FinishStopped(drivers::DRIVER_ERROR_TIMEOUT,
                          ROAD_ROUTE_RESULT_REACQUIRE_FAILED);
        }
        return;
    }

    if ((g_state.phase == ROAD_CONTROL_PHASE_STOPPED) &&
        (LF_GetState()->mode == LF_FOLLOW)) {
        g_state.phase = ROAD_CONTROL_PHASE_IDLE;
    }

    if ((data != 0) && (data->road_event_sequence != 0U) &&
        (data->road_event_sequence != g_state.handled_event_sequence)) {
        HandleNewEvent(data);
    }
}

drivers::DriverStatus RoadEventController_SetMode(RoadControlMode mode)
{
    if ((mode != ROAD_CONTROL_DETECT_ONLY) &&
        (mode != ROAD_CONTROL_AUTO_CORNERS)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if ((mode == ROAD_CONTROL_AUTO_CORNERS) &&
        (!LineSensor_IsRoadCapable())) {
        return drivers::DRIVER_ERROR_UNSUPPORTED;
    }
    if (g_state.route_active ||
        ((g_state.phase != ROAD_CONTROL_PHASE_IDLE) &&
         (g_state.phase != ROAD_CONTROL_PHASE_STOPPED))) {
        (void) RoadEventController_Cancel();
    }
    g_state.mode = mode;
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus RoadEventController_SetTurnConfig(
    int32_t left_turn_deg,
    int32_t right_turn_deg,
    int32_t align_distance_mm,
    uint32_t align_rpm,
    uint32_t reacquire_timeout_ms)
{
    if ((left_turn_deg <= 0) || (left_turn_deg > kMaximumTurnDeg) ||
        (right_turn_deg >= 0) || (right_turn_deg < -kMaximumTurnDeg) ||
        (align_distance_mm < 0) ||
        (align_distance_mm > kMaximumAlignDistanceMm) ||
        (align_rpm == 0U) || (align_rpm > kMaximumAlignRpm) ||
        (reacquire_timeout_ms == 0U) ||
        (reacquire_timeout_ms > kMaximumReacquireTimeoutMs)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    const drivers::DriverStatus align_status =
        RoadEventController_SetAlignConfig(align_distance_mm, align_rpm);
    if (align_status != drivers::DRIVER_OK) {
        return align_status;
    }
    g_state.config.left_turn_deg = static_cast<int16_t>(left_turn_deg);
    g_state.config.right_turn_deg = static_cast<int16_t>(right_turn_deg);
    g_state.config.reacquire_timeout_ms =
        static_cast<uint16_t>(reacquire_timeout_ms);
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus RoadEventController_SetAlignConfig(
    int32_t align_distance_mm,
    uint32_t align_rpm)
{
    if ((align_distance_mm < 0) ||
        (align_distance_mm > kMaximumAlignDistanceMm) ||
        (align_rpm == 0U) || (align_rpm > kMaximumAlignRpm)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }

    drivers::DriverStatus status = ConfigStore_Set(
        "road_align_distance_mm",
        align_distance_mm);
    if (status == drivers::DRIVER_OK) {
        status = ConfigStore_Set("road_align_rpm",
                                 static_cast<int32_t>(align_rpm));
    }
    if (status != drivers::DRIVER_OK) {
        g_state.last_status = status;
        return status;
    }
    g_state.config.align_distance_mm =
        static_cast<int16_t>(align_distance_mm);
    g_state.config.align_rpm = static_cast<uint16_t>(align_rpm);
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus RoadEventController_StartRoute(
    RoadRoute route,
    int32_t rpm,
    uint32_t timeout_ms)
{
    if (!LineSensor_IsRoadCapable()) {
        g_state.last_status = drivers::DRIVER_ERROR_UNSUPPORTED;
        g_state.route_result = ROAD_ROUTE_RESULT_UNAVAILABLE;
        return g_state.last_status;
    }
    const ChassisState *chassis = Chassis_GetState();
    if ((route >= ROAD_ROUTE_COUNT) || (rpm <= 0) ||
        (chassis == 0) ||
        (rpm > static_cast<int32_t>(chassis->config.max_wheel_rpm)) ||
        (timeout_ms < 50U) || (timeout_ms > 30000U) ||
        ((timeout_ms % 50U) != 0U)) {
        g_state.last_status = drivers::DRIVER_ERROR_INVALID_ARG;
        return g_state.last_status;
    }
    if (g_state.route_active ||
        ((g_state.phase != ROAD_CONTROL_PHASE_IDLE) &&
         (g_state.phase != ROAD_CONTROL_PHASE_STOPPED))) {
        g_state.last_status = drivers::DRIVER_ERROR_BUSY;
        return g_state.last_status;
    }
    if (services::Fault_HasFault()) {
        g_state.last_status = drivers::DRIVER_ERROR_NOT_INITIALIZED;
        return g_state.last_status;
    }

    drivers::DriverStatus status = drivers::DRIVER_OK;
    const LFState *lf = LF_GetState();
    if ((lf != 0) && (lf->mode == LF_FOLLOW)) {
        status = LF_ContinueForMotionHandoff(rpm, 0U);
    } else if ((lf != 0) && (lf->mode == LF_IDLE)) {
        status = LF_Start(rpm, 0U);
    } else {
        status = drivers::DRIVER_ERROR_BUSY;
    }
    if (status != drivers::DRIVER_OK) {
        (void) Heading_Stop();
        (void) LF_Stop();
        (void) Chassis_Stop();
        g_state.last_status = status;
        return status;
    }

    const uint32_t now = services::Time_Millis();
    const AppGrayscaleData *data = App_GrayscaleGetData();
    g_state.route = route;
    g_state.route_result = ROAD_ROUTE_RESULT_RUNNING;
    g_state.route_active = true;
    g_state.route_start_ms = now;
    g_state.route_timeout_ms = timeout_ms;
    g_state.saved_base_rpm = rpm;
    g_state.saved_duration_ms = 0U;
    g_state.road_base_rpm = 0;
    g_state.handled_event_sequence =
        (data == 0) ? 0U : data->road_event_sequence;
    g_state.handled_event_type = GRAYSCALE_ROAD_UNKNOWN;
    g_state.last_policy = ROAD_POLICY_ACTION_SEQUENCE;
    ResetReacquire(data);
    g_state.phase = ((data != 0) &&
                     (data->road_phase == GRAYSCALE_ROAD_PHASE_NORMAL))
        ? ROAD_CONTROL_PHASE_FOLLOWING
        : ROAD_CONTROL_PHASE_ARMING;
    g_state.phase_start_ms = now;
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus RoadEventController_Cancel(void)
{
    const bool was_active = g_state.route_active;
    (void) Heading_Stop();
    (void) LF_Stop();
    (void) Chassis_Stop();
    g_state.phase = ROAD_CONTROL_PHASE_IDLE;
    g_state.route_active = false;
    if (was_active) {
        g_state.route_result = ROAD_ROUTE_RESULT_CANCELLED;
    }
    g_state.road_base_rpm = 0;
    g_state.reacquire_last_sequence = 0U;
    g_state.reacquire_valid_frames = 0U;
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

bool RoadEventController_IsRouteActive(void)
{
    return g_state.route_active;
}

void RoadEventController_ClearEvent(void)
{
    const AppGrayscaleData *data = App_GrayscaleGetData();
    g_state.handled_event_sequence =
        (data == 0) ? 0U : data->road_event_sequence;
    App_GrayscaleClearRoadEvent();
    g_state.handled_event_type = GRAYSCALE_ROAD_UNKNOWN;
    g_state.last_policy = ROAD_POLICY_DEFAULT;
    if (g_state.phase == ROAD_CONTROL_PHASE_STOPPED) {
        g_state.phase = ROAD_CONTROL_PHASE_IDLE;
    }
}

const RoadControlState *RoadEventController_GetState(void)
{
    return &g_state;
}

RoadEventPolicy RoadEventController_GetPolicy(GrayscaleRoadType type)
{
    switch (type) {
    case GRAYSCALE_ROAD_LEFT_CORNER:
        return ROAD_POLICY_TURN_LEFT;
    case GRAYSCALE_ROAD_RIGHT_CORNER:
        return ROAD_POLICY_TURN_RIGHT;
    case GRAYSCALE_ROAD_T:
    case GRAYSCALE_ROAD_UNKNOWN:
        return ROAD_POLICY_STOP;
    case GRAYSCALE_ROAD_LEFT_BRANCH:
    case GRAYSCALE_ROAD_RIGHT_BRANCH:
    case GRAYSCALE_ROAD_CROSS:
    case GRAYSCALE_ROAD_STRAIGHT:
    case GRAYSCALE_ROAD_LOST:
    default:
        return ROAD_POLICY_DEFAULT;
    }
}

const char *RoadEventController_ModeText(RoadControlMode mode)
{
    return (mode == ROAD_CONTROL_AUTO_CORNERS) ? "corner" : "detect";
}

const char *RoadEventController_PhaseText(RoadControlPhase phase)
{
    switch (phase) {
    case ROAD_CONTROL_PHASE_IDLE:
        return "idle";
    case ROAD_CONTROL_PHASE_ARMING:
        return "arming";
    case ROAD_CONTROL_PHASE_FOLLOWING:
        return "following";
    case ROAD_CONTROL_PHASE_ALIGNING:
        return "aligning";
    case ROAD_CONTROL_PHASE_TURNING:
        return "turning";
    case ROAD_CONTROL_PHASE_PIVOT_TURNING:
        return "pivot_turn";
    case ROAD_CONTROL_PHASE_REACQUIRE:
        return "reacquire";
    case ROAD_CONTROL_PHASE_PIVOT_REACQUIRE:
        return "pivot_reacquire";
    case ROAD_CONTROL_PHASE_STOPPED:
        return "stopped";
    default:
        return "unknown";
    }
}

const char *RoadEventController_RouteText(RoadRoute route)
{
    switch (route) {
    case ROAD_ROUTE_LEFT:
        return "left";
    case ROAD_ROUTE_STRAIGHT:
        return "straight";
    case ROAD_ROUTE_RIGHT:
        return "right";
    case ROAD_ROUTE_UTURN_LEFT_ARC:
        return "uturn_left_arc";
    case ROAD_ROUTE_UTURN_RIGHT_ARC:
        return "uturn_right_arc";
    case ROAD_ROUTE_UTURN_LEFT_PIVOT:
        return "uturn_left_pivot";
    case ROAD_ROUTE_UTURN_RIGHT_PIVOT:
        return "uturn_right_pivot";
    case ROAD_ROUTE_COUNT:
    default:
        return "unknown";
    }
}

const char *RoadEventController_RouteResultText(RoadRouteResult result)
{
    switch (result) {
    case ROAD_ROUTE_RESULT_IDLE:
        return "idle";
    case ROAD_ROUTE_RESULT_RUNNING:
        return "running";
    case ROAD_ROUTE_RESULT_SUCCESS:
        return "success";
    case ROAD_ROUTE_RESULT_UNAVAILABLE:
        return "route_unavailable";
    case ROAD_ROUTE_RESULT_REACQUIRE_FAILED:
        return "route_reacquire_failed";
    case ROAD_ROUTE_RESULT_CANCELLED:
        return "cancelled";
    case ROAD_ROUTE_RESULT_CONTROL_ERROR:
        return "control_error";
    case ROAD_ROUTE_RESULT_TIMEOUT:
        return "timeout";
    case ROAD_ROUTE_RESULT_FAULT:
        return "fault";
    default:
        return "unknown";
    }
}

const char *RoadEventController_PolicyText(RoadEventPolicy policy)
{
    switch (policy) {
    case ROAD_POLICY_TURN_LEFT:
        return "turn_left";
    case ROAD_POLICY_TURN_RIGHT:
        return "turn_right";
    case ROAD_POLICY_STOP:
        return "stop";
    case ROAD_POLICY_ACTION_SEQUENCE:
        return "sequence";
    case ROAD_POLICY_DEFAULT:
    default:
        return "default";
    }
}

} /* namespace app */
