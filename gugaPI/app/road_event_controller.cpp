#include "app/road_event_controller.h"

#include "app/app_grayscale.h"
#include "app/heading.h"
#include "app/linefollow.h"
#include "config/feature_config.h"
#include "services/fault.h"
#include "services/time.h"

namespace app {
namespace {

static const int16_t kDefaultLeftTurnDeg = 90;
static const int16_t kDefaultRightTurnDeg = -90;
static const int16_t kDefaultAlignDistanceMm = 0;
static const uint16_t kDefaultAlignRpm = 30U;
static const uint16_t kDefaultReacquireTimeoutMs = 800U;
static const int32_t kMaximumTurnDeg = 180;
static const int32_t kMaximumAlignDistanceMm = 300;
static const uint32_t kMaximumAlignRpm = 300U;
static const uint32_t kMaximumReacquireTimeoutMs = 5000U;

RoadControlState g_state = {};

int32_t AbsoluteInt32(int32_t value)
{
    return (value < 0) ? -value : value;
}

bool IsTrackingPositionReady(void)
{
    const AppGrayscaleData *data = App_GrayscaleGetData();
    const uint32_t now = services::Time_Millis();
    return (data != 0) && data->valid && data->processed_valid &&
           ((now - data->last_update_ms) <= 50U) &&
           data->line_detected && data->position_valid &&
           (data->track_state == drivers::GRAYSCALE_TRACK_VALID) &&
           (data->channel_anomaly_mask == 0U);
}

void FinishStopped(drivers::DriverStatus status)
{
    (void) Heading_Stop();
    (void) LF_Stop();
    g_state.phase = ROAD_CONTROL_PHASE_STOPPED;
    g_state.phase_start_ms = services::Time_Millis();
    g_state.last_status = status;
}

drivers::DriverStatus StartTurn(GrayscaleRoadType type)
{
    const int32_t angle = (type == GRAYSCALE_ROAD_LEFT_CORNER)
        ? g_state.config.left_turn_deg
        : g_state.config.right_turn_deg;
    const drivers::DriverStatus status = Heading_TurnStart(angle);
    if (status == drivers::DRIVER_OK) {
        g_state.phase = ROAD_CONTROL_PHASE_TURNING;
        g_state.phase_start_ms = services::Time_Millis();
    } else {
        FinishStopped(status);
    }
    g_state.last_status = status;
    return status;
}

drivers::DriverStatus StartCorner(GrayscaleRoadType type)
{
    const LFState *lf = LF_GetState();
    if ((lf == 0) || (lf->mode != LF_FOLLOW)) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }

    const uint32_t now = services::Time_Millis();
    g_state.saved_base_rpm = lf->base_rpm;
    if (lf->follow_duration_ms == 0U) {
        g_state.saved_duration_ms = 0U;
    } else {
        const uint32_t elapsed = now - lf->follow_start_ms;
        g_state.saved_duration_ms = (elapsed < lf->follow_duration_ms)
            ? (lf->follow_duration_ms - elapsed)
            : 1U;
    }

    drivers::DriverStatus status = LF_Stop();
    if (status != drivers::DRIVER_OK) {
        FinishStopped(status);
        return status;
    }

    if (g_state.config.align_distance_mm == 0) {
        return StartTurn(type);
    }

    int32_t align_rpm = static_cast<int32_t>(g_state.config.align_rpm);
    const int32_t base_magnitude = AbsoluteInt32(g_state.saved_base_rpm);
    if ((base_magnitude > 0) && (align_rpm > base_magnitude)) {
        align_rpm = base_magnitude;
    }
    if (align_rpm == 0) {
        align_rpm = 1;
    }
    status = Heading_DistanceStart(g_state.config.align_distance_mm,
                                   align_rpm,
                                   0U);
    if (status == drivers::DRIVER_OK) {
        g_state.phase = ROAD_CONTROL_PHASE_ALIGNING;
        g_state.phase_start_ms = now;
    } else {
        FinishStopped(status);
    }
    g_state.last_status = status;
    return status;
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
    g_state.handled_event_type = GRAYSCALE_ROAD_UNKNOWN;
    g_state.last_policy = ROAD_POLICY_DEFAULT;
    g_state.last_status = drivers::DRIVER_OK;
}

void RoadEventController_Update(void)
{
    const uint32_t now = services::Time_Millis();
    if (services::Fault_HasFault()) {
        if ((g_state.phase != ROAD_CONTROL_PHASE_IDLE) &&
            (g_state.phase != ROAD_CONTROL_PHASE_STOPPED)) {
            FinishStopped(drivers::DRIVER_ERROR_NOT_INITIALIZED);
        }
        return;
    }

    if (g_state.phase == ROAD_CONTROL_PHASE_ALIGNING) {
        const HeadingState *heading = Heading_GetState();
        if (heading->mode == HEADING_IDLE) {
            if (heading->last_status != drivers::DRIVER_OK) {
                FinishStopped(heading->last_status);
            } else {
                (void) StartTurn(g_state.handled_event_type);
            }
        }
        return;
    }

    if (g_state.phase == ROAD_CONTROL_PHASE_TURNING) {
        const HeadingState *heading = Heading_GetState();
        if (heading->mode == HEADING_IDLE) {
            if (heading->last_status != drivers::DRIVER_OK) {
                FinishStopped(heading->last_status);
            } else {
                g_state.phase = ROAD_CONTROL_PHASE_REACQUIRE;
                g_state.phase_start_ms = now;
            }
        }
        return;
    }

    if (g_state.phase == ROAD_CONTROL_PHASE_REACQUIRE) {
        if (IsTrackingPositionReady()) {
            const drivers::DriverStatus status =
                LF_Start(g_state.saved_base_rpm, g_state.saved_duration_ms);
            if (status == drivers::DRIVER_OK) {
                g_state.phase = ROAD_CONTROL_PHASE_IDLE;
                g_state.last_status = status;
                return;
            }
            if (status != drivers::DRIVER_ERROR_NOT_INITIALIZED) {
                FinishStopped(status);
                return;
            }
        }
        if ((now - g_state.phase_start_ms) >=
            g_state.config.reacquire_timeout_ms) {
            FinishStopped(drivers::DRIVER_ERROR_TIMEOUT);
        }
        return;
    }

    if ((g_state.phase == ROAD_CONTROL_PHASE_STOPPED) &&
        (LF_GetState()->mode == LF_FOLLOW)) {
        g_state.phase = ROAD_CONTROL_PHASE_IDLE;
    }

    const AppGrayscaleData *data = App_GrayscaleGetData();
    if ((data != 0) && (data->road_event_sequence != 0U) &&
        (data->road_event_sequence != g_state.handled_event_sequence)) {
        HandleNewEvent(data);
    }
}

drivers::DriverStatus RoadEventController_SetMode(RoadControlMode mode)
{
    if (!FEATURE_ENABLE_DIFFERENTIAL_CHASSIS) {
        (void) RoadEventController_Cancel();
        return drivers::DRIVER_ERROR_UNSUPPORTED;
    }
    if ((mode != ROAD_CONTROL_DETECT_ONLY) &&
        (mode != ROAD_CONTROL_AUTO_CORNERS)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if ((g_state.phase != ROAD_CONTROL_PHASE_IDLE) &&
        (g_state.phase != ROAD_CONTROL_PHASE_STOPPED)) {
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
    if (!FEATURE_ENABLE_DIFFERENTIAL_CHASSIS) {
        (void) RoadEventController_Cancel();
        return drivers::DRIVER_ERROR_UNSUPPORTED;
    }
    if ((left_turn_deg <= 0) || (left_turn_deg > kMaximumTurnDeg) ||
        (right_turn_deg >= 0) || (right_turn_deg < -kMaximumTurnDeg) ||
        (align_distance_mm < 0) ||
        (align_distance_mm > kMaximumAlignDistanceMm) ||
        (align_rpm == 0U) || (align_rpm > kMaximumAlignRpm) ||
        (reacquire_timeout_ms == 0U) ||
        (reacquire_timeout_ms > kMaximumReacquireTimeoutMs)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    g_state.config.left_turn_deg = static_cast<int16_t>(left_turn_deg);
    g_state.config.right_turn_deg = static_cast<int16_t>(right_turn_deg);
    g_state.config.align_distance_mm =
        static_cast<int16_t>(align_distance_mm);
    g_state.config.align_rpm = static_cast<uint16_t>(align_rpm);
    g_state.config.reacquire_timeout_ms =
        static_cast<uint16_t>(reacquire_timeout_ms);
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus RoadEventController_Cancel(void)
{
    (void) Heading_Stop();
    (void) LF_Stop();
    g_state.phase = ROAD_CONTROL_PHASE_IDLE;
    g_state.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
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
    case ROAD_CONTROL_PHASE_ALIGNING:
        return "aligning";
    case ROAD_CONTROL_PHASE_TURNING:
        return "turning";
    case ROAD_CONTROL_PHASE_REACQUIRE:
        return "reacquire";
    case ROAD_CONTROL_PHASE_STOPPED:
        return "stopped";
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
