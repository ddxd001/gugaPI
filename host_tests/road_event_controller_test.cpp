#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/app_grayscale.h"
#include "app/config_store.h"
#include "app/heading.h"
#include "app/linefollow.h"
#include "app/road_event_controller.h"
#include "services/fault.h"
#include "services/time.h"

namespace {

uint32_t g_now_ms = 0U;
bool g_fault = false;
app::AppGrayscaleData g_gray = {};
app::ConfigStoreParams g_params = {};
app::LFState g_lf = {};
app::HeadingState g_heading = {};
uint32_t g_lf_stop_calls = 0U;
uint32_t g_heading_stop_calls = 0U;
uint32_t g_lf_release_calls = 0U;
uint32_t g_heading_release_calls = 0U;
uint32_t g_lf_start_calls = 0U;
uint32_t g_arc_turn_calls = 0U;
int32_t g_last_turn_deg = 0;
int32_t g_last_arc_base_rpm = 0;
uint32_t g_distance_calls = 0U;
int32_t g_last_distance_mm = 0;
int32_t g_last_distance_rpm = 0;
int32_t g_last_initial_rpm = 0;

void ResetHarness(int32_t align_mm, uint16_t road_rpm)
{
    g_now_ms = 0U;
    g_fault = false;
    g_gray = {};
    g_lf = {};
    g_heading = {};
    g_heading.mode = app::HEADING_IDLE;
    g_heading.last_status = drivers::DRIVER_OK;
    g_lf_stop_calls = 0U;
    g_heading_stop_calls = 0U;
    g_lf_release_calls = 0U;
    g_heading_release_calls = 0U;
    g_lf_start_calls = 0U;
    g_arc_turn_calls = 0U;
    g_last_turn_deg = 0;
    g_last_arc_base_rpm = 0;
    g_distance_calls = 0U;
    g_last_distance_mm = 0;
    g_last_distance_rpm = 0;
    g_last_initial_rpm = 0;
    g_params.road_align_distance_mm = static_cast<uint16_t>(align_mm);
    g_params.road_align_rpm = road_rpm;
    app::RoadEventController_Init();
    assert(app::RoadEventController_SetMode(
        app::ROAD_CONTROL_AUTO_CORNERS) == drivers::DRIVER_OK);
}

void PrepareFollow(uint32_t start_ms, int32_t base_rpm)
{
    g_lf = {};
    g_lf.mode = app::LF_FOLLOW;
    g_lf.base_rpm = base_rpm;
    g_lf.follow_start_ms = start_ms;
    g_lf.follow_duration_ms = 0U;
    g_lf.last_status = drivers::DRIVER_OK;
}

void PublishCorner(uint32_t sequence, app::GrayscaleRoadType type)
{
    g_gray.road_event_sequence = sequence;
    g_gray.road_event_type = type;
}

void PublishTrackingFrame(uint32_t sequence, bool valid)
{
    g_gray.sequence = sequence;
    g_gray.last_update_ms = g_now_ms;
    g_gray.valid = valid;
    g_gray.processed_valid = valid;
    g_gray.line_detected = valid;
    g_gray.position_valid = valid;
    g_gray.track_state = valid
        ? drivers::GRAYSCALE_TRACK_VALID
        : drivers::GRAYSCALE_TRACK_LOST;
    g_gray.channel_anomaly_mask = 0U;
}

} /* namespace */

namespace services {

uint32_t Time_Millis(void)
{
    return g_now_ms;
}

bool Fault_HasFault(void)
{
    return g_fault;
}

} /* namespace services */

namespace app {

const AppGrayscaleData *App_GrayscaleGetData(void)
{
    return &g_gray;
}

void App_GrayscaleClearRoadEvent(void)
{
    g_gray.road_event_sequence = 0U;
    g_gray.road_event_type = GRAYSCALE_ROAD_UNKNOWN;
}

const ConfigStoreParams *ConfigStore_Get(void)
{
    return &g_params;
}

drivers::DriverStatus ConfigStore_Set(const char *name, int32_t value)
{
    if (name == 0) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (strcmp(name, "road_align_distance_mm") == 0) {
        if ((value < 0) || (value > 300)) {
            return drivers::DRIVER_ERROR_INVALID_ARG;
        }
        g_params.road_align_distance_mm = static_cast<uint16_t>(value);
        return drivers::DRIVER_OK;
    }
    if (strcmp(name, "road_align_rpm") == 0) {
        if ((value < 1) || (value > 300)) {
            return drivers::DRIVER_ERROR_INVALID_ARG;
        }
        g_params.road_align_rpm = static_cast<uint16_t>(value);
        return drivers::DRIVER_OK;
    }
    return drivers::DRIVER_ERROR_INVALID_ARG;
}

const LFState *LF_GetState(void)
{
    return &g_lf;
}

drivers::DriverStatus LF_Stop(void)
{
    g_lf_stop_calls++;
    g_lf.mode = LF_IDLE;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus LF_ReleaseForMotionHandoff(void)
{
    if (g_lf.mode != LF_FOLLOW) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    g_lf_release_calls++;
    g_lf.mode = LF_IDLE;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus LF_Start(int32_t base_rpm, uint32_t duration_ms)
{
    g_lf_start_calls++;
    g_lf.mode = LF_FOLLOW;
    g_lf.base_rpm = base_rpm;
    g_lf.follow_duration_ms = duration_ms;
    return drivers::DRIVER_OK;
}

const HeadingState *Heading_GetState(void)
{
    return &g_heading;
}

drivers::DriverStatus Heading_Stop(void)
{
    g_heading_stop_calls++;
    g_heading.mode = HEADING_IDLE;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Heading_ArcTurnStart(int32_t delta_deg,
                                           int32_t base_rpm)
{
    g_arc_turn_calls++;
    g_last_turn_deg = delta_deg;
    g_last_arc_base_rpm = base_rpm;
    g_heading.mode = HEADING_ARC_TURN;
    g_heading.at_target = false;
    g_heading.error_mdeg = delta_deg * 1000;
    g_heading.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Heading_DistanceStartForRollingHandoff(
    int32_t distance_mm,
    int32_t max_rpm,
    uint32_t,
    int32_t initial_rpm)
{
    g_distance_calls++;
    g_last_distance_mm = distance_mm;
    g_last_distance_rpm = max_rpm;
    g_last_initial_rpm = initial_rpm;
    g_heading.mode = HEADING_DISTANCE;
    g_heading.last_status = drivers::DRIVER_OK;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Heading_ReleaseForMotionHandoff(void)
{
    if ((g_heading.mode != HEADING_ARC_TURN) || (!g_heading.at_target)) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    g_heading_release_calls++;
    g_heading.mode = HEADING_IDLE;
    return drivers::DRIVER_OK;
}

} /* namespace app */

int main(void)
{
    using namespace app;

    /* Nonzero alignment, arc turn and reacquisition never send a stop. */
    ResetHarness(20, 35U);
    PrepareFollow(0U, 40);
    PublishCorner(1U, GRAYSCALE_ROAD_LEFT_CORNER);
    g_now_ms = 100U;
    RoadEventController_Update();
    assert(g_lf_release_calls == 1U);
    assert(g_lf_stop_calls == 0U);
    assert(g_heading_stop_calls == 0U);
    assert(g_distance_calls == 1U);
    assert(g_last_distance_mm == 20);
    assert(g_last_distance_rpm == 35);
    assert(g_last_initial_rpm == 35);
    assert(RoadEventController_GetState()->phase ==
           ROAD_CONTROL_PHASE_ALIGNING);

    g_heading.mode = HEADING_IDLE;
    g_heading.last_status = drivers::DRIVER_OK;
    g_now_ms = 200U;
    RoadEventController_Update();
    assert(g_arc_turn_calls == 1U);
    assert(g_last_turn_deg == 90);
    assert(g_last_arc_base_rpm == 35);
    assert(RoadEventController_GetState()->phase ==
           ROAD_CONTROL_PHASE_TURNING);

    /* Two valid frames in the final 20-degree window are retained while the
     * arc is still active, so target arrival hands directly to LF_FOLLOW. */
    g_heading.error_mdeg = 15000;
    g_now_ms = 210U;
    PublishTrackingFrame(10U, true);
    RoadEventController_Update();
    assert(RoadEventController_GetState()->reacquire_valid_frames == 1U);
    assert(g_lf_start_calls == 0U);

    g_now_ms = 220U;
    PublishTrackingFrame(11U, true);
    RoadEventController_Update();
    assert(RoadEventController_GetState()->reacquire_valid_frames == 2U);
    assert(g_lf_start_calls == 0U);

    g_heading.error_mdeg = 2000;
    g_heading.at_target = true;
    g_now_ms = 230U;
    RoadEventController_Update();
    assert(g_lf_start_calls == 1U);
    assert(g_heading_release_calls == 1U);
    assert(RoadEventController_GetState()->phase ==
           ROAD_CONTROL_PHASE_IDLE);
    assert(g_lf_stop_calls == 0U);
    assert(g_heading_stop_calls == 0U);

    /* A zero alignment distance enters the arc directly and keeps direction. */
    ResetHarness(0, 30U);
    PrepareFollow(0U, -40);
    PublishCorner(2U, GRAYSCALE_ROAD_RIGHT_CORNER);
    g_now_ms = 100U;
    RoadEventController_Update();
    assert(g_distance_calls == 0U);
    assert(g_arc_turn_calls == 1U);
    assert(g_last_turn_deg == -90);
    assert(g_last_arc_base_rpm == -30);
    assert(g_lf_stop_calls == 0U);
    assert(g_heading_stop_calls == 0U);

    /* An invalid new frame resets the two-frame reacquisition gate. */
    PublishTrackingFrame(20U, false);
    g_heading.error_mdeg = 2000;
    g_heading.at_target = true;
    g_now_ms = 110U;
    RoadEventController_Update();
    assert(RoadEventController_GetState()->phase ==
           ROAD_CONTROL_PHASE_REACQUIRE);
    g_now_ms = 120U;
    PublishTrackingFrame(21U, true);
    RoadEventController_Update();
    assert(RoadEventController_GetState()->reacquire_valid_frames == 1U);
    g_now_ms = 130U;
    PublishTrackingFrame(22U, false);
    RoadEventController_Update();
    assert(RoadEventController_GetState()->reacquire_valid_frames == 0U);
    g_now_ms = 140U;
    PublishTrackingFrame(23U, true);
    RoadEventController_Update();
    g_now_ms = 150U;
    PublishTrackingFrame(24U, true);
    RoadEventController_Update();
    assert(g_lf_start_calls == 1U);
    assert(g_heading_release_calls == 1U);
    assert(g_lf_stop_calls == 0U);

    /* Reacquisition timeout and an active fault retain the safe-stop path. */
    ResetHarness(0, 30U);
    PrepareFollow(0U, 40);
    PublishCorner(3U, GRAYSCALE_ROAD_LEFT_CORNER);
    g_now_ms = 100U;
    RoadEventController_Update();
    PublishTrackingFrame(30U, false);
    g_heading.at_target = true;
    g_now_ms = 110U;
    RoadEventController_Update();
    g_now_ms = 910U;
    RoadEventController_Update();
    assert(RoadEventController_GetState()->phase ==
           ROAD_CONTROL_PHASE_STOPPED);
    assert(g_lf_stop_calls == 1U);
    assert(g_heading_stop_calls == 1U);

    ResetHarness(20, 35U);
    PrepareFollow(0U, 40);
    PublishCorner(4U, GRAYSCALE_ROAD_LEFT_CORNER);
    g_now_ms = 100U;
    RoadEventController_Update();
    g_fault = true;
    g_now_ms = 110U;
    RoadEventController_Update();
    assert(RoadEventController_GetState()->phase ==
           ROAD_CONTROL_PHASE_STOPPED);
    assert(g_lf_stop_calls == 1U);
    assert(g_heading_stop_calls == 1U);

    ResetHarness(0, 30U);
    PrepareFollow(0U, 40);
    PublishCorner(5U, GRAYSCALE_ROAD_LEFT_CORNER);
    g_now_ms = 100U;
    RoadEventController_Update();
    assert(RoadEventController_Cancel() == drivers::DRIVER_OK);
    assert(RoadEventController_GetState()->phase ==
           ROAD_CONTROL_PHASE_IDLE);
    assert(g_lf_stop_calls == 1U);
    assert(g_heading_stop_calls == 1U);

    /* A corner event arriving after LF has already stopped is visible as an
     * explicit failed/stopped handoff instead of phase=idle,last=ok. */
    ResetHarness(20, 35U);
    g_lf.mode = LF_IDLE;
    PublishCorner(6U, GRAYSCALE_ROAD_LEFT_CORNER);
    g_now_ms = 100U;
    RoadEventController_Update();
    assert(RoadEventController_GetState()->phase ==
           ROAD_CONTROL_PHASE_STOPPED);
    assert(RoadEventController_GetState()->last_status ==
           drivers::DRIVER_ERROR_NOT_INITIALIZED);
    assert(g_lf_stop_calls == 1U);
    assert(g_heading_stop_calls == 1U);

    assert(RoadEventController_SetAlignConfig(301, 30U) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(RoadEventController_SetAlignConfig(20, 0U) ==
           drivers::DRIVER_ERROR_INVALID_ARG);

    puts("road event controller rolling corner ok");
    return 0;
}
