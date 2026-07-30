#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "app/app_grayscale.h"
#include "app/chassis.h"
#include "app/line_sensor.h"
#include "app/linefollow.h"
#include "app/track_course.h"
#include "services/fault.h"
#include "services/time.h"

namespace {

uint32_t g_now = 100U;
bool g_fault = false;
app::ChassisState g_chassis = {};
app::LineSensorSnapshot g_line = {};
app::AppGrayscaleData g_gray = {};
app::LFState g_lf = {};
uint32_t g_stop_calls = 0U;

int32_t CountsForMillimeters(int32_t millimeters)
{
    const double circumference_mm =
        2.0 * 3.141593 * 33.05;
    return static_cast<int32_t>(
        lround(static_cast<double>(millimeters) * 1456.0 /
               circumference_mm));
}

void SetDistance(int32_t millimeters)
{
    const int32_t counts = CountsForMillimeters(millimeters);
    g_chassis.left.encoder_count = counts;
    g_chassis.right.encoder_count = counts;
    g_chassis.feedback_sequence++;
    g_chassis.last_feedback_ms = g_now;
}

void PublishMask(uint8_t mask)
{
    g_gray.active_mask = mask;
    g_gray.sequence++;
    g_gray.last_update_ms = g_now;
    g_line.sequence = g_gray.sequence;
    g_line.last_update_ms = g_now;
}

void Reset()
{
    g_now = 100U;
    g_fault = false;
    g_stop_calls = 0U;
    g_chassis = app::ChassisState();
    g_chassis.initialized = true;
    g_chassis.config.max_wheel_rpm = 500U;
    g_chassis.config.wheel_radius_um = 33050U;
    g_chassis.config.left_counts_per_rev = 1456U;
    g_chassis.config.right_counts_per_rev = 1456U;
    g_chassis.feedback_sequence = 1U;
    g_chassis.last_feedback_ms = g_now;
    g_chassis.last_feedback_status = drivers::DRIVER_OK;

    g_line = app::LineSensorSnapshot();
    g_line.source = app::LINE_SENSOR_ADC8;
    g_line.valid = true;
    g_line.fresh = true;
    g_line.calibrated = true;
    g_line.line_detected = true;
    g_line.position_valid = true;
    g_line.track_state = drivers::GRAYSCALE_TRACK_VALID;
    g_line.last_status = drivers::DRIVER_OK;

    g_gray = app::AppGrayscaleData();
    g_gray.valid = true;
    g_gray.processed_valid = true;
    g_gray.position_valid = true;
    g_gray.line_detected = true;
    g_gray.processing_status = drivers::DRIVER_OK;
    g_gray.sequence = 1U;
    g_gray.last_update_ms = g_now;

    g_lf = app::LFState();
    g_lf.mode = app::LF_IDLE;
    app::TrackCourse_Init();
}

void Start()
{
    assert(app::TrackCourse_Start(110U, 60U, 6142U) ==
           drivers::DRIVER_OK);
    assert(app::TrackCourse_GetState()->result ==
           app::TRACK_COURSE_RESULT_RUNNING);
    assert(g_lf.mode == app::LF_FOLLOW);
    assert(g_lf.base_rpm == 110);
}

} /* namespace */

namespace services {

uint32_t Time_Millis(void) { return g_now; }
bool Fault_HasFault(void) { return g_fault; }

} /* namespace services */

namespace app {

const ChassisState *Chassis_GetState(void) { return &g_chassis; }
const LineSensorSnapshot *LineSensor_GetSnapshot(void) { return &g_line; }
LineSensorSource LineSensor_GetSource(void) { return g_line.source; }
bool LineSensor_IsReadyForMotion(void)
{
    return g_line.valid && g_line.fresh && g_line.calibrated &&
           g_line.line_detected && g_line.position_valid;
}
const AppGrayscaleData *App_GrayscaleGetData(void) { return &g_gray; }
const LFState *LF_GetState(void) { return &g_lf; }

drivers::DriverStatus LF_Start(int32_t base_rpm, uint32_t duration_ms)
{
    g_lf.mode = LF_FOLLOW;
    g_lf.base_rpm = base_rpm;
    g_lf.follow_duration_ms = duration_ms;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus LF_ContinueForMotionHandoff(
    int32_t base_rpm,
    uint32_t duration_ms)
{
    if (g_lf.mode != LF_FOLLOW) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    g_lf.base_rpm = base_rpm;
    g_lf.follow_duration_ms = duration_ms;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus LF_Stop(void)
{
    g_stop_calls++;
    g_lf.mode = LF_IDLE;
    return drivers::DRIVER_OK;
}

} /* namespace app */

int main()
{
    using namespace app;

    /* The competition controller can reject an invalid launch before it
     * opens the timed OLED page, without mutating the course state. */
    Reset();
    assert(TrackCourse_CheckStartReady(110U, 60U, 6142U) ==
           drivers::DRIVER_OK);
    g_line.line_detected = false;
    g_line.position_valid = false;
    assert(TrackCourse_CheckStartReady(110U, 60U, 6142U) ==
           drivers::DRIVER_ERROR_NOT_INITIALIZED);
    assert(TrackCourse_GetState()->result == TRACK_COURSE_RESULT_IDLE);

    /* A start-line pattern cannot finish the task before the distance gate. */
    Reset();
    Start();
    SetDistance(4800);
    PublishMask(0x7EU);
    TrackCourse_Update();
    PublishMask(0x7EU);
    TrackCourse_Update();
    assert(TrackCourse_GetState()->result == TRACK_COURSE_RESULT_RUNNING);
    assert(TrackCourse_GetState()->finish_count == 0U);

    /* Center-marker evidence is still diagnosed, but it cannot finish the
     * temporary encoder-only task before the configured lap distance. */
    const uint8_t finish_masks[] = { 0x3EU, 0x7CU, 0x7EU };
    for (uint8_t mask : finish_masks) {
        Reset();
        Start();
        SetDistance(5000);
        PublishMask(mask);
        TrackCourse_Update();
        assert(TrackCourse_GetState()->finish_count == 1U);
        PublishMask(mask);
        TrackCourse_Update();
        assert(TrackCourse_GetState()->result ==
               TRACK_COURSE_RESULT_RUNNING);
        assert(TrackCourse_GetState()->completion ==
               TRACK_COURSE_COMPLETION_NONE);
        assert(TrackCourse_GetState()->finish_count == 2U);
        assert(TrackCourse_GetState()->finish_mask ==
               static_cast<uint8_t>(mask & 0x7EU));
        assert(g_stop_calls == 0U);
    }

    /* Five separated center hits do not satisfy the contiguous-run rule. */
    Reset();
    Start();
    SetDistance(5000);
    PublishMask(0x5EU);
    TrackCourse_Update();
    PublishMask(0x5EU);
    TrackCourse_Update();
    assert(TrackCourse_GetState()->result == TRACK_COURSE_RESULT_RUNNING);
    assert(TrackCourse_GetState()->finish_count == 0U);

    /* Approach speed changes without a stop; reaching lap distance is the
     * sole normal completion trigger. */
    Reset();
    Start();
    SetDistance(5250);
    PublishMask(0x00U);
    TrackCourse_Update();
    assert(TrackCourse_GetState()->phase == TRACK_COURSE_PHASE_APPROACH);
    assert(g_lf.base_rpm == 60);
    assert(g_stop_calls == 0U);
    SetDistance(6150);
    PublishMask(0x00U);
    TrackCourse_Update();
    assert(TrackCourse_GetState()->result == TRACK_COURSE_RESULT_SUCCESS);
    assert(TrackCourse_GetState()->completion ==
           TRACK_COURSE_COMPLETION_ENCODER);

    /* Encoder completion has priority even if a confirmed center marker is
     * present on the same controller update. */
    Reset();
    Start();
    SetDistance(5000);
    PublishMask(0x7EU);
    TrackCourse_Update();
    PublishMask(0x7EU);
    TrackCourse_Update();
    assert(TrackCourse_GetState()->result == TRACK_COURSE_RESULT_RUNNING);
    SetDistance(6150);
    PublishMask(0x7EU);
    TrackCourse_Update();
    assert(TrackCourse_GetState()->result == TRACK_COURSE_RESULT_SUCCESS);
    assert(TrackCourse_GetState()->completion ==
           TRACK_COURSE_COMPLETION_ENCODER);
    assert(g_stop_calls == 1U);

    /* Stale encoder feedback and excessive line recovery are fail-safe. */
    Reset();
    Start();
    g_now += 101U;
    PublishMask(0x00U);
    TrackCourse_Update();
    assert(TrackCourse_GetState()->failure ==
           TRACK_COURSE_FAILURE_ENCODER_STALE);

    Reset();
    Start();
    g_lf.recovery_mode = LF_RECOVERY_HOLD;
    g_lf.recovery_elapsed_ms = 501U;
    PublishMask(0x00U);
    TrackCourse_Update();
    assert(TrackCourse_GetState()->failure ==
           TRACK_COURSE_FAILURE_RECOVERY_TIMEOUT);

    printf("track course ok: marker diagnostics, encoder-only finish, "
           "approach handoff, fail-safe stops\n");
    return 0;
}
