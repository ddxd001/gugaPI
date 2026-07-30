#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/action.h"
#include "app/app_grayscale.h"
#include "app/app_imu.h"
#include "app/app_main.h"
#include "app/ball_balance.h"
#include "app/chassis.h"
#include "app/dm_g6220_controller.h"
#include "app/heading.h"
#include "app/linefollow.h"
#include "app/line_sensor.h"
#include "app/road_event_controller.h"
#include "app/seq_store.h"
#include "app/track_course.h"
#include "board/board_button.h"
#include "board/board_buzzer.h"
#include "board/board_fram.h"
#include "board/board_led.h"
#include "services/fault.h"
#include "services/time.h"

namespace {
uint32_t g_now = 100U;
bool g_fault = false;
bool g_line = false;
bool g_button = false;
bool g_buttons[board::BOARD_BUTTON_COUNT] = {};
bool g_led[board::BOARD_LED_ID_COUNT] = {};
uint16_t g_ledToggleCount[board::BOARD_LED_ID_COUNT] = {};
bool g_buzzer = false;
uint8_t g_fram[8192] = {};
app::ChassisState g_chassis = {};
app::HeadingState g_heading = {};
app::LFState g_lf = {};
app::AppGrayscaleData g_gray = {};
app::LineSensorSnapshot g_line_sensor = {};
app::AppImuData g_imu = {};
app::AppState g_app = {};
app::RoadControlState g_road = {};
uint32_t g_road_start_calls = 0U;
uint32_t g_road_cancel_calls = 0U;
app::DmG6220ControlState g_dm = {};
app::BallBalanceState g_ball = {};
app::TrackCourseState g_course = {};
uint32_t g_dm_emergency_disable_calls = 0U;
uint32_t g_ball_emergency_stop_calls = 0U;
uint32_t g_ball_stop_calls = 0U;
uint32_t g_dm_stop_speed_calls = 0U;

void Reset()
{
    g_now = 100U;
    g_fault = false;
    g_line = false;
    g_button = false;
    memset(g_buttons, 0, sizeof(g_buttons));
    memset(g_ledToggleCount, 0, sizeof(g_ledToggleCount));
    g_led[0] = g_led[1] = g_led[2] = false;
    g_buzzer = false;
    g_chassis = app::ChassisState();
    g_chassis.initialized = true;
    g_chassis.config.max_wheel_rpm = 500U;
    g_chassis.config.wheel_radius_um = 33050U;
    g_chassis.config.left_counts_per_rev = 1456U;
    g_chassis.config.right_counts_per_rev = 1456U;
    g_chassis.feedback_sequence = 1U;
    g_chassis.last_feedback_ms = g_now;
    g_chassis.last_feedback_status = drivers::DRIVER_OK;
    g_heading = app::HeadingState();
    g_heading.mode = app::HEADING_IDLE;
    g_lf = app::LFState();
    g_lf.mode = app::LF_IDLE;
    g_gray = app::AppGrayscaleData();
    g_gray.valid = true;
    g_gray.processed_valid = true;
    g_gray.position_valid = true;
    g_gray.last_update_ms = g_now;
    g_line_sensor = app::LineSensorSnapshot();
    g_line_sensor.source = app::LINE_SENSOR_ADC8;
    g_line_sensor.valid = true;
    g_line_sensor.fresh = true;
    g_line_sensor.calibrated = true;
    g_imu = app::AppImuData();
    g_imu.valid = true;
    g_imu.last_update_ms = g_now;
    g_app.mode = app::APP_MODE_RUNNING;
    g_road = app::RoadControlState();
    g_road.route_result = app::ROAD_ROUTE_RESULT_IDLE;
    g_road_start_calls = 0U;
    g_road_cancel_calls = 0U;
    g_dm = app::DmG6220ControlState();
    g_dm.initialized = true;
    g_dm.mode = app::DM_CONTROL_READY;
    g_dm.operation_result = app::DM_OPERATION_IDLE;
    g_ball = app::BallBalanceState();
    g_ball.mode = app::BALL_BALANCE_IDLE;
    g_ball.result = app::BALL_BALANCE_RESULT_IDLE;
    g_course = app::TrackCourseState();
    g_course.result = app::TRACK_COURSE_RESULT_IDLE;
    g_dm_emergency_disable_calls = 0U;
    g_ball_stop_calls = 0U;
    g_dm_stop_speed_calls = 0U;
    app::ActionRunner_Init();
}

void AddEnd()
{
    assert(app::ActionRunner_AddInstr(app::ACT_OP_END, 0, 0,
                                      app::ACT_COND_IMMEDIATE,
                                      app::ACT_NEXT, app::ACT_NEXT) ==
           drivers::DRIVER_OK);
}

void RunUntilStopped(uint16_t max_updates)
{
    for (uint16_t i = 0U;
         (i < max_updates) && app::ActionRunner_GetState()->running;
         i++) {
        app::ActionRunner_Update();
    }
    assert(!app::ActionRunner_GetState()->running);
}

} /* namespace */

namespace services {
uint32_t Time_Millis(void) { return g_now; }
bool Time_HasElapsed(uint32_t start, uint32_t interval)
{
    return static_cast<uint32_t>(g_now - start) >= interval;
}
bool Fault_HasFault(void) { return g_fault; }
} /* namespace services */

namespace app {
const AppState *App_GetState(void) { return &g_app; }
const AppGrayscaleData *App_GrayscaleGetData(void) { return &g_gray; }
const LineSensorSnapshot *LineSensor_GetSnapshot(void)
{
    g_line_sensor.line_detected = g_line;
    g_line_sensor.position_valid = g_line;
    g_line_sensor.road_event_sequence = g_gray.road_event_sequence;
    g_line_sensor.road_event_type = g_gray.road_event_type;
    g_line_sensor.road_event_paths = g_gray.road_event_paths;
    return &g_line_sensor;
}
const AppImuData *App_ImuGetData(void) { return &g_imu; }
const ChassisState *Chassis_GetState(void) { return &g_chassis; }
drivers::DriverStatus Chassis_Stop(void)
{
    g_chassis.left.target_rpm = 0;
    g_chassis.right.target_rpm = 0;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus Heading_Stop(void)
{
    g_heading.mode = HEADING_IDLE;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus Heading_HoldStart(int32_t rpm)
{
    g_heading.mode = HEADING_HOLD;
    g_heading.base_rpm = rpm;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus Heading_TurnStart(int32_t angle)
{
    (void) angle;
    g_heading.mode = HEADING_TURN;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus Heading_DistanceStart(int32_t mm, int32_t rpm,
                                            uint32_t timeout)
{
    (void) mm;
    (void) rpm;
    (void) timeout;
    g_heading.mode = HEADING_DISTANCE;
    return drivers::DRIVER_OK;
}
const HeadingState *Heading_GetState(void) { return &g_heading; }
drivers::DriverStatus LF_Start(int32_t rpm, uint32_t duration)
{
    g_lf.mode = LF_FOLLOW;
    g_lf.base_rpm = rpm;
    g_lf.follow_duration_ms = duration;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus LF_Stop(void)
{
    g_lf.mode = LF_IDLE;
    return drivers::DRIVER_OK;
}
const LFState *LF_GetState(void) { return &g_lf; }
bool LF_IsLineDetected(void) { return g_line; }
drivers::DriverStatus RoadEventController_StartRoute(
    RoadRoute route,
    int32_t rpm,
    uint32_t timeout_ms)
{
    (void) timeout_ms;
    g_road_start_calls++;
    g_road.route = route;
    g_road.route_active = true;
    g_road.route_result = ROAD_ROUTE_RESULT_RUNNING;
    g_lf.mode = LF_FOLLOW;
    g_lf.base_rpm = rpm;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus RoadEventController_Cancel(void)
{
    g_road_cancel_calls++;
    g_road.route_active = false;
    if (g_road.route_result == ROAD_ROUTE_RESULT_RUNNING) {
        g_road.route_result = ROAD_ROUTE_RESULT_CANCELLED;
    }
    (void) Heading_Stop();
    (void) LF_Stop();
    return Chassis_Stop();
}
const RoadControlState *RoadEventController_GetState(void)
{
    return &g_road;
}
drivers::DriverStatus DmG6220Controller_StartPosition(
    DmG6220PositionFrame frame,
    int32_t target_mrad,
    int32_t max_velocity_mrad_s,
    uint32_t timeout_ms)
{
    (void) frame;
    g_dm.mode = DM_CONTROL_POSITION;
    g_dm.operation_result = DM_OPERATION_RUNNING;
    g_dm.target_position_mrad = target_mrad;
    g_dm.reference_velocity_mrad_s = max_velocity_mrad_s;
    g_dm.operation_timeout_ms = timeout_ms;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus DmG6220Controller_StartSpeed(int32_t velocity_mrad_s)
{
    g_dm.mode = DM_CONTROL_SPEED;
    g_dm.operation_result = DM_OPERATION_RUNNING;
    g_dm.target_velocity_mrad_s = velocity_mrad_s;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus DmG6220Controller_StopSpeedAndHold(void)
{
    g_dm_stop_speed_calls++;
    g_dm.mode = DM_CONTROL_SPEED_STOPPING;
    g_dm.operation_result = DM_OPERATION_RUNNING;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus DmG6220Controller_Disable(void)
{
    g_dm.mode = DM_CONTROL_DISABLING;
    g_dm.operation_result = DM_OPERATION_RUNNING;
    return drivers::DRIVER_OK;
}
void DmG6220Controller_EmergencyDisable(void)
{
    g_dm_emergency_disable_calls++;
    g_dm.mode = DM_CONTROL_DISABLING;
}
const DmG6220ControlState *DmG6220Controller_GetState(void)
{
    return &g_dm;
}
drivers::DriverStatus BallBalance_StartHold(int16_t target_0p1mm)
{
    g_ball.mode = BALL_BALANCE_HOLD;
    g_ball.result = BALL_BALANCE_RESULT_SUCCESS;
    g_ball.target_position_0p1mm = target_0p1mm;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus BallBalance_StartMove(int16_t target_0p1mm,
                                            uint32_t timeout_ms)
{
    g_ball.mode = BALL_BALANCE_MOVE;
    g_ball.result = BALL_BALANCE_RESULT_RUNNING;
    g_ball.target_position_0p1mm = target_0p1mm;
    g_ball.timeout_ms = timeout_ms;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus BallBalance_Stop(bool disable)
{
    (void) disable;
    g_ball_stop_calls++;
    g_ball.mode = BALL_BALANCE_IDLE;
    g_ball.result = BALL_BALANCE_RESULT_IDLE;
    return drivers::DRIVER_OK;
}
void BallBalance_EmergencyStop(void)
{
    g_ball_emergency_stop_calls++;
    g_ball.mode = BALL_BALANCE_IDLE;
    g_ball.result = BALL_BALANCE_RESULT_IDLE;
}
const BallBalanceState *BallBalance_GetState(void)
{
    return &g_ball;
}
drivers::DriverStatus TrackCourse_Start(uint32_t cruise_rpm,
                                        uint32_t approach_rpm,
                                        uint32_t lap_distance_mm)
{
    g_course.result = TRACK_COURSE_RESULT_RUNNING;
    g_course.cruise_rpm = static_cast<uint16_t>(cruise_rpm);
    g_course.approach_rpm = static_cast<uint16_t>(approach_rpm);
    g_course.lap_distance_mm = static_cast<uint16_t>(lap_distance_mm);
    return drivers::DRIVER_OK;
}
drivers::DriverStatus TrackCourse_Cancel(void)
{
    if (g_course.result == TRACK_COURSE_RESULT_RUNNING) {
        g_course.result = TRACK_COURSE_RESULT_CANCELLED;
    }
    return drivers::DRIVER_OK;
}
const TrackCourseState *TrackCourse_GetState(void)
{
    return &g_course;
}
} /* namespace app */

namespace board {
bool Board_ButtonIsReady(BoardButtonId id)
{
    return id < BOARD_BUTTON_COUNT;
}
bool Board_ButtonIsPressed(BoardButtonId id)
{
    return (id < BOARD_BUTTON_COUNT) && g_buttons[id];
}
uint32_t Board_ButtonTakeEvents(BoardButtonId id, uint32_t event_mask)
{
    if ((id >= BOARD_BUTTON_COUNT) ||
        ((event_mask & drivers::BUTTON_EVENT_PRESSED) == 0U) ||
        !g_buttons[id]) {
        return 0U;
    }
    g_buttons[id] = false;
    return drivers::BUTTON_EVENT_PRESSED;
}
bool Board_ButtonWasPressed(BoardButtonId id)
{
    (void) id;
    const bool pressed = g_button;
    g_button = false;
    return pressed;
}
bool Board_LedIsReady(BoardLedId id) { return id < BOARD_LED_ID_COUNT; }
bool Board_LedIsOn(BoardLedId id) { return g_led[id]; }
drivers::DriverStatus Board_LedOn(BoardLedId id)
{
    g_led[id] = true;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus Board_LedOff(BoardLedId id)
{
    g_led[id] = false;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus Board_LedToggle(BoardLedId id)
{
    g_led[id] = !g_led[id];
    g_ledToggleCount[id]++;
    return drivers::DRIVER_OK;
}
bool Board_BuzzerIsReady(void) { return true; }
bool Board_BuzzerIsOn(void) { return g_buzzer; }
drivers::DriverStatus Board_BuzzerOn(void)
{
    g_buzzer = true;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus Board_BuzzerOff(void)
{
    g_buzzer = false;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus Board_BuzzerToggle(void)
{
    g_buzzer = !g_buzzer;
    return drivers::DRIVER_OK;
}
drivers::DriverStatus Board_FramRead(uint16_t address, uint8_t *data,
                                     uint16_t length)
{
    if (static_cast<uint32_t>(address) + length > sizeof(g_fram)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    memcpy(data, &g_fram[address], length);
    return drivers::DRIVER_OK;
}
drivers::DriverStatus Board_FramWrite(uint16_t address, const uint8_t *data,
                                      uint16_t length)
{
    if (static_cast<uint32_t>(address) + length > sizeof(g_fram)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    memcpy(&g_fram[address], data, length);
    return drivers::DRIVER_OK;
}
drivers::DriverStatus Board_FramWriteByte(uint16_t address, uint8_t data)
{
    if (address >= sizeof(g_fram)) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    g_fram[address] = data;
    return drivers::DRIVER_OK;
}
} /* namespace board */

int main()
{
    using namespace app;
    Reset();

    struct ValidCase {
        ActionOp op;
        int32_t p1;
        int32_t p2;
        ActionCond cond;
    } cases[] = {
        { ACT_OP_DRIVE, -80, 1000, ACT_COND_TIMEOUT },
        { ACT_OP_TURN, -180, 3000, ACT_COND_HEADING_REACHED },
        { ACT_OP_FOLLOW, 80, 5000, ACT_COND_LINE_LOST },
        { ACT_OP_WAIT, 0, 10, ACT_COND_TIMEOUT },
        { ACT_OP_STOP, 0, 0, ACT_COND_IMMEDIATE },
        { ACT_OP_BRANCH, 0, 0, ACT_COND_BUTTON },
        { ACT_OP_END, 0, 0, ACT_COND_IMMEDIATE },
        { ACT_OP_DRIVE_MM, -10000, 500, ACT_COND_DISTANCE_REACHED },
        { ACT_OP_LED_ON, 0, 50, ACT_COND_IMMEDIATE },
        { ACT_OP_LED_OFF, 2, 0, ACT_COND_IMMEDIATE },
        { ACT_OP_LED_TOGGLE, 3, 0, ACT_COND_IMMEDIATE },
        { ACT_OP_BUZZER_ON, 0, 50, ACT_COND_IMMEDIATE },
        { ACT_OP_BUZZER_OFF, 0, 0, ACT_COND_IMMEDIATE },
        { ACT_OP_BUZZER_TOGGLE, 0, 0, ACT_COND_IMMEDIATE },
        { ACT_OP_BALL_HOLD, 0, 0, ACT_COND_IMMEDIATE },
        { ACT_OP_BALL_MOVE, 500, 5000, ACT_COND_IMMEDIATE },
        { ACT_OP_BALL_DISABLE, 0, 0, ACT_COND_IMMEDIATE }
    };
    for (uint32_t i = 0U; i < sizeof(cases) / sizeof(cases[0]); i++) {
        assert(ActionRunner_Clear() == drivers::DRIVER_OK);
        assert(ActionRunner_AddInstr(cases[i].op, cases[i].p1, cases[i].p2,
                                     cases[i].cond, ACT_NEXT, ACT_NEXT) ==
               drivers::DRIVER_OK);
        ActionValidationResult validation;
        assert(ActionRunner_Validate(&validation) == drivers::DRIVER_OK);
        assert(validation.valid);
    }

    assert(ActionRunner_Clear() == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_TURN, 181, 1000,
                                 ACT_COND_HEADING_REACHED,
                                 ACT_NEXT, ACT_NEXT) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_AddInstr(ACT_OP_WAIT, 1, 10, ACT_COND_TIMEOUT,
                                 ACT_NEXT, ACT_NEXT) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_AddInstr(ACT_OP_LOOP, 0, 0, ACT_COND_IMMEDIATE,
                                 0U, 0U) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_AddInstr(ACT_OP_LOOP, 1001, 0, ACT_COND_IMMEDIATE,
                                 0U, 0U) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_AddRoadNav(
        ROAD_ROUTE_COUNT, 80, 15000, 0U, 0U) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_AddRoadNav(
        ROAD_ROUTE_STRAIGHT, 0, 15000, 0U, 0U) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_AddRoadNav(
        ROAD_ROUTE_STRAIGHT, 80, 75, 0U, 0U) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_AddDmPosition(
        false, 12501, 200, 5000, 0U, 0U) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_AddDmPosition(
        true, 100, 20001, 5000, 0U, 0U) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_AddDmPosition(
        true, 100, 200, 75, 0U, 0U) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_AddInstr(
        ACT_OP_DM_SPEED, 0, 1000, ACT_COND_IMMEDIATE, 0U, 0U) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_AddInstr(
        ACT_OP_DM_SPEED, 20001, 1000, ACT_COND_IMMEDIATE, 0U, 0U) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_AddInstr(
        ACT_OP_DM_SPEED, 200, 75, ACT_COND_IMMEDIATE, 0U, 0U) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_AddInstr(
        ACT_OP_DM_DISABLE, 1, 0, ACT_COND_IMMEDIATE, 0U, 0U) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_AddInstr(ACT_OP_WAIT, 0, 10, ACT_COND_TIMEOUT,
                                 4U, ACT_NEXT) == drivers::DRIVER_OK);
    ActionValidationResult validation;
    assert(ActionRunner_Validate(&validation) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(validation.index == 0U);
    assert(validation.field == ACT_VALID_FIELD_ON_SUCCESS);
    assert(ActionRunner_Start() == drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_GetState()->result == ACT_RUN_INVALID);

    /* Loop targets must both be explicit valid indices. */
    Reset();
    assert(ActionRunner_AddInstr(ACT_OP_LOOP, 1, 0, ACT_COND_IMMEDIATE,
                                 ACT_NEXT, ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Validate(&validation) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(validation.index == 0U);
    assert(validation.field == ACT_VALID_FIELD_ON_SUCCESS);

    Reset();
    assert(ActionRunner_AddInstr(ACT_OP_WAIT, 0, 10, ACT_COND_TIMEOUT,
                                 1U, ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    g_now = 0U; /* boot-time timestamp zero must not restart the instruction */
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    g_now += 11U;
    ActionRunner_Update();
    ActionRunner_Update();
    assert(!ActionRunner_GetState()->running);
    assert(ActionRunner_GetState()->result == ACT_RUN_SUCCESS);

    Reset();
    assert(ActionRunner_AddInstr(ACT_OP_WAIT, 0, 5,
                                 ACT_COND_LINE_DETECTED,
                                 2U, 1U) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_STOP, 0, 0, ACT_COND_IMMEDIATE,
                                 2U, ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    g_now += 6U;
    ActionRunner_Update();
    assert(ActionRunner_GetState()->running);
    assert(ActionRunner_GetState()->current == 1U);
    assert(ActionRunner_GetState()->failure_reason ==
           ACT_FAIL_INSTR_TIMEOUT);
    ActionRunner_Update();
    ActionRunner_Update();
    assert(ActionRunner_GetState()->result == ACT_RUN_SUCCESS);

    Reset();
    assert(ActionRunner_AddInstr(ACT_OP_BRANCH, 0, 0,
                                 ACT_COND_IMMEDIATE, 0U,
                                 ACT_NEXT) == drivers::DRIVER_OK);
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    g_now += 60001U;
    ActionRunner_Update();
    assert(ActionRunner_GetState()->running);
    assert(ActionRunner_GetState()->result == ACT_RUN_RUNNING);
    g_now += 239999U;
    ActionRunner_Update();
    assert(ActionRunner_GetState()->running);
    g_now += 1U;
    ActionRunner_Update();
    assert(ActionRunner_GetState()->result == ACT_RUN_TIMEOUT);
    assert(ActionRunner_GetState()->failure_reason ==
           ACT_FAIL_SEQUENCE_TIMEOUT);

    /* Consecutive road actions adopt LF_FOLLOW without an intervening stop.
     * Leaving the road-nav chain for END restores the normal safe stop. */
    Reset();
    assert(ActionRunner_AddRoadNav(
        ROAD_ROUTE_STRAIGHT, 80, 15000, 1U, ACT_NEXT) ==
           drivers::DRIVER_OK);
    assert(ActionRunner_AddRoadNav(
        ROAD_ROUTE_RIGHT, 70, 15000, 2U, ACT_NEXT) ==
           drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    assert(g_road_start_calls == 1U);
    g_road.route_active = false;
    g_road.route_result = ROAD_ROUTE_RESULT_SUCCESS;
    ActionRunner_Update();
    assert(ActionRunner_GetState()->current == 1U);
    assert(g_road_cancel_calls == 0U);
    assert(g_lf.mode == LF_FOLLOW);
    ActionRunner_Update();
    assert(g_road_start_calls == 2U);
    assert(g_road_cancel_calls == 0U);
    assert(g_lf.base_rpm == 70);
    g_road.route_active = false;
    g_road.route_result = ROAD_ROUTE_RESULT_SUCCESS;
    ActionRunner_Update();
    assert(g_road_cancel_calls != 0U);
    assert(g_lf.mode == LF_IDLE);
    ActionRunner_Update();
    assert(ActionRunner_GetState()->result == ACT_RUN_SUCCESS);
    assert(g_road_cancel_calls != 0U);

    /* A counted loop node preserves the handoff while its body returns to
     * another road action. */
    Reset();
    assert(ActionRunner_AddInstr(
        ACT_OP_LOOP, 2, 0, ACT_COND_IMMEDIATE, 1U, 2U) ==
           drivers::DRIVER_OK);
    assert(ActionRunner_AddRoadNav(
        ROAD_ROUTE_STRAIGHT, 80, 15000, 0U, ACT_NEXT) ==
           drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    const uint32_t stops_before_body = g_road_cancel_calls;
    ActionRunner_Update();
    g_road.route_active = false;
    g_road.route_result = ROAD_ROUTE_RESULT_SUCCESS;
    ActionRunner_Update();
    ActionRunner_Update();
    assert(g_road_cancel_calls == stops_before_body);
    assert(ActionRunner_GetState()->current == 1U);
    ActionRunner_Update();
    assert(g_road_start_calls == 2U);

    /* Route-specific failures keep their own reason and use the failure
     * target. */
    Reset();
    assert(ActionRunner_AddRoadNav(
        ROAD_ROUTE_LEFT, 80, 15000, 1U, 1U) ==
           drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    g_road.route_active = false;
    g_road.route_result = ROAD_ROUTE_RESULT_UNAVAILABLE;
    ActionRunner_Update();
    assert(ActionRunner_GetState()->current == 1U);
    assert(ActionRunner_GetState()->failure_reason ==
           ACT_FAIL_ROUTE_UNAVAILABLE);

    Reset();
    assert(ActionRunner_AddRoadNav(
        ROAD_ROUTE_LEFT, 80, 15000, 1U, 1U) ==
           drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    g_road.route_active = false;
    g_road.route_result = ROAD_ROUTE_RESULT_REACQUIRE_FAILED;
    ActionRunner_Update();
    assert(ActionRunner_GetState()->failure_reason ==
           ACT_FAIL_ROUTE_REACQUIRE_FAILED);

    Reset();
    assert(ActionRunner_AddRoadNav(
        ROAD_ROUTE_LEFT, 80, 15000, 1U, 1U) ==
           drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    g_road.route_active = false;
    g_road.route_result = ROAD_ROUTE_RESULT_CONTROL_ERROR;
    ActionRunner_Update();
    assert(ActionRunner_GetState()->result == ACT_RUN_FAULT);
    assert(ActionRunner_GetState()->failure_reason == ACT_FAIL_FAULT);

    /* DM positioning remains in hold across ordinary successors. Sequence
     * completion, cancellation and implicit termination always request the
     * repeated emergency-disable path. */
    Reset();
    assert(ActionRunner_AddDmPosition(
        true, 87, 201, 5000, 1U, ACT_NEXT) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_WAIT, 0, 100,
                                 ACT_COND_TIMEOUT, 2U,
                                 ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    assert(g_dm.mode == DM_CONTROL_POSITION);
    assert(g_dm.target_position_mrad == 87);
    assert(g_dm.operation_timeout_ms == 5000U);
    g_dm.mode = DM_CONTROL_HOLD;
    g_dm.operation_result = DM_OPERATION_SUCCESS;
    ActionRunner_Update();
    assert(ActionRunner_GetState()->current == 1U);
    assert(g_dm_emergency_disable_calls == 0U);
    ActionRunner_Update();
    g_now += 100U;
    ActionRunner_Update();
    assert(g_dm_emergency_disable_calls == 0U);
    ActionRunner_Update();
    assert(ActionRunner_GetState()->result == ACT_RUN_SUCCESS);
    assert(g_dm_emergency_disable_calls == 1U);

    Reset();
    assert(ActionRunner_AddDmPosition(
        false, 1000, 200, 5000, 1U, 1U) == drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    g_dm.mode = DM_CONTROL_HOLD;
    g_dm.operation_result = DM_OPERATION_TARGET_TIMEOUT;
    ActionRunner_Update();
    assert(ActionRunner_GetState()->current == 1U);
    assert(ActionRunner_GetState()->failure_reason ==
           ACT_FAIL_INSTR_TIMEOUT);

    Reset();
    assert(ActionRunner_AddInstr(
        ACT_OP_DM_SPEED, -200, 1000, ACT_COND_IMMEDIATE,
        1U, ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    g_now += 1000U;
    ActionRunner_Update();
    assert(g_dm_stop_speed_calls == 1U);
    g_dm.mode = DM_CONTROL_HOLD;
    g_dm.operation_result = DM_OPERATION_SUCCESS;
    ActionRunner_Update();
    assert(ActionRunner_GetState()->current == 1U);
    assert(ActionRunner_Cancel() == drivers::DRIVER_OK);
    assert(g_dm_emergency_disable_calls == 1U);

    /* A foreground ball-move failure follows the explicit red target and
     * releases continuous control into a safe DM hold. */
    Reset();
    assert(ActionRunner_AddInstr(
        ACT_OP_BALL_MOVE, 500, 5000, ACT_COND_IMMEDIATE,
        1U, 1U) == drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    g_ball.mode = BALL_BALANCE_FAILED;
    g_ball.result = BALL_BALANCE_RESULT_VISION_LOST;
    ActionRunner_Update();
    assert(ActionRunner_GetState()->running);
    assert(ActionRunner_GetState()->current == 1U);
    assert(ActionRunner_GetState()->failure_reason ==
           ACT_FAIL_BALL_VISION_LOST);
    assert(g_ball_stop_calls == 1U);
    ActionRunner_Update();
    assert(ActionRunner_GetState()->result == ACT_RUN_SUCCESS);

    /* Once ball_hold has handed control to a later action, loss of the
     * background balance loop aborts the complete sequence. */
    Reset();
    assert(ActionRunner_AddInstr(
        ACT_OP_BALL_HOLD, 0, 0, ACT_COND_IMMEDIATE,
        1U, ACT_NEXT) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(
        ACT_OP_WAIT, 0, 1000, ACT_COND_TIMEOUT,
        2U, ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    assert(ActionRunner_GetState()->current == 1U);
    g_ball.mode = BALL_BALANCE_FAILED;
    g_ball.result = BALL_BALANCE_RESULT_ENDPOINT;
    ActionRunner_Update();
    assert(ActionRunner_GetState()->result == ACT_RUN_ABORTED);
    assert(ActionRunner_GetState()->failure_reason ==
           ACT_FAIL_BALL_ENDPOINT);

    /* Counted loop executes its body exactly N times; done is success. */
    Reset();
    assert(ActionRunner_AddInstr(ACT_OP_LOOP, 1, 0, ACT_COND_IMMEDIATE,
                                 1U, 2U) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_LED_TOGGLE, 2, 0,
                                 ACT_COND_IMMEDIATE, 0U,
                                 ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    RunUntilStopped(8U);
    assert(g_ledToggleCount[board::BOARD_LED_ID_2] == 1U);
    assert(ActionRunner_GetState()->result == ACT_RUN_SUCCESS);
    assert(ActionRunner_GetState()->failure_reason == ACT_FAIL_NONE);

    Reset();
    assert(ActionRunner_AddInstr(ACT_OP_LOOP, 3, 0, ACT_COND_IMMEDIATE,
                                 1U, 2U) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_LED_TOGGLE, 2, 0,
                                 ACT_COND_IMMEDIATE, 0U,
                                 ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    RunUntilStopped(16U);
    assert(g_ledToggleCount[board::BOARD_LED_ID_2] == 3U);
    assert(ActionRunner_GetState()->failure_reason == ACT_FAIL_NONE);

    Reset();
    assert(ActionRunner_AddInstr(ACT_OP_LOOP, 1000, 0,
                                 ACT_COND_IMMEDIATE, 1U, 2U) ==
           drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_LED_TOGGLE, 2, 0,
                                 ACT_COND_IMMEDIATE, 0U,
                                 ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    RunUntilStopped(2100U);
    assert(g_ledToggleCount[board::BOARD_LED_ID_2] == 1000U);

    /* Sequential loops keep independent counters. */
    Reset();
    assert(ActionRunner_AddInstr(ACT_OP_LOOP, 2, 0, ACT_COND_IMMEDIATE,
                                 1U, 2U) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_LED_TOGGLE, 2, 0,
                                 ACT_COND_IMMEDIATE, 0U,
                                 ACT_NEXT) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_LOOP, 3, 0, ACT_COND_IMMEDIATE,
                                 3U, 4U) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_LED_TOGGLE, 3, 0,
                                 ACT_COND_IMMEDIATE, 2U,
                                 ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    RunUntilStopped(24U);
    assert(g_ledToggleCount[board::BOARD_LED_ID_2] == 2U);
    assert(g_ledToggleCount[board::BOARD_LED_ID_3] == 3U);

    /* Nested 2 x 3 loop executes the inner body six times. */
    Reset();
    assert(ActionRunner_AddInstr(ACT_OP_LOOP, 2, 0, ACT_COND_IMMEDIATE,
                                 1U, 4U) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_LOOP, 3, 0, ACT_COND_IMMEDIATE,
                                 2U, 3U) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_LED_TOGGLE, 2, 0,
                                 ACT_COND_IMMEDIATE, 1U,
                                 ACT_NEXT) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_STOP, 0, 0, ACT_COND_IMMEDIATE,
                                 0U, ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    RunUntilStopped(40U);
    assert(g_ledToggleCount[board::BOARD_LED_ID_2] == 6U);

    /* Cancel clears loop state so a restart begins again at iteration one. */
    Reset();
    assert(ActionRunner_AddInstr(ACT_OP_LOOP, 3, 0, ACT_COND_IMMEDIATE,
                                 1U, 2U) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_LED_TOGGLE, 2, 0,
                                 ACT_COND_IMMEDIATE, 0U,
                                 ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    ActionRunner_Update();
    assert(g_ledToggleCount[board::BOARD_LED_ID_2] == 1U);
    assert(ActionRunner_Cancel() == drivers::DRIVER_OK);
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    RunUntilStopped(16U);
    assert(g_ledToggleCount[board::BOARD_LED_ID_2] == 4U);

    /* A loop-body action failure keeps that action's failure reason. */
    Reset();
    assert(ActionRunner_AddInstr(ACT_OP_LOOP, 3, 0, ACT_COND_IMMEDIATE,
                                 1U, 2U) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_WAIT, 0, 5,
                                 ACT_COND_LINE_DETECTED, 0U,
                                 ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    ActionRunner_Update();
    g_now += 6U;
    ActionRunner_Update();
    assert(!ActionRunner_GetState()->running);
    assert(ActionRunner_GetState()->result == ACT_RUN_ABORTED);
    assert(ActionRunner_GetState()->failure_reason ==
           ACT_FAIL_INSTR_TIMEOUT);

    Reset();
    assert(ActionRunner_AddInstr(ACT_OP_LED_ON, 2, 50,
                                 ACT_COND_IMMEDIATE, 1U,
                                 ACT_NEXT) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_WAIT, 0, 100,
                                 ACT_COND_TIMEOUT, 2U,
                                 ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    assert(g_led[board::BOARD_LED_ID_2]);
    ActionRunner_Update();
    g_now += 51U;
    ActionRunner_Update();
    assert(!g_led[board::BOARD_LED_ID_2]);
    assert(ActionRunner_Cancel() == drivers::DRIVER_OK);
    assert(ActionRunner_GetState()->result == ACT_RUN_CANCELLED);

    /* Generic condition: wait for a stable line, then complete. */
    Reset();
    ActionConditionConfig line_condition = {
        ACT_SOURCE_LINE_DETECTED,
        ACT_COMPARE_EQ,
        1,
        ACT_CONDITION_WAIT,
        1000U,
        100U
    };
    assert(ActionRunner_AddCompareInstr(
               ACT_OP_CONDITION, 0, &line_condition, 1U, ACT_NEXT) ==
           drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    g_line = true;
    g_now += 50U;
    g_gray.last_update_ms = g_now;
    ActionRunner_Update();
    g_now += 50U;
    g_gray.last_update_ms = g_now;
    ActionRunner_Update();
    g_now += 50U;
    g_gray.last_update_ms = g_now;
    ActionRunner_Update();
    ActionRunner_Update();
    assert(ActionRunner_GetState()->result == ACT_RUN_SUCCESS);

    /* Invalid live sensor data takes the failure path immediately. */
    Reset();
    ActionConditionConfig yaw_condition = {
        ACT_SOURCE_IMU_YAW_MDEG,
        ACT_COMPARE_GE,
        1000,
        ACT_CONDITION_WAIT,
        1000U,
        0U
    };
    assert(ActionRunner_AddCompareInstr(
               ACT_OP_CONDITION, 0, &yaw_condition, 1U, 1U) ==
           drivers::DRIVER_OK);
    AddEnd();
    g_imu.valid = false;
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    assert(ActionRunner_GetState()->current == 1U);
    assert(ActionRunner_GetState()->failure_reason ==
           ACT_FAIL_CONDITION_UNAVAILABLE);

    /* Conditioned drive measures distance relative to instruction entry. */
    Reset();
    ActionConditionConfig distance_condition = {
        ACT_SOURCE_AVERAGE_DISTANCE_MM,
        ACT_COMPARE_GE,
        100,
        ACT_CONDITION_WAIT,
        5000U,
        0U
    };
    assert(ActionRunner_AddCompareInstr(
               ACT_OP_DRIVE_IF, 60, &distance_condition, 1U, ACT_NEXT) ==
           drivers::DRIVER_OK);
    AddEnd();
    assert(ActionRunner_Start() == drivers::DRIVER_OK);
    ActionRunner_Update();
    g_chassis.left.encoder_count += 800;
    g_chassis.right.encoder_count += 800;
    g_now += 50U;
    g_chassis.last_feedback_ms = g_now;
    ActionRunner_Update();
    ActionRunner_Update();
    assert(ActionRunner_GetState()->result == ACT_RUN_SUCCESS);

    /* Button2 remains reserved for competition stop. */
    Reset();
    ActionConditionConfig button2_condition = {
        ACT_SOURCE_BUTTON2_PRESSED,
        ACT_COMPARE_EQ,
        1,
        ACT_CONDITION_WAIT,
        1000U,
        0U
    };
    assert(ActionRunner_AddCompareInstr(
               ACT_OP_CONDITION, 0, &button2_condition,
               ACT_NEXT, ACT_NEXT) == drivers::DRIVER_OK);
    assert(ActionRunner_Validate(&validation) == drivers::DRIVER_OK);
    assert(ActionRunner_ValidateCompetition(&validation) ==
           drivers::DRIVER_ERROR_INVALID_ARG);

    /* ActionRunner and SeqStore share the same 54-instruction ceiling. */
    Reset();
    for (uint8_t i = 0U; i < ACTION_MAX_INSTRS; i++) {
        assert(ActionRunner_AddInstr(
                   ACT_OP_STOP, 0, 0, ACT_COND_IMMEDIATE,
                   ACT_NEXT, ACT_NEXT) == drivers::DRIVER_OK);
    }
    assert(ActionRunner_AddInstr(
               ACT_OP_END, 0, 0, ACT_COND_IMMEDIATE,
               ACT_NEXT, ACT_NEXT) ==
           drivers::DRIVER_ERROR);

    /* Clean SeqStore v1 keeps eight 54-step slots with 14-byte instructions. */
    Reset();
    memset(g_fram, 0, sizeof(g_fram));
    assert(ActionRunner_AddInstr(ACT_OP_LOOP, 3, 0,
                                 ACT_COND_IMMEDIATE, 1U, 2U) ==
           drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(ACT_OP_STOP, 0, 0,
                                 ACT_COND_IMMEDIATE, 0U,
                                 ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(SeqStore_Save(7U) == drivers::DRIVER_OK);
    assert(g_fram[0x0800] == 'G' && g_fram[0x0801] == 'S' &&
           g_fram[0x0802] == 'Q' && g_fram[0x0803] == '1');
    assert(g_fram[0x0804] == 1U && g_fram[0x0805] == 8U &&
           g_fram[0x0806] == 54U);
    const uint16_t slot7 = 0x1CFAU;
    assert(g_fram[slot7] == 0xA5U &&
           g_fram[slot7 + 1U] == 3U);
    assert(g_fram[slot7 + 6U] ==
           static_cast<uint8_t>(ACT_OP_LOOP));
    assert(g_fram[slot7 + 20U] ==
           static_cast<uint8_t>(ACT_OP_STOP));
    assert(g_fram[slot7 + 34U] ==
           static_cast<uint8_t>(ACT_OP_END));
    assert(ActionRunner_Clear() == drivers::DRIVER_OK);
    assert(SeqStore_Load(7U) == drivers::DRIVER_OK);
    assert(ActionRunner_GetState()->count == 3U);
    assert(ActionRunner_GetState()->instrs[0].op == ACT_OP_LOOP);
    assert(ActionRunner_GetState()->instrs[0].param1 == 3);
    assert(ActionRunner_GetState()->instrs[0].on_success == 1U);
    assert(ActionRunner_GetState()->instrs[0].on_timeout == 2U);

    /* Opcode 19 and its route enum survive the unchanged 14-byte slot
     * record and CRC round-trip. */
    assert(ActionRunner_Clear() == drivers::DRIVER_OK);
    assert(ActionRunner_AddRoadNav(
        ROAD_ROUTE_UTURN_RIGHT_PIVOT, 80, 15000, 1U, ACT_NEXT) ==
           drivers::DRIVER_OK);
    AddEnd();
    assert(SeqStore_Save(6U) == drivers::DRIVER_OK);
    assert(ActionRunner_Clear() == drivers::DRIVER_OK);
    assert(SeqStore_Load(6U) == drivers::DRIVER_OK);
    assert(ActionRunner_GetState()->count == 2U);
    assert(ActionRunner_GetState()->instrs[0].op == ACT_OP_ROAD_NAV);
    assert(ActionRunner_GetState()->instrs[0].param1 == 80);
    assert(ActionRunner_GetState()->instrs[0].param2 == 15000);
    assert(ActionRunner_GetState()->instrs[0].condition_value ==
           ROAD_ROUTE_UTURN_RIGHT_PIVOT);

    /* Opcodes 20..22 keep the existing 14-byte SeqStore record,
     * including dm_position's timeout in condition_value. */
    assert(ActionRunner_Clear() == drivers::DRIVER_OK);
    assert(ActionRunner_AddDmPosition(
        true, 0, 20000, 5000, 1U, ACT_NEXT) == drivers::DRIVER_OK);
    assert(ActionRunner_Clear() == drivers::DRIVER_OK);
    assert(ActionRunner_AddDmPosition(
        true, 0, 20001, 5000, 1U, ACT_NEXT) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_AddInstr(
        ACT_OP_DM_SPEED, 20000, 1000, ACT_COND_IMMEDIATE,
        1U, ACT_NEXT) == drivers::DRIVER_OK);
    assert(ActionRunner_Clear() == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(
        ACT_OP_DM_SPEED, 20001, 1000, ACT_COND_IMMEDIATE,
        1U, ACT_NEXT) == drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_Clear() == drivers::DRIVER_OK);
    assert(ActionRunner_AddDmPosition(
        true, -87, 201, 5000, 1U, ACT_NEXT) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(
        ACT_OP_DM_SPEED, 200, 1000, ACT_COND_IMMEDIATE,
        2U, ACT_NEXT) == drivers::DRIVER_OK);
    assert(ActionRunner_AddInstr(
        ACT_OP_DM_DISABLE, 0, 0, ACT_COND_IMMEDIATE,
        3U, ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(SeqStore_Save(5U) == drivers::DRIVER_OK);
    assert(ActionRunner_Clear() == drivers::DRIVER_OK);
    assert(SeqStore_Load(5U) == drivers::DRIVER_OK);
    assert(ActionRunner_GetState()->count == 4U);
    assert(ActionRunner_GetState()->instrs[0].op == ACT_OP_DM_POSITION);
    assert(ActionRunner_GetState()->instrs[0].param1 == -87);
    assert(ActionRunner_GetState()->instrs[0].param2 == 201);
    assert(ActionRunner_GetState()->instrs[0].until ==
           ACT_COND_DM_RELATIVE);
    assert(ActionRunner_GetState()->instrs[0].condition_value == 5000);
    assert(ActionRunner_GetState()->instrs[1].op == ACT_OP_DM_SPEED);
    assert(ActionRunner_GetState()->instrs[2].op == ACT_OP_DM_DISABLE);

    /* Opcode 26 preserves cruise, approach and lap distance, while slot 0
     * remains reserved for the built-in H2 task. */
    assert(ActionRunner_Clear() == drivers::DRIVER_OK);
    assert(ActionRunner_AddTrackCourse(
        110, 60, 6142, 1U, ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(SeqStore_Save(4U) == drivers::DRIVER_OK);
    assert(ActionRunner_Clear() == drivers::DRIVER_OK);
    assert(SeqStore_Load(4U) == drivers::DRIVER_OK);
    assert(ActionRunner_GetState()->instrs[0].op == ACT_OP_TRACK_COURSE);
    assert(ActionRunner_GetState()->instrs[0].param1 == 110);
    assert(ActionRunner_GetState()->instrs[0].param2 == 60);
    assert(ActionRunner_GetState()->instrs[0].condition_value == 6142);
    assert(SeqStore_Save(0U) == drivers::DRIVER_ERROR_INVALID_ARG);
    assert(SeqStore_Load(0U) == drivers::DRIVER_ERROR_INVALID_ARG);
    assert(SeqStore_Delete(0U) == drivers::DRIVER_ERROR_INVALID_ARG);

    /* Former layouts are deliberately discarded, with all slots invalid. */
    Reset();
    memset(g_fram, 0xCC, sizeof(g_fram));
    assert(SeqStore_Init() == drivers::DRIVER_OK);
    assert(g_fram[0x0800] == 'G' && g_fram[0x0804] == 1U);
    assert(g_fram[0x0806] == 54U);
    assert(g_fram[0x0808] == 0U);
    assert(g_fram[0x1CFA] == 0U);
    assert(SeqStore_Load(7U) ==
           drivers::DRIVER_ERROR_NOT_INITIALIZED);

    printf("action runner ok: 300s guard, DM-G6220, counted/nested loops, "
           "generic conditions, competition validation, clean SeqStore v1\n");
    return 0;
}
