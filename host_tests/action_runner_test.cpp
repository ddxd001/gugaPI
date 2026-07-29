#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/action.h"
#include "app/app_grayscale.h"
#include "app/app_imu.h"
#include "app/app_main.h"
#include "app/chassis.h"
#include "app/heading.h"
#include "app/linefollow.h"
#include "app/road_event_controller.h"
#include "app/seq_store.h"
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
app::AppImuData g_imu = {};
app::AppState g_app = {};
app::RoadControlState g_road = {};
uint32_t g_road_start_calls = 0U;
uint32_t g_road_cancel_calls = 0U;

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
    g_imu = app::AppImuData();
    g_imu.valid = true;
    g_imu.last_update_ms = g_now;
    g_app.mode = app::APP_MODE_RUNNING;
    g_road = app::RoadControlState();
    g_road.route_result = app::ROAD_ROUTE_RESULT_IDLE;
    g_road_start_calls = 0U;
    g_road_cancel_calls = 0U;
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

uint32_t TestCrc32(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint16_t i = 0U; i < length; i++) {
        crc ^= static_cast<uint32_t>(data[i]) << 24U;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = ((crc & 0x80000000U) != 0U) ?
                ((crc << 1U) ^ 0x04C11DB7U) : (crc << 1U);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

void TestWriteU32(uint8_t *data, uint32_t value)
{
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8U);
    data[2] = static_cast<uint8_t>(value >> 16U);
    data[3] = static_cast<uint8_t>(value >> 24U);
}

void PrepareLegacyV1Slot7()
{
    memset(g_fram, 0, sizeof(g_fram));
    TestWriteU32(&g_fram[0x0100], 0x53455131U);
    g_fram[0x0104] = 1U;
    g_fram[0x0105] = 0U;
    g_fram[0x0106] = 8U;
    const uint16_t slot =
        static_cast<uint16_t>(0x0100U + 8U + 7U * 646U);
    g_fram[slot] = 1U;
    g_fram[slot + 1U] = 1U;
    uint8_t *instr = &g_fram[slot + 2U];
    instr[0] = static_cast<uint8_t>(app::ACT_OP_END);
    instr[7] = static_cast<uint8_t>(app::ACT_COND_IMMEDIATE);
    instr[8] = app::ACT_NEXT;
    instr[9] = app::ACT_NEXT;
    const uint16_t payload_length = 12U;
    TestWriteU32(&g_fram[slot + payload_length],
                 TestCrc32(&g_fram[slot], payload_length));
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
        { ACT_OP_BUZZER_TOGGLE, 0, 0, ACT_COND_IMMEDIATE }
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

    /* SeqStore v2 keeps eight 64-step slots with 14-byte instructions. */
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
    assert(g_fram[0x0100] == '1' && g_fram[0x0101] == 'Q' &&
           g_fram[0x0102] == 'E' && g_fram[0x0103] == 'S');
    assert(g_fram[0x0104] == 2U && g_fram[0x0105] == 0U);
    const uint16_t slot7 = static_cast<uint16_t>(0x0100U + 8U + 7U * 902U);
    assert(g_fram[slot7] == 1U && g_fram[slot7 + 1U] == 3U);
    assert(g_fram[slot7 + 2U] == static_cast<uint8_t>(ACT_OP_LOOP));
    assert(g_fram[slot7 + 16U] == static_cast<uint8_t>(ACT_OP_STOP));
    assert(g_fram[slot7 + 30U] == static_cast<uint8_t>(ACT_OP_END));
    assert(ActionRunner_Clear() == drivers::DRIVER_OK);
    assert(SeqStore_Load(7U) == drivers::DRIVER_OK);
    assert(ActionRunner_GetState()->count == 3U);
    assert(ActionRunner_GetState()->instrs[0].op == ACT_OP_LOOP);
    assert(ActionRunner_GetState()->instrs[0].param1 == 3);
    assert(ActionRunner_GetState()->instrs[0].on_success == 1U);
    assert(ActionRunner_GetState()->instrs[0].on_timeout == 2U);

    /* Opcode 19 and its route enum survive the unchanged 14-byte v2 slot
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

    /* Existing v1 slots migrate in place without changing their behavior. */
    Reset();
    PrepareLegacyV1Slot7();
    assert(SeqStore_Init() == drivers::DRIVER_OK);
    assert(g_fram[0x0104] == 2U && g_fram[0x0105] == 0U);
    assert(SeqStore_Load(7U) == drivers::DRIVER_OK);
    assert(ActionRunner_GetState()->count == 1U);
    assert(ActionRunner_GetState()->instrs[0].op == ACT_OP_END);

    printf("action runner ok: 300s guard, counted/nested loops, generic "
           "conditions, competition validation, SeqStore v1/v2\n");
    return 0;
}
