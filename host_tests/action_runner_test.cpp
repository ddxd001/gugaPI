#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/action.h"
#include "app/chassis.h"
#include "app/heading.h"
#include "app/linefollow.h"
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
bool g_led[board::BOARD_LED_ID_COUNT] = {};
bool g_buzzer = false;
uint8_t g_fram[8192] = {};
app::ChassisState g_chassis = {};
app::HeadingState g_heading = {};
app::LFState g_lf = {};

void Reset()
{
    g_now = 100U;
    g_fault = false;
    g_line = false;
    g_button = false;
    g_led[0] = g_led[1] = g_led[2] = false;
    g_buzzer = false;
    g_chassis = app::ChassisState();
    g_chassis.initialized = true;
    g_chassis.config.max_wheel_rpm = 500U;
    g_heading = app::HeadingState();
    g_heading.mode = app::HEADING_IDLE;
    g_lf = app::LFState();
    g_lf.mode = app::LF_IDLE;
    app::ActionRunner_Init();
}

void AddEnd()
{
    assert(app::ActionRunner_AddInstr(app::ACT_OP_END, 0, 0,
                                      app::ACT_COND_IMMEDIATE,
                                      app::ACT_NEXT, app::ACT_NEXT) ==
           drivers::DRIVER_OK);
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
bool LF_IsLineDetected(void) { return g_line; }
} /* namespace app */

namespace board {
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
    assert(ActionRunner_AddInstr(ACT_OP_WAIT, 0, 10, ACT_COND_TIMEOUT,
                                 4U, ACT_NEXT) == drivers::DRIVER_OK);
    ActionValidationResult validation;
    assert(ActionRunner_Validate(&validation) ==
           drivers::DRIVER_ERROR_INVALID_ARG);
    assert(validation.index == 0U);
    assert(validation.field == ACT_VALID_FIELD_ON_SUCCESS);
    assert(ActionRunner_Start() == drivers::DRIVER_ERROR_INVALID_ARG);
    assert(ActionRunner_GetState()->result == ACT_RUN_INVALID);

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
    assert(ActionRunner_GetState()->result == ACT_RUN_TIMEOUT);
    assert(ActionRunner_GetState()->failure_reason ==
           ACT_FAIL_SEQUENCE_TIMEOUT);

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

    /* SeqStore v1 remains an eight-slot, 10-byte instruction format. */
    Reset();
    memset(g_fram, 0, sizeof(g_fram));
    assert(ActionRunner_AddInstr(ACT_OP_DRIVE_MM, 250, 60,
                                 ACT_COND_DISTANCE_REACHED, 1U,
                                 ACT_NEXT) == drivers::DRIVER_OK);
    AddEnd();
    assert(SeqStore_Save(7U) == drivers::DRIVER_OK);
    assert(g_fram[0x0100] == '1' && g_fram[0x0101] == 'Q' &&
           g_fram[0x0102] == 'E' && g_fram[0x0103] == 'S');
    assert(g_fram[0x0104] == 1U && g_fram[0x0105] == 0U);
    const uint16_t slot7 = static_cast<uint16_t>(0x0100U + 8U + 7U * 646U);
    assert(g_fram[slot7] == 1U && g_fram[slot7 + 1U] == 2U);
    assert(g_fram[slot7 + 2U] == static_cast<uint8_t>(ACT_OP_DRIVE_MM));
    assert(g_fram[slot7 + 12U] == static_cast<uint8_t>(ACT_OP_END));
    assert(ActionRunner_Clear() == drivers::DRIVER_OK);
    assert(SeqStore_Load(7U) == drivers::DRIVER_OK);
    assert(ActionRunner_GetState()->count == 2U);
    assert(ActionRunner_GetState()->instrs[0].param1 == 250);

    printf("action runner ok: validation, jumps, timeout, cancel, outputs, SeqStore v1\n");
    return 0;
}
