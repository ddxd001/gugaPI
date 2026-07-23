#include "app/action.h"

#include "app/chassis.h"
#include "app/heading.h"
#include "app/linefollow.h"
#include "drivers/common/driver_status.h"
#include "services/fault.h"
#include "services/time.h"

namespace app {
namespace {

static const uint8_t kMaxActions = 16U;
static const uint32_t kActionDriveSafetyMs = 30000U;  /* per-drive runaway cap */
static const uint32_t kActionTurnTimeoutMs = 12000U;  /* backup of heading's 8s */
static const uint32_t kActionLineFollowTimeoutMs = 30000U; /* per-follow cap */
static const uint32_t kSequenceTimeoutMs = 30000U;     /* whole-sequence cap */

ActionRunnerState g_state = {
    { { ACTION_NONE, 0, 0U, ACTION_PENDING, 0U } },
    0U, 0U, false, ACTION_DONE, 0U
};

/* Start an action. Returns true if started (or instantaneous), false if it
 * could not start (heading/chassis rejected it). */
bool StartAction(Action *a)
{
    a->start_ms = services::Time_Millis();
    switch (a->type) {
    case ACTION_DRIVE:
        return (Heading_HoldStart(a->param1) == drivers::DRIVER_OK);
    case ACTION_TURN:
        return (Heading_TurnStart(a->param1) == drivers::DRIVER_OK);
    case ACTION_WAIT:
        return true;
    case ACTION_STOP:
        (void) Heading_Stop();
        (void) Chassis_Stop();
        return true;
    case ACTION_LINE_FOLLOW:
        return (LF_Start(a->param1, a->param2) == drivers::DRIVER_OK);
    default:
        return false;
    }
}

/* Poll a running action. TURN completes when the heading returns to IDLE (it
 * has its own stop); DRIVE/WAIT complete on duration. */
ActionStatus UpdateAction(Action *a)
{
    const uint32_t now = services::Time_Millis();

    switch (a->type) {
    case ACTION_DRIVE: {
        const uint32_t elapsed = now - a->start_ms;
        if (elapsed >= a->param2) {
            return ACTION_DONE;
        }
        if (elapsed > kActionDriveSafetyMs) {
            return ACTION_FAILED;
        }
        return ACTION_RUNNING;
    }
    case ACTION_TURN: {
        const HeadingState *h = Heading_GetState();
        if (h->mode == HEADING_IDLE) {
            /* Heading_Update stopped itself (turn done) or faulted. */
            return services::Fault_HasFault() ? ACTION_FAILED : ACTION_DONE;
        }
        if ((now - a->start_ms) > kActionTurnTimeoutMs) {
            return ACTION_FAILED;
        }
        return ACTION_RUNNING;
    }
    case ACTION_WAIT:
        return ((now - a->start_ms) >= a->param2) ? ACTION_DONE : ACTION_RUNNING;
    case ACTION_STOP:
        return ACTION_DONE;
    case ACTION_LINE_FOLLOW: {
        const LFState *lf = LF_GetState();
        if (lf->mode == LF_IDLE) {
            return services::Fault_HasFault() ? ACTION_FAILED : ACTION_DONE;
        }
        if ((now - a->start_ms) > kActionLineFollowTimeoutMs) {
            return ACTION_FAILED;
        }
        return ACTION_RUNNING;
    }
    default:
        return ACTION_FAILED;
    }
}

void FinishSequence(void)
{
    g_state.running = false;
    g_state.overall = ACTION_DONE;
    (void) Heading_Stop();
    (void) Chassis_Stop();
}

void AbortSequence(void)
{
    g_state.running = false;
    g_state.overall = ACTION_FAILED;
    (void) Heading_Stop();
    (void) Chassis_Stop();
}

drivers::DriverStatus AddAction(ActionType type,
                                int32_t param1,
                                uint32_t param2)
{
    if (g_state.running) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (g_state.count >= kMaxActions) {
        return drivers::DRIVER_ERROR;
    }
    Action *a = &g_state.actions[g_state.count];
    a->type = type;
    a->param1 = param1;
    a->param2 = param2;
    a->status = ACTION_PENDING;
    a->start_ms = 0U;
    g_state.count++;
    return drivers::DRIVER_OK;
}

} /* namespace */

void ActionRunner_Init(void)
{
    g_state.count = 0U;
    g_state.current = 0U;
    g_state.running = false;
    g_state.overall = ACTION_DONE;
    g_state.seq_start_ms = 0U;
    for (uint8_t i = 0U; i < kMaxActions; i++) {
        g_state.actions[i].type = ACTION_NONE;
        g_state.actions[i].status = ACTION_PENDING;
        g_state.actions[i].start_ms = 0U;
    }
}

drivers::DriverStatus ActionRunner_Clear(void)
{
    if (g_state.running) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    g_state.count = 0U;
    g_state.current = 0U;
    g_state.overall = ACTION_DONE;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus ActionRunner_AddDrive(int32_t rpm, uint32_t duration_ms)
{
    return AddAction(ACTION_DRIVE, rpm, duration_ms);
}

drivers::DriverStatus ActionRunner_AddTurn(int32_t delta_deg)
{
    return AddAction(ACTION_TURN, delta_deg, 0U);
}

drivers::DriverStatus ActionRunner_AddWait(uint32_t duration_ms)
{
    return AddAction(ACTION_WAIT, 0, duration_ms);
}

drivers::DriverStatus ActionRunner_AddStop(void)
{
    return AddAction(ACTION_STOP, 0, 0U);
}

drivers::DriverStatus ActionRunner_AddLineFollow(int32_t base_rpm,
                                                   uint32_t duration_ms)
{
    return AddAction(ACTION_LINE_FOLLOW, base_rpm, duration_ms);
}

drivers::DriverStatus ActionRunner_Start(void)
{
    if (g_state.running) {
        return drivers::DRIVER_ERROR_BUSY;
    }
    if (g_state.count == 0U) {
        return drivers::DRIVER_ERROR_INVALID_ARG;
    }
    if (services::Fault_HasFault()) {
        return drivers::DRIVER_ERROR_NOT_INITIALIZED;
    }
    for (uint8_t i = 0U; i < g_state.count; i++) {
        g_state.actions[i].status = ACTION_PENDING;
        g_state.actions[i].start_ms = 0U;
    }
    g_state.current = 0U;
    g_state.running = true;
    g_state.overall = ACTION_RUNNING;
    g_state.seq_start_ms = services::Time_Millis();
    return drivers::DRIVER_OK;
}

drivers::DriverStatus ActionRunner_Cancel(void)
{
    AbortSequence();
    return drivers::DRIVER_OK;
}

void ActionRunner_Update(void)
{
    if (!g_state.running) {
        return;
    }

    const uint32_t now = services::Time_Millis();

    if (services::Fault_HasFault()) {
        AbortSequence();
        return;
    }
    if ((now - g_state.seq_start_ms) > kSequenceTimeoutMs) {
        AbortSequence();
        return;
    }
    if (g_state.current >= g_state.count) {
        FinishSequence();
        return;
    }

    Action *a = &g_state.actions[g_state.current];
    if (a->status == ACTION_PENDING) {
        if (StartAction(a)) {
            a->status = ACTION_RUNNING;
        } else {
            a->status = ACTION_FAILED;
        }
    }

    if (a->status == ACTION_FAILED) {
        AbortSequence();
        return;
    }

    const ActionStatus r = UpdateAction(a);
    if (r == ACTION_DONE) {
        a->status = ACTION_DONE;
        g_state.current++;
        if (g_state.current >= g_state.count) {
            FinishSequence();
        }
    } else if (r == ACTION_FAILED) {
        a->status = ACTION_FAILED;
        AbortSequence();
    }
}

const ActionRunnerState *ActionRunner_GetState(void)
{
    return &g_state;
}

} /* namespace app */
