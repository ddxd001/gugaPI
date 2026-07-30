#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/config_store.h"
#include "app/dm_g6220_controller.h"
#include "board/board_can.h"
#include "services/fault.h"
#include "services/time.h"

namespace {

uint32_t g_now = 0U;
services::FaultCode g_fault = services::FAULT_NONE;
app::ConfigStoreParams g_params = {};
drivers::CanStatus g_can_status = {};
drivers::DriverStatus g_send_status = drivers::DRIVER_OK;
drivers::CanFrame g_sent[256] = {};
uint16_t g_sent_count = 0U;

uint32_t ToUnsigned(int32_t value, int32_t minimum, int32_t maximum,
                    uint8_t bits)
{
    if (value < minimum) value = minimum;
    if (value > maximum) value = maximum;
    const uint32_t full_scale = (1UL << bits) - 1UL;
    return static_cast<uint32_t>(
        (static_cast<int64_t>(value - minimum) * full_scale +
         (maximum - minimum) / 2) /
        (maximum - minimum));
}

void Feed(int32_t position_mrad, int32_t velocity_mrad_s, uint8_t state)
{
    const uint32_t position = ToUnsigned(
        position_mrad, -drivers::DM_G6220_POSITION_LIMIT_MRAD,
        drivers::DM_G6220_POSITION_LIMIT_MRAD, 16U);
    const uint32_t velocity = ToUnsigned(
        velocity_mrad_s, -drivers::DM_G6220_VELOCITY_LIMIT_MRAD_S,
        drivers::DM_G6220_VELOCITY_LIMIT_MRAD_S, 12U);
    const uint32_t torque = ToUnsigned(
        0, -drivers::DM_G6220_TORQUE_LIMIT_MNM,
        drivers::DM_G6220_TORQUE_LIMIT_MNM, 12U);
    drivers::CanFrame frame = {};
    frame.id = 0x000U;
    frame.length = 8U;
    frame.data[0] = static_cast<uint8_t>((state << 4U) | 0x01U);
    frame.data[1] = static_cast<uint8_t>(position >> 8U);
    frame.data[2] = static_cast<uint8_t>(position);
    frame.data[3] = static_cast<uint8_t>(velocity >> 4U);
    frame.data[4] = static_cast<uint8_t>(
        ((velocity & 0x0FU) << 4U) | (torque >> 8U));
    frame.data[5] = static_cast<uint8_t>(torque);
    frame.data[6] = 35U;
    frame.data[7] = 34U;
    assert(app::DmG6220Controller_ProcessFrame(&frame, g_now) ==
           drivers::DM_G6220_FRAME_FEEDBACK);
}

void Reset()
{
    g_now = 0U;
    g_fault = services::FAULT_NONE;
    g_can_status = {};
    g_can_status.initialized = true;
    g_send_status = drivers::DRIVER_OK;
    g_sent_count = 0U;
    memset(g_sent, 0, sizeof(g_sent));
    g_params = {};
    g_params.dm_position_kp_milli = 4000U;
    g_params.dm_position_kd_milli = 400U;
    g_params.dm_speed_kd_milli = 500U;
    g_params.dm_max_velocity_mrad_s = 2000U;
    g_params.dm_max_tracking_error_mrad = 250U;
    g_params.dm_speed_slew_mrad_s2 = 2000U;
    g_params.dm_position_tolerance_mrad = 10U;
    g_params.dm_velocity_tolerance_mrad_s = 80U;
    g_params.dm_settle_ms = 200U;
    g_params.dm_feedback_timeout_ms = 100U;
    app::DmG6220Controller_Init();
}

void AdvanceBootToReady()
{
    assert(app::DmG6220Controller_GetState()->mode ==
           app::DM_CONTROL_BOOT_WAIT);
    app::DmG6220Controller_Update();
    assert(g_sent_count == 0U);
    g_now = 1000U;
    app::DmG6220Controller_Update();
    app::DmG6220Controller_Update();
    g_now += 20U;
    app::DmG6220Controller_Update();
    g_now += 20U;
    app::DmG6220Controller_Update();
    app::DmG6220Controller_Update();
    app::DmG6220Controller_Update();
    g_now += 20U;
    app::DmG6220Controller_Update();
    g_now += 20U;
    app::DmG6220Controller_Update();
    assert(g_sent_count == 6U);
    for (uint8_t index = 0U; index < 3U; index++) {
        assert(g_sent[index].data[7] == 0xFBU);
    }
    for (uint8_t index = 3U; index < 6U; index++) {
        assert(g_sent[index].data[7] == 0xFDU);
    }
    g_now += 50U;
    app::DmG6220Controller_Update();
    assert(g_sent_count == 7U);
    assert(g_sent[6].data[0] == 0x80U);
    Feed(0, 0, 0U);
    app::DmG6220Controller_Update();
    assert(app::DmG6220Controller_GetState()->mode ==
           app::DM_CONTROL_READY);
    assert(!app::DmG6220Controller_GetState()->enabled);
}

} /* namespace */

namespace services {

uint32_t Time_Millis(void) { return g_now; }
void Fault_Set(FaultCode code)
{
    if ((g_fault == FAULT_NONE) && (code != FAULT_NONE)) g_fault = code;
}
FaultCode Fault_Get(void) { return g_fault; }
bool Fault_HasFault(void) { return g_fault != FAULT_NONE; }

} /* namespace services */

namespace app {

const ConfigStoreParams *ConfigStore_Get(void) { return &g_params; }

} /* namespace app */

namespace board {

drivers::DriverStatus Board_CanSend(const drivers::CanFrame *frame)
{
    if (g_send_status != drivers::DRIVER_OK) {
        const drivers::DriverStatus status = g_send_status;
        g_send_status = drivers::DRIVER_OK;
        return status;
    }
    assert(frame != 0);
    assert(g_sent_count < 256U);
    g_sent[g_sent_count++] = *frame;
    return drivers::DRIVER_OK;
}

drivers::DriverStatus Board_CanGetStatus(drivers::CanStatus *status)
{
    assert(status != 0);
    *status = g_can_status;
    return drivers::DRIVER_OK;
}

} /* namespace board */

int main()
{
    using namespace app;

    /* Disabled READY mode keeps feedback alive without enabling the motor. */
    Reset();
    AdvanceBootToReady();
    const uint16_t ready_sent_count = g_sent_count;
    g_now += 50U;
    DmG6220Controller_Update();
    assert(g_sent_count == ready_sent_count + 1U);
    assert(g_sent[g_sent_count - 1U].data[0] == 0x80U);
    assert(g_sent[g_sent_count - 1U].data[1] == 0x00U);
    assert(!DmG6220Controller_GetState()->enabled);
    Feed(25, 0, 0U);
    DmG6220Controller_Update();
    assert(DmG6220Controller_IsFeedbackFresh(g_now));
    assert(DmG6220Controller_GetState()->mode == DM_CONTROL_READY);

    /* A stale idle sample starts a safe probe and resumes the motion request. */
    Reset();
    AdvanceBootToReady();
    g_now += 101U;
    assert(DmG6220Controller_StartPosition(
               DM_POSITION_RELATIVE, 100, 2000, 5000U) ==
           drivers::DRIVER_OK);
    assert(g_fault == services::FAULT_NONE);
    assert(DmG6220Controller_GetState()->mode == DM_CONTROL_PROBING);
    assert(DmG6220Controller_GetState()->operation_result ==
           DM_OPERATION_RUNNING);
    DmG6220Controller_Update();
    Feed(50, 0, 0U);
    DmG6220Controller_Update();
    assert(g_fault == services::FAULT_NONE);
    assert(DmG6220Controller_GetState()->mode == DM_CONTROL_ENABLING);
    assert(DmG6220Controller_GetState()->target_position_mrad == 150);

    /* A real disconnect still escalates once the automatic probe expires. */
    Reset();
    AdvanceBootToReady();
    g_now += 101U;
    assert(DmG6220Controller_StartPosition(
               DM_POSITION_RELATIVE, 100, 2000, 5000U) ==
           drivers::DRIVER_OK);
    g_now += 501U;
    DmG6220Controller_Update();
    assert(g_fault == services::FAULT_DM_TIMEOUT);
    assert(DmG6220Controller_GetState()->operation_result ==
           DM_OPERATION_FAULT);
    assert(!DmG6220Controller_GetState()->enabled);

    /* The configurable motion ceiling accepts the requested 20 rad/s. */
    Reset();
    AdvanceBootToReady();
    g_params.dm_max_velocity_mrad_s = 20000U;
    Feed(0, 0, 1U);
    assert(DmG6220Controller_StartSpeed(20000) == drivers::DRIVER_OK);
    assert(DmG6220Controller_GetState()->target_velocity_mrad_s == 20000);
    assert(DmG6220Controller_GetState()->mode == DM_CONTROL_SPEED);

    Reset();
    AdvanceBootToReady();
    assert(g_fault == services::FAULT_NONE);
    assert(DmG6220Controller_EnableHold() == drivers::DRIVER_OK);
    DmG6220Controller_Update();
    g_now += 20U;
    DmG6220Controller_Update();
    g_now += 20U;
    DmG6220Controller_Update();
    Feed(0, 0, 1U);
    DmG6220Controller_Update();
    assert(DmG6220Controller_GetState()->mode == DM_CONTROL_HOLD);

    assert(DmG6220Controller_StartPosition(
               DM_POSITION_RELATIVE, 100, 2000, 5000U) ==
           drivers::DRIVER_OK);
    for (uint8_t index = 0U; index < 30U; index++) {
        g_now += 10U;
        const int32_t reference =
            DmG6220Controller_GetState()->reference_position_mrad;
        Feed(reference < 100 ? reference : 100, 0, 1U);
        DmG6220Controller_Update();
    }
    assert(DmG6220Controller_GetState()->mode == DM_CONTROL_HOLD);
    assert(DmG6220Controller_GetState()->operation_result ==
           DM_OPERATION_SUCCESS);

    assert(DmG6220Controller_StartPosition(
               DM_POSITION_RELATIVE, 100, 2000, 5000U) ==
           drivers::DRIVER_OK);
    const int32_t reference_before =
        DmG6220Controller_GetState()->reference_position_mrad;
    g_send_status = drivers::DRIVER_ERROR_BUSY;
    g_now += 10U;
    Feed(reference_before, 0, 1U);
    DmG6220Controller_Update();
    assert(DmG6220Controller_GetState()->reference_position_mrad ==
           reference_before);
    assert(DmG6220Controller_GetState()->tx_busy_count == 1U);

    Reset();
    AdvanceBootToReady();
    Feed(0, 0, 1U);
    assert(DmG6220Controller_StartSpeed(200) == drivers::DRIVER_OK);
    g_now += 101U;
    DmG6220Controller_Update();
    assert(g_fault == services::FAULT_DM_TIMEOUT);
    assert(DmG6220Controller_GetState()->operation_result ==
           DM_OPERATION_FAULT);
    assert(!DmG6220Controller_GetState()->enabled);
    DmG6220Controller_Update();
    assert(g_sent[g_sent_count - 1U].data[7] == 0xFDU);

    Reset();
    AdvanceBootToReady();
    Feed(0, 0, 9U);
    DmG6220Controller_Update();
    assert(g_fault == services::FAULT_DM_MOTOR);

    Reset();
    AdvanceBootToReady();
    Feed(0, 0, 1U);
    assert(DmG6220Controller_StartSpeed(-200) == drivers::DRIVER_OK);
    g_can_status.bus_off = true;
    DmG6220Controller_Update();
    assert(g_fault == services::FAULT_DM_TIMEOUT);

    Reset();
    AdvanceBootToReady();
    assert(DmG6220Controller_SetZero() == drivers::DRIVER_OK);
    DmG6220Controller_Update();
    DmG6220Controller_Update();
    assert(g_sent[g_sent_count - 1U].data[7] == 0xFEU);

    /* The ball controller owns a continuous position reference exclusively. */
    Reset();
    AdvanceBootToReady();
    Feed(100, 0, 1U);
    assert(DmG6220Controller_ExternalAcquire(
               DM_EXTERNAL_OWNER_BALL_BALANCE) == drivers::DRIVER_OK);
    assert(DmG6220Controller_GetState()->mode ==
           DM_CONTROL_EXTERNAL_POSITION);
    assert(DmG6220Controller_GetState()->external_owner ==
           DM_EXTERNAL_OWNER_BALL_BALANCE);
    assert(DmG6220Controller_StartSpeed(100) ==
           drivers::DRIVER_ERROR_BUSY);
    assert(DmG6220Controller_ExternalSetPosition(
               DM_EXTERNAL_OWNER_BALL_BALANCE, 350) ==
           drivers::DRIVER_OK);
    g_now += 10U;
    Feed(100, 0, 1U);
    DmG6220Controller_Update();
    assert(DmG6220Controller_GetState()->reference_position_mrad > 100);
    assert(DmG6220Controller_ExternalRelease(
               DM_EXTERNAL_OWNER_BALL_BALANCE, false) ==
           drivers::DRIVER_OK);
    assert(DmG6220Controller_GetState()->mode == DM_CONTROL_HOLD);
    assert(DmG6220Controller_GetState()->external_owner ==
           DM_EXTERNAL_OWNER_NONE);

    puts("dm g6220 controller ok");
    return 0;
}
