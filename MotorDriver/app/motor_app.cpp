#include "app/motor_app.h"

#include <stdint.h>

#include "board/board_encoders.h"
#include "board/board_uart.h"
#include "control/motor_control.h"
#include "protocol/register_map.h"
#include "protocol/serial_protocol.h"
#include "ti_msp_dl_config.h"

namespace app {
namespace {

constexpr int32_t kEncoderCountsPerMotorRev = 448;
constexpr int32_t kMotorGearRatio = 50;
constexpr int32_t kEncoderCountsPerOutputRev =
    kEncoderCountsPerMotorRev * kMotorGearRatio;

struct MotorFeedback {
    int16_t m1_rpm;
    int16_t m2_rpm;
};

protocol::RegisterMap g_registers;
protocol::SerialProtocol g_serial;
volatile uint32_t g_ms_ticks = 0U;
uint32_t g_last_successful_write_ms = 0U;
bool g_have_successful_write = false;
bool g_timeout_active = false;
bool g_output_dirty = true;

uint32_t Millis(void)
{
    return g_ms_ticks;
}

uint32_t WatchdogTimeoutMs(void)
{
    return static_cast<uint32_t>(g_registers.WatchdogTimeout10ms()) * 10U;
}

int16_t EncoderCountsPerSecondToRpm(int32_t counts_per_second)
{
    int64_t numerator = static_cast<int64_t>(counts_per_second) * 60;
    const int64_t denominator = kEncoderCountsPerOutputRev;

    if (numerator >= 0) {
        numerator += denominator / 2;
    } else {
        numerator -= denominator / 2;
    }

    int64_t rpm = numerator / denominator;
    if (rpm > 32767) {
        rpm = 32767;
    } else if (rpm < -32768) {
        rpm = -32768;
    }

    return static_cast<int16_t>(rpm);
}

void HandleSuccessfulWrite(void)
{
    g_last_successful_write_ms = Millis();
    g_have_successful_write = true;
    g_timeout_active = false;
    g_registers.SetTimeoutActive(false);
    g_output_dirty = true;
}

void CheckWatchdog(void)
{
    const uint32_t timeout_ms = WatchdogTimeoutMs();

    if ((!g_have_successful_write) || (timeout_ms == 0U)) {
        return;
    }

    if ((!g_timeout_active) &&
        ((Millis() - g_last_successful_write_ms) > timeout_ms)) {
        g_timeout_active = true;
        g_registers.ForceCoastOnTimeout();
        g_output_dirty = true;
    }
}

MotorFeedback SyncEncoderRegisters(void)
{
    const drivers::EncoderSnapshot m1 =
        board::BoardEncoders_GetSnapshot(drivers::EncoderId::Encoder1);
    const drivers::EncoderSnapshot m2 =
        board::BoardEncoders_GetSnapshot(drivers::EncoderId::Encoder2);
    const MotorFeedback feedback = {
        EncoderCountsPerSecondToRpm(m1.counts_per_second),
        EncoderCountsPerSecondToRpm(m2.counts_per_second),
    };

    g_registers.UpdateEncoderSnapshot(m1.count,
                                      m1.counts_per_second,
                                      m1.state,
                                      m2.count,
                                      m2.counts_per_second,
                                      m2.state);
    g_registers.UpdateMeasuredRpm(feedback.m1_rpm, feedback.m2_rpm);
    return feedback;
}

void HandleEncoderControl(void)
{
    const uint8_t control = g_registers.EncoderControl();

    if ((control & protocol::ENCODER_CONTROL_RESET_M1) != 0U) {
        board::BoardEncoders_Reset(drivers::EncoderId::Encoder1);
    }
    if ((control & protocol::ENCODER_CONTROL_RESET_M2) != 0U) {
        board::BoardEncoders_Reset(drivers::EncoderId::Encoder2);
    }

    if (control != 0U) {
        g_registers.ClearEncoderControl();
        (void) SyncEncoderRegisters();
    }
}

void ProcessSerialRx(void)
{
    uint8_t value = 0U;

    while (board::BoardUart_ReadByte(&value)) {
        protocol::SerialResponse response = {};
        bool write_committed = false;

        if (g_serial.ProcessByte(value, &response, &write_committed)) {
            (void) board::BoardUart_Write(response.bytes, response.length);
        }

        if (write_committed) {
            HandleEncoderControl();
            HandleSuccessfulWrite();
        }
    }
}

}  // namespace

void MotorApp_Init(void)
{
    g_registers.Init();
    g_serial.Init(&g_registers);

    control::MotorControl_Init();
    board::BoardEncoders_Init();
    (void) board::BoardUart_Init();

    SysTick_Config(CPUCLK_FREQ / 1000U);
    g_registers.RefreshStatus();
    (void) SyncEncoderRegisters();
    control::MotorControl_Apply(g_registers, true);
    g_output_dirty = false;
}

void MotorApp_Process(void)
{
    board::BoardEncoders_Process(Millis());
    const MotorFeedback feedback = SyncEncoderRegisters();
    ProcessSerialRx();
    CheckWatchdog();

    if ((!g_timeout_active) && g_registers.IsEnabled() &&
        control::MotorControl_UpdateSpeed(g_registers,
                                          feedback.m1_rpm,
                                          feedback.m2_rpm,
                                          Millis())) {
        g_output_dirty = true;
    }

    if (g_output_dirty) {
        g_registers.RefreshStatus();
        control::MotorControl_Apply(g_registers, g_timeout_active);
        g_output_dirty = false;
    }
}

void MotorApp_TickFromIsr(void)
{
    g_ms_ticks++;
}

}  // namespace app

extern "C" void SysTick_Handler(void)
{
    app::MotorApp_TickFromIsr();
}
