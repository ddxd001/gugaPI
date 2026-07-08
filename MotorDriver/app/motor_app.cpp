#include "app/motor_app.h"

#include <stdint.h>

#include "board/board_encoders.h"
#include "board/board_i2c_target.h"
#include "board/board_uart.h"
#include "control/motor_control.h"
#include "protocol/register_map.h"
#include "protocol/serial_protocol.h"
#include "ti_msp_dl_config.h"

namespace app {
namespace {

struct MotorFeedback {
    int32_t m1_count;
    int16_t m1_rpm;
    int32_t m2_count;
    int16_t m2_rpm;
};

constexpr uint32_t kM1GpioReenableDelayAfterFastModeMs = 3000U;

protocol::RegisterMap g_registers;
protocol::SerialProtocol g_serial;
volatile uint32_t g_ms_ticks = 0U;
uint32_t g_last_watchdog_refresh_ms = 0U;
uint32_t g_m1_encoder_cooldown_start_ms = 0U;
bool g_watchdog_armed = false;
bool g_timeout_active = false;
bool g_output_dirty = true;
bool g_m1_encoder_cooldown_active = false;

uint32_t Millis(void)
{
    return g_ms_ticks;
}

uint32_t WatchdogTimeoutMs(void)
{
    return static_cast<uint32_t>(g_registers.WatchdogTimeout10ms()) * 10U;
}

int16_t EncoderCountsPerSecondToRpm(int32_t counts_per_second,
                                    uint32_t counts_per_rev)
{
    if (counts_per_rev == 0U) {
        return 0;
    }

    int64_t numerator = static_cast<int64_t>(counts_per_second) * 60;
    const int64_t denominator = counts_per_rev;

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

int32_t NegateInt32Saturating(int32_t value)
{
    if (value == static_cast<int32_t>(-2147483647 - 1)) {
        return 2147483647;
    }
    return -value;
}

void RefreshWatchdogLeaseFromWrite(void)
{
    g_timeout_active = false;
    g_registers.SetTimeoutActive(false);
    if (g_registers.HasActiveCommand()) {
        g_last_watchdog_refresh_ms = Millis();
        g_watchdog_armed = true;
    } else {
        g_watchdog_armed = false;
    }
    g_output_dirty = true;
}

void RefreshWatchdogLeaseFromKeepalive(void)
{
    if ((!g_timeout_active) && g_registers.HasActiveCommand()) {
        g_last_watchdog_refresh_ms = Millis();
        g_watchdog_armed = true;
    }
}

void CheckWatchdog(void)
{
    const uint32_t timeout_ms = WatchdogTimeoutMs();

    if ((timeout_ms == 0U) || (!g_registers.HasActiveCommand())) {
        g_watchdog_armed = false;
        return;
    }

    if (!g_watchdog_armed) {
        g_last_watchdog_refresh_ms = Millis();
        g_watchdog_armed = true;
        return;
    }

    if ((!g_timeout_active) && g_watchdog_armed &&
        ((Millis() - g_last_watchdog_refresh_ms) > timeout_ms)) {
        g_timeout_active = true;
        g_watchdog_armed = false;
        g_registers.ForceCoastOnTimeout();
        g_output_dirty = true;
    }
}

bool IsOpenLoopRunActive(uint8_t mode, uint8_t duty)
{
    return (mode == protocol::MOTOR_MODE_RUN) && (duty > 0U);
}

bool IsM1FastEncoderMode(uint8_t mode, uint8_t duty)
{
    if (IsOpenLoopRunActive(mode, duty)) {
        return true;
    }

    return (mode == protocol::MOTOR_MODE_SPEED) &&
           (g_registers.M1TargetRpm() > 0U);
}

bool M1NeedsGpioQuadrature(uint8_t mode)
{
    if (mode == protocol::MOTOR_MODE_POSITION) {
        return true;
    }

    return (mode == protocol::MOTOR_MODE_SPEED) &&
           (g_registers.M1TargetRpm() == 0U);
}

int16_t ApplyM1FastEncoderDirection(int16_t rpm)
{
    const uint8_t mode = g_registers.Read(protocol::REG_M1_MODE);
    const uint8_t duty = g_registers.Read(protocol::REG_M1_DUTY);
    const bool m1_fast_encoder =
        IsM1FastEncoderMode(mode, duty) ||
        (g_m1_encoder_cooldown_active && (!M1NeedsGpioQuadrature(mode)));

    if (!m1_fast_encoder) {
        return rpm;
    }

    int32_t magnitude = rpm;
    if (magnitude < 0) {
        magnitude = -magnitude;
    }
    if (magnitude > 32767) {
        magnitude = 32767;
    }

    if (g_registers.Read(protocol::REG_M1_DIRECTION) ==
        protocol::MOTOR_DIRECTION_REVERSE) {
        return static_cast<int16_t>(-magnitude);
    }
    return static_cast<int16_t>(magnitude);
}

void UpdateEncoderModes(void)
{
    const uint32_t now = Millis();
    const uint8_t m1_mode = g_registers.Read(protocol::REG_M1_MODE);
    const uint8_t m1_duty = g_registers.Read(protocol::REG_M1_DUTY);

    if (M1NeedsGpioQuadrature(m1_mode)) {
        g_m1_encoder_cooldown_active = false;
        board::BoardEncoders_SetEnabled(drivers::EncoderId::Encoder1, true);
    } else if (IsM1FastEncoderMode(m1_mode, m1_duty)) {
        g_m1_encoder_cooldown_start_ms = now;
        g_m1_encoder_cooldown_active = true;
        board::BoardEncoders_SetEnabled(drivers::EncoderId::Encoder1, false);
    } else if (g_m1_encoder_cooldown_active) {
        if ((now - g_m1_encoder_cooldown_start_ms) >=
            kM1GpioReenableDelayAfterFastModeMs) {
            g_m1_encoder_cooldown_active = false;
            board::BoardEncoders_SetEnabled(drivers::EncoderId::Encoder1, true);
        } else {
            board::BoardEncoders_SetEnabled(drivers::EncoderId::Encoder1, false);
        }
    } else {
        board::BoardEncoders_SetEnabled(drivers::EncoderId::Encoder1, true);
    }

    board::BoardEncoders_SetEnabled(drivers::EncoderId::Encoder2, true);
}

MotorFeedback SyncEncoderRegisters(void)
{
    const drivers::EncoderSnapshot m1 =
        board::BoardEncoders_GetSnapshot(drivers::EncoderId::Encoder1);
    const drivers::EncoderSnapshot m2 =
        board::BoardEncoders_GetSnapshot(drivers::EncoderId::Encoder2);
    int32_t m1_count = m1.count;
    int32_t m1_cps = m1.counts_per_second;
    int32_t m2_count = m2.count;
    int32_t m2_cps = m2.counts_per_second;

    if (g_registers.EncoderInverted(true)) {
        m1_count = NegateInt32Saturating(m1_count);
        m1_cps = NegateInt32Saturating(m1_cps);
    }
    if (g_registers.EncoderInverted(false)) {
        m2_count = NegateInt32Saturating(m2_count);
        m2_cps = NegateInt32Saturating(m2_cps);
    }

    int16_t m1_rpm =
        EncoderCountsPerSecondToRpm(m1_cps,
                                    g_registers.M1CountsPerRev());
    const int16_t m2_rpm =
        EncoderCountsPerSecondToRpm(m2_cps,
                                    g_registers.M2CountsPerRev());
    m1_rpm = ApplyM1FastEncoderDirection(m1_rpm);

    const MotorFeedback feedback = {
        m1_count,
        m1_rpm,
        m2_count,
        m2_rpm,
    };

    g_registers.UpdateEncoderSnapshot(m1_count,
                                      m1_cps,
                                      m1.state,
                                      m2_count,
                                      m2_cps,
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
        bool keepalive_received = false;

        if (g_serial.ProcessByte(value,
                                 &response,
                                 &write_committed,
                                 &keepalive_received)) {
            (void) board::BoardUart_Write(response.bytes, response.length);
        }

        if (keepalive_received) {
            RefreshWatchdogLeaseFromKeepalive();
        }

        if (write_committed) {
            HandleEncoderControl();
            RefreshWatchdogLeaseFromWrite();
        }
    }
}

void ProcessI2cWrites(void)
{
    board::BoardI2cTargetWrite write = {};

    while (board::BoardI2cTarget_TakeWrite(&write)) {
        bool control_changed = false;
        if (g_registers.CommitWrite(write.reg,
                                    write.data,
                                    write.length,
                                    &control_changed)) {
            (void) control_changed;
            HandleEncoderControl();
            RefreshWatchdogLeaseFromWrite();
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
    board::BoardI2cTarget_Init(&g_registers);
    g_registers.SetI2cAddress(board::BoardI2cTarget_Address());

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
    ProcessI2cWrites();
    CheckWatchdog();
    UpdateEncoderModes();

    if ((!g_timeout_active) && g_registers.IsEnabled() &&
        control::MotorControl_UpdateSpeed(g_registers,
                                          feedback.m1_count,
                                          feedback.m1_rpm,
                                          feedback.m2_count,
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
