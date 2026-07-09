#include "drivers/quadrature_encoder.h"

#include "board/board_encoder_pins.h"

namespace drivers {
namespace {

constexpr uint32_t kCounterLoad = 0xFFFFU;

struct EncoderState {
    volatile int32_t count;
    int32_t counts_per_second;
    int32_t last_sample_count;
    uint32_t last_sample_ms;
    uint8_t state;
};

struct QeiCounterState {
    uint16_t last_hw_count;
};

EncoderState g_m1 = {};
EncoderState g_m2 = {};
QeiCounterState g_m1_qei = {};
QeiCounterState g_m2_qei = {};

uint8_t ReadM1State(void)
{
    uint8_t state = 0U;

    if ((DL_GPIO_readPins(BOARD_M1_ENC_A_PORT, BOARD_M1_ENC_A_PIN) &
         BOARD_M1_ENC_A_PIN) != 0U) {
        state = static_cast<uint8_t>(state | 0x01U);
    }
    if ((DL_GPIO_readPins(BOARD_M1_ENC_B_PORT, BOARD_M1_ENC_B_PIN) &
         BOARD_M1_ENC_B_PIN) != 0U) {
        state = static_cast<uint8_t>(state | 0x02U);
    }

    return state;
}

uint8_t ReadM2State(void)
{
    uint8_t state = 0U;

    if ((DL_GPIO_readPins(BOARD_M2_ENC_A_PORT, BOARD_M2_ENC_A_PIN) &
         BOARD_M2_ENC_A_PIN) != 0U) {
        state = static_cast<uint8_t>(state | 0x01U);
    }
    if ((DL_GPIO_readPins(BOARD_M2_ENC_B_PORT, BOARD_M2_ENC_B_PIN) &
         BOARD_M2_ENC_B_PIN) != 0U) {
        state = static_cast<uint8_t>(state | 0x02U);
    }

    return state;
}

void UpdateSample(EncoderState *encoder, int32_t current_count, uint32_t now_ms)
{
    if (encoder->last_sample_ms == 0U) {
        encoder->last_sample_count = current_count;
        encoder->last_sample_ms = now_ms;
        encoder->counts_per_second = 0;
        return;
    }

    const uint32_t elapsed_ms = now_ms - encoder->last_sample_ms;
    if (elapsed_ms < board::kEncoderSamplePeriodMs) {
        return;
    }

    const int32_t delta = current_count - encoder->last_sample_count;
    encoder->counts_per_second =
        static_cast<int32_t>((static_cast<int64_t>(delta) * 1000) /
                             static_cast<int64_t>(elapsed_ms));
    encoder->last_sample_count = current_count;
    encoder->last_sample_ms = now_ms;
}

void ResetState(EncoderState *encoder, uint8_t state, uint32_t now_ms)
{
    __disable_irq();
    encoder->count = 0;
    __enable_irq();

    encoder->counts_per_second = 0;
    encoder->last_sample_count = 0;
    encoder->last_sample_ms = now_ms;
    encoder->state = static_cast<uint8_t>(state & 0x03U);
}

void ConfigurePeripheralInput(uint32_t iomux, uint32_t function)
{
    DL_GPIO_initPeripheralInputFunctionFeatures(iomux,
                                                function,
                                                DL_GPIO_INVERSION_DISABLE,
                                                DL_GPIO_RESISTOR_PULL_UP,
                                                DL_GPIO_HYSTERESIS_ENABLE,
                                                DL_GPIO_WAKEUP_DISABLE);
}

uint16_t ReadM1QeiCounter(void)
{
    return static_cast<uint16_t>(
        DL_TimerG_getTimerCount(BOARD_M1_QEI_INST) & kCounterLoad);
}

uint16_t ReadM2QeiCounter(void)
{
    return static_cast<uint16_t>(
        DL_TimerG_getTimerCount(BOARD_M2_QEI_INST) & kCounterLoad);
}

void ConfigureTimerPower(void)
{
    DL_TimerG_reset(BOARD_M1_QEI_INST);
    DL_TimerG_reset(BOARD_M2_QEI_INST);
    DL_TimerG_enablePower(BOARD_M1_QEI_INST);
    DL_TimerG_enablePower(BOARD_M2_QEI_INST);
}

void ConfigureM1Qei(void)
{
    static DL_TimerG_ClockConfig kClockConfig = {
        DL_TIMER_CLOCK_BUSCLK,
        DL_TIMER_CLOCK_DIVIDE_1,
        0U,
    };

    ConfigurePeripheralInput(BOARD_M1_QEI_PHA_IOMUX,
                             BOARD_M1_QEI_PHA_IOMUX_FUNC);
    ConfigurePeripheralInput(BOARD_M1_QEI_PHB_IOMUX,
                             BOARD_M1_QEI_PHB_IOMUX_FUNC);

    DL_TimerG_stopCounter(BOARD_M1_QEI_INST);
    DL_TimerG_setClockConfig(BOARD_M1_QEI_INST, &kClockConfig);
    DL_TimerG_configQEI(BOARD_M1_QEI_INST,
                        DL_TIMER_QEI_MODE_2_INPUT,
                        DL_TIMER_CC_INPUT_INV_NOINVERT,
                        DL_TIMER_CC_0_INDEX);
    DL_TimerG_setLoadValue(BOARD_M1_QEI_INST, kCounterLoad);
    DL_TimerG_setTimerCount(BOARD_M1_QEI_INST, 0U);
    DL_TimerG_disableInterrupt(BOARD_M1_QEI_INST, 0xFFFFFFFFU);
    DL_TimerG_enableClock(BOARD_M1_QEI_INST);
    DL_TimerG_startCounter(BOARD_M1_QEI_INST);

    g_m1_qei.last_hw_count = ReadM1QeiCounter();
}

void ConfigureM2Qei(void)
{
    static DL_TimerG_ClockConfig kClockConfig = {
        DL_TIMER_CLOCK_BUSCLK,
        DL_TIMER_CLOCK_DIVIDE_1,
        0U,
    };

    ConfigurePeripheralInput(BOARD_M2_QEI_PHA_IOMUX,
                             BOARD_M2_QEI_PHA_IOMUX_FUNC);
    ConfigurePeripheralInput(BOARD_M2_QEI_PHB_IOMUX,
                             BOARD_M2_QEI_PHB_IOMUX_FUNC);

    DL_TimerG_stopCounter(BOARD_M2_QEI_INST);
    DL_TimerG_setClockConfig(BOARD_M2_QEI_INST, &kClockConfig);
    DL_TimerG_configQEI(BOARD_M2_QEI_INST,
                        DL_TIMER_QEI_MODE_2_INPUT,
                        DL_TIMER_CC_INPUT_INV_NOINVERT,
                        DL_TIMER_CC_0_INDEX);
    DL_TimerG_setLoadValue(BOARD_M2_QEI_INST, kCounterLoad);
    DL_TimerG_setTimerCount(BOARD_M2_QEI_INST, 0U);
    DL_TimerG_disableInterrupt(BOARD_M2_QEI_INST, 0xFFFFFFFFU);
    DL_TimerG_enableClock(BOARD_M2_QEI_INST);
    DL_TimerG_startCounter(BOARD_M2_QEI_INST);

    g_m2_qei.last_hw_count = ReadM2QeiCounter();
}

void SyncM1QeiCounter(void)
{
    const uint16_t current = ReadM1QeiCounter();
    const int16_t raw_delta =
        static_cast<int16_t>(current - g_m1_qei.last_hw_count);
    g_m1_qei.last_hw_count = current;
    g_m1.count +=
        static_cast<int32_t>(raw_delta) *
        static_cast<int32_t>(board::kM1QeiCountSign);
}

void SyncM2QeiCounter(void)
{
    const uint16_t current = ReadM2QeiCounter();
    const int16_t raw_delta =
        static_cast<int16_t>(current - g_m2_qei.last_hw_count);
    g_m2_qei.last_hw_count = current;
    g_m2.count +=
        static_cast<int32_t>(raw_delta) *
        static_cast<int32_t>(board::kM2QeiCountSign);
}

}  // namespace

void QuadratureEncoder_Init(void)
{
    ConfigureTimerPower();

    ConfigureM1Qei();
    g_m1.state = ReadM1State();

    ConfigureM2Qei();
    g_m2.state = ReadM2State();
}

void QuadratureEncoder_Process(uint32_t now_ms)
{
    SyncM1QeiCounter();
    SyncM2QeiCounter();
    UpdateSample(&g_m1, g_m1.count, now_ms);
    UpdateSample(&g_m2, g_m2.count, now_ms);
    g_m1.state = ReadM1State();
    g_m2.state = ReadM2State();
}

void QuadratureEncoder_SetEnabled(EncoderId encoder, bool enabled)
{
    (void) encoder;
    (void) enabled;
}

void QuadratureEncoder_HandleGpioInterrupt(void)
{
}

void QuadratureEncoder_Reset(EncoderId encoder)
{
    if (encoder == EncoderId::Encoder1) {
        ResetState(&g_m1, ReadM1State(), g_m1.last_sample_ms);
        g_m1_qei.last_hw_count = ReadM1QeiCounter();
    } else {
        ResetState(&g_m2, ReadM2State(), g_m2.last_sample_ms);
        g_m2_qei.last_hw_count = ReadM2QeiCounter();
    }
}

EncoderSnapshot QuadratureEncoder_GetSnapshot(EncoderId encoder)
{
    EncoderState *state = (encoder == EncoderId::Encoder1) ? &g_m1 : &g_m2;
    EncoderSnapshot snapshot = {};

    if (encoder == EncoderId::Encoder1) {
        SyncM1QeiCounter();
    } else if (encoder == EncoderId::Encoder2) {
        SyncM2QeiCounter();
    }

    __disable_irq();
    snapshot.count = state->count;
    __enable_irq();

    snapshot.counts_per_second = state->counts_per_second;
    snapshot.state = state->state;

    return snapshot;
}

}  // namespace drivers
