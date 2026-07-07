#include "drivers/quadrature_encoder.h"

#include "board/board_encoder_pins.h"

namespace drivers {
namespace {

constexpr uint32_t kCounterLoad = 0xFFFFU;
constexpr uint8_t kM1EdgeToQuadratureScale = 2U;

struct EncoderState {
    volatile int32_t count;
    int32_t counts_per_second;
    int32_t last_sample_count;
    uint32_t last_sample_ms;
    uint8_t state;
    bool gpio_enabled;
};

struct EdgeCounterState {
    uint32_t last_remaining;
    int32_t scaled_count;
};

struct QeiCounterState {
    uint16_t last_hw_count;
};

EncoderState g_m1 = {};
EncoderState g_m2 = {};
EdgeCounterState g_m1_edge = {};
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

void ApplyTransition(EncoderState *encoder, uint8_t next_state)
{
    static const int8_t kTransitionDelta[16] = {
        0,  1, -1, 0,
        -1, 0,  0, 1,
        1,  0,  0, -1,
        0, -1,  1, 0,
    };

    const uint8_t index = static_cast<uint8_t>(((encoder->state & 0x03U) << 2U) |
                                              (next_state & 0x03U));
    encoder->count += kTransitionDelta[index];
    encoder->state = static_cast<uint8_t>(next_state & 0x03U);
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

void ConfigureDigitalInput(uint32_t iomux)
{
    DL_GPIO_initDigitalInputFeatures(iomux,
                                     DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_ENABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
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

uint32_t ReadM1EdgeRemaining(void)
{
    return DL_TimerG_getTimerCount(BOARD_M1_EDGE_COUNTER_INST) & kCounterLoad;
}

uint16_t ReadM2QeiCounter(void)
{
    return static_cast<uint16_t>(
        DL_TimerG_getTimerCount(BOARD_M2_QEI_INST) & kCounterLoad);
}

uint32_t EdgeDelta(uint32_t previous, uint32_t current)
{
    if (previous >= current) {
        return previous - current;
    }

    return previous + (kCounterLoad - current) + 1U;
}

void ConfigureTimerPower(void)
{
    DL_TimerG_reset(BOARD_M1_EDGE_COUNTER_INST);
    DL_TimerG_reset(BOARD_M2_QEI_INST);
    DL_TimerG_enablePower(BOARD_M1_EDGE_COUNTER_INST);
    DL_TimerG_enablePower(BOARD_M2_QEI_INST);
}

void ConfigureM1GpioInterrupts(void)
{
    DL_GPIO_setLowerPinsInputFilter(GPIOB,
                                    DL_GPIO_PIN_7_INPUT_FILTER_3_CYCLES |
                                        DL_GPIO_PIN_9_INPUT_FILTER_3_CYCLES);
    DL_GPIO_setLowerPinsPolarity(GPIOB,
                                 DL_GPIO_PIN_7_EDGE_RISE_FALL |
                                     DL_GPIO_PIN_9_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOB,
                                 BOARD_M1_ENC_A_PIN | BOARD_M1_ENC_B_PIN);
    DL_GPIO_enableInterrupt(GPIOB, BOARD_M1_ENC_A_PIN | BOARD_M1_ENC_B_PIN);
    NVIC_ClearPendingIRQ(BOARD_ENCODER_IRQN);
    NVIC_EnableIRQ(BOARD_ENCODER_IRQN);
}

void ConfigureM1EdgeCounter(void)
{
    static DL_TimerG_ClockConfig kClockConfig = {
        DL_TIMER_CLOCK_BUSCLK,
        DL_TIMER_CLOCK_DIVIDE_1,
        0U,
    };
    static DL_TimerG_CompareConfig kCompareConfig = {
        DL_TIMER_COMPARE_MODE_EDGE_COUNT,
        kCounterLoad,
        DL_TIMER_COMPARE_EDGE_DETECTION_MODE_EDGE,
        BOARD_M1_EDGE_COUNTER_INPUT_CHAN,
        DL_TIMER_CC_INPUT_INV_NOINVERT,
        DL_TIMER_STOP,
    };

    ConfigurePeripheralInput(BOARD_M1_ENC_A_IOMUX,
                             BOARD_M1_EDGE_COUNTER_IOMUX_FUNC);
    ConfigureDigitalInput(BOARD_M1_ENC_B_IOMUX);

    DL_TimerG_stopCounter(BOARD_M1_EDGE_COUNTER_INST);
    DL_TimerG_setClockConfig(BOARD_M1_EDGE_COUNTER_INST, &kClockConfig);
    DL_TimerG_initCompareMode(
        BOARD_M1_EDGE_COUNTER_INST,
        &kCompareConfig);
    DL_TimerG_disableInterrupt(BOARD_M1_EDGE_COUNTER_INST, 0xFFFFFFFFU);
    DL_TimerG_enableClock(BOARD_M1_EDGE_COUNTER_INST);
    DL_TimerG_setTimerCount(BOARD_M1_EDGE_COUNTER_INST, kCounterLoad);
    DL_TimerG_startCounter(BOARD_M1_EDGE_COUNTER_INST);

    g_m1_edge.last_remaining = ReadM1EdgeRemaining();
    g_m1_edge.scaled_count = 0;
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

void SetM1GpioMode(void)
{
    DL_TimerG_stopCounter(BOARD_M1_EDGE_COUNTER_INST);
    ConfigureDigitalInput(BOARD_M1_ENC_A_IOMUX);
    ConfigureDigitalInput(BOARD_M1_ENC_B_IOMUX);
    g_m1.state = ReadM1State();
    ConfigureM1GpioInterrupts();
    g_m1.gpio_enabled = true;
    g_m1.counts_per_second = 0;
    g_m1.last_sample_count = g_m1.count;
    g_m1.last_sample_ms = 0U;
}

void SetM1EdgeMode(void)
{
    const uint32_t pins = BOARD_M1_ENC_A_PIN | BOARD_M1_ENC_B_PIN;

    DL_GPIO_disableInterrupt(GPIOB, pins);
    DL_GPIO_clearInterruptStatus(GPIOB, pins);
    ConfigureM1EdgeCounter();
    g_m1.gpio_enabled = false;
    g_m1.counts_per_second = 0;
    g_m1.last_sample_count = 0;
    g_m1.last_sample_ms = 0U;
}

void SyncM1EdgeCounter(void)
{
    const uint32_t current = ReadM1EdgeRemaining();
    const uint32_t raw_delta = EdgeDelta(g_m1_edge.last_remaining, current);
    g_m1_edge.last_remaining = current;

    const uint32_t scaled_delta =
        raw_delta * static_cast<uint32_t>(kM1EdgeToQuadratureScale);
    if (scaled_delta >
        static_cast<uint32_t>(2147483647 - g_m1_edge.scaled_count)) {
        g_m1_edge.scaled_count = 2147483647;
    } else {
        g_m1_edge.scaled_count += static_cast<int32_t>(scaled_delta);
    }
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

    ConfigureDigitalInput(BOARD_M1_ENC_A_IOMUX);
    ConfigureDigitalInput(BOARD_M1_ENC_B_IOMUX);
    g_m1.state = ReadM1State();
    g_m1.gpio_enabled = true;
    ConfigureM1GpioInterrupts();

    ConfigureM2Qei();
    g_m2.state = ReadM2State();
    g_m2.gpio_enabled = false;
}

void QuadratureEncoder_Process(uint32_t now_ms)
{
    if (g_m1.gpio_enabled) {
        int32_t count = 0;
        __disable_irq();
        count = g_m1.count;
        __enable_irq();
        UpdateSample(&g_m1, count, now_ms);
    } else {
        SyncM1EdgeCounter();
        UpdateSample(&g_m1, g_m1_edge.scaled_count, now_ms);
    }

    SyncM2QeiCounter();
    UpdateSample(&g_m2, g_m2.count, now_ms);
    g_m1.state = ReadM1State();
    g_m2.state = ReadM2State();
}

void QuadratureEncoder_SetEnabled(EncoderId encoder, bool enabled)
{
    if (encoder != EncoderId::Encoder1) {
        (void) enabled;
        return;
    }

    if (g_m1.gpio_enabled == enabled) {
        return;
    }

    __disable_irq();
    if (enabled) {
        SetM1GpioMode();
    } else {
        SetM1EdgeMode();
    }
    __enable_irq();
}

void QuadratureEncoder_HandleGpioInterrupt(void)
{
    const uint32_t m1_status =
        DL_GPIO_getEnabledInterruptStatus(GPIOB,
                                          BOARD_M1_ENC_A_PIN |
                                              BOARD_M1_ENC_B_PIN);

    if (m1_status != 0U) {
        if (g_m1.gpio_enabled) {
            ApplyTransition(&g_m1, ReadM1State());
        }
        DL_GPIO_clearInterruptStatus(GPIOB, m1_status);
    }
}

void QuadratureEncoder_Reset(EncoderId encoder)
{
    if (encoder == EncoderId::Encoder1) {
        ResetState(&g_m1, ReadM1State(), g_m1.last_sample_ms);
        if (!g_m1.gpio_enabled) {
            g_m1_edge.scaled_count = 0;
            g_m1_edge.last_remaining = ReadM1EdgeRemaining();
        }
    } else {
        ResetState(&g_m2, ReadM2State(), g_m2.last_sample_ms);
        g_m2_qei.last_hw_count = ReadM2QeiCounter();
    }
}

EncoderSnapshot QuadratureEncoder_GetSnapshot(EncoderId encoder)
{
    EncoderState *state = (encoder == EncoderId::Encoder1) ? &g_m1 : &g_m2;
    EncoderSnapshot snapshot = {};

    if ((encoder == EncoderId::Encoder1) && (!g_m1.gpio_enabled)) {
        SyncM1EdgeCounter();
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
