#include "drivers/quadrature_encoder.h"

#include "board/board_encoder_pins.h"

namespace drivers {
namespace {

struct EncoderState {
    volatile int32_t count;
    int32_t counts_per_second;
    int32_t last_sample_count;
    uint32_t last_sample_ms;
    uint8_t state;
};

EncoderState g_m1 = {};
EncoderState g_m2 = {};

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

void UpdateSample(EncoderState *encoder, uint32_t now_ms)
{
    const uint32_t elapsed_ms = now_ms - encoder->last_sample_ms;

    if (elapsed_ms < board::kEncoderSamplePeriodMs) {
        return;
    }

    __disable_irq();
    const int32_t count = encoder->count;
    __enable_irq();

    const int32_t delta = count - encoder->last_sample_count;
    encoder->counts_per_second =
        static_cast<int32_t>((static_cast<int64_t>(delta) * 1000) /
                             static_cast<int64_t>(elapsed_ms));
    encoder->last_sample_count = count;
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

void ConfigureInput(uint32_t iomux)
{
    DL_GPIO_initDigitalInputFeatures(iomux,
                                     DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_ENABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
}

void ConfigureInterrupts(void)
{
    DL_GPIO_setLowerPinsInputFilter(GPIOB,
                                    DL_GPIO_PIN_7_INPUT_FILTER_3_CYCLES |
                                        DL_GPIO_PIN_9_INPUT_FILTER_3_CYCLES);
    DL_GPIO_setUpperPinsInputFilter(GPIOA,
                                    DL_GPIO_PIN_21_INPUT_FILTER_3_CYCLES |
                                        DL_GPIO_PIN_22_INPUT_FILTER_3_CYCLES);

    DL_GPIO_setLowerPinsPolarity(GPIOB,
                                 DL_GPIO_PIN_7_EDGE_RISE_FALL |
                                     DL_GPIO_PIN_9_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsPolarity(GPIOA,
                                 DL_GPIO_PIN_21_EDGE_RISE_FALL |
                                     DL_GPIO_PIN_22_EDGE_RISE_FALL);

    DL_GPIO_clearInterruptStatus(GPIOB,
                                 BOARD_M1_ENC_A_PIN | BOARD_M1_ENC_B_PIN);
    DL_GPIO_clearInterruptStatus(GPIOA,
                                 BOARD_M2_ENC_A_PIN | BOARD_M2_ENC_B_PIN);

    DL_GPIO_enableInterrupt(GPIOB, BOARD_M1_ENC_A_PIN | BOARD_M1_ENC_B_PIN);
    DL_GPIO_enableInterrupt(GPIOA, BOARD_M2_ENC_A_PIN | BOARD_M2_ENC_B_PIN);

    NVIC_ClearPendingIRQ(BOARD_ENCODER_IRQN);
    NVIC_EnableIRQ(BOARD_ENCODER_IRQN);
}

}  // namespace

void QuadratureEncoder_Init(void)
{
    ConfigureInput(BOARD_M1_ENC_A_IOMUX);
    ConfigureInput(BOARD_M1_ENC_B_IOMUX);
    ConfigureInput(BOARD_M2_ENC_A_IOMUX);
    ConfigureInput(BOARD_M2_ENC_B_IOMUX);

    g_m1.state = ReadM1State();
    g_m2.state = ReadM2State();
    g_m1.last_sample_ms = 0U;
    g_m2.last_sample_ms = 0U;

    ConfigureInterrupts();
}

void QuadratureEncoder_Process(uint32_t now_ms)
{
    UpdateSample(&g_m1, now_ms);
    UpdateSample(&g_m2, now_ms);
}

void QuadratureEncoder_HandleGpioInterrupt(void)
{
    const uint32_t m1_status =
        DL_GPIO_getEnabledInterruptStatus(GPIOB,
                                          BOARD_M1_ENC_A_PIN |
                                              BOARD_M1_ENC_B_PIN);
    const uint32_t m2_status =
        DL_GPIO_getEnabledInterruptStatus(GPIOA,
                                          BOARD_M2_ENC_A_PIN |
                                              BOARD_M2_ENC_B_PIN);

    if (m1_status != 0U) {
        ApplyTransition(&g_m1, ReadM1State());
        DL_GPIO_clearInterruptStatus(GPIOB, m1_status);
    }

    if (m2_status != 0U) {
        ApplyTransition(&g_m2, ReadM2State());
        DL_GPIO_clearInterruptStatus(GPIOA, m2_status);
    }
}

void QuadratureEncoder_Reset(EncoderId encoder)
{
    if (encoder == EncoderId::Encoder1) {
        ResetState(&g_m1, ReadM1State(), g_m1.last_sample_ms);
    } else {
        ResetState(&g_m2, ReadM2State(), g_m2.last_sample_ms);
    }
}

EncoderSnapshot QuadratureEncoder_GetSnapshot(EncoderId encoder)
{
    EncoderState *state = (encoder == EncoderId::Encoder1) ? &g_m1 : &g_m2;
    EncoderSnapshot snapshot = {};

    __disable_irq();
    snapshot.count = state->count;
    __enable_irq();

    snapshot.counts_per_second = state->counts_per_second;
    snapshot.state = state->state;

    return snapshot;
}

}  // namespace drivers
