#include "motor_open_loop.h"

#include "drv8323rs.h"
#include "ti_msp_dl_config.h"

/* TIMA0 runs center-aligned: LOAD=800 gives a 1600-count, 20 kHz period. */
#define PWM_PERIOD_COUNTS                  FOC_CFG_PWM_PERIOD_COUNTS
#define PWM_LOAD_COUNTS                    (PWM_PERIOD_COUNTS / 2U)
#define PWM_NEUTRAL_COMPARE                (PWM_LOAD_COUNTS / 2U)
#define PWM_MAX_PHASE_DELTA                FOC_CFG_PWM_MAX_PHASE_DELTA
#define ALIGN_ALPHA_VOLTAGE                FOC_CFG_ALIGNMENT_ALPHA_VOLTAGE

#define MOTOR_POLE_PAIRS                   FOC_CFG_MOTOR_POLE_PAIRS
#define CURRENT_LOOP_HZ                    FOC_CFG_CURRENT_LOOP_HZ
#define ENCODER_SPEED_WINDOW_TICKS         FOC_CFG_SPEED_WINDOW_TICKS
/* Fast sampling is 1 kHz; stop after five missed samples. */
#define ENCODER_STALE_TICKS                FOC_CFG_ENCODER_STALE_TICKS
#define ELECTRICAL_PREDICTION_MAX_TICKS    \
    FOC_CFG_ELECTRICAL_PREDICTION_MAX_TICKS
#define ELECTRICAL_VELOCITY_FILTER_SHIFT   \
    FOC_CFG_ELECTRICAL_VELOCITY_FILTER_SHIFT

/* 1 ADC count is approximately 2.015 mA with 20 mOhm and 20 V/V. */
#define IQ_REFERENCE_LIMIT_COUNTS          FOC_CFG_IQ_REFERENCE_LIMIT_COUNTS
#define HARD_CURRENT_LIMIT_COUNTS          FOC_CFG_HARD_CURRENT_LIMIT_COUNTS
#define HARD_CURRENT_TRIP_SAMPLES          FOC_CFG_HARD_CURRENT_TRIP_SAMPLES

/* The speed PI runs at 100 Hz and stores its integral in Q6 current counts. */
#define SPEED_INTEGRAL_SHIFT               FOC_CFG_SPEED_INTEGRAL_SHIFT
#define SPEED_MEASUREMENT_FILTER_SHIFT     \
    FOC_CFG_SPEED_MEASUREMENT_FILTER_SHIFT
#define POSITION_INTEGRAL_RELEASE_RPM      \
    FOC_CFG_POSITION_INTEGRAL_RELEASE_RPM

/* 100 Hz single-turn position loop: position error -> limited speed target. */
#define ENCODER_COUNTS_PER_REVOLUTION      \
    FOC_CFG_ENCODER_COUNTS_PER_REVOLUTION
#define POSITION_COUNTS_PER_RPM            FOC_CFG_POSITION_COUNTS_PER_RPM
#define POSITION_SPEED_LIMIT_RPM           FOC_CFG_POSITION_SPEED_LIMIT_RPM
#define POSITION_DIRECT_ZONE_COUNTS        \
    FOC_CFG_POSITION_DIRECT_ZONE_COUNTS
#define POSITION_DIRECT_ERROR_DIVISOR      \
    FOC_CFG_POSITION_DIRECT_ERROR_DIVISOR
#define POSITION_DIRECT_DAMPING_PER_RPM    \
    FOC_CFG_POSITION_DIRECT_DAMPING_PER_RPM
#define POSITION_DIRECT_IQ_LIMIT_COUNTS    \
    FOC_CFG_POSITION_DIRECT_IQ_LIMIT_COUNTS

/* Current PI output is in PWM delta counts, with Q8 coefficients/state. */
#define CURRENT_KP_Q8                      FOC_CFG_CURRENT_KP_Q8
#define CURRENT_KI_Q8                      FOC_CFG_CURRENT_KI_Q8
#define CURRENT_VOLTAGE_LIMIT              FOC_CFG_CURRENT_VOLTAGE_LIMIT
#define CURRENT_VOLTAGE_LIMIT_Q8           (CURRENT_VOLTAGE_LIMIT << 8)

#define THREE_PHASE_ENABLE_MASK            \
    (MOTOR_CTRL_INLA_STATE_A_PIN | MOTOR_CTRL_INLB_STATE_C_PIN | \
        MOTOR_CTRL_INLC_BRAKE_N_PIN)

/* Quarter-wave Q15 sine table, 0 through pi/2 in 64 equal steps. */
static const int16_t gSinQuarterQ15[65] = {
    0, 804, 1608, 2410, 3212, 4011, 4808, 5602,
    6393, 7179, 7962, 8739, 9512, 10278, 11039, 11793,
    12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530,
    18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
    23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
    27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
    30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971,
    32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
    32767,
};

static volatile MOTOR_FOC_State gState;
static volatile MOTOR_FOC_StopReason gStopReason;

static volatile uint16_t gEncoderRaw;
static volatile uint16_t gElectricalAngle;
static volatile uint16_t gPredictedElectricalAngle;
static volatile int32_t gElectricalVelocityQ16;
static volatile uint32_t gElectricalAngleSampleTick;
static volatile uint16_t gElectricalOffset;
static volatile uint32_t gLastEncoderSampleTick;
static volatile bool gEncoderReady;

static uint16_t gPreviousEncoderRaw;
static uint32_t gPreviousEncoderTick;
static bool gElectricalVelocityReady;
static int32_t gSpeedWindowDelta;
static uint32_t gSpeedWindowTicks;
static volatile int32_t gMeasuredRpm;
static volatile int32_t gTargetRpm = MOTOR_FOC_DEFAULT_TARGET_RPM;
static int32_t gSpeedIntegralQ6;
static volatile int32_t gIqReferenceCounts;
static volatile MOTOR_FOC_ControlMode gControlMode;
static volatile uint16_t gTargetPositionRaw;
static volatile int32_t gPositionErrorCounts;

static int32_t gIdIntegralQ8;
static int32_t gIqIntegralQ8;
static volatile int32_t gLastIdCounts;
static volatile int32_t gLastIqCounts;
static uint8_t gOvercurrentCount;

static volatile uint16_t gCompareA = PWM_NEUTRAL_COMPARE;
static volatile uint16_t gCompareB = PWM_NEUTRAL_COMPARE;
static volatile uint16_t gCompareC = PWM_NEUTRAL_COMPARE;

static int32_t clampSigned(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int32_t absoluteSigned(int32_t value)
{
    return (value < 0) ? -value : value;
}

static bool limitDqVoltageVector(int32_t *dVoltageQ8, int32_t *qVoltageQ8)
{
    int32_t absoluteD = absoluteSigned(*dVoltageQ8);
    int32_t absoluteQ = absoluteSigned(*qVoltageQ8);
    int32_t maximum = (absoluteD > absoluteQ) ? absoluteD : absoluteQ;
    int32_t minimum = (absoluteD > absoluteQ) ? absoluteQ : absoluteD;
    int32_t conservativeMagnitude = maximum + (minimum / 2);

    /*
     * max(|d|, |q|) + min(|d|, |q|)/2 is a cheap upper bound on the
     * Euclidean magnitude.  Scaling both axes by the same factor preserves
     * the voltage-vector direction and guarantees it remains inside the
     * configured circular limit without a square root in the 20 kHz ISR.
     */
    if (conservativeMagnitude <= CURRENT_VOLTAGE_LIMIT_Q8) {
        return false;
    }

    *dVoltageQ8 = (int32_t) (((int64_t) *dVoltageQ8 *
        CURRENT_VOLTAGE_LIMIT_Q8) / conservativeMagnitude);
    *qVoltageQ8 = (int32_t) (((int64_t) *qVoltageQ8 *
        CURRENT_VOLTAGE_LIMIT_Q8) / conservativeMagnitude);
    return true;
}

/*
 * Integer division truncates toward zero.  A conventional
 * current += (target - current) / 2^shift filter can therefore get stuck
 * forever when the remaining difference is smaller than 2^shift.  Force a
 * one-count final approach so zero speed and zero predicted velocity are
 * actually reachable.
 */
static int32_t approachFilteredValue(
    int32_t current, int32_t target, uint32_t shift)
{
    int32_t difference = target - current;
    int32_t step;

    if (difference == 0) {
        return current;
    }

    step = difference / (1L << shift);
    if (step == 0) {
        step = (difference > 0) ? 1 : -1;
    }
    return current + step;
}

static int32_t wrapEncoderError(int32_t error)
{
    if (error > 8191) {
        error -= ENCODER_COUNTS_PER_REVOLUTION;
    } else if (error < -8192) {
        error += ENCODER_COUNTS_PER_REVOLUTION;
    }

    return error;
}

static int16_t sinQ15(uint16_t angle)
{
    uint8_t index = (uint8_t) (angle >> 8);
    uint8_t quadrant = index >> 6;
    uint8_t offset = index & 0x3FU;

    switch (quadrant) {
        case 0U:
            return gSinQuarterQ15[offset];
        case 1U:
            return gSinQuarterQ15[64U - offset];
        case 2U:
            return (int16_t) -gSinQuarterQ15[offset];
        default:
            return (int16_t) -gSinQuarterQ15[64U - offset];
    }
}

static void writePwmCompare(int32_t phaseA, int32_t phaseB, int32_t phaseC)
{
    uint16_t compareA;
    uint16_t compareB;
    uint16_t compareC;

    phaseA = clampSigned(
        phaseA, -PWM_MAX_PHASE_DELTA, PWM_MAX_PHASE_DELTA);
    phaseB = clampSigned(
        phaseB, -PWM_MAX_PHASE_DELTA, PWM_MAX_PHASE_DELTA);
    phaseC = clampSigned(
        phaseC, -PWM_MAX_PHASE_DELTA, PWM_MAX_PHASE_DELTA);

    /* Center-aligned output duty rises as the compare value falls. */
    compareA = (uint16_t) (PWM_NEUTRAL_COMPARE - phaseA);
    compareB = (uint16_t) (PWM_NEUTRAL_COMPARE - phaseB);
    compareC = (uint16_t) (PWM_NEUTRAL_COMPARE - phaseC);

    DL_TimerA_setCaptureCompareValue(
        TIMA0, compareA, DL_TIMER_CC_1_INDEX); /* PA22, INHA */
    DL_TimerA_setCaptureCompareValue(
        TIMA0, compareB, DL_TIMER_CC_2_INDEX); /* PA3, INHB */
    DL_TimerA_setCaptureCompareValue(
        TIMA0, compareC, DL_TIMER_CC_3_INDEX); /* PA12, INHC */

    gCompareA = compareA;
    gCompareB = compareB;
    gCompareC = compareC;
}

static void applyAlphaBetaVoltage(int32_t alpha, int32_t beta)
{
    int32_t phaseA = alpha;
    int32_t phaseB = (-alpha / 2) + ((beta * 28378) >> 15);
    int32_t phaseC = (-alpha / 2) - ((beta * 28378) >> 15);
    int32_t maximum = phaseA;
    int32_t minimum = phaseA;
    int32_t commonMode;

    if (phaseB > maximum) {
        maximum = phaseB;
    }
    if (phaseC > maximum) {
        maximum = phaseC;
    }
    if (phaseB < minimum) {
        minimum = phaseB;
    }
    if (phaseC < minimum) {
        minimum = phaseC;
    }

    /* Min/max zero-sequence injection gives space-vector PWM utilization. */
    commonMode = -(maximum + minimum) / 2;
    writePwmCompare(
        phaseA + commonMode, phaseB + commonMode, phaseC + commonMode);
}

static void applyDqVoltage(int32_t dVoltage, int32_t qVoltage, uint16_t angle)
{
    int32_t sine = sinQ15(angle);
    int32_t cosine = sinQ15((uint16_t) (angle + 16384U));
    int32_t alpha = ((dVoltage * cosine) - (qVoltage * sine)) >> 15;
    int32_t beta = ((dVoltage * sine) + (qVoltage * cosine)) >> 15;

    applyAlphaBetaVoltage(alpha, beta);
}

static void disablePhases(void)
{
    DL_GPIO_clearPins(MOTOR_CTRL_PORT, THREE_PHASE_ENABLE_MASK);
    writePwmCompare(0, 0, 0);
}

static void enablePhases(void)
{
    DL_GPIO_setPins(MOTOR_CTRL_PORT, THREE_PHASE_ENABLE_MASK);
}

static void initializeThreePhasePwm(void)
{
    static const DL_TimerA_ClockConfig clockConfig = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U,
    };
    static const DL_TimerA_PWMConfig pwmConfig = {
        .pwmMode = DL_TIMER_PWM_MODE_CENTER_ALIGN,
        .period = PWM_PERIOD_COUNTS,
        .isTimerWithFourCC = true,
        .startTimer = DL_TIMER_STOP,
    };
    uint8_t channel;

    DL_TimerA_reset(TIMA0);
    DL_TimerA_enablePower(TIMA0);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_TimerA_setClockConfig(TIMA0, (DL_TimerA_ClockConfig *) &clockConfig);
    DL_TimerA_initPWMMode(TIMA0, (DL_TimerA_PWMConfig *) &pwmConfig);

    DL_TimerA_setCCPDirection(TIMA0,
        DL_TIMER_CC1_OUTPUT | DL_TIMER_CC2_OUTPUT | DL_TIMER_CC3_OUTPUT);

    /* Existing board wiring: high-side control inputs become TIMA0 PWM. */
    DL_GPIO_initPeripheralOutputFunction(MOTOR_CTRL_INHA_PWM_IOMUX,
        IOMUX_PINCM47_PF_TIMA0_CCP1);
    DL_GPIO_initPeripheralOutputFunction(MOTOR_CTRL_INHB_STATE_B_IOMUX,
        IOMUX_PINCM8_PF_TIMA0_CCP2);
    DL_GPIO_initPeripheralOutputFunction(MOTOR_CTRL_INHC_DIR_IOMUX,
        IOMUX_PINCM34_PF_TIMA0_CCP3);
    DL_GPIO_enableOutput(MOTOR_CTRL_PORT,
        MOTOR_CTRL_INHA_PWM_PIN | MOTOR_CTRL_INHB_STATE_B_PIN |
            MOTOR_CTRL_INHC_DIR_PIN);

    /* Keep every INHx low while DRV8323RS PWM_MODE is being changed. */
    DL_TimerA_setCaptureCompareValue(
        TIMA0, PWM_LOAD_COUNTS, DL_TIMER_CC_1_INDEX);
    DL_TimerA_setCaptureCompareValue(
        TIMA0, PWM_LOAD_COUNTS, DL_TIMER_CC_2_INDEX);
    DL_TimerA_setCaptureCompareValue(
        TIMA0, PWM_LOAD_COUNTS, DL_TIMER_CC_3_INDEX);
    gCompareA = PWM_LOAD_COUNTS;
    gCompareB = PWM_LOAD_COUNTS;
    gCompareC = PWM_LOAD_COUNTS;

    /*
     * Subsequent duty writes go to the three shadow registers.  Hardware
     * transfers all channels just after the same counter-zero event, so no
     * phase can change compare value halfway through an up/down PWM cycle.
     */
    for (channel = 1U; channel <= 3U; channel++) {
        DL_TimerA_setCaptCompUpdateMethod(TIMA0,
            DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT,
            (DL_TIMER_CC_INDEX) channel);
    }

    DL_TimerA_enableClock(TIMA0);
    DL_TimerA_startCounter(TIMA0);
}

static void resetControlState(void)
{
    gSpeedWindowDelta = 0;
    gSpeedWindowTicks = 0U;
    gMeasuredRpm = 0;
    gSpeedIntegralQ6 = 0;
    gIqReferenceCounts = 0;
    gPositionErrorCounts = 0;
    gIdIntegralQ8 = 0;
    gIqIntegralQ8 = 0;
    gLastIdCounts = 0;
    gLastIqCounts = 0;
    gOvercurrentCount = 0U;
}

static void updateSpeedController(void)
{
    int32_t error = gTargetRpm - gMeasuredRpm;
    int32_t proportional;
    int32_t candidateIntegralQ6;
    int32_t candidateOutput;
    int32_t integralLimitQ6 =
        (IQ_REFERENCE_LIMIT_COUNTS << SPEED_INTEGRAL_SHIFT);

    if ((gControlMode == MOTOR_FOC_MODE_POSITION) &&
        (absoluteSigned(gPositionErrorCounts) <=
            POSITION_DIRECT_ZONE_COUNTS)) {
        int32_t directIq =
            (gPositionErrorCounts / POSITION_DIRECT_ERROR_DIVISOR) -
            (gMeasuredRpm * POSITION_DIRECT_DAMPING_PER_RPM);

        gSpeedIntegralQ6 = 0;
        gIqReferenceCounts = clampSigned(directIq,
            -POSITION_DIRECT_IQ_LIMIT_COUNTS,
            POSITION_DIRECT_IQ_LIMIT_COUNTS);
        return;
    }

    /* At a zero-speed command, remove residual integral once almost stopped. */
    if ((gControlMode == MOTOR_FOC_MODE_SPEED) &&
        (gTargetRpm == 0) && (absoluteSigned(gMeasuredRpm) <= 1)) {
        gSpeedIntegralQ6 = 0;
        gIqReferenceCounts = 0;
        return;
    }

    /* Position commands change direction as the shaft crosses the target. */
    if ((gControlMode == MOTOR_FOC_MODE_POSITION) &&
        ((((error <= -POSITION_INTEGRAL_RELEASE_RPM) &&
              (gSpeedIntegralQ6 > 0))) ||
         ((error >= POSITION_INTEGRAL_RELEASE_RPM) &&
              (gSpeedIntegralQ6 < 0)))) {
        gSpeedIntegralQ6 = 0;
    }

    /* 100 Hz outer loop; one current count/RPM gives useful active damping. */
    /* Q6 integral retains sub-count torque commands at low target speeds. */
    proportional = error;
    candidateIntegralQ6 = clampSigned(
        gSpeedIntegralQ6 + error, -integralLimitQ6, integralLimitQ6);
    candidateOutput = proportional +
        (candidateIntegralQ6 >> SPEED_INTEGRAL_SHIFT);

    /* Conditional integration prevents additional windup at either limit. */
    if (!(((candidateOutput > IQ_REFERENCE_LIMIT_COUNTS) && (error > 0)) ||
            ((candidateOutput < -IQ_REFERENCE_LIMIT_COUNTS) &&
                (error < 0)))) {
        gSpeedIntegralQ6 = candidateIntegralQ6;
    }

    gIqReferenceCounts = clampSigned(
        proportional + (gSpeedIntegralQ6 >> SPEED_INTEGRAL_SHIFT),
        -IQ_REFERENCE_LIMIT_COUNTS, IQ_REFERENCE_LIMIT_COUNTS);
}

static void updatePositionController(void)
{
    int32_t error;
    int32_t speedTarget;

    error = wrapEncoderError(
        (int32_t) gTargetPositionRaw - (int32_t) gEncoderRaw);
    gPositionErrorCounts = error;

    if (absoluteSigned(error) <= POSITION_DIRECT_ZONE_COUNTS) {
        speedTarget = 0;
    } else {
        speedTarget = error / POSITION_COUNTS_PER_RPM;
        if (speedTarget == 0) {
            speedTarget = (error > 0) ? 1 : -1;
        }
        speedTarget = clampSigned(speedTarget,
            -POSITION_SPEED_LIMIT_RPM, POSITION_SPEED_LIMIT_RPM);
    }

    gTargetRpm = speedTarget;
}

static void setTargetRpm(int32_t newTargetRpm)
{
    int32_t newError;

    gTargetRpm = clampSigned(newTargetRpm,
        -MOTOR_FOC_MAX_TARGET_RPM, MOTOR_FOC_MAX_TARGET_RPM);

    if (gState != MOTOR_FOC_STATE_RUNNING) {
        return;
    }

    newError = gTargetRpm - gMeasuredRpm;

    /*
     * A target step that asks for torque opposite to the stored command must
     * brake now, not wait seconds for the old Q6 integral to unwind.
     */
    if ((gTargetRpm == 0) ||
        ((newError < 0) && (gIqReferenceCounts > 0)) ||
        ((newError > 0) && (gIqReferenceCounts < 0))) {
        gSpeedIntegralQ6 = 0;
    }

    /* Refresh Iq_ref immediately instead of waiting for the next 10 ms tick. */
    updateSpeedController();
}

void MOTOR_FOC_initialize(void)
{
    gState = MOTOR_FOC_STATE_STOPPED;
    gStopReason = MOTOR_FOC_STOP_NONE;
    gEncoderReady = false;
    gEncoderRaw = 0U;
    gElectricalAngle = 0U;
    gPredictedElectricalAngle = 0U;
    gElectricalVelocityQ16 = 0;
    gElectricalAngleSampleTick = 0U;
    gElectricalOffset = 0U;
    gLastEncoderSampleTick = 0U;
    gControlMode = MOTOR_FOC_MODE_SPEED;
    gElectricalVelocityReady = false;
    gTargetPositionRaw = 0U;
    gTargetRpm = MOTOR_FOC_DEFAULT_TARGET_RPM;
    resetControlState();

    /* INLx low guarantees Hi-Z while PWM mode or pin mux is changing. */
    DL_GPIO_clearPins(MOTOR_CTRL_PORT, THREE_PHASE_ENABLE_MASK);
    initializeThreePhasePwm();
}

void MOTOR_FOC_enableAdcTrigger(void)
{
    uint32_t pwmCount;

    /*
     * Keep the proven TIMG0 -> event channel 14 -> ADC path from SysConfig.
     * Phase-lock TIMG0's 50 us period to TIMA0 near its zero point so current
     * is sampled during the all-low PWM vector.
     */
    DL_TimerG_stopCounter(CURRENT_SAMPLE_TIMER_INST);
    DL_TimerA_disableEvent(
        TIMA0, DL_TIMERA_EVENT_ROUTE_1, DL_TIMER_EVENT_ZERO_EVENT);
    DL_TimerA_setPublisherChanID(
        TIMA0, DL_TIMERA_PUBLISHER_INDEX_0, 0U);

    /* The driver is now in 3x PWM mode; neutral PWM is safe with INLx low. */
    writePwmCompare(0, 0, 0);

    DL_TimerG_setTimerCount(
        CURRENT_SAMPLE_TIMER_INST, CURRENT_SAMPLE_TIMER_INST_LOAD_VALUE);

    /* Leave any current zero region, then wait for the next approach to zero. */
    do {
        pwmCount = DL_TimerA_getTimerCount(TIMA0);
    } while (pwmCount <= 4U);
    do {
        pwmCount = DL_TimerA_getTimerCount(TIMA0);
    } while (pwmCount > 4U);

    DL_TimerG_startCounter(CURRENT_SAMPLE_TIMER_INST);
}

bool MOTOR_FOC_beginAlignment(void)
{
    if ((gState != MOTOR_FOC_STATE_STOPPED) ||
        DRV8323RS_isFaultActive()) {
        return false;
    }

    gStopReason = MOTOR_FOC_STOP_NONE;
    gEncoderReady = false;
    gElectricalVelocityReady = false;
    gElectricalVelocityQ16 = 0;
    gControlMode = MOTOR_FOC_MODE_SPEED;
    /* Every run starts from the same conservative, known speed command. */
    gTargetRpm = MOTOR_FOC_DEFAULT_TARGET_RPM;
    resetControlState();

    /* A small stationary alpha-axis field establishes electrical angle zero. */
    applyAlphaBetaVoltage(ALIGN_ALPHA_VOLTAGE, 0);
    enablePhases();
    gState = MOTOR_FOC_STATE_ALIGNING;
    return true;
}

bool MOTOR_FOC_finishAlignment(
    uint16_t encoderRaw, uint32_t currentSampleSequence)
{
    uint32_t mechanicalAngle;

    if ((gState != MOTOR_FOC_STATE_ALIGNING) ||
        DRV8323RS_isFaultActive()) {
        MOTOR_FOC_emergencyStop(MOTOR_FOC_STOP_DRIVER_FAULT);
        return false;
    }

    encoderRaw &= 0x3FFFU;
    mechanicalAngle = (uint32_t) encoderRaw << 2;
    gElectricalOffset = (uint16_t)
        (0U - (uint16_t) (mechanicalAngle * MOTOR_POLE_PAIRS));
    {
        uint16_t electricalAngle = (uint16_t)
            ((mechanicalAngle * MOTOR_POLE_PAIRS) + gElectricalOffset);
        uint32_t interruptState = __get_PRIMASK();

        __disable_irq();
        gEncoderRaw = encoderRaw;
        gElectricalAngle = electricalAngle;
        gPredictedElectricalAngle = electricalAngle;
        gElectricalVelocityQ16 = 0;
        gElectricalAngleSampleTick = currentSampleSequence;
        gLastEncoderSampleTick = currentSampleSequence;
        if (interruptState == 0U) {
            __enable_irq();
        }
    }
    gPreviousEncoderRaw = encoderRaw;
    gPreviousEncoderTick = currentSampleSequence;
    gElectricalVelocityReady = false;
    gEncoderReady = true;
    resetControlState();
    gState = MOTOR_FOC_STATE_RUNNING;
    return true;
}

void MOTOR_FOC_stop(void)
{
    disablePhases();
    gState = MOTOR_FOC_STATE_STOPPED;
    gStopReason = MOTOR_FOC_STOP_COMMAND;
    gEncoderReady = false;
    gElectricalVelocityReady = false;
    gElectricalVelocityQ16 = 0;
    gControlMode = MOTOR_FOC_MODE_SPEED;
    resetControlState();
}

void MOTOR_FOC_emergencyStop(MOTOR_FOC_StopReason reason)
{
    disablePhases();
    gState = MOTOR_FOC_STATE_FAULT;
    gStopReason = reason;
    gEncoderReady = false;
    gElectricalVelocityReady = false;
    gElectricalVelocityQ16 = 0;
    gIqReferenceCounts = 0;
}

void MOTOR_FOC_onEncoderSample(
    uint16_t encoderRaw, bool valid, uint32_t currentSampleSequence)
{
    int32_t delta;
    int32_t electricalVelocityQ16;
    int32_t rpm = 0;
    uint16_t electricalAngle;
    uint32_t elapsed;
    uint32_t interruptState;
    bool updateOuterControllers;

    if (!valid) {
        return;
    }

    encoderRaw &= 0x3FFFU;
    electricalAngle = (uint16_t)
        ((((uint32_t) encoderRaw << 2) * MOTOR_POLE_PAIRS) +
            gElectricalOffset);

    if ((gState != MOTOR_FOC_STATE_RUNNING) || !gEncoderReady) {
        interruptState = __get_PRIMASK();
        __disable_irq();
        gEncoderRaw = encoderRaw;
        gElectricalAngle = electricalAngle;
        gPredictedElectricalAngle = electricalAngle;
        gElectricalAngleSampleTick = currentSampleSequence;
        gLastEncoderSampleTick = currentSampleSequence;
        if (interruptState == 0U) {
            __enable_irq();
        }
        return;
    }

    elapsed = currentSampleSequence - gPreviousEncoderTick;
    if (elapsed == 0U) {
        return;
    }

    delta = (int32_t) encoderRaw - (int32_t) gPreviousEncoderRaw;
    if (delta > 8191) {
        delta -= 16384;
    } else if (delta < -8192) {
        delta += 16384;
    }

    gPreviousEncoderRaw = encoderRaw;
    gPreviousEncoderTick = currentSampleSequence;
    gSpeedWindowDelta += delta;
    gSpeedWindowTicks += elapsed;
    electricalVelocityQ16 = gElectricalVelocityQ16;
    updateOuterControllers = false;

    if (gSpeedWindowTicks >= ENCODER_SPEED_WINDOW_TICKS) {
        int32_t windowElectricalVelocityQ16 = (int32_t)
            (((int64_t) gSpeedWindowDelta * 4LL *
                (int64_t) MOTOR_POLE_PAIRS * 65536LL) /
                (int64_t) gSpeedWindowTicks);

        rpm = (int32_t)
            (((int64_t) gSpeedWindowDelta * CURRENT_LOOP_HZ * 60) /
                ((int64_t) 16384 * gSpeedWindowTicks));

        if (gElectricalVelocityReady) {
            electricalVelocityQ16 = approachFilteredValue(
                gElectricalVelocityQ16, windowElectricalVelocityQ16,
                ELECTRICAL_VELOCITY_FILTER_SHIFT);
        } else {
            electricalVelocityQ16 = windowElectricalVelocityQ16;
            gElectricalVelocityReady = true;
        }

        gMeasuredRpm = approachFilteredValue(
            gMeasuredRpm, rpm, SPEED_MEASUREMENT_FILTER_SHIFT);
        gSpeedWindowDelta = 0;
        gSpeedWindowTicks = 0U;
        updateOuterControllers = true;
    }

    interruptState = __get_PRIMASK();
    __disable_irq();
    gEncoderRaw = encoderRaw;
    gElectricalAngle = electricalAngle;
    gPredictedElectricalAngle = electricalAngle;
    gElectricalVelocityQ16 = electricalVelocityQ16;
    gElectricalAngleSampleTick = currentSampleSequence;
    gLastEncoderSampleTick = currentSampleSequence;
    if (interruptState == 0U) {
        __enable_irq();
    }

    if (updateOuterControllers) {
        if (gControlMode == MOTOR_FOC_MODE_POSITION) {
            updatePositionController();
        }
        updateSpeedController();
    }
}

void MOTOR_FOC_currentLoopISR(int16_t phaseACounts, int16_t phaseBCounts,
    int16_t phaseCCounts, uint32_t currentSampleSequence)
{
    int32_t phaseA;
    int32_t phaseB;
    int32_t phaseC;
    int32_t zeroSequence;
    int32_t alphaCurrent;
    int32_t betaCurrent;
    int32_t sine;
    int32_t cosine;
    int32_t dCurrent;
    int32_t qCurrent;
    int32_t dError;
    int32_t qError;
    int32_t candidateIdIntegralQ8;
    int32_t candidateIqIntegralQ8;
    int32_t dOutputQ8;
    int32_t qOutputQ8;
    bool voltageSaturated;
    int32_t electricalVelocityQ16;
    uint16_t electricalAngle;
    uint32_t electricalAngleSampleTick;
    uint32_t predictionTicks;

    if ((gState != MOTOR_FOC_STATE_ALIGNING) &&
        (gState != MOTOR_FOC_STATE_RUNNING)) {
        return;
    }

    if ((absoluteSigned(phaseACounts) > HARD_CURRENT_LIMIT_COUNTS) ||
        (absoluteSigned(phaseBCounts) > HARD_CURRENT_LIMIT_COUNTS) ||
        (absoluteSigned(phaseCCounts) > HARD_CURRENT_LIMIT_COUNTS)) {
        if (++gOvercurrentCount >= HARD_CURRENT_TRIP_SAMPLES) {
            MOTOR_FOC_emergencyStop(MOTOR_FOC_STOP_OVERCURRENT);
            return;
        }
    } else {
        gOvercurrentCount = 0U;
    }

    if (gState == MOTOR_FOC_STATE_ALIGNING) {
        return;
    }

    if (!gEncoderReady ||
        ((currentSampleSequence - gLastEncoderSampleTick) >
            ENCODER_STALE_TICKS)) {
        MOTOR_FOC_emergencyStop(MOTOR_FOC_STOP_ENCODER_LOST);
        return;
    }

    electricalAngle = gElectricalAngle;
    electricalVelocityQ16 = gElectricalVelocityQ16;
    electricalAngleSampleTick = gElectricalAngleSampleTick;
    predictionTicks = currentSampleSequence - electricalAngleSampleTick;
    if (predictionTicks > ELECTRICAL_PREDICTION_MAX_TICKS) {
        predictionTicks = ELECTRICAL_PREDICTION_MAX_TICKS;
    }
    electricalAngle = (uint16_t) ((int32_t) electricalAngle +
        (int32_t) (((int64_t) electricalVelocityQ16 * predictionTicks) >> 16));
    gPredictedElectricalAngle = electricalAngle;

    /*
     * This board's SOA/SOB/SOC polarity was verified during six-step tests:
     * positive phase current produces ADC above the calibrated VREF/2 zero.
     * Therefore the phase-current convention is ADC minus offset (the delta
     * values passed into this ISR), not the opposite polarity used by some
     * DRV8323RS evaluation-module shunt layouts.
     */
    phaseA = (int32_t) phaseACounts;
    phaseB = (int32_t) phaseBCounts;
    phaseC = (int32_t) phaseCCounts;

    zeroSequence = (phaseA + phaseB + phaseC) / 3;
    phaseA -= zeroSequence;
    phaseB -= zeroSequence;
    phaseC -= zeroSequence;

    alphaCurrent = phaseA;
    betaCurrent = ((phaseB - phaseC) * 18919) >> 15; /* 1/sqrt(3). */

    sine = sinQ15(electricalAngle);
    cosine = sinQ15((uint16_t) (electricalAngle + 16384U));
    dCurrent = ((alphaCurrent * cosine) + (betaCurrent * sine)) >> 15;
    qCurrent = ((-alphaCurrent * sine) + (betaCurrent * cosine)) >> 15;
    gLastIdCounts = dCurrent;
    gLastIqCounts = qCurrent;

    dError = -dCurrent;
    qError = gIqReferenceCounts - qCurrent;

    candidateIdIntegralQ8 = clampSigned(
        gIdIntegralQ8 + (dError * CURRENT_KI_Q8),
        -CURRENT_VOLTAGE_LIMIT_Q8, CURRENT_VOLTAGE_LIMIT_Q8);
    candidateIqIntegralQ8 = clampSigned(
        gIqIntegralQ8 + (qError * CURRENT_KI_Q8),
        -CURRENT_VOLTAGE_LIMIT_Q8, CURRENT_VOLTAGE_LIMIT_Q8);

    dOutputQ8 = (dError * CURRENT_KP_Q8) + candidateIdIntegralQ8;
    qOutputQ8 = (qError * CURRENT_KP_Q8) + candidateIqIntegralQ8;
    voltageSaturated = limitDqVoltageVector(&dOutputQ8, &qOutputQ8);

    /*
     * Keep the candidate integral unless saturation and the error are driving
     * the requested vector farther outward.  Error opposing the saturated
     * vector is still allowed to unwind the integrators.
     */
    if (!voltageSaturated ||
        ((((int64_t) dError * dOutputQ8) +
             ((int64_t) qError * qOutputQ8)) <= 0)) {
        gIdIntegralQ8 = candidateIdIntegralQ8;
        gIqIntegralQ8 = candidateIqIntegralQ8;
    } else {
        dOutputQ8 = (dError * CURRENT_KP_Q8) + gIdIntegralQ8;
        qOutputQ8 = (qError * CURRENT_KP_Q8) + gIqIntegralQ8;
        (void) limitDqVoltageVector(&dOutputQ8, &qOutputQ8);
    }

    applyDqVoltage(dOutputQ8 >> 8, qOutputQ8 >> 8, electricalAngle);
}

bool MOTOR_FOC_isRunning(void)
{
    return gState == MOTOR_FOC_STATE_RUNNING;
}

bool MOTOR_FOC_isActive(void)
{
    return (gState == MOTOR_FOC_STATE_ALIGNING) ||
        (gState == MOTOR_FOC_STATE_RUNNING);
}

MOTOR_FOC_State MOTOR_FOC_getState(void)
{
    return gState;
}

MOTOR_FOC_StopReason MOTOR_FOC_getStopReason(void)
{
    return gStopReason;
}

void MOTOR_FOC_clearStopReason(void)
{
    if (gState != MOTOR_FOC_STATE_RUNNING) {
        gStopReason = MOTOR_FOC_STOP_NONE;
        if (gState == MOTOR_FOC_STATE_FAULT) {
            gState = MOTOR_FOC_STATE_STOPPED;
        }
    }
}

void MOTOR_FOC_adjustTargetRpm(int32_t deltaRpm)
{
    setTargetRpm(gTargetRpm + deltaRpm);
}

void MOTOR_FOC_reverse(void)
{
    int32_t reversedTarget = -gTargetRpm;

    /* Let 'r' select reverse motion even when the present target is zero. */
    if (reversedTarget == 0) {
        reversedTarget = -MOTOR_FOC_TARGET_RPM_STEP;
    }

    gSpeedIntegralQ6 = 0;
    setTargetRpm(reversedTarget);
}

bool MOTOR_FOC_enterPositionMode(void)
{
    if ((gState != MOTOR_FOC_STATE_RUNNING) || !gEncoderReady) {
        return false;
    }

    gControlMode = MOTOR_FOC_MODE_POSITION;
    gTargetPositionRaw = gEncoderRaw;
    gPositionErrorCounts = 0;
    gTargetRpm = 0;
    gSpeedIntegralQ6 = 0;
    gIqReferenceCounts = 0;
    updatePositionController();
    updateSpeedController();
    return true;
}

void MOTOR_FOC_enterSpeedMode(void)
{
    gControlMode = MOTOR_FOC_MODE_SPEED;
    gPositionErrorCounts = 0;
    gSpeedIntegralQ6 = 0;
    setTargetRpm(0);
}

void MOTOR_FOC_adjustTargetPositionDegrees(int32_t deltaDegrees)
{
    int32_t deltaCounts;
    int32_t newTarget;

    if ((gState != MOTOR_FOC_STATE_RUNNING) ||
        (gControlMode != MOTOR_FOC_MODE_POSITION)) {
        return;
    }

    deltaCounts = (int32_t)
        (((int64_t) deltaDegrees * ENCODER_COUNTS_PER_REVOLUTION) / 360);
    newTarget = ((int32_t) gTargetPositionRaw + deltaCounts) %
        ENCODER_COUNTS_PER_REVOLUTION;
    if (newTarget < 0) {
        newTarget += ENCODER_COUNTS_PER_REVOLUTION;
    }

    gTargetPositionRaw = (uint16_t) newTarget;
    gSpeedIntegralQ6 = 0;
    updatePositionController();
    updateSpeedController();
}

MOTOR_FOC_ControlMode MOTOR_FOC_getControlMode(void)
{
    return gControlMode;
}

int32_t MOTOR_FOC_getTargetRpm(void)
{
    return gTargetRpm;
}

int32_t MOTOR_FOC_getMeasuredRpm(void)
{
    return gMeasuredRpm;
}

static int32_t currentCountsToMilliamps(int32_t counts)
{
    return (int32_t) (((int64_t) counts * 3300000LL) /
        (4095LL * 20LL * 20LL));
}

int32_t MOTOR_FOC_getIqReferenceMilliamps(void)
{
    return currentCountsToMilliamps(gIqReferenceCounts);
}

int32_t MOTOR_FOC_getIdMilliamps(void)
{
    return currentCountsToMilliamps(gLastIdCounts);
}

int32_t MOTOR_FOC_getIqMilliamps(void)
{
    return currentCountsToMilliamps(gLastIqCounts);
}

uint16_t MOTOR_FOC_getElectricalOffset(void)
{
    return gElectricalOffset;
}

uint16_t MOTOR_FOC_getEncoderRaw(void)
{
    return gEncoderRaw;
}

static uint16_t compareToDutyPermille(uint16_t compare)
{
    return (uint16_t)
        (((uint32_t) (PWM_LOAD_COUNTS - compare) * 1000U) /
            PWM_LOAD_COUNTS);
}

void MOTOR_FOC_getStatus(MOTOR_FOC_Status *status)
{
    uint32_t interruptState;
    int32_t iqReferenceCounts;
    int32_t idCounts;
    int32_t iqCounts;
    uint16_t compareA;
    uint16_t compareB;
    uint16_t compareC;

    if (status == 0) {
        return;
    }

    interruptState = __get_PRIMASK();
    __disable_irq();
    status->state = gState;
    status->stopReason = gStopReason;
    status->controlMode = gControlMode;
    status->targetRpm = gTargetRpm;
    status->measuredRpm = gMeasuredRpm;
    status->encoderRaw = gEncoderRaw;
    status->targetPositionRaw = gTargetPositionRaw;
    status->electricalAngleSample = gElectricalAngle;
    status->electricalAnglePredicted = gPredictedElectricalAngle;
    status->electricalVelocityQ16 = gElectricalVelocityQ16;
    iqReferenceCounts = gIqReferenceCounts;
    idCounts = gLastIdCounts;
    iqCounts = gLastIqCounts;
    compareA = gCompareA;
    compareB = gCompareB;
    compareC = gCompareC;
    if (interruptState == 0U) {
        __enable_irq();
    }

    status->iqReferenceMilliamps =
        currentCountsToMilliamps(iqReferenceCounts);
    status->idMilliamps = currentCountsToMilliamps(idCounts);
    status->iqMilliamps = currentCountsToMilliamps(iqCounts);
    status->positionErrorCounts = wrapEncoderError(
        (int32_t) status->targetPositionRaw - (int32_t) status->encoderRaw);
    status->dutyApermille = compareToDutyPermille(compareA);
    status->dutyBpermille = compareToDutyPermille(compareB);
    status->dutyCpermille = compareToDutyPermille(compareC);
}

uint16_t MOTOR_FOC_getDutyApermille(void)
{
    return compareToDutyPermille(gCompareA);
}

uint16_t MOTOR_FOC_getDutyBpermille(void)
{
    return compareToDutyPermille(gCompareB);
}

uint16_t MOTOR_FOC_getDutyCpermille(void)
{
    return compareToDutyPermille(gCompareC);
}
