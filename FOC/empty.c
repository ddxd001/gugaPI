#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drv8323rs.h"
#include "motor_open_loop.h"

#define ADC_REFERENCE_MV          (3300U)
#define ADC_FULL_SCALE_COUNTS     (4095U)
#define CURRENT_SHUNT_MILLIOHMS   (20U)
#define CURRENT_CSA_GAIN          (20U)
#define CURRENT_CALIBRATION_SAMPLES (64U)
#define CURRENT_DIAGNOSTIC_DECIMATION (4U)
#define CURRENT_AVERAGE_SAMPLES      (64U)
#define AS5048B_I2C_ADDRESS          (0x40U)
#define AS5048B_REG_AGC              (0xFAU)
#define AS5048B_REG_ANGLE            (0xFEU)
#define AS5048B_READOUT_LENGTH       (6U)
#define ENCODER_I2C_TIMEOUT_LOOPS    (CPUCLK_FREQ / 100U)
#define ENCODER_I2C_ERROR_EVENTS                                         \
    (DL_I2C_INTERRUPT_CONTROLLER_NACK |                                 \
        DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST)
#define ENCODER_I2C_TRANSFER_EVENTS                                      \
    (DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |                              \
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE | ENCODER_I2C_ERROR_EVENTS)
#define ENCODER_FAST_PERIOD_TICKS       (20U)  /* 1 kHz at 20 kHz ADC tick. */
#define ENCODER_FAST_TIMEOUT_TICKS      (40U)  /* 2 ms transaction timeout. */
#define CAN_DEBUG_BIT_RATE             (500000U)
#define CAN_DEBUG_TX_ID                (0x321U)
#define CAN_DEBUG_TX_BUFFER            (0U)
#define CAN_DEBUG_ERROR_EVENTS                                           \
    (DL_MCAN_INTR_SRC_MSG_RAM_ACCESS_FAILURE |                          \
        DL_MCAN_INTR_SRC_ERR_PASSIVE | DL_MCAN_INTR_SRC_WARNING_STATUS | \
        DL_MCAN_INTR_SRC_BUS_OFF_STATUS |                               \
        DL_MCAN_INTR_SRC_PROTOCOL_ERR_ARB |                             \
        DL_MCAN_INTR_SRC_PROTOCOL_ERR_DATA)
#define UART_TX_BUFFER_SIZE             (1024U)
#define CURRENT_ISR_PERIOD_TICKS        \
    (CURRENT_SAMPLE_TIMER_INST_LOAD_VALUE + 1U)
#define CURRENT_ISR_BUDGET_TICKS        \
    ((CURRENT_ISR_PERIOD_TICKS * 3U) / 4U)
#define CURRENT_ADC_ERROR_INTERRUPTS     \
    (DL_ADC12_INTERRUPT_OVERFLOW | DL_ADC12_INTERRUPT_TRIG_OVF |          \
        DL_ADC12_INTERRUPT_UNDERFLOW)

static volatile bool gFaultInterrupt;
static bool gFaultReported;
static bool gDriverReady;
static MOTOR_FOC_StopReason gReportedMotorStopReason;

typedef struct {
    uint16_t phaseA;
    uint16_t phaseB;
    uint16_t phaseC;
} CurrentAdcSample;

typedef struct {
    uint8_t agc;
    uint8_t diagnostics;
    uint16_t magnitude;
    uint16_t angle;
} AS5048B_Sample;

typedef enum {
    ENCODER_FAST_IDLE = 0,
    ENCODER_FAST_WAIT_TX,
    ENCODER_FAST_WAIT_RX
} EncoderFastState;

static volatile CurrentAdcSample gCurrentOffset;
static volatile CurrentAdcSample gCurrentLatest;
static volatile bool gCurrentSenseReady;
static volatile uint32_t gCurrentSampleSequence;

static volatile bool gCurrentCalibrationActive;
static volatile uint16_t gCurrentCalibrationCount;
static volatile uint32_t gCurrentCalibrationSumA;
static volatile uint32_t gCurrentCalibrationSumB;
static volatile uint32_t gCurrentCalibrationSumC;

static volatile uint16_t gCurrentWindowCount;
static volatile uint8_t gCurrentDiagnosticDecimationCount;
static volatile int32_t gCurrentWindowSumA;
static volatile int32_t gCurrentWindowSumB;
static volatile int32_t gCurrentWindowSumC;
static volatile uint16_t gCurrentWindowPeakA;
static volatile uint16_t gCurrentWindowPeakB;
static volatile uint16_t gCurrentWindowPeakC;
static volatile int16_t gCurrentAverageA;
static volatile int16_t gCurrentAverageB;
static volatile int16_t gCurrentAverageC;
static volatile uint16_t gCurrentPeakA;
static volatile uint16_t gCurrentPeakB;
static volatile uint16_t gCurrentPeakC;
static volatile uint32_t gCurrentIsrLastTicks;
static volatile uint32_t gCurrentIsrMaxTicks;
static volatile uint32_t gCurrentIsrBudgetExceededCount;
static volatile bool gCurrentIsrActive;
static volatile uint32_t gCurrentIsrReentryCount;
static volatile uint32_t gCurrentAdcValidSampleCount;
static volatile uint32_t gCurrentAdcPairMissCount;
static volatile uint32_t gCurrentAdcUnexpectedIrqCount;
static volatile uint32_t gCurrentAdc0OverflowCount;
static volatile uint32_t gCurrentAdc0TriggerOverflowCount;
static volatile uint32_t gCurrentAdc0UnderflowCount;
static volatile uint32_t gCurrentAdc1OverflowCount;
static volatile uint32_t gCurrentAdc1TriggerOverflowCount;
static volatile uint32_t gCurrentAdc1UnderflowCount;
static uint32_t gEncoderI2CStartDelayCycles = 3U;
static EncoderFastState gEncoderFastState;
static uint8_t gEncoderFastData[2];
static uint8_t gEncoderFastRxCount;
static uint32_t gEncoderFastStartTick;
static uint32_t gEncoderFastNextTick;
static uint32_t gEncoderFastLastCompleteTick;
static uint32_t gEncoderFastRequestCount;
static uint32_t gEncoderFastSuccessCount;
static uint32_t gEncoderFastErrorCount;
static uint32_t gEncoderFastTimeoutCount;
static uint32_t gEncoderFastMissedPeriodCount;
static uint32_t gEncoderFastMaxIntervalTicks;
static bool gEncoderFastPaused;

static volatile uint32_t gCanInterruptStatus;
static volatile bool gCanServicePending;
static uint32_t gCanLastInterruptStatus;
static uint32_t gCanRxCount;
static uint32_t gCanRxLostCount;
static uint32_t gCanTxRequestCount;
static uint32_t gCanTxCompleteCount;
static uint32_t gCanErrorEventCount;
static uint8_t gCanTxSequence;
static bool gCanLastRxValid;
static DL_MCAN_RxBufElement gCanLastRx;

static void serviceEncoderFast(void);

static uint8_t gUartTxBuffer[UART_TX_BUFFER_SIZE];
static volatile uint16_t gUartTxHead;
static volatile uint16_t gUartTxTail;
static volatile uint32_t gUartTxDroppedCount;

static uint16_t uartNextTxIndex(uint16_t index)
{
    index++;
    if (index >= UART_TX_BUFFER_SIZE) {
        index = 0U;
    }
    return index;
}

static void initializeDebugUartAsync(void)
{
    NVIC_DisableIRQ(DEBUG_UART_INST_INT_IRQN);
    DL_UART_Main_disableInterrupt(
        DEBUG_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    DL_UART_Main_clearInterruptStatus(
        DEBUG_UART_INST, DL_UART_MAIN_INTERRUPT_TX);

    gUartTxHead = 0U;
    gUartTxTail = 0U;
    gUartTxDroppedCount = 0U;
    NVIC_ClearPendingIRQ(DEBUG_UART_INST_INT_IRQN);
}

static void serviceDebugUartTx(void)
{
    while ((gUartTxTail != gUartTxHead) &&
        !DL_UART_Main_isTXFIFOFull(DEBUG_UART_INST)) {
        DL_UART_Main_transmitData(
            DEBUG_UART_INST, gUartTxBuffer[gUartTxTail]);
        gUartTxTail = uartNextTxIndex(gUartTxTail);
    }
}

static void uartPutChar(char character)
{
    uint16_t nextHead = uartNextTxIndex(gUartTxHead);

    if (nextHead == gUartTxTail) {
        gUartTxDroppedCount++;
        return;
    }

    gUartTxBuffer[gUartTxHead] = (uint8_t) character;
    gUartTxHead = nextHead;
    serviceDebugUartTx();
}

static void uartPutString(const char *string)
{
    while (*string != '\0') {
        uartPutChar(*string++);
    }
}

static void uartPutHex16(uint16_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    int8_t shift;

    uartPutString("0x");
    for (shift = 12; shift >= 0; shift -= 4) {
        uartPutChar(hex[(value >> shift) & 0x0FU]);
    }
}

static void uartPutHex8(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    uartPutString("0x");
    uartPutChar(hex[(value >> 4U) & 0x0FU]);
    uartPutChar(hex[value & 0x0FU]);
}

static void uartPutHex32(uint32_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    int8_t shift;

    uartPutString("0x");
    for (shift = 28; shift >= 0; shift -= 4) {
        uartPutChar(hex[(value >> shift) & 0x0FU]);
    }
}

static void uartPutUnsigned(uint32_t value)
{
    char buffer[10];
    uint8_t length = 0U;

    do {
        buffer[length++] = (char) ('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (length < sizeof(buffer)));

    while (length > 0U) {
        uartPutChar(buffer[--length]);
    }
}

static void uartPutSigned(int32_t value)
{
    if (value < 0) {
        uartPutChar('-');
        uartPutUnsigned((uint32_t) (-value));
    } else {
        uartPutChar('+');
        uartPutUnsigned((uint32_t) value);
    }
}

static void uartPutThreeDigits(uint16_t value)
{
    uartPutChar((char) ('0' + ((value / 100U) % 10U)));
    uartPutChar((char) ('0' + ((value / 10U) % 10U)));
    uartPutChar((char) ('0' + (value % 10U)));
}

static uint16_t uartGetTxQueuedCount(void)
{
    uint16_t head;
    uint16_t tail;
    uint32_t interruptState = __get_PRIMASK();

    __disable_irq();
    head = gUartTxHead;
    tail = gUartTxTail;
    if (interruptState == 0U) {
        __enable_irq();
    }

    if (head >= tail) {
        return (uint16_t) (head - tail);
    }
    return (uint16_t) (UART_TX_BUFFER_SIZE - tail + head);
}

static void printUartStatus(void)
{
    const uint16_t queuedCount = uartGetTxQueuedCount();
    const uint32_t droppedCount = gUartTxDroppedCount;

    uartPutString("UART TX async queued=");
    uartPutUnsigned(queuedCount);
    uartPutString(" dropped=");
    uartPutUnsigned(droppedCount);
    uartPutString(" capacity=");
    uartPutUnsigned(UART_TX_BUFFER_SIZE - 1U);
    uartPutString(".\r\n");
}

static void initializeCanDebug(void)
{
    /* TCAN3413 STB is active high; hold PA23 low for Normal mode. */
    DL_GPIO_clearPins(CAN_CTRL_PORT, CAN_CTRL_STB_PIN);

    gCanInterruptStatus = 0U;
    gCanServicePending = false;
    gCanLastInterruptStatus = 0U;
    gCanRxCount = 0U;
    gCanRxLostCount = 0U;
    gCanTxRequestCount = 0U;
    gCanTxCompleteCount = 0U;
    gCanErrorEventCount = 0U;
    gCanTxSequence = 0U;
    gCanLastRxValid = false;

    /* TC is gated by the dedicated-buffer interrupt enable as well as IE.TC. */
    (void) DL_MCAN_TXBufTransIntrEnable(
        CAN_DEBUG_INST, CAN_DEBUG_TX_BUFFER, true);
    NVIC_SetPriority(CAN_DEBUG_INST_INT_IRQN, 2U);
    NVIC_ClearPendingIRQ(CAN_DEBUG_INST_INT_IRQN);
    NVIC_EnableIRQ(CAN_DEBUG_INST_INT_IRQN);
}

static void serviceCanDebug(void)
{
    DL_MCAN_RxFIFOStatus fifoStatus;
    uint32_t interruptStatus;

    if (!gCanServicePending) {
        return;
    }

    /* Preserve an interrupt arriving while the main loop takes the snapshot. */
    __disable_irq();
    interruptStatus = gCanInterruptStatus;
    gCanInterruptStatus = 0U;
    gCanServicePending = false;
    __enable_irq();

    gCanLastInterruptStatus = interruptStatus;
    if ((interruptStatus & DL_MCAN_INTR_SRC_TRANS_COMPLETE) != 0U) {
        gCanTxCompleteCount++;
    }
    if ((interruptStatus & DL_MCAN_INTR_SRC_RX_FIFO0_MSG_LOST) != 0U) {
        gCanRxLostCount++;
    }
    if ((interruptStatus & CAN_DEBUG_ERROR_EVENTS) != 0U) {
        gCanErrorEventCount++;
    }

    fifoStatus.num = DL_MCAN_RX_FIFO_NUM_0;
    for (;;) {
        DL_MCAN_RxBufElement message;

        DL_MCAN_getRxFIFOStatus(CAN_DEBUG_INST, &fifoStatus);
        if (fifoStatus.fillLvl == 0U) {
            break;
        }

        DL_MCAN_readMsgRam(CAN_DEBUG_INST, DL_MCAN_MEM_TYPE_FIFO, 0U,
            fifoStatus.num, &message);
        (void) DL_MCAN_writeRxFIFOAck(
            CAN_DEBUG_INST, fifoStatus.num, fifoStatus.getIdx);
        gCanLastRx = message;
        gCanLastRxValid = true;
        gCanRxCount++;
    }
}

static bool sendCanDebugFrame(void)
{
    DL_MCAN_TxBufElement message;
    MOTOR_FOC_Status motor;
    int16_t targetRpm;
    int16_t measuredRpm;

    if (DL_MCAN_getOpMode(CAN_DEBUG_INST) !=
        DL_MCAN_OPERATION_MODE_NORMAL) {
        uartPutString("CAN TX rejected: controller is not in NORMAL mode.\r\n");
        return false;
    }
    if ((DL_MCAN_getTxBufReqPend(CAN_DEBUG_INST) &
            (1UL << CAN_DEBUG_TX_BUFFER)) != 0U) {
        uartPutString("CAN TX busy: previous frame is still pending (check ACK/wiring).\r\n");
        return false;
    }

    MOTOR_FOC_getStatus(&motor);
    targetRpm = (int16_t) motor.targetRpm;
    measuredRpm = (int16_t) motor.measuredRpm;

    message.id = CAN_DEBUG_TX_ID << 18U;
    message.rtr = 0U;
    message.xtd = 0U;
    message.esi = 0U;
    message.dlc = 8U;
    message.brs = 0U;
    message.fdf = 0U;
    message.efc = 0U;
    message.mm = gCanTxSequence;
    message.data[0] = 0xA5U;
    message.data[1] = gCanTxSequence++;
    message.data[2] = (uint8_t) motor.state;
    message.data[3] = (uint8_t) motor.controlMode;
    message.data[4] = (uint8_t) targetRpm;
    message.data[5] = (uint8_t) ((uint16_t) targetRpm >> 8U);
    message.data[6] = (uint8_t) measuredRpm;
    message.data[7] = (uint8_t) ((uint16_t) measuredRpm >> 8U);

    DL_MCAN_writeMsgRam(CAN_DEBUG_INST, DL_MCAN_MEM_TYPE_BUF,
        CAN_DEBUG_TX_BUFFER, &message);
    if (DL_MCAN_TXBufAddReq(
            CAN_DEBUG_INST, CAN_DEBUG_TX_BUFFER) != 0) {
        uartPutString("CAN TX request failed.\r\n");
        return false;
    }

    gCanTxRequestCount++;
    uartPutString("CAN TX requested: STD ID=0x321 DLC=8 data=A5 ");
    uartPutHex8(message.data[1]);
    uartPutString(" state/mode/rpm.\r\n");
    return true;
}

static uint8_t canPayloadLength(const DL_MCAN_RxBufElement *message)
{
    return (message->dlc > 8U) ? 8U : (uint8_t) message->dlc;
}

static void printCanDebugStatus(void)
{
    DL_MCAN_ProtocolStatus protocol;
    DL_MCAN_ErrCntStatus counters;
    uint32_t operationMode = DL_MCAN_getOpMode(CAN_DEBUG_INST);

    serviceCanDebug();
    DL_MCAN_getProtocolStatus(CAN_DEBUG_INST, &protocol);
    DL_MCAN_getErrCounters(CAN_DEBUG_INST, &counters);

    uartPutString("CAN 500kbps classic PA26=TX PA27=RX PA23=STB(");
    uartPutString((DL_GPIO_readPins(CAN_CTRL_PORT, CAN_CTRL_STB_PIN) &
        CAN_CTRL_STB_PIN) ? "STANDBY" : "NORMAL");
    uartPutString(") op=");
    uartPutString((operationMode == DL_MCAN_OPERATION_MODE_NORMAL) ?
        "NORMAL" : "SW_INIT");
    uartPutString(" tx_req/done=");
    uartPutUnsigned(gCanTxRequestCount);
    uartPutChar('/');
    uartPutUnsigned(gCanTxCompleteCount);
    uartPutString(" rx/lost=");
    uartPutUnsigned(gCanRxCount);
    uartPutChar('/');
    uartPutUnsigned(gCanRxLostCount);
    uartPutString(" err_evt=");
    uartPutUnsigned(gCanErrorEventCount);
    uartPutString(" TEC/REC=");
    uartPutUnsigned(counters.transErrLogCnt);
    uartPutChar('/');
    uartPutUnsigned(counters.recErrCnt);
    uartPutString(" LEC=");
    uartPutUnsigned(protocol.lastErrCode);
    uartPutString(" BO/EP/EW=");
    uartPutUnsigned(protocol.busOffStatus);
    uartPutChar('/');
    uartPutUnsigned(protocol.errPassive);
    uartPutChar('/');
    uartPutUnsigned(protocol.warningStatus);
    uartPutString(" pending=");
    uartPutUnsigned(DL_MCAN_getTxBufReqPend(CAN_DEBUG_INST) & 1U);
    uartPutString(" irq=");
    uartPutHex32(gCanLastInterruptStatus);
    uartPutString("\r\n");

    if (!gCanLastRxValid) {
        uartPutString("CAN RX: none.\r\n");
    } else {
        uint32_t identifier = gCanLastRx.xtd ?
            (gCanLastRx.id & 0x1FFFFFFFU) :
            ((gCanLastRx.id >> 18U) & 0x7FFU);
        uint8_t length = canPayloadLength(&gCanLastRx);
        uint8_t index;

        uartPutString("CAN RX last: ");
        uartPutString(gCanLastRx.xtd ? "EXT" : "STD");
        uartPutString(" ID=");
        uartPutHex32(identifier);
        uartPutString(" DLC=");
        uartPutUnsigned(gCanLastRx.dlc);
        uartPutString(gCanLastRx.rtr ? " RTR" : " data=");
        if (!gCanLastRx.rtr) {
            for (index = 0U; index < length; index++) {
                if (index != 0U) {
                    uartPutChar(' ');
                }
                uartPutHex8(gCanLastRx.data[index]);
            }
        }
        uartPutString("\r\n");
    }
}

static void initializeEncoderI2C(void)
{
    DL_I2C_ClockConfig clockConfig;
    uint32_t clockFrequency = CPUCLK_FREQ;

    DL_I2C_getClockConfig(ENCODER_I2C_INST, &clockConfig);
    if (clockConfig.clockSel == DL_I2C_CLOCK_MFCLK) {
        clockFrequency = 4000000U;
    }

    /* MSPM0 I2C_ERR_13: wait at least three I2C functional clocks. */
    gEncoderI2CStartDelayCycles =
        (3U * ((uint32_t) clockConfig.divideRatio + 1U)) *
        (CPUCLK_FREQ / clockFrequency);
    if (gEncoderI2CStartDelayCycles == 0U) {
        gEncoderI2CStartDelayCycles = 3U;
    }

    gEncoderFastState = ENCODER_FAST_IDLE;
    gEncoderFastRxCount = 0U;
    gEncoderFastStartTick = 0U;
    gEncoderFastNextTick = 0U;
    gEncoderFastLastCompleteTick = 0U;
    gEncoderFastRequestCount = 0U;
    gEncoderFastSuccessCount = 0U;
    gEncoderFastErrorCount = 0U;
    gEncoderFastTimeoutCount = 0U;
    gEncoderFastMissedPeriodCount = 0U;
    gEncoderFastMaxIntervalTicks = 0U;
    gEncoderFastPaused = false;
}

static void resetEncoderI2CTransfer(void)
{
    DL_I2C_resetControllerTransfer(ENCODER_I2C_INST);
    DL_I2C_flushControllerTXFIFO(ENCODER_I2C_INST);
    DL_I2C_flushControllerRXFIFO(ENCODER_I2C_INST);
    DL_I2C_clearInterruptStatus(
        ENCODER_I2C_INST, ENCODER_I2C_TRANSFER_EVENTS);
}

static bool waitEncoderI2CIdle(void)
{
    uint32_t timeout = ENCODER_I2C_TIMEOUT_LOOPS;

    while (timeout > 0U) {
        uint32_t status = DL_I2C_getControllerStatus(ENCODER_I2C_INST);

        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            resetEncoderI2CTransfer();
            return false;
        }
        if ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) {
            return true;
        }
        timeout--;
    }

    resetEncoderI2CTransfer();
    return false;
}

static bool waitEncoderI2CEvent(uint32_t completionEvent)
{
    uint32_t timeout = ENCODER_I2C_TIMEOUT_LOOPS;

    while (timeout > 0U) {
        uint32_t events = DL_I2C_getRawInterruptStatus(ENCODER_I2C_INST,
            completionEvent | ENCODER_I2C_ERROR_EVENTS);

        if ((events & ENCODER_I2C_ERROR_EVENTS) != 0U) {
            resetEncoderI2CTransfer();
            return false;
        }
        if ((events & completionEvent) != 0U) {
            DL_I2C_clearInterruptStatus(
                ENCODER_I2C_INST, completionEvent);
            return true;
        }
        timeout--;
    }

    resetEncoderI2CTransfer();
    return false;
}

static bool readEncoderRegisters(
    uint8_t startRegister, uint8_t *data, uint8_t length)
{
    uint8_t index;
    uint32_t pauseTimeout;
    bool restoreFastSampler;
    bool success = false;

    if ((data == NULL) || (length == 0U) || (length > 8U)) {
        return false;
    }

    /* Let an in-flight fast transaction finish before borrowing the bus. */
    restoreFastSampler = !gEncoderFastPaused;
    gEncoderFastPaused = true;
    pauseTimeout = ENCODER_I2C_TIMEOUT_LOOPS;
    while ((gEncoderFastState != ENCODER_FAST_IDLE) &&
        (pauseTimeout > 0U)) {
        serviceEncoderFast();
        pauseTimeout--;
    }
    if (gEncoderFastState != ENCODER_FAST_IDLE) {
        resetEncoderI2CTransfer();
        gEncoderFastState = ENCODER_FAST_IDLE;
        gEncoderFastRxCount = 0U;
        goto cleanup;
    }

    resetEncoderI2CTransfer();
    if (!waitEncoderI2CIdle()) {
        goto cleanup;
    }

    /* Random read: write register address, retain the bus for RESTART. */
    DL_I2C_transmitControllerData(ENCODER_I2C_INST, startRegister);
    DL_I2C_startControllerTransferAdvanced(ENCODER_I2C_INST,
        AS5048B_I2C_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_TX, 1U,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(gEncoderI2CStartDelayCycles);
    if (!waitEncoderI2CEvent(DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)) {
        goto cleanup;
    }

    /* Repeated START, sequentially read AGC through angle-low, then STOP. */
    DL_I2C_startControllerTransferAdvanced(ENCODER_I2C_INST,
        AS5048B_I2C_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_RX, length,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(gEncoderI2CStartDelayCycles);

    for (index = 0U; index < length; index++) {
        uint32_t timeout = ENCODER_I2C_TIMEOUT_LOOPS;

        while (DL_I2C_isControllerRXFIFOEmpty(ENCODER_I2C_INST)) {
            uint32_t events = DL_I2C_getRawInterruptStatus(
                ENCODER_I2C_INST, ENCODER_I2C_ERROR_EVENTS);
            if ((events & ENCODER_I2C_ERROR_EVENTS) != 0U) {
                resetEncoderI2CTransfer();
                goto cleanup;
            }
            if (timeout == 0U) {
                resetEncoderI2CTransfer();
                goto cleanup;
            }
            timeout--;
        }
        data[index] = DL_I2C_receiveControllerData(ENCODER_I2C_INST);
    }

    if (!waitEncoderI2CEvent(DL_I2C_INTERRUPT_CONTROLLER_RX_DONE)) {
        goto cleanup;
    }

    success = waitEncoderI2CIdle();

cleanup:
    gEncoderFastNextTick = gCurrentSampleSequence +
        ENCODER_FAST_PERIOD_TICKS;
    if (restoreFastSampler) {
        gEncoderFastPaused = false;
    }
    return success;
}

static bool readEncoderSample(AS5048B_Sample *sample)
{
    uint8_t data[AS5048B_READOUT_LENGTH];

    if ((sample == NULL) ||
        !readEncoderRegisters(
            AS5048B_REG_AGC, data, AS5048B_READOUT_LENGTH)) {
        return false;
    }

    sample->agc = data[0];
    sample->diagnostics = data[1];
    sample->magnitude =
        ((uint16_t) data[2] << 6) | ((uint16_t) data[3] & 0x3FU);
    sample->angle =
        ((uint16_t) data[4] << 6) | ((uint16_t) data[5] & 0x3FU);
    return true;
}

static bool isEncoderSampleValid(const AS5048B_Sample *sample)
{
    return (sample != NULL) &&
        ((sample->diagnostics & 0x01U) != 0U) &&
        ((sample->diagnostics & 0x0EU) == 0U);
}

static bool encoderFastDeadlineReached(uint32_t now, uint32_t deadline)
{
    return ((int32_t) (now - deadline) >= 0);
}

static void encoderFastAbort(bool timeout, uint32_t now)
{
    resetEncoderI2CTransfer();
    gEncoderFastState = ENCODER_FAST_IDLE;
    gEncoderFastRxCount = 0U;
    gEncoderFastErrorCount++;
    if (timeout) {
        gEncoderFastTimeoutCount++;
    }
    gEncoderFastNextTick = now + ENCODER_FAST_PERIOD_TICKS;
}

static bool encoderFastStart(uint32_t now)
{
    uint32_t status;

    resetEncoderI2CTransfer();
    status = DL_I2C_getControllerStatus(ENCODER_I2C_INST);
    if (((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) ||
        ((status & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U)) {
        encoderFastAbort(false, now);
        return false;
    }

    DL_I2C_transmitControllerData(ENCODER_I2C_INST, AS5048B_REG_ANGLE);
    DL_I2C_startControllerTransferAdvanced(ENCODER_I2C_INST,
        AS5048B_I2C_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_TX, 1U,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(gEncoderI2CStartDelayCycles);

    gEncoderFastRequestCount++;
    gEncoderFastStartTick = now;
    gEncoderFastRxCount = 0U;
    gEncoderFastState = ENCODER_FAST_WAIT_TX;
    return true;
}

static void encoderFastComplete(uint32_t now)
{
    uint16_t angle = ((uint16_t) gEncoderFastData[0] << 6) |
        ((uint16_t) gEncoderFastData[1] & 0x3FU);

    if (gEncoderFastSuccessCount != 0U) {
        uint32_t interval = now - gEncoderFastLastCompleteTick;
        if (interval > gEncoderFastMaxIntervalTicks) {
            gEncoderFastMaxIntervalTicks = interval;
        }
    }

    gEncoderFastLastCompleteTick = now;
    gEncoderFastSuccessCount++;
    gEncoderFastState = ENCODER_FAST_IDLE;
    gEncoderFastNextTick += ENCODER_FAST_PERIOD_TICKS;
    if (encoderFastDeadlineReached(now, gEncoderFastNextTick)) {
        uint32_t lateTicks = now - gEncoderFastNextTick;
        gEncoderFastMissedPeriodCount +=
            1U + (lateTicks / ENCODER_FAST_PERIOD_TICKS);
        gEncoderFastNextTick = now + ENCODER_FAST_PERIOD_TICKS;
    }

    MOTOR_FOC_onEncoderSample(angle, true, now);
}

static void serviceEncoderFast(void)
{
    uint32_t now = gCurrentSampleSequence;
    uint32_t events;

    if (gEncoderFastState == ENCODER_FAST_IDLE) {
        if (gEncoderFastPaused) {
            return;
        }
        if (encoderFastDeadlineReached(now, gEncoderFastNextTick)) {
            (void) encoderFastStart(now);
        }
        return;
    }

    events = DL_I2C_getRawInterruptStatus(ENCODER_I2C_INST,
        ENCODER_I2C_TRANSFER_EVENTS);
    if ((events & ENCODER_I2C_ERROR_EVENTS) != 0U) {
        encoderFastAbort(false, now);
        return;
    }
    if ((now - gEncoderFastStartTick) > ENCODER_FAST_TIMEOUT_TICKS) {
        encoderFastAbort(true, now);
        return;
    }

    if (gEncoderFastState == ENCODER_FAST_WAIT_TX) {
        if ((events & DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) == 0U) {
            return;
        }

        DL_I2C_clearInterruptStatus(
            ENCODER_I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
        DL_I2C_startControllerTransferAdvanced(ENCODER_I2C_INST,
            AS5048B_I2C_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_RX, 2U,
            DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE,
            DL_I2C_CONTROLLER_ACK_DISABLE);
        delay_cycles(gEncoderI2CStartDelayCycles);
        gEncoderFastState = ENCODER_FAST_WAIT_RX;
        return;
    }

    while ((gEncoderFastRxCount < 2U) &&
        !DL_I2C_isControllerRXFIFOEmpty(ENCODER_I2C_INST)) {
        gEncoderFastData[gEncoderFastRxCount++] =
            DL_I2C_receiveControllerData(ENCODER_I2C_INST);
    }

    if ((events & DL_I2C_INTERRUPT_CONTROLLER_RX_DONE) != 0U) {
        DL_I2C_clearInterruptStatus(
            ENCODER_I2C_INST, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
        if (gEncoderFastRxCount == 2U) {
            encoderFastComplete(now);
        } else {
            encoderFastAbort(false, now);
        }
    }
}

static const char *encoderFastStateName(void)
{
    switch (gEncoderFastState) {
        case ENCODER_FAST_WAIT_TX:
            return "WAIT_TX";
        case ENCODER_FAST_WAIT_RX:
            return "WAIT_RX";
        default:
            return "IDLE";
    }
}

static void printEncoderFastStatus(void)
{
    uint32_t now = gCurrentSampleSequence;
    MOTOR_FOC_Status motor;

    MOTOR_FOC_getStatus(&motor);

    uartPutString("ENC fast state=");
    uartPutString(encoderFastStateName());
    uartPutString(" req/ok/err/to/miss=");
    uartPutUnsigned(gEncoderFastRequestCount);
    uartPutChar('/');
    uartPutUnsigned(gEncoderFastSuccessCount);
    uartPutChar('/');
    uartPutUnsigned(gEncoderFastErrorCount);
    uartPutChar('/');
    uartPutUnsigned(gEncoderFastTimeoutCount);
    uartPutChar('/');
    uartPutUnsigned(gEncoderFastMissedPeriodCount);
    uartPutString(" max_interval_us=");
    uartPutUnsigned(gEncoderFastMaxIntervalTicks * 50U);
    uartPutString(" age_us=");
    uartPutUnsigned((now - gEncoderFastLastCompleteTick) * 50U);
    uartPutString(" eangle=");
    uartPutUnsigned(motor.electricalAngleSample);
    uartPutChar('/');
    uartPutUnsigned(motor.electricalAnglePredicted);
    uartPutString(" evel_q16=");
    uartPutSigned(motor.electricalVelocityQ16);
    uartPutString(".\r\n");
}

static void resetEncoderFastRuntimeTiming(void)
{
    uint32_t now = gCurrentSampleSequence;

    gEncoderFastLastCompleteTick = now;
    gEncoderFastMaxIntervalTicks = 0U;
    gEncoderFastMissedPeriodCount = 0U;
    gEncoderFastNextTick = now;
}

static bool printEncoderStatus(void)
{
    AS5048B_Sample sample;
    uint32_t angleMilliDegrees;
    bool valid;

    if (MOTOR_FOC_isActive()) {
        uartPutString(
            "AS5048B full diagnostic rejected while FOC is active.\r\n");
        printEncoderFastStatus();
        return false;
    }

    if (!readEncoderSample(&sample)) {
        uartPutString(
            "AS5048B I2C read failed (addr=0x40, SDA=PA0, SCL=PA1).\r\n");
        return false;
    }

    /* 360000 / 16384 reduces to 22500 / 1024. */
    angleMilliDegrees =
        (((uint32_t) sample.angle * 22500U) + 512U) / 1024U;
    valid = isEncoderSampleValid(&sample);

    uartPutString("ENC angle=");
    uartPutUnsigned(angleMilliDegrees / 1000U);
    uartPutChar('.');
    uartPutThreeDigits((uint16_t) (angleMilliDegrees % 1000U));
    uartPutString("deg raw=");
    uartPutUnsigned(sample.angle);
    uartPutString(" mag=");
    uartPutUnsigned(sample.magnitude);
    uartPutString(" agc=");
    uartPutUnsigned(sample.agc);
    uartPutString(" diag=");
    uartPutHex16(sample.diagnostics);
    uartPutString(" OCF=");
    uartPutChar(((sample.diagnostics & 0x01U) != 0U) ? '1' : '0');
    uartPutString(" COF=");
    uartPutChar(((sample.diagnostics & 0x02U) != 0U) ? '1' : '0');
    uartPutString(" strong=");
    uartPutChar(((sample.diagnostics & 0x04U) != 0U) ? '1' : '0');
    uartPutString(" weak=");
    uartPutChar(((sample.diagnostics & 0x08U) != 0U) ? '1' : '0');
    uartPutString(valid ? " valid=1\r\n" : " valid=0\r\n");
    return true;
}

static uint32_t adcCountsToMillivolts(uint16_t counts)
{
    return (((uint32_t) counts * ADC_REFERENCE_MV) +
        (ADC_FULL_SCALE_COUNTS / 2U)) / ADC_FULL_SCALE_COUNTS;
}

static int32_t adcDeltaToMilliamps(int32_t deltaCounts)
{
    int64_t numerator = (int64_t) deltaCounts *
        (int64_t) ADC_REFERENCE_MV * 1000LL;
    int64_t denominator = (int64_t) ADC_FULL_SCALE_COUNTS *
        (int64_t) CURRENT_SHUNT_MILLIOHMS * (int64_t) CURRENT_CSA_GAIN;

    return (int32_t) (numerator / denominator);
}

static uint16_t absoluteDelta(int32_t value)
{
    return (uint16_t) ((value < 0) ? -value : value);
}

static void startCurrentSampling(void)
{
    /*
     * Keep the ADC trigger stopped until the driver and UART are initialized.
     * Starting it here also prevents a pending 20 kHz ADC interrupt from
     * taking over the CPU before the boot messages can be transmitted.
     */
    NVIC_DisableIRQ(CURRENT_ADC0_INST_INT_IRQN);
    DL_TimerG_stopCounter(CURRENT_SAMPLE_TIMER_INST);
    DL_ADC12_clearInterruptStatus(CURRENT_ADC0_INST,
        DL_ADC12_INTERRUPT_MEM1_RESULT_LOADED |
            CURRENT_ADC_ERROR_INTERRUPTS);
    DL_ADC12_clearInterruptStatus(CURRENT_ADC1_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED |
            CURRENT_ADC_ERROR_INTERRUPTS);

    gCurrentIsrActive = false;
    gCurrentIsrReentryCount = 0U;
    gCurrentAdcValidSampleCount = 0U;
    gCurrentAdcPairMissCount = 0U;
    gCurrentAdcUnexpectedIrqCount = 0U;
    gCurrentAdc0OverflowCount = 0U;
    gCurrentAdc0TriggerOverflowCount = 0U;
    gCurrentAdc0UnderflowCount = 0U;
    gCurrentAdc1OverflowCount = 0U;
    gCurrentAdc1TriggerOverflowCount = 0U;
    gCurrentAdc1UnderflowCount = 0U;

    NVIC_ClearPendingIRQ(CURRENT_ADC0_INST_INT_IRQN);
    NVIC_EnableIRQ(CURRENT_ADC0_INST_INT_IRQN);

    /* TIMG0 is phase-locked to TIMA0's quiet all-low PWM interval. */
    MOTOR_FOC_enableAdcTrigger();
}

static void resetCurrentStatistics(void)
{
    uint32_t interruptState = __get_PRIMASK();

    __disable_irq();
    gCurrentWindowCount = 0U;
    gCurrentDiagnosticDecimationCount = 0U;
    gCurrentWindowSumA = 0;
    gCurrentWindowSumB = 0;
    gCurrentWindowSumC = 0;
    gCurrentWindowPeakA = 0U;
    gCurrentWindowPeakB = 0U;
    gCurrentWindowPeakC = 0U;
    gCurrentAverageA = 0;
    gCurrentAverageB = 0;
    gCurrentAverageC = 0;
    gCurrentPeakA = 0U;
    gCurrentPeakB = 0U;
    gCurrentPeakC = 0U;
    if (interruptState == 0U) {
        __enable_irq();
    }
}

static bool calibrateCurrentOffsets(void)
{
    uint32_t timeout;
    uint32_t interruptState;
    uint32_t sumA;
    uint32_t sumB;
    uint32_t sumC;
    uint16_t count;
    uint32_t sequence;

    /* CAL shorts all three CSA inputs so the ADC measures true offsets. */
    gCurrentSenseReady = false;
    DRV8323RS_setCurrentSenseCalibration(true);
    delay_cycles((CPUCLK_FREQ / 1000000U) * 150U);

    interruptState = __get_PRIMASK();
    __disable_irq();
    gCurrentCalibrationCount = 0U;
    gCurrentCalibrationSumA = 0U;
    gCurrentCalibrationSumB = 0U;
    gCurrentCalibrationSumC = 0U;
    gCurrentCalibrationActive = true;
    if (interruptState == 0U) {
        __enable_irq();
    }

    /* Allow up to 100 ms for 64 synchronized samples (normally 3.2 ms). */
    timeout = 100U;
    while ((gCurrentCalibrationCount < CURRENT_CALIBRATION_SAMPLES) &&
           (timeout > 0U)) {
        delay_cycles(CPUCLK_FREQ / 1000U);
        timeout--;
    }

    interruptState = __get_PRIMASK();
    __disable_irq();
    gCurrentCalibrationActive = false;
    count = gCurrentCalibrationCount;
    sumA = gCurrentCalibrationSumA;
    sumB = gCurrentCalibrationSumB;
    sumC = gCurrentCalibrationSumC;
    if (interruptState == 0U) {
        __enable_irq();
    }

    DRV8323RS_setCurrentSenseCalibration(false);

    if ((timeout == 0U) || (count < CURRENT_CALIBRATION_SAMPLES)) {
        return false;
    }

    gCurrentOffset.phaseA = (uint16_t) (sumA / count);
    gCurrentOffset.phaseB = (uint16_t) (sumB / count);
    gCurrentOffset.phaseC = (uint16_t) (sumC / count);

    /* Discard a few CAL-to-normal transition samples. */
    sequence = gCurrentSampleSequence;
    timeout = 10U;
    while (((gCurrentSampleSequence - sequence) < 4U) && (timeout > 0U)) {
        delay_cycles(CPUCLK_FREQ / 1000U);
        timeout--;
    }

    gCurrentSenseReady = true;
    resetCurrentStatistics();
    return true;
}

static void printOneCurrentChannel(
    const char *name, uint16_t counts, uint16_t offset,
    int16_t averageDelta, uint16_t peakDelta)
{
    int32_t deltaCounts = (int32_t) counts - (int32_t) offset;

    uartPutString(name);
    uartPutChar('=');
    uartPutUnsigned(counts);
    uartPutChar('/');
    uartPutUnsigned(adcCountsToMillivolts(counts));
    uartPutString("mV d=");
    uartPutSigned(deltaCounts);
    uartPutString(" I=");
    uartPutSigned(adcDeltaToMilliamps(deltaCounts));
    uartPutString("mA avg=");
    uartPutSigned(adcDeltaToMilliamps(averageDelta));
    uartPutString("mA pk=");
    uartPutUnsigned((uint32_t) adcDeltaToMilliamps(peakDelta));
    uartPutString("mA");
}

static void printCurrentStatus(void)
{
    CurrentAdcSample sample;
    CurrentAdcSample offset;
    int16_t averageA;
    int16_t averageB;
    int16_t averageC;
    uint16_t peakA;
    uint16_t peakB;
    uint16_t peakC;
    uint32_t sequence;
    uint32_t interruptState;

    if (!gCurrentSenseReady) {
        uartPutString("Current ADC is not calibrated. Stop motor and send 'z'.\r\n");
        return;
    }

    interruptState = __get_PRIMASK();
    __disable_irq();
    sample.phaseA = gCurrentLatest.phaseA;
    sample.phaseB = gCurrentLatest.phaseB;
    sample.phaseC = gCurrentLatest.phaseC;
    offset.phaseA = gCurrentOffset.phaseA;
    offset.phaseB = gCurrentOffset.phaseB;
    offset.phaseC = gCurrentOffset.phaseC;
    averageA = gCurrentAverageA;
    averageB = gCurrentAverageB;
    averageC = gCurrentAverageC;
    peakA = gCurrentPeakA;
    peakB = gCurrentPeakB;
    peakC = gCurrentPeakC;
    sequence = gCurrentSampleSequence;
    if (interruptState == 0U) {
        __enable_irq();
    }

    if (sequence == 0U) {
        uartPutString("No PWM-synchronized current sample yet.\r\n");
        return;
    }

    printOneCurrentChannel(
        "SOA", sample.phaseA, offset.phaseA, averageA, peakA);
    uartPutString("  ");
    printOneCurrentChannel(
        "SOB", sample.phaseB, offset.phaseB, averageB, peakB);
    uartPutString("  ");
    printOneCurrentChannel(
        "SOC", sample.phaseC, offset.phaseC, averageC, peakC);
    uartPutString("  n=");
    uartPutUnsigned(sequence);
    uartPutString("\r\n");
}

static void printCurrentCalibrationFailure(void)
{
    uartPutString("Current ADC calibration failed: samples=");
    uartPutUnsigned(gCurrentCalibrationCount);
    uartPutString(" n=");
    uartPutUnsigned(gCurrentSampleSequence);
    uartPutString(".\r\n");
}

static void printCurrentIsrTiming(void)
{
    uint32_t lastTicks;
    uint32_t maxTicks;
    uint32_t budgetExceeded;
    uint32_t validSamples;
    uint32_t pairMisses;
    uint32_t unexpectedIrqs;
    uint32_t reentries;
    uint32_t adc0Overflow;
    uint32_t adc0TriggerOverflow;
    uint32_t adc0Underflow;
    uint32_t adc1Overflow;
    uint32_t adc1TriggerOverflow;
    uint32_t adc1Underflow;
    uint32_t interruptState = __get_PRIMASK();

    __disable_irq();
    lastTicks = gCurrentIsrLastTicks;
    maxTicks = gCurrentIsrMaxTicks;
    budgetExceeded = gCurrentIsrBudgetExceededCount;
    validSamples = gCurrentAdcValidSampleCount;
    pairMisses = gCurrentAdcPairMissCount;
    unexpectedIrqs = gCurrentAdcUnexpectedIrqCount;
    reentries = gCurrentIsrReentryCount;
    adc0Overflow = gCurrentAdc0OverflowCount;
    adc0TriggerOverflow = gCurrentAdc0TriggerOverflowCount;
    adc0Underflow = gCurrentAdc0UnderflowCount;
    adc1Overflow = gCurrentAdc1OverflowCount;
    adc1TriggerOverflow = gCurrentAdc1TriggerOverflowCount;
    adc1Underflow = gCurrentAdc1UnderflowCount;
    if (interruptState == 0U) {
        __enable_irq();
    }

    uartPutString("ADC ISR last/max/budget_ticks=");
    uartPutUnsigned(lastTicks);
    uartPutChar('/');
    uartPutUnsigned(maxTicks);
    uartPutChar('/');
    uartPutUnsigned(CURRENT_ISR_BUDGET_TICKS);
    uartPutString(" max_ns=");
    uartPutUnsigned((maxTicks * 1000U + 16U) / 32U);
    uartPutString(" budget_exceeded=");
    uartPutUnsigned(budgetExceeded);
    uartPutString(".\r\nADC sync valid/pair_miss/unexpected/reentry=");
    uartPutUnsigned(validSamples);
    uartPutChar('/');
    uartPutUnsigned(pairMisses);
    uartPutChar('/');
    uartPutUnsigned(unexpectedIrqs);
    uartPutChar('/');
    uartPutUnsigned(reentries);
    uartPutString(" adc0_ovf/trig/under=");
    uartPutUnsigned(adc0Overflow);
    uartPutChar('/');
    uartPutUnsigned(adc0TriggerOverflow);
    uartPutChar('/');
    uartPutUnsigned(adc0Underflow);
    uartPutString(" adc1_ovf/trig/under=");
    uartPutUnsigned(adc1Overflow);
    uartPutChar('/');
    uartPutUnsigned(adc1TriggerOverflow);
    uartPutChar('/');
    uartPutUnsigned(adc1Underflow);
    uartPutString(" pwm_update=zero.\r\n");
}

static void printFaultStatus(void)
{
    DRV8323RS_FaultStatus fault = DRV8323RS_readFaults();

    uartPutString("FAULT1=");
    uartPutHex16(fault.faultStatus1);
    uartPutString(" FAULT2=");
    uartPutHex16(fault.faultStatus2);
    uartPutString(" nFAULT=");
    uartPutChar(DRV8323RS_isFaultActive() ? '0' : '1');
    uartPutString("\r\n");
}

static const char *focStateName(MOTOR_FOC_State state)
{
    switch (state) {
        case MOTOR_FOC_STATE_ALIGNING:
            return "ALIGN";
        case MOTOR_FOC_STATE_RUNNING:
            return "RUN";
        case MOTOR_FOC_STATE_FAULT:
            return "FAULT";
        default:
            return "STOP";
    }
}

static const char *focStopReasonName(MOTOR_FOC_StopReason reason)
{
    switch (reason) {
        case MOTOR_FOC_STOP_COMMAND:
            return "command";
        case MOTOR_FOC_STOP_DRIVER_FAULT:
            return "driver-fault";
        case MOTOR_FOC_STOP_ENCODER_LOST:
            return "encoder-lost";
        case MOTOR_FOC_STOP_OVERCURRENT:
            return "overcurrent";
        default:
            return "none";
    }
}

static void printDutyPermille(uint16_t duty)
{
    uartPutUnsigned(duty / 10U);
    uartPutChar('.');
    uartPutChar((char) ('0' + (duty % 10U)));
}

static const char *focControlModeName(MOTOR_FOC_ControlMode mode)
{
    return (mode == MOTOR_FOC_MODE_POSITION) ? "POSITION" : "SPEED";
}

static void printEncoderAngleTenths(uint16_t raw)
{
    uint32_t tenths = ((uint32_t) raw * 3600U) / 16384U;

    uartPutUnsigned(tenths / 10U);
    uartPutChar('.');
    uartPutChar((char) ('0' + (tenths % 10U)));
}

static void printEncoderErrorTenths(int32_t counts)
{
    uint32_t magnitude;
    uint32_t tenths;

    if (counts < 0) {
        uartPutChar('-');
        magnitude = (uint32_t) (-counts);
    } else {
        uartPutChar('+');
        magnitude = (uint32_t) counts;
    }

    tenths = (magnitude * 3600U) / 16384U;
    uartPutUnsigned(tenths / 10U);
    uartPutChar('.');
    uartPutChar((char) ('0' + (tenths % 10U)));
}

static bool startFocMotor(void)
{
    AS5048B_Sample encoder;
    uint32_t millisecond;

    if (!gDriverReady) {
        uartPutString("FOC start rejected: driver initialization failed.\r\n");
        return false;
    }
    if (!gCurrentSenseReady) {
        uartPutString("FOC start rejected: current ADC is not calibrated.\r\n");
        return false;
    }
    if (!readEncoderSample(&encoder)) {
        uartPutString("FOC start rejected: AS5048B I2C read failed.\r\n");
        return false;
    }
    if (!isEncoderSampleValid(&encoder)) {
        uartPutString("FOC start rejected: AS5048B diagnostic is invalid.\r\n");
        printEncoderStatus();
        return false;
    }
    if (!MOTOR_FOC_beginAlignment()) {
        uartPutString("FOC start rejected: driver/state is not ready.\r\n");
        return false;
    }

    gReportedMotorStopReason = MOTOR_FOC_STOP_NONE;

    uartPutString("FOC rotor alignment (keep shaft unloaded)...\r\n");
    for (millisecond = 0U; millisecond < 500U; millisecond++) {
        serviceDebugUartTx();
        delay_cycles(CPUCLK_FREQ / 1000U);
        if (MOTOR_FOC_getState() != MOTOR_FOC_STATE_ALIGNING) {
            uartPutString("FOC alignment aborted by protection.\r\n");
            return false;
        }
    }

    if (!readEncoderSample(&encoder) || !isEncoderSampleValid(&encoder)) {
        MOTOR_FOC_emergencyStop(MOTOR_FOC_STOP_ENCODER_LOST);
        uartPutString("FOC alignment failed: encoder became invalid.\r\n");
        return false;
    }

    if (!MOTOR_FOC_finishAlignment(
            encoder.angle, gCurrentSampleSequence)) {
        uartPutString("FOC alignment failed.\r\n");
        return false;
    }

    MOTOR_FOC_onEncoderSample(
        encoder.angle, true, gCurrentSampleSequence);
    resetEncoderFastRuntimeTiming();
    uartPutString("FOC closed-loop started; electrical_offset=");
    uartPutHex16(MOTOR_FOC_getElectricalOffset());
    uartPutString(".\r\n");
    return true;
}

static void printMotorStatus(void)
{
    MOTOR_FOC_Status status;

    MOTOR_FOC_getStatus(&status);
    uartPutString("FOC state=");
    uartPutString(focStateName(status.state));
    uartPutString(" mode=");
    uartPutString(focControlModeName(status.controlMode));
    if (status.controlMode == MOTOR_FOC_MODE_POSITION) {
        uartPutString(" pos=");
        printEncoderAngleTenths(status.encoderRaw);
        uartPutString("deg target_pos=");
        printEncoderAngleTenths(status.targetPositionRaw);
        uartPutString("deg err=");
        printEncoderErrorTenths(status.positionErrorCounts);
        uartPutString("deg");
    }
    uartPutString(" target_rpm=");
    uartPutSigned(status.targetRpm);
    uartPutString(" speed_rpm=");
    uartPutSigned(status.measuredRpm);
    uartPutString(" Iq_ref=");
    uartPutSigned(status.iqReferenceMilliamps);
    uartPutString("mA Id/Iq=");
    uartPutSigned(status.idMilliamps);
    uartPutChar('/');
    uartPutSigned(status.iqMilliamps);
    uartPutString("mA PWM(A/B/C)=");
    printDutyPermille(status.dutyApermille);
    uartPutChar('/');
    printDutyPermille(status.dutyBpermille);
    uartPutChar('/');
    printDutyPermille(status.dutyCpermille);
    uartPutString("% enc=");
    uartPutUnsigned(status.encoderRaw);
    uartPutString(" stop=");
    uartPutString(focStopReasonName(status.stopReason));
    uartPutString("\r\n");
}

static void printHelp(void)
{
    uartPutString("\r\nDRV8323RS encoder FOC console, 115200 8N1\r\n");
    uartPutString("s=align/start SPEED FOC  p=start/hold POSITION  v=SPEED at 0rpm\r\n");
    uartPutString("SPEED: +=25rpm  -=25rpm magnitude  r=reverse\r\n");
    uartPutString("POSITION: +=+10deg  -=-10deg (single-turn shortest path)\r\n");
    uartPutString("x=stop/coast\r\n");
    uartPutString("f=fault/status  c=clear fault  i=current ADC\r\n");
    uartPutString("a=AS5048B angle/status  z=zero current ADC (stopped)\r\n");
    uartPutString("t=send CAN 0x321 debug frame  n=CAN status/last RX\r\n");
    uartPutString("e=encoder fast sampler  j=ADC ISR timing\r\n");
    uartPutString("u=UART TX queue status  h=help\r\n");
}

static void processCommand(uint8_t command)
{
    int32_t targetRpm;

    switch (command) {
        case 's':
        case 'S':
            if (MOTOR_FOC_isActive()) {
                uartPutString("Start ignored: motor is already running.\r\n");
            } else if (DRV8323RS_isFaultActive()) {
                uartPutString("Start rejected: nFAULT is low.\r\n");
                printFaultStatus();
            } else {
                (void) startFocMotor();
            }
            break;

        case 'x':
        case 'X':
            MOTOR_FOC_stop();
            uartPutString("Motor stopped (coast).\r\n");
            break;

        case 'r':
        case 'R':
            if (MOTOR_FOC_getControlMode() == MOTOR_FOC_MODE_POSITION) {
                uartPutString("Reverse is only for SPEED mode; use +/- position steps.\r\n");
            } else {
                MOTOR_FOC_reverse();
                uartPutString("FOC speed direction reversed.\r\n");
            }
            printMotorStatus();
            break;

        case '+':
            if (MOTOR_FOC_getControlMode() == MOTOR_FOC_MODE_POSITION) {
                MOTOR_FOC_adjustTargetPositionDegrees(
                    MOTOR_FOC_POSITION_STEP_DEGREES);
            } else {
                targetRpm = MOTOR_FOC_getTargetRpm();
                MOTOR_FOC_adjustTargetRpm((targetRpm < 0) ?
                    -MOTOR_FOC_TARGET_RPM_STEP : MOTOR_FOC_TARGET_RPM_STEP);
            }
            printMotorStatus();
            break;

        case '-':
            if (MOTOR_FOC_getControlMode() == MOTOR_FOC_MODE_POSITION) {
                MOTOR_FOC_adjustTargetPositionDegrees(
                    -MOTOR_FOC_POSITION_STEP_DEGREES);
            } else {
                targetRpm = MOTOR_FOC_getTargetRpm();
                if (targetRpm < 0) {
                    MOTOR_FOC_adjustTargetRpm(MOTOR_FOC_TARGET_RPM_STEP);
                } else if (targetRpm > 0) {
                    MOTOR_FOC_adjustTargetRpm(-MOTOR_FOC_TARGET_RPM_STEP);
                }
            }
            printMotorStatus();
            break;

        case 'p':
        case 'P':
            if (!MOTOR_FOC_isRunning()) {
                if (MOTOR_FOC_isActive()) {
                    uartPutString("Position start ignored: alignment is active.\r\n");
                    break;
                }
                if (DRV8323RS_isFaultActive()) {
                    uartPutString("Position start rejected: nFAULT is low.\r\n");
                    printFaultStatus();
                    break;
                }
                if (!startFocMotor()) {
                    break;
                }
            }
            if (MOTOR_FOC_enterPositionMode()) {
                uartPutString("POSITION hold enabled at current shaft angle.\r\n");
                printMotorStatus();
            } else {
                uartPutString("Position mode rejected: FOC is not running.\r\n");
            }
            break;

        case 'v':
        case 'V':
            if (MOTOR_FOC_isRunning()) {
                MOTOR_FOC_enterSpeedMode();
                uartPutString("SPEED mode selected; target is 0 rpm.\r\n");
                printMotorStatus();
            } else {
                uartPutString("Speed-mode change rejected: FOC is not running.\r\n");
            }
            break;

        case 'f':
        case 'F':
            printMotorStatus();
            printFaultStatus();
            break;

        case 'c':
        case 'C':
            MOTOR_FOC_stop();
            if (DRV8323RS_clearFaults()) {
                MOTOR_FOC_clearStopReason();
                gFaultInterrupt = false;
                gFaultReported = false;
                uartPutString("Faults cleared.\r\n");
            } else {
                uartPutString("Fault remains active.\r\n");
                printFaultStatus();
            }
            break;

        case 'i':
        case 'I':
            printCurrentStatus();
            break;

        case 'z':
        case 'Z':
            if (MOTOR_FOC_isActive()) {
                uartPutString("Current zero rejected: stop motor first.\r\n");
            } else if (calibrateCurrentOffsets()) {
                uartPutString("Current ADC zero calibrated: ");
                printCurrentStatus();
            } else {
                printCurrentCalibrationFailure();
            }
            break;

        case 'a':
        case 'A':
            printEncoderStatus();
            break;

        case 'e':
        case 'E':
            printEncoderFastStatus();
            break;

        case 'j':
        case 'J':
            printCurrentIsrTiming();
            break;

        case 't':
        case 'T':
            (void) sendCanDebugFrame();
            break;

        case 'n':
        case 'N':
            printCanDebugStatus();
            break;

        case 'u':
        case 'U':
            printUartStatus();
            break;

        case 'h':
        case 'H':
        case '?':
            printHelp();
            break;

        case '\r':
        case '\n':
            break;

        default:
            uartPutString("Unknown command. Send 'h'.\r\n");
            break;
    }
}

int main(void)
{
    SYSCFG_DL_init();
    initializeDebugUartAsync();
    initializeEncoderI2C();
    initializeCanDebug();
    uartPutString("\r\nBoot: peripherals initialized.\r\n");
    uartPutString("Boot: CAN initialized (500 kbps classic, PA26 TX, PA27 RX, PA23 STB=LOW).\r\n");
    MOTOR_FOC_initialize();
    uartPutString("Boot: PWM initialized.\r\n");
    gDriverReady = DRV8323RS_initialize();
    uartPutString(gDriverReady ? "Boot: driver initialized.\r\n" :
                                "Boot: driver initialization failed.\r\n");
    startCurrentSampling();
    gCurrentSenseReady = calibrateCurrentOffsets();

    DL_GPIO_clearInterruptStatus(DRV_FAULT_PORT,
        DRV_FAULT_NFAULT_PIN);
    NVIC_EnableIRQ(DRV_FAULT_INT_IRQN);

    printHelp();
    uartPutString(gDriverReady ? "Driver ready; motor is stopped.\r\n" :
                                "Driver initialization failed.\r\n");
    if (gCurrentSenseReady) {
        uartPutString("Current ADC ready.\r\n");
    } else {
        printCurrentCalibrationFailure();
    }
    printFaultStatus();
    if (gCurrentSenseReady) {
        printCurrentStatus();
    }
    /* AS5048B requires up to 10 ms after power-up before valid data. */
    delay_cycles(CPUCLK_FREQ / 100U);
    printEncoderStatus();

    while (1) {
        MOTOR_FOC_StopReason stopReason = MOTOR_FOC_getStopReason();

        serviceDebugUartTx();
        serviceCanDebug();
        serviceEncoderFast();

        if (gFaultInterrupt && !gFaultReported) {
            gFaultReported = true;
            uartPutString("Emergency coast: DRV8323RS fault.\r\n");
            printFaultStatus();
        }

        if ((MOTOR_FOC_getState() == MOTOR_FOC_STATE_FAULT) &&
            (stopReason != MOTOR_FOC_STOP_NONE) &&
            (stopReason != gReportedMotorStopReason)) {
            gReportedMotorStopReason = stopReason;
            uartPutString("FOC protection stop: ");
            uartPutString(focStopReasonName(stopReason));
            uartPutString(".\r\n");
            printMotorStatus();
        }

        if (!DL_UART_isRXFIFOEmpty(DEBUG_UART_INST)) {
            processCommand(DL_UART_receiveData(DEBUG_UART_INST));
        }

        if (!MOTOR_FOC_isActive()) {
            /* The 20 kHz ADC interrupt wakes the cooperative I/O services. */
            __WFI();
        }
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(
        DRV_FAULT_PORT, DRV_FAULT_NFAULT_PIN);

    if ((status & DRV_FAULT_NFAULT_PIN) != 0U) {
        MOTOR_FOC_emergencyStop(MOTOR_FOC_STOP_DRIVER_FAULT);
        gFaultInterrupt = true;
        DL_GPIO_clearInterruptStatus(
            DRV_FAULT_PORT, DRV_FAULT_NFAULT_PIN);
    }
}

void CURRENT_ADC0_INST_IRQHandler(void)
{
    uint32_t isrEntryCount =
        DL_TimerG_getTimerCount(CURRENT_SAMPLE_TIMER_INST);
    uint32_t adc0Status = DL_ADC12_getRawInterruptStatus(
        CURRENT_ADC0_INST,
        DL_ADC12_INTERRUPT_MEM1_RESULT_LOADED |
            CURRENT_ADC_ERROR_INTERRUPTS);
    uint32_t adc1Status;

    if (gCurrentIsrActive) {
        gCurrentIsrReentryCount++;
    }
    gCurrentIsrActive = true;

    if ((adc0Status & DL_ADC12_INTERRUPT_OVERFLOW) != 0U) {
        gCurrentAdc0OverflowCount++;
    }
    if ((adc0Status & DL_ADC12_INTERRUPT_TRIG_OVF) != 0U) {
        gCurrentAdc0TriggerOverflowCount++;
    }
    if ((adc0Status & DL_ADC12_INTERRUPT_UNDERFLOW) != 0U) {
        gCurrentAdc0UnderflowCount++;
    }
    DL_ADC12_clearInterruptStatus(CURRENT_ADC0_INST,
        adc0Status & CURRENT_ADC_ERROR_INTERRUPTS);

    if ((adc0Status & DL_ADC12_INTERRUPT_MEM1_RESULT_LOADED) == 0U) {
        gCurrentAdcUnexpectedIrqCount++;
        goto isrExit;
    }
    DL_ADC12_clearInterruptStatus(CURRENT_ADC0_INST,
        DL_ADC12_INTERRUPT_MEM1_RESULT_LOADED);
    gCurrentSampleSequence++;

    adc1Status = DL_ADC12_getRawInterruptStatus(CURRENT_ADC1_INST,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED |
            CURRENT_ADC_ERROR_INTERRUPTS);
    if ((adc1Status & DL_ADC12_INTERRUPT_OVERFLOW) != 0U) {
        gCurrentAdc1OverflowCount++;
    }
    if ((adc1Status & DL_ADC12_INTERRUPT_TRIG_OVF) != 0U) {
        gCurrentAdc1TriggerOverflowCount++;
    }
    if ((adc1Status & DL_ADC12_INTERRUPT_UNDERFLOW) != 0U) {
        gCurrentAdc1UnderflowCount++;
    }
    DL_ADC12_clearInterruptStatus(CURRENT_ADC1_INST,
        adc1Status & CURRENT_ADC_ERROR_INTERRUPTS);

    /* Never combine fresh A/B values with a stale phase-C result. */
    if ((adc1Status & DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) == 0U) {
        gCurrentAdcPairMissCount++;
        goto isrExit;
    }

    {
        uint16_t rawA = DL_ADC12_getMemResult(
            CURRENT_ADC0_INST, CURRENT_ADC0_ADCMEM_0); /* PA24, SOA */
        uint16_t rawB = DL_ADC12_getMemResult(
            CURRENT_ADC0_INST, CURRENT_ADC0_ADCMEM_1); /* PA25, SOB */
        uint16_t rawC = DL_ADC12_getMemResult(
            CURRENT_ADC1_INST, CURRENT_ADC1_ADCMEM_0); /* PA15, SOC */

        DL_ADC12_clearInterruptStatus(CURRENT_ADC1_INST,
            DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
        gCurrentLatest.phaseA = rawA;
        gCurrentLatest.phaseB = rawB;
        gCurrentLatest.phaseC = rawC;
        gCurrentAdcValidSampleCount++;

        if (gCurrentCalibrationActive) {
            if (gCurrentCalibrationCount < CURRENT_CALIBRATION_SAMPLES) {
                gCurrentCalibrationSumA += rawA;
                gCurrentCalibrationSumB += rawB;
                gCurrentCalibrationSumC += rawC;
                gCurrentCalibrationCount++;
            }
        } else if (gCurrentSenseReady) {
            int32_t deltaA = (int32_t) rawA -
                (int32_t) gCurrentOffset.phaseA;
            int32_t deltaB = (int32_t) rawB -
                (int32_t) gCurrentOffset.phaseB;
            int32_t deltaC = (int32_t) rawC -
                (int32_t) gCurrentOffset.phaseC;

            MOTOR_FOC_currentLoopISR((int16_t) deltaA, (int16_t) deltaB,
                (int16_t) deltaC, gCurrentSampleSequence);

            /* Diagnostic averages are decimated; protection above is not. */
            if (gCurrentDiagnosticDecimationCount == 0U) {
                uint16_t magnitudeA = absoluteDelta(deltaA);
                uint16_t magnitudeB = absoluteDelta(deltaB);
                uint16_t magnitudeC = absoluteDelta(deltaC);

                gCurrentWindowSumA += deltaA;
                gCurrentWindowSumB += deltaB;
                gCurrentWindowSumC += deltaC;
                if (magnitudeA > gCurrentWindowPeakA) {
                    gCurrentWindowPeakA = magnitudeA;
                }
                if (magnitudeB > gCurrentWindowPeakB) {
                    gCurrentWindowPeakB = magnitudeB;
                }
                if (magnitudeC > gCurrentWindowPeakC) {
                    gCurrentWindowPeakC = magnitudeC;
                }

                gCurrentWindowCount++;
                if (gCurrentWindowCount >= CURRENT_AVERAGE_SAMPLES) {
                    gCurrentAverageA = (int16_t)
                        (gCurrentWindowSumA /
                            (int32_t) CURRENT_AVERAGE_SAMPLES);
                    gCurrentAverageB = (int16_t)
                        (gCurrentWindowSumB /
                            (int32_t) CURRENT_AVERAGE_SAMPLES);
                    gCurrentAverageC = (int16_t)
                        (gCurrentWindowSumC /
                            (int32_t) CURRENT_AVERAGE_SAMPLES);
                    gCurrentPeakA = gCurrentWindowPeakA;
                    gCurrentPeakB = gCurrentWindowPeakB;
                    gCurrentPeakC = gCurrentWindowPeakC;

                    gCurrentWindowCount = 0U;
                    gCurrentWindowSumA = 0;
                    gCurrentWindowSumB = 0;
                    gCurrentWindowSumC = 0;
                    gCurrentWindowPeakA = 0U;
                    gCurrentWindowPeakB = 0U;
                    gCurrentWindowPeakC = 0U;
                }
            }

            gCurrentDiagnosticDecimationCount++;
            if (gCurrentDiagnosticDecimationCount >=
                CURRENT_DIAGNOSTIC_DECIMATION) {
                gCurrentDiagnosticDecimationCount = 0U;
            }
        }

    }

isrExit:
    {
        uint32_t isrExitCount =
            DL_TimerG_getTimerCount(CURRENT_SAMPLE_TIMER_INST);
        uint32_t elapsedTicks = (isrEntryCount >= isrExitCount) ?
            (isrEntryCount - isrExitCount) :
            (isrEntryCount + CURRENT_ISR_PERIOD_TICKS - isrExitCount);

        gCurrentIsrLastTicks = elapsedTicks;
        if (elapsedTicks > gCurrentIsrMaxTicks) {
            gCurrentIsrMaxTicks = elapsedTicks;
        }
        if (elapsedTicks > CURRENT_ISR_BUDGET_TICKS) {
            gCurrentIsrBudgetExceededCount++;
        }
    }
    gCurrentIsrActive = false;
}

void CAN_DEBUG_INST_IRQHandler(void)
{
    if (DL_MCAN_getPendingInterrupt(CAN_DEBUG_INST) ==
        DL_MCAN_IIDX_LINE1) {
        uint32_t status = DL_MCAN_getIntrStatus(CAN_DEBUG_INST);

        DL_MCAN_clearIntrStatus(CAN_DEBUG_INST, status,
            DL_MCAN_INTR_SRC_MCAN_LINE_1);
        gCanInterruptStatus |= status;
        gCanServicePending = true;
    }
}
