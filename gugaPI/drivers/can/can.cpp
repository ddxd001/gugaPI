#include "drivers/can/can.h"

namespace drivers {
namespace {

const uint32_t kCanErrorEvents =
    DL_MCAN_INTR_SRC_MSG_RAM_ACCESS_FAILURE |
    DL_MCAN_INTR_SRC_ERR_PASSIVE |
    DL_MCAN_INTR_SRC_WARNING_STATUS |
    DL_MCAN_INTR_SRC_BUS_OFF_STATUS |
    DL_MCAN_INTR_SRC_PROTOCOL_ERR_ARB |
    DL_MCAN_INTR_SRC_PROTOCOL_ERR_DATA;

bool IsConfigValid(const CanConfig *config)
{
    return (config != 0) && (config->mcan != 0) &&
           (config->stb_port != 0) && (config->stb_pin != 0U) &&
           (config->bitrate != 0U) && (config->tx_buffer < 32U) &&
           (config->rx_queue != 0) && (config->rx_queue_size > 1U);
}

bool IsFrameValid(const CanFrame *frame)
{
    if ((frame == 0) || (frame->length > 8U)) {
        return false;
    }
    return frame->extended ?
        (frame->id <= 0x1FFFFFFFU) : (frame->id <= 0x7FFU);
}

uint16_t NextRxIndex(const CanConfig *config, uint16_t index)
{
    index++;
    if (index >= config->rx_queue_size) {
        index = 0U;
    }
    return index;
}

uint16_t RxAvailable(const CanContext *context)
{
    const uint16_t head = context->rx_head;
    const uint16_t tail = context->rx_tail;
    if (head >= tail) {
        return static_cast<uint16_t>(head - tail);
    }
    return static_cast<uint16_t>(
        context->config->rx_queue_size - tail + head);
}

void PushRxFromIsr(CanContext *context,
                   const DL_MCAN_RxBufElement *message)
{
    const uint16_t next = NextRxIndex(context->config, context->rx_head);
    if (next == context->rx_tail) {
        context->rx_dropped_count++;
        return;
    }

    CanFrame &frame = context->config->rx_queue[context->rx_head];
    frame.extended = (message->xtd != 0U);
    frame.id = frame.extended ?
        (message->id & 0x1FFFFFFFU) :
        ((message->id >> 18U) & 0x7FFU);
    frame.length = (message->dlc > 8U) ?
        8U : static_cast<uint8_t>(message->dlc);
    for (uint8_t i = 0U; i < frame.length; i++) {
        frame.data[i] = message->data[i];
    }
    for (uint8_t i = frame.length; i < 8U; i++) {
        frame.data[i] = 0U;
    }
    context->rx_head = next;
    context->rx_count++;
}

void ResetSoftwareState(CanContext *context)
{
    context->rx_head = 0U;
    context->rx_tail = 0U;
    context->rx_count = 0U;
    context->rx_dropped_count = 0U;
    context->rx_fifo_lost_count = 0U;
    context->tx_request_count = 0U;
    context->tx_complete_count = 0U;
    context->tx_cancel_count = 0U;
    context->error_event_count = 0U;
    context->last_interrupt_status = 0U;
}

} /* namespace */

DriverStatus Can_Init(CanContext *context, const CanConfig *config)
{
    if ((context == 0) || (!IsConfigValid(config))) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    NVIC_DisableIRQ(config->irq);
    context->config = config;
    context->mode = CAN_TRANSCEIVER_NORMAL;
    context->initialized = false;
    ResetSoftwareState(context);

    /* STB has an internal pull-up; explicitly drive low for Normal mode. */
    DL_GPIO_clearPins(config->stb_port, config->stb_pin);
    DL_MCAN_setOpMode(config->mcan, DL_MCAN_OPERATION_MODE_NORMAL);
    (void) DL_MCAN_TXBufTransIntrEnable(
        config->mcan, config->tx_buffer, true);
    (void) DL_MCAN_getTxBufCancellationIntrEnable(
        config->mcan, config->tx_buffer, true);

    NVIC_SetPriority(config->irq, config->irq_priority);
    NVIC_ClearPendingIRQ(config->irq);
    context->initialized = true;
    NVIC_EnableIRQ(config->irq);
    return DRIVER_OK;
}

bool Can_IsReady(const CanContext *context)
{
    return (context != 0) && context->initialized &&
           (context->config != 0);
}

DriverStatus Can_SetMode(CanContext *context, CanTransceiverMode mode)
{
    if (!Can_IsReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if ((mode != CAN_TRANSCEIVER_NORMAL) &&
        (mode != CAN_TRANSCEIVER_STANDBY)) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    if (mode == CAN_TRANSCEIVER_STANDBY) {
        DL_MCAN_setOpMode(
            context->config->mcan, DL_MCAN_OPERATION_MODE_SW_INIT);
        DL_GPIO_setPins(
            context->config->stb_port, context->config->stb_pin);
    } else {
        DL_GPIO_clearPins(
            context->config->stb_port, context->config->stb_pin);
        DL_MCAN_setOpMode(
            context->config->mcan, DL_MCAN_OPERATION_MODE_NORMAL);
    }
    context->mode = mode;
    return DRIVER_OK;
}

DriverStatus Can_Send(CanContext *context, const CanFrame *frame)
{
    if (!Can_IsReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (!IsFrameValid(frame)) {
        return DRIVER_ERROR_INVALID_ARG;
    }
    if ((context->mode != CAN_TRANSCEIVER_NORMAL) ||
        (DL_MCAN_getOpMode(context->config->mcan) !=
         DL_MCAN_OPERATION_MODE_NORMAL)) {
        return DRIVER_ERROR_BUSY;
    }

    const uint32_t tx_mask = 1UL << context->config->tx_buffer;
    if ((DL_MCAN_getTxBufReqPend(context->config->mcan) & tx_mask) != 0U) {
        return DRIVER_ERROR_BUSY;
    }

    DL_MCAN_TxBufElement message = {};
    message.id = frame->extended ?
        frame->id : (frame->id << 18U);
    message.rtr = 0U;
    message.xtd = frame->extended ? 1U : 0U;
    message.esi = 0U;
    message.dlc = frame->length;
    message.brs = 0U;
    message.fdf = 0U;
    message.efc = 0U;
    message.mm = 0U;
    for (uint8_t i = 0U; i < frame->length; i++) {
        message.data[i] = frame->data[i];
    }

    DL_MCAN_writeMsgRam(context->config->mcan,
                        DL_MCAN_MEM_TYPE_BUF,
                        context->config->tx_buffer,
                        &message);
    if (DL_MCAN_TXBufAddReq(
            context->config->mcan, context->config->tx_buffer) != 0) {
        return DRIVER_ERROR;
    }
    context->tx_request_count++;
    return DRIVER_OK;
}

bool Can_Read(CanContext *context, CanFrame *frame)
{
    if ((!Can_IsReady(context)) || (frame == 0) ||
        (context->rx_head == context->rx_tail)) {
        return false;
    }
    *frame = context->config->rx_queue[context->rx_tail];
    context->rx_tail = NextRxIndex(context->config, context->rx_tail);
    return true;
}

uint16_t Can_GetRxAvailable(const CanContext *context)
{
    if (!Can_IsReady(context)) {
        return 0U;
    }
    return RxAvailable(context);
}

DriverStatus Can_GetStatus(const CanContext *context, CanStatus *status)
{
    if (!Can_IsReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (status == 0) {
        return DRIVER_ERROR_INVALID_ARG;
    }

    DL_MCAN_ErrCntStatus counters = {};
    DL_MCAN_ProtocolStatus protocol = {};
    DL_MCAN_getErrCounters(context->config->mcan, &counters);
    DL_MCAN_getProtocolStatus(context->config->mcan, &protocol);

    status->initialized = context->initialized;
    status->mode = context->mode;
    status->bitrate = context->config->bitrate;
    status->rx_available = RxAvailable(context);
    status->rx_count = context->rx_count;
    status->rx_dropped_count = context->rx_dropped_count;
    status->rx_fifo_lost_count = context->rx_fifo_lost_count;
    status->tx_request_count = context->tx_request_count;
    status->tx_complete_count = context->tx_complete_count;
    status->tx_cancel_count = context->tx_cancel_count;
    status->error_event_count = context->error_event_count;
    status->last_interrupt_status = context->last_interrupt_status;
    status->tx_error_count = counters.transErrLogCnt;
    status->rx_error_count = counters.recErrCnt;
    status->last_error_code = protocol.lastErrCode;
    status->error_passive = (protocol.errPassive != 0U);
    status->warning = (protocol.warningStatus != 0U);
    status->bus_off = (protocol.busOffStatus != 0U);
    status->tx_pending =
        (DL_MCAN_getTxBufReqPend(context->config->mcan) &
         (1UL << context->config->tx_buffer)) != 0U;
    return DRIVER_OK;
}

DriverStatus Can_Clear(CanContext *context)
{
    if (!Can_IsReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    NVIC_DisableIRQ(context->config->irq);
    ResetSoftwareState(context);
    NVIC_ClearPendingIRQ(context->config->irq);
    NVIC_EnableIRQ(context->config->irq);
    return DRIVER_OK;
}

DriverStatus Can_CancelTx(CanContext *context)
{
    if (!Can_IsReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (DL_MCAN_txBufCancellationReq(
            context->config->mcan, context->config->tx_buffer) != 0) {
        return DRIVER_ERROR;
    }
    return DRIVER_OK;
}

DriverStatus Can_Recover(CanContext *context)
{
    if (!Can_IsReady(context)) {
        return DRIVER_ERROR_NOT_INITIALIZED;
    }
    if (context->mode != CAN_TRANSCEIVER_NORMAL) {
        return DRIVER_ERROR_BUSY;
    }

    DL_MCAN_setOpMode(
        context->config->mcan, DL_MCAN_OPERATION_MODE_SW_INIT);
    DL_MCAN_setOpMode(
        context->config->mcan, DL_MCAN_OPERATION_MODE_NORMAL);
    return DRIVER_OK;
}

void Can_IrqHandler(CanContext *context)
{
    if (!Can_IsReady(context)) {
        return;
    }
    if (DL_MCAN_getPendingInterrupt(context->config->mcan) !=
        DL_MCAN_IIDX_LINE1) {
        return;
    }

    const uint32_t interrupt_status =
        DL_MCAN_getIntrStatus(context->config->mcan);
    DL_MCAN_clearIntrStatus(context->config->mcan,
                            interrupt_status,
                            DL_MCAN_INTR_SRC_MCAN_LINE_1);
    context->last_interrupt_status = interrupt_status;

    if ((interrupt_status & DL_MCAN_INTR_SRC_TRANS_COMPLETE) != 0U) {
        context->tx_complete_count++;
    }
    if ((interrupt_status & DL_MCAN_INTR_SRC_TRANS_CANCEL_FINISH) != 0U) {
        context->tx_cancel_count++;
    }
    if ((interrupt_status & DL_MCAN_INTR_SRC_RX_FIFO0_MSG_LOST) != 0U) {
        context->rx_fifo_lost_count++;
    }
    if ((interrupt_status & kCanErrorEvents) != 0U) {
        context->error_event_count++;
    }

    DL_MCAN_RxFIFOStatus fifo_status = {};
    fifo_status.num = DL_MCAN_RX_FIFO_NUM_0;
    for (;;) {
        DL_MCAN_getRxFIFOStatus(context->config->mcan, &fifo_status);
        if (fifo_status.fillLvl == 0U) {
            break;
        }

        DL_MCAN_RxBufElement message = {};
        DL_MCAN_readMsgRam(context->config->mcan,
                           DL_MCAN_MEM_TYPE_FIFO,
                           0U,
                           fifo_status.num,
                           &message);
        (void) DL_MCAN_writeRxFIFOAck(context->config->mcan,
                                      fifo_status.num,
                                      fifo_status.getIdx);
        if ((message.rtr == 0U) && (message.fdf == 0U)) {
            PushRxFromIsr(context, &message);
        }
    }
}

} /* namespace drivers */
