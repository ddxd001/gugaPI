#include "board/board_can.h"

#include "board/board_pins.h"

namespace board {
namespace {

drivers::CanFrame g_rx_queue[BOARD_CAN_RX_QUEUE_SIZE];
drivers::CanContext g_context = {};

const drivers::CanConfig g_config = {
    BOARD_CAN_INST,
    BOARD_CAN_IRQN,
    BOARD_CAN_STB_PORT,
    BOARD_CAN_STB_PIN,
    BOARD_CAN_BITRATE,
    BOARD_CAN_TX_BUFFER,
    BOARD_CAN_IRQ_PRIORITY,
    g_rx_queue,
    BOARD_CAN_RX_QUEUE_SIZE
};

} /* namespace */

drivers::DriverStatus Board_CanInit(void)
{
    return drivers::Can_Init(&g_context, &g_config);
}

bool Board_CanIsReady(void)
{
    return drivers::Can_IsReady(&g_context);
}

drivers::DriverStatus Board_CanSend(const drivers::CanFrame *frame)
{
    return drivers::Can_Send(&g_context, frame);
}

bool Board_CanRead(drivers::CanFrame *frame)
{
    return drivers::Can_Read(&g_context, frame);
}

drivers::DriverStatus Board_CanGetStatus(drivers::CanStatus *status)
{
    return drivers::Can_GetStatus(&g_context, status);
}

drivers::DriverStatus Board_CanRecover(void)
{
    return drivers::Can_Recover(&g_context);
}

void Board_CanIrqHandler(void)
{
    drivers::Can_IrqHandler(&g_context);
}

} /* namespace board */

extern "C" void CAN_BUS_INST_IRQHandler(void)
{
    board::Board_CanIrqHandler();
}
