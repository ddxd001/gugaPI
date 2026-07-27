#include "board/board_can.h"

#include "board/board_pins.h"
#include "config/feature_config.h"

namespace board {
namespace {

#if FEATURE_ENABLE_CAN
drivers::CanFrame g_rxQueue[BOARD_CAN_RX_QUEUE_SIZE];
drivers::CanContext g_context;

const drivers::CanConfig g_config = {
    BOARD_CAN_INST,
    BOARD_CAN_IRQN,
    BOARD_CAN_STB_PORT,
    BOARD_CAN_STB_PIN,
    BOARD_CAN_BITRATE,
    BOARD_CAN_TX_BUFFER,
    BOARD_CAN_IRQ_PRIORITY,
    g_rxQueue,
    BOARD_CAN_RX_QUEUE_SIZE
};
#endif

} /* namespace */

drivers::DriverStatus Board_CanInit(void)
{
#if FEATURE_ENABLE_CAN
    return drivers::Can_Init(&g_context, &g_config);
#else
    return drivers::DRIVER_ERROR_UNSUPPORTED;
#endif
}

bool Board_CanIsReady(void)
{
#if FEATURE_ENABLE_CAN
    return drivers::Can_IsReady(&g_context);
#else
    return false;
#endif
}

drivers::DriverStatus Board_CanSetMode(
    drivers::CanTransceiverMode mode)
{
#if FEATURE_ENABLE_CAN
    return drivers::Can_SetMode(&g_context, mode);
#else
    (void) mode;
    return drivers::DRIVER_ERROR_UNSUPPORTED;
#endif
}

drivers::DriverStatus Board_CanSend(const drivers::CanFrame *frame)
{
#if FEATURE_ENABLE_CAN
    return drivers::Can_Send(&g_context, frame);
#else
    (void) frame;
    return drivers::DRIVER_ERROR_UNSUPPORTED;
#endif
}

bool Board_CanRead(drivers::CanFrame *frame)
{
#if FEATURE_ENABLE_CAN
    return drivers::Can_Read(&g_context, frame);
#else
    (void) frame;
    return false;
#endif
}

uint16_t Board_CanRxAvailable(void)
{
#if FEATURE_ENABLE_CAN
    return drivers::Can_GetRxAvailable(&g_context);
#else
    return 0U;
#endif
}

drivers::DriverStatus Board_CanGetStatus(drivers::CanStatus *status)
{
#if FEATURE_ENABLE_CAN
    return drivers::Can_GetStatus(&g_context, status);
#else
    (void) status;
    return drivers::DRIVER_ERROR_UNSUPPORTED;
#endif
}

drivers::DriverStatus Board_CanClear(void)
{
#if FEATURE_ENABLE_CAN
    return drivers::Can_Clear(&g_context);
#else
    return drivers::DRIVER_ERROR_UNSUPPORTED;
#endif
}

drivers::DriverStatus Board_CanCancelTx(void)
{
#if FEATURE_ENABLE_CAN
    return drivers::Can_CancelTx(&g_context);
#else
    return drivers::DRIVER_ERROR_UNSUPPORTED;
#endif
}

drivers::DriverStatus Board_CanRecover(void)
{
#if FEATURE_ENABLE_CAN
    return drivers::Can_Recover(&g_context);
#else
    return drivers::DRIVER_ERROR_UNSUPPORTED;
#endif
}

void Board_CanIrqHandler(void)
{
#if FEATURE_ENABLE_CAN
    drivers::Can_IrqHandler(&g_context);
#endif
}

} /* namespace board */

#if FEATURE_ENABLE_CAN
extern "C" void CAN_BUS_INST_IRQHandler(void)
{
    board::Board_CanIrqHandler();
}
#endif
