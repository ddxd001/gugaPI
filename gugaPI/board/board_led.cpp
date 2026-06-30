#include "board/board_led.h"

#include "board/board_pins.h"
#include "drivers/led/led.h"

namespace board {

static const drivers::LedConfig g_statusLedConfig = {
    BOARD_STATUS_LED_PORT,
    BOARD_STATUS_LED_PIN,
    (BOARD_STATUS_LED_ACTIVE_LOW != 0U),
    (BOARD_STATUS_LED_INITIAL_ON != 0U)
};

static drivers::LedContext g_statusLedContext = {
    0,
    false,
    false
};

drivers::DriverStatus Board_StatusLedInit(void)
{
    return drivers::Led_Init(&g_statusLedContext, &g_statusLedConfig);
}

drivers::DriverStatus Board_StatusLedOn(void)
{
    return drivers::Led_On(&g_statusLedContext);
}

drivers::DriverStatus Board_StatusLedOff(void)
{
    return drivers::Led_Off(&g_statusLedContext);
}

drivers::DriverStatus Board_StatusLedToggle(void)
{
    return drivers::Led_Toggle(&g_statusLedContext);
}

bool Board_StatusLedIsReady(void)
{
    return drivers::Led_IsReady(&g_statusLedContext);
}

bool Board_StatusLedIsOn(void)
{
    return drivers::Led_IsOn(&g_statusLedContext);
}

} /* namespace board */