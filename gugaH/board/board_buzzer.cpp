#include "board/board_buzzer.h"

#include "board/board_pins.h"
#include "drivers/buzzer/buzzer.h"

namespace board {

static const drivers::BuzzerConfig g_buzzerConfig = {
    BOARD_BUZZER_PORT,
    BOARD_BUZZER_PIN,
    (BOARD_BUZZER_ACTIVE_HIGH != 0U),
    (BOARD_BUZZER_INITIAL_ON != 0U)
};

static drivers::BuzzerContext g_buzzerContext = {
    0,
    false,
    false
};

drivers::DriverStatus Board_BuzzerInit(void)
{
    return drivers::Buzzer_Init(&g_buzzerContext, &g_buzzerConfig);
}

drivers::DriverStatus Board_BuzzerOn(void)
{
    return drivers::Buzzer_On(&g_buzzerContext);
}

drivers::DriverStatus Board_BuzzerOff(void)
{
    return drivers::Buzzer_Off(&g_buzzerContext);
}

drivers::DriverStatus Board_BuzzerToggle(void)
{
    return drivers::Buzzer_Toggle(&g_buzzerContext);
}

bool Board_BuzzerIsReady(void)
{
    return drivers::Buzzer_IsReady(&g_buzzerContext);
}

bool Board_BuzzerIsOn(void)
{
    return drivers::Buzzer_IsOn(&g_buzzerContext);
}

} /* namespace board */