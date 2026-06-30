#include "board/board.h"

#include "board/board_button.h"

#include "board/board_buzzer.h"
#include "board/board_config.h"
#include "board/board_fram.h"
#include "board/board_led.h"
#include "board/board_pins.h"
#include "config/feature_config.h"

namespace board {

void Board_Init(void)
{
    /*
     * SysConfig already performed low-level clock, power, and pin mux setup.
     * Put custom board bring-up here: power rails, enables, default GPIO
     * states, and board-level self checks.
     */
#if FEATURE_ENABLE_STATUS_LED
    (void) Board_StatusLedInit();
#endif

#if FEATURE_ENABLE_BUZZER
    (void) Board_BuzzerInit();
#endif

#if FEATURE_ENABLE_BUTTONS
    (void) Board_ButtonsInit();
#endif

#if FEATURE_ENABLE_FRAM
    (void) Board_FramInit();
#endif
}

void Board_LateInit(void)
{
    /*
     * Use this hook after all drivers are initialized, for devices that need
     * a known system state before being enabled.
     */
}

} /* namespace board */