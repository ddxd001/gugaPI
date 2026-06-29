#include "board/board.h"

#include "board/board_config.h"
#include "board/board_pins.h"

namespace board {

void Board_Init(void)
{
    /*
     * SysConfig already performed low-level clock, power, and pin mux setup.
     * Put custom board bring-up here: power rails, enables, default GPIO
     * states, and board-level self checks.
     */
}

void Board_LateInit(void)
{
    /*
     * Use this hook after all drivers are initialized, for devices that need
     * a known system state before being enabled.
     */
}

} /* namespace board */