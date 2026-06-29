#ifndef BOARD_BOARD_PINS_H_
#define BOARD_BOARD_PINS_H_

#include "ti_msp_dl_config.h"

/*
 * Central pin ownership table for the custom development board.
 *
 * Rules:
 * 1. Reserve debug/programming pins here.
 * 2. Add board-level aliases for new hardware here.
 * 3. Let SysConfig generate the final peripheral pin mux code.
 * 4. Do not hard-code board pins inside driver modules.
 */

#define BOARD_PIN_SWDIO_PORT            GPIOA
#define BOARD_PIN_SWDIO_PIN             DL_GPIO_PIN_19

#define BOARD_PIN_SWCLK_PORT            GPIOA
#define BOARD_PIN_SWCLK_PIN             DL_GPIO_PIN_20

#endif /* BOARD_BOARD_PINS_H_ */