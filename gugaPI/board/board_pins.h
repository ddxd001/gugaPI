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

#define BOARD_STATUS_LED_PORT           GPIO_LEDS_PORT
#define BOARD_STATUS_LED_PIN            GPIO_LEDS_STATUS_LED_PIN
#define BOARD_STATUS_LED_ACTIVE_LOW     (1U)
#define BOARD_STATUS_LED_INITIAL_ON     (0U)

#define BOARD_BUZZER_PORT               GPIO_BUZZER_PORT
#define BOARD_BUZZER_PIN                GPIO_BUZZER_BUZZER_PIN
#define BOARD_BUZZER_ACTIVE_HIGH        (1U)
#define BOARD_BUZZER_INITIAL_ON         (0U)

#define BOARD_FRAM_I2C_INST             FRAM_I2C_INST
#define BOARD_FRAM_I2C_SCL_PORT         GPIO_FRAM_I2C_SCL_PORT
#define BOARD_FRAM_I2C_SCL_PIN          GPIO_FRAM_I2C_SCL_PIN
#define BOARD_FRAM_I2C_SCL_IOMUX        GPIO_FRAM_I2C_IOMUX_SCL
#define BOARD_FRAM_I2C_SCL_IOMUX_FUNC   GPIO_FRAM_I2C_IOMUX_SCL_FUNC
#define BOARD_FRAM_I2C_SDA_PORT         GPIO_FRAM_I2C_SDA_PORT
#define BOARD_FRAM_I2C_SDA_PIN          GPIO_FRAM_I2C_SDA_PIN
#define BOARD_FRAM_I2C_SDA_IOMUX        GPIO_FRAM_I2C_IOMUX_SDA
#define BOARD_FRAM_I2C_SDA_IOMUX_FUNC   GPIO_FRAM_I2C_IOMUX_SDA_FUNC
#define BOARD_FRAM_I2C_ADDRESS          (0x50U)
#define BOARD_FRAM_TIMEOUT_ITERATIONS   (100000U)
#define BOARD_FRAM_SELF_TEST_ADDRESS    (0x1FF0U)

#endif /* BOARD_BOARD_PINS_H_ */