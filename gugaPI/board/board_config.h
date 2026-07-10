#ifndef BOARD_BOARD_CONFIG_H_
#define BOARD_BOARD_CONFIG_H_

/*
 * Board-wide constants. Pin ownership stays in board_pins.h, while device
 * specific behavior belongs in each driver.
 */

#define BOARD_NAME                      "gugaPI"
#define BOARD_MCU_NAME                  "MSPM0G3519"

#define BOARD_SCHEDULER_MAX_TASKS       (16U)
#define BOARD_DEFAULT_LOG_BAUDRATE      (115200U)

#endif /* BOARD_BOARD_CONFIG_H_ */
