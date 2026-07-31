#ifndef BOARD_BOARD_BALL_VISION_H_
#define BOARD_BOARD_BALL_VISION_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace board {

drivers::DriverStatus Board_BallVisionInit(void);
drivers::DriverStatus Board_BallVisionWrite(const uint8_t *data,
                                            uint16_t length);
bool Board_BallVisionReadByte(uint8_t *data);
void Board_BallVisionClear(void);
bool Board_BallVisionIsReady(void);
uint32_t Board_BallVisionGetDroppedBytes(void);
uint32_t Board_BallVisionGetUartErrors(void);
uint32_t Board_BallVisionGetIrqCount(void);
void Board_BallVisionIrqHandler(void);

} /* namespace board */

#endif /* BOARD_BOARD_BALL_VISION_H_ */
