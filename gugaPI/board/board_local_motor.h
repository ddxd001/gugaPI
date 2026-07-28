#ifndef BOARD_BOARD_LOCAL_MOTOR_H_
#define BOARD_BOARD_LOCAL_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"

namespace board {

enum LocalMotorWheel : uint8_t {
    LOCAL_MOTOR_LEFT = 0U,
    LOCAL_MOTOR_RIGHT = 1U
};

/* This MR-only build enables the complete drive + hardware-QEI path only for
 * LOCAL_MOTOR_RIGHT. Left-wheel operations return DRIVER_ERROR_UNSUPPORTED. */

struct LocalMotorEncoderSnapshot {
    int32_t count;
    int32_t counts_per_second;
    uint8_t state;
};

drivers::DriverStatus Board_LocalMotorInit(void);
drivers::DriverStatus Board_LocalMotorProcessEncoders(uint32_t now_ms);
drivers::DriverStatus Board_LocalMotorGetEncoder(
    LocalMotorWheel wheel,
    LocalMotorEncoderSnapshot *snapshot);
drivers::DriverStatus Board_LocalMotorRun(LocalMotorWheel wheel,
                                          bool reverse,
                                          uint16_t duty_q8,
                                          uint32_t now_ms);
drivers::DriverStatus Board_LocalMotorSleep(LocalMotorWheel wheel);
drivers::DriverStatus Board_LocalMotorSleepAll(void);
bool Board_LocalMotorIsReady(void);
bool Board_LocalMotorIsAwake(LocalMotorWheel wheel);

} /* namespace board */

#endif /* BOARD_BOARD_LOCAL_MOTOR_H_ */
