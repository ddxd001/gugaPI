#ifndef DRIVERS_UART_CONTROL_H_
#define DRIVERS_UART_CONTROL_H_

#include <stdint.h>

#include "protocol/motor_protocol.h"

void UartControl_init(MotorRegisterMap* registers);
void UartControl_process(void);
bool UartControl_consumeWriteActivity(void);

#endif  // DRIVERS_UART_CONTROL_H_
