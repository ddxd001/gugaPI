#ifndef DRIVERS_MOTOR_DRIVER_MOTOR_DRIVER_UART_H_
#define DRIVERS_MOTOR_DRIVER_MOTOR_DRIVER_UART_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

struct MotorDriverUartConfig {
    UART_Regs *uart;
    IRQn_Type irq;
    uint32_t baudrate;
    uint32_t tx_timeout_iterations;
    uint8_t *rx_buffer;
    uint16_t rx_buffer_size;
};

struct MotorDriverUartContext {
    const MotorDriverUartConfig *config;
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint32_t rx_dropped_count;
    bool initialized;
};

DriverStatus MotorDriverUart_Init(MotorDriverUartContext *context,
                                  const MotorDriverUartConfig *config);
bool MotorDriverUart_IsReady(const MotorDriverUartContext *context);
DriverStatus MotorDriverUart_WriteByte(MotorDriverUartContext *context,
                                       uint8_t data);
DriverStatus MotorDriverUart_Write(MotorDriverUartContext *context,
                                   const uint8_t *data,
                                   uint16_t length);
bool MotorDriverUart_ReadByte(MotorDriverUartContext *context, uint8_t *data);
uint16_t MotorDriverUart_GetRxAvailable(const MotorDriverUartContext *context);
uint32_t MotorDriverUart_GetRxDroppedCount(const MotorDriverUartContext *context);
uint32_t MotorDriverUart_GetBaudrate(const MotorDriverUartContext *context);
DriverStatus MotorDriverUart_GetControllerStatus(
    const MotorDriverUartContext *context,
    uint32_t *status);
DriverStatus MotorDriverUart_ClearRxBuffer(MotorDriverUartContext *context);
void MotorDriverUart_IrqHandler(MotorDriverUartContext *context);

} /* namespace drivers */

#endif /* DRIVERS_MOTOR_DRIVER_MOTOR_DRIVER_UART_H_ */
