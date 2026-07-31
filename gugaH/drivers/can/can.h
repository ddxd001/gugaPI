#ifndef DRIVERS_CAN_CAN_H_
#define DRIVERS_CAN_CAN_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

enum CanTransceiverMode : uint8_t {
    CAN_TRANSCEIVER_NORMAL = 0U,
    CAN_TRANSCEIVER_STANDBY
};

struct CanFrame {
    uint32_t id;
    uint8_t length;
    bool extended;
    uint8_t data[8];
};

struct CanStatus {
    bool initialized;
    CanTransceiverMode mode;
    uint32_t bitrate;
    uint16_t rx_available;
    uint32_t rx_count;
    uint32_t rx_dropped_count;
    uint32_t rx_fifo_lost_count;
    uint32_t tx_request_count;
    uint32_t tx_complete_count;
    uint32_t tx_cancel_count;
    uint32_t error_event_count;
    uint32_t last_interrupt_status;
    uint32_t tx_error_count;
    uint32_t rx_error_count;
    uint32_t last_error_code;
    bool error_passive;
    bool warning;
    bool bus_off;
    bool tx_pending;
};

struct CanConfig {
    MCAN_Regs *mcan;
    IRQn_Type irq;
    GPIO_Regs *stb_port;
    uint32_t stb_pin;
    uint32_t bitrate;
    uint8_t tx_buffer;
    uint8_t irq_priority;
    CanFrame *rx_queue;
    uint16_t rx_queue_size;
};

struct CanContext {
    const CanConfig *config;
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint32_t rx_count;
    volatile uint32_t rx_dropped_count;
    volatile uint32_t rx_fifo_lost_count;
    volatile uint32_t tx_request_count;
    volatile uint32_t tx_complete_count;
    volatile uint32_t tx_cancel_count;
    volatile uint32_t error_event_count;
    volatile uint32_t last_interrupt_status;
    CanTransceiverMode mode;
    bool initialized;
};

DriverStatus Can_Init(CanContext *context, const CanConfig *config);
bool Can_IsReady(const CanContext *context);
DriverStatus Can_SetMode(CanContext *context, CanTransceiverMode mode);
DriverStatus Can_Send(CanContext *context, const CanFrame *frame);
bool Can_Read(CanContext *context, CanFrame *frame);
uint16_t Can_GetRxAvailable(const CanContext *context);
DriverStatus Can_GetStatus(const CanContext *context, CanStatus *status);
DriverStatus Can_Clear(CanContext *context);
DriverStatus Can_CancelTx(CanContext *context);
DriverStatus Can_Recover(CanContext *context);
void Can_IrqHandler(CanContext *context);

} /* namespace drivers */

#endif /* DRIVERS_CAN_CAN_H_ */
