#ifndef DRIVERS_DRV8876_DRV8876_H_
#define DRIVERS_DRV8876_DRV8876_H_

#include <stdbool.h>
#include <stdint.h>

#include "drivers/common/driver_status.h"
#include "ti_msp_dl_config.h"

namespace drivers {

static const uint16_t DRV8876_DUTY_Q8_SCALE = 256U;

enum Drv8876PhaseLevel : uint8_t {
    DRV8876_PHASE_LOW = 0U,
    DRV8876_PHASE_HIGH = 1U
};

struct Drv8876Config {
    GPTIMER_Regs *pwm_timer;
    DL_TIMER_CC_INDEX pwm_index;
    GPIO_Regs *phase_port;
    uint32_t phase_pin;
    GPIO_Regs *sleep_port;
    uint32_t sleep_pin;
    uint16_t pwm_period_counts;
    uint16_t wake_delay_ms;
};

struct Drv8876Context {
    const Drv8876Config *config;
    uint32_t wake_start_ms;
    uint32_t direction_change_start_ms;
    uint16_t duty_q8;
    Drv8876PhaseLevel phase_level;
    Drv8876PhaseLevel pending_phase_level;
    bool initialized;
    bool awake;
    bool direction_valid;
    bool direction_change_pending;
};

DriverStatus Drv8876_Init(Drv8876Context *ctx,
                          const Drv8876Config *config);
DriverStatus Drv8876_Sleep(Drv8876Context *ctx);
DriverStatus Drv8876_Brake(Drv8876Context *ctx, uint32_t now_ms);
DriverStatus Drv8876_Run(Drv8876Context *ctx,
                         Drv8876PhaseLevel phase_level,
                         uint16_t duty_q8,
                         uint32_t now_ms);
bool Drv8876_IsReady(const Drv8876Context *ctx);
bool Drv8876_IsAwake(const Drv8876Context *ctx);
uint16_t Drv8876_GetDutyQ8(const Drv8876Context *ctx);

} /* namespace drivers */

#endif /* DRIVERS_DRV8876_DRV8876_H_ */
