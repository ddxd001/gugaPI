#include "services/time.h"

#include "ti_msp_dl_config.h"

namespace services {

static volatile uint32_t g_millis = 0U;

void Time_Init(void)
{
    g_millis = 0U;
    (void) SysTick_Config(CPUCLK_FREQ / 1000U);
}

void Time_Tick1ms(void)
{
    g_millis++;
}

uint32_t Time_Millis(void)
{
    return g_millis;
}

bool Time_HasElapsed(uint32_t start_ms, uint32_t interval_ms)
{
    return ((uint32_t) (Time_Millis() - start_ms) >= interval_ms);
}

void Time_DelayMs(uint32_t delay_ms)
{
    const uint32_t start = Time_Millis();

    while (!Time_HasElapsed(start, delay_ms)) {
    }
}

} /* namespace services */

extern "C" void SysTick_Handler(void)
{
    services::Time_Tick1ms();
}