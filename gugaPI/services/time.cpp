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

uint32_t Time_Micros(void)
{
    uint32_t millis_before = 0U;
    uint32_t millis_after = 0U;
    uint32_t systick_value = 0U;

    do {
        millis_before = g_millis;
        systick_value = SysTick->VAL;
        millis_after = g_millis;
    } while (millis_before != millis_after);

    const uint32_t ticks_per_us = CPUCLK_FREQ / 1000000U;
    const uint32_t reload_ticks = SysTick->LOAD + 1U;
    const uint32_t elapsed_ticks = reload_ticks - systick_value;
    const uint32_t elapsed_us =
        (ticks_per_us == 0U) ? 0U : (elapsed_ticks / ticks_per_us);

    return (millis_before * 1000U) + elapsed_us;
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
