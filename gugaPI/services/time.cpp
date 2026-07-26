#include "services/time.h"

#include "ti_msp_dl_config.h"

namespace services {

namespace {

void DelayCpuCycles(uint64_t cycles)
{
    /* DL_Common_delayCycles(0) wraps to its maximum delay. Split long delays
     * and never pass zero. Current callers are far below one 32-bit chunk,
     * but keeping the conversion bounded makes the public API safe. */
    while (cycles != 0ULL) {
        const uint32_t chunk = (cycles > 0xFFFFFFFFULL) ?
            0xFFFFFFFFU : static_cast<uint32_t>(cycles);
        DL_Common_delayCycles(chunk);
        cycles -= chunk;
    }
}

uint64_t CyclesForDuration(uint32_t duration,
                           uint32_t units_per_second)
{
    if (duration == 0U) {
        return 0ULL;
    }

    /* Round upward so short hardware timing requirements are never shortened
     * when CPUCLK_FREQ is not an exact multiple of the requested time unit. */
    const uint64_t numerator =
        static_cast<uint64_t>(CPUCLK_FREQ) * duration;
    return (numerator + units_per_second - 1ULL) / units_per_second;
}

} /* namespace */

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

void Time_DelayNs(uint32_t delay_ns)
{
    DelayCpuCycles(CyclesForDuration(delay_ns, 1000000000U));
}

void Time_DelayUs(uint32_t delay_us)
{
    DelayCpuCycles(CyclesForDuration(delay_us, 1000000U));
}

void Time_DelayMsBusy(uint32_t delay_ms)
{
    DelayCpuCycles(CyclesForDuration(delay_ms, 1000U));
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
