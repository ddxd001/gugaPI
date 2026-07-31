#include "app/h_runtime.h"
#include "app/h_shell.h"
#include "services/debug_uart.h"
#include "services/shell.h"
#include "services/time.h"
#include "ti_msp_dl_config.h"

int main(void)
{
    SYSCFG_DL_init();
    services::Time_Init();
    services::DebugUart_Init();
    services::Shell_Init();
    gugah::HRuntime_Init();
    gugah::HShell_Init();

    uint32_t last_1ms = services::Time_Millis();
    uint32_t last_2ms = last_1ms;
    uint32_t last_5ms = last_1ms;
    uint32_t last_50ms = last_1ms;

    while (true) {
        const uint32_t now_ms = services::Time_Millis();
        if ((now_ms - last_1ms) >= 1U) {
            last_1ms = now_ms;
            gugah::HRuntime_Update1ms(now_ms);
        }
        if ((now_ms - last_2ms) >= 2U) {
            last_2ms = now_ms;
            gugah::HRuntime_Update2ms(now_ms);
        }
        if ((now_ms - last_5ms) >= 5U) {
            last_5ms = now_ms;
            gugah::HRuntime_Update5ms(now_ms);
        }
        if ((now_ms - last_50ms) >= 50U) {
            last_50ms = now_ms;
            gugah::HRuntime_Update50ms(now_ms);
        }
        gugah::HRuntime_Service();
        __WFI();
    }
}
