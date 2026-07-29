/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"

#include "app/app_main.h"
#include "app/app_shell.h"
#include "board/board.h"
#include "board/board_buzzer.h"
#include "board/board_led.h"
#include "board/board_oled.h"
#include "config/feature_config.h"
#include "drivers/common/driver_status.h"
#include "services/debug_uart.h"
#include "services/fault.h"
#include "services/log.h"
#include "services/scheduler.h"
#include "services/shell.h"
#include "services/time.h"

#if FEATURE_PROFILE_COMPETITION
namespace {

constexpr bool kUsesSensorI2c =
    (FEATURE_ENABLE_FRAM != 0U) ||
    (FEATURE_ENABLE_INA219 != 0U) ||
    (FEATURE_ENABLE_OLED != 0U);

void CompetitionSyscfgInitPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_GPIO_reset(GPIOC);
#if FEATURE_ENABLE_FRAM || FEATURE_ENABLE_INA219 || FEATURE_ENABLE_OLED
    DL_I2C_reset(SENSOR_I2C_INST);
#endif
#if FEATURE_ENABLE_MOTOR_DRIVER
    DL_I2C_reset(MOTOR_I2C_INST);
    DL_UART_Main_reset(MOTOR_UART_INST);
#endif
#if FEATURE_ENABLE_LORA
    DL_UART_Main_reset(LORA_UART_INST);
#endif
#if FEATURE_ENABLE_IMU
    DL_SPI_reset(IMU_SPI_INST);
#endif
#if FEATURE_ENABLE_DEBUG_UART
    DL_UART_Main_reset(DEBUG_UART_INST);
#endif
#if FEATURE_ENABLE_GRAYSCALE
    DL_ADC12_reset(GRAYSCALE_ADC_INST);
#endif
#if FEATURE_ENABLE_CAN
    DL_MCAN_reset(CAN_BUS_INST);
#endif

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_GPIO_enablePower(GPIOC);
#if FEATURE_ENABLE_FRAM || FEATURE_ENABLE_INA219 || FEATURE_ENABLE_OLED
    DL_I2C_enablePower(SENSOR_I2C_INST);
#endif
#if FEATURE_ENABLE_MOTOR_DRIVER
    DL_I2C_enablePower(MOTOR_I2C_INST);
    DL_UART_Main_enablePower(MOTOR_UART_INST);
#endif
#if FEATURE_ENABLE_LORA
    DL_UART_Main_enablePower(LORA_UART_INST);
#endif
#if FEATURE_ENABLE_IMU
    DL_SPI_enablePower(IMU_SPI_INST);
#endif
#if FEATURE_ENABLE_DEBUG_UART
    DL_UART_Main_enablePower(DEBUG_UART_INST);
#endif
#if FEATURE_ENABLE_GRAYSCALE
    DL_ADC12_enablePower(GRAYSCALE_ADC_INST);
#endif
#if FEATURE_ENABLE_CAN
    DL_MCAN_enablePower(CAN_BUS_INST);
#endif
    /* Preserve the generated 16-cycle startup guard used at the original
     * 40 MHz CPU clock (400 ns). */
    services::Time_DelayNs(400U);
}

void CompetitionSyscfgInitGpio(void)
{
    if (kUsesSensorI2c) {
        DL_GPIO_initPeripheralInputFunctionFeatures(
            GPIO_SENSOR_I2C_IOMUX_SDA,
            GPIO_SENSOR_I2C_IOMUX_SDA_FUNC,
            DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR_NONE,
            DL_GPIO_HYSTERESIS_DISABLE,
            DL_GPIO_WAKEUP_DISABLE);
        DL_GPIO_initPeripheralInputFunctionFeatures(
            GPIO_SENSOR_I2C_IOMUX_SCL,
            GPIO_SENSOR_I2C_IOMUX_SCL_FUNC,
            DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR_NONE,
            DL_GPIO_HYSTERESIS_DISABLE,
            DL_GPIO_WAKEUP_DISABLE);
        DL_GPIO_enableHiZ(GPIO_SENSOR_I2C_IOMUX_SDA);
        DL_GPIO_enableHiZ(GPIO_SENSOR_I2C_IOMUX_SCL);
    }

#if FEATURE_ENABLE_MOTOR_DRIVER
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_MOTOR_I2C_IOMUX_SDA,
        GPIO_MOTOR_I2C_IOMUX_SDA_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_MOTOR_I2C_IOMUX_SCL,
        GPIO_MOTOR_I2C_IOMUX_SCL_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_MOTOR_I2C_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_MOTOR_I2C_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(GPIO_MOTOR_UART_IOMUX_TX,
                                         GPIO_MOTOR_UART_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_MOTOR_UART_IOMUX_RX,
                                        GPIO_MOTOR_UART_IOMUX_RX_FUNC);
#endif

#if FEATURE_ENABLE_DEBUG_UART
    DL_GPIO_initPeripheralOutputFunction(GPIO_DEBUG_UART_IOMUX_TX,
                                         GPIO_DEBUG_UART_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_DEBUG_UART_IOMUX_RX,
                                        GPIO_DEBUG_UART_IOMUX_RX_FUNC);
#endif

#if FEATURE_ENABLE_LORA
    DL_GPIO_initPeripheralOutputFunction(GPIO_LORA_UART_IOMUX_TX,
                                         GPIO_LORA_UART_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_LORA_UART_IOMUX_RX,
                                        GPIO_LORA_UART_IOMUX_RX_FUNC);
#endif

#if FEATURE_ENABLE_IMU
    DL_GPIO_initPeripheralOutputFunction(GPIO_IMU_SPI_IOMUX_SCLK,
                                         GPIO_IMU_SPI_IOMUX_SCLK_FUNC);
    DL_GPIO_initPeripheralOutputFunction(GPIO_IMU_SPI_IOMUX_PICO,
                                         GPIO_IMU_SPI_IOMUX_PICO_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_IMU_SPI_IOMUX_POCI,
                                        GPIO_IMU_SPI_IOMUX_POCI_FUNC);

    DL_GPIO_initDigitalInputFeatures(GPIO_IMU_C_ICM45686_INT1_IOMUX,
                                     DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_NONE,
                                     DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_IMU_A_LIS3MDL_DRDY_IOMUX,
                                     DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_NONE,
                                     DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        GPIO_IMU_A_ICM45686_INT2_FSYNC_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalOutput(GPIO_IMU_C_ICM45686_CS_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_IMU_C_LIS3MDL_CS_IOMUX);
    DL_GPIO_setPins(GPIOC,
                    GPIO_IMU_C_ICM45686_CS_PIN |
                        GPIO_IMU_C_LIS3MDL_CS_PIN);
    DL_GPIO_enableOutput(GPIOC,
                         GPIO_IMU_C_ICM45686_CS_PIN |
                             GPIO_IMU_C_LIS3MDL_CS_PIN);
#endif

#if FEATURE_ENABLE_STATUS_LED
    DL_GPIO_initDigitalOutput(GPIO_LED_A_LED1_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_LED_A_LED2_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_LED_B_LED3_IOMUX);
    DL_GPIO_setPins(GPIOA, GPIO_LED_A_LED1_PIN | GPIO_LED_A_LED2_PIN);
    DL_GPIO_setPins(GPIOB, GPIO_LED_B_LED3_PIN);
    DL_GPIO_enableOutput(GPIOA,
                         GPIO_LED_A_LED1_PIN | GPIO_LED_A_LED2_PIN);
    DL_GPIO_enableOutput(GPIOB, GPIO_LED_B_LED3_PIN);
#endif

#if FEATURE_ENABLE_BUZZER
    DL_GPIO_initDigitalOutput(GPIO_BUZZER_BUZZER_IOMUX);
    DL_GPIO_clearPins(GPIOC, GPIO_BUZZER_BUZZER_PIN);
    DL_GPIO_enableOutput(GPIOC, GPIO_BUZZER_BUZZER_PIN);
#endif

#if FEATURE_ENABLE_BUTTONS
    DL_GPIO_initDigitalInputFeatures(GPIO_BUTTON_C_BUTTON1_IOMUX,
                                     DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_BUTTON_B_BUTTON2_IOMUX,
                                     DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_BUTTON_B_BUTTON3_IOMUX,
                                     DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
#endif

#if FEATURE_ENABLE_GY931
    DL_GPIO_initDigitalInputFeatures(GPIO_GY931_I2C_GY931_SCL_IOMUX,
                                     DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_GY931_I2C_GY931_SDA_IOMUX,
                                     DL_GPIO_INVERSION_DISABLE,
                                     DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_DISABLE,
                                     DL_GPIO_WAKEUP_DISABLE);
#endif

#if FEATURE_ENABLE_GRAYSCALE
    DL_GPIO_initDigitalOutput(GPIO_GRAY_C_GRAY_SEL0_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GRAY_C_GRAY_SEL1_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_GRAY_A_GRAY_SEL2_IOMUX);
    DL_GPIO_clearPins(GPIOA, GPIO_GRAY_A_GRAY_SEL2_PIN);
    DL_GPIO_enableOutput(GPIOA, GPIO_GRAY_A_GRAY_SEL2_PIN);
    DL_GPIO_clearPins(GPIOC,
                      GPIO_GRAY_C_GRAY_SEL0_PIN |
                          GPIO_GRAY_C_GRAY_SEL1_PIN);
    DL_GPIO_enableOutput(GPIOC,
                         GPIO_GRAY_C_GRAY_SEL0_PIN |
                             GPIO_GRAY_C_GRAY_SEL1_PIN);
#endif

#if FEATURE_ENABLE_CAN
    DL_GPIO_initPeripheralOutputFunction(GPIO_CAN_BUS_IOMUX_CAN_TX,
                                         GPIO_CAN_BUS_IOMUX_CAN_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_CAN_BUS_IOMUX_CAN_RX,
                                        GPIO_CAN_BUS_IOMUX_CAN_RX_FUNC);
    DL_GPIO_initDigitalOutput(GPIO_CAN_CTRL_CAN_STB_IOMUX);
    DL_GPIO_clearPins(GPIO_CAN_CTRL_PORT, GPIO_CAN_CTRL_CAN_STB_PIN);
    DL_GPIO_enableOutput(GPIO_CAN_CTRL_PORT, GPIO_CAN_CTRL_CAN_STB_PIN);
#endif
}

} /* namespace */

extern "C" void SYSCFG_DL_init(void)
{
    CompetitionSyscfgInitPower();
    CompetitionSyscfgInitGpio();
    SYSCFG_DL_SYSCTL_init();
#if FEATURE_ENABLE_FRAM || FEATURE_ENABLE_INA219 || FEATURE_ENABLE_OLED
    SYSCFG_DL_SENSOR_I2C_init();
#endif
#if FEATURE_ENABLE_MOTOR_DRIVER
    SYSCFG_DL_MOTOR_I2C_init();
#endif
#if FEATURE_ENABLE_LORA
    SYSCFG_DL_LORA_UART_init();
#endif
#if FEATURE_ENABLE_INFRARED_LINE_SENSOR
    SYSCFG_DL_INFRARED_UART_init();
#endif
#if FEATURE_ENABLE_IMU
    SYSCFG_DL_IMU_SPI_init();
#endif
#if FEATURE_ENABLE_DEBUG_UART
    SYSCFG_DL_DEBUG_UART_init();
#endif
#if FEATURE_ENABLE_DEBUG_UART || FEATURE_ENABLE_OLED
    SYSCFG_DL_DMA_init();
#endif
#if FEATURE_ENABLE_MOTOR_DRIVER
    SYSCFG_DL_MOTOR_UART_init();
#endif
#if FEATURE_ENABLE_GRAYSCALE
    SYSCFG_DL_GRAYSCALE_ADC_init();
#endif
#if FEATURE_ENABLE_CAN
    SYSCFG_DL_CAN_BUS_init();
#endif
    SYSCFG_DL_SYSCTL_CLK_init();
}
#endif

#if FEATURE_ENABLE_OLED
extern "C" void SENSOR_I2C_INST_IRQHandler(void)
{
    board::Board_OledHandleI2cInterrupt();
}
#endif

/* Observable panic: blink the status LED (and attempt one log line) instead
 * of a silent hang. Registered with services::Fault so the services layer does
 * not need to depend on board/app. Owns the loop; Fault_Panic hangs if this
 * ever returns. */
static void PanicHandler(services::FaultCode code)
{
#if FEATURE_ENABLE_LOG
    services::Log_Error("panic");
#endif
#if FEATURE_ENABLE_BUZZER
    if (board::Board_BuzzerIsReady()) {
        (void) board::Board_BuzzerOff();
    }
#endif
#if FEATURE_ENABLE_OLED
    if (board::Board_OledIsReady()) {
        (void) board::Board_OledClear();
        (void) board::Board_OledWriteText(0U, 0U, "SYSTEM FAULT");
        (void) board::Board_OledWriteText(
            1U,
            0U,
            (code == services::FAULT_ASSERT) ? "ASSERT" : "PANIC");
        (void) board::Board_OledWriteText(2U, 0U, "EXECUTION HALTED");
        (void) board::Board_OledWriteText(3U, 0U, "RESET TO RECOVER");
    }
#else
    (void) code;
#endif
    for (;;) {
#if FEATURE_ENABLE_OLED
        (void) board::Board_OledService();
#endif
#if FEATURE_ENABLE_STATUS_LED
        (void) board::Board_StatusLedOn();
#endif
        services::Time_DelayMsBusy(200U);
#if FEATURE_ENABLE_STATUS_LED
        (void) board::Board_StatusLedOff();
#endif
        services::Time_DelayMsBusy(200U);
    }
}

int main(void)
{
    SYSCFG_DL_init();

    services::Fault_Init();
    services::Scheduler_Init();
    services::Time_Init();
    services::DebugUart_Init();
    services::Log_Init();
    services::Fault_SetPanicHandler(&PanicHandler);

    /* Retry the complete checked initialization once because some I2C/SPI
     * devices need additional power-up time. The final report still records
     * every enabled peripheral and distinguishes degraded failures from
     * failures that must inhibit motion. */
    if (board::Board_Init() != drivers::DRIVER_OK) {
        LOG_WARN("board init incomplete; retrying");
        services::Time_DelayMs(100U);
        if (board::Board_Init() != drivers::DRIVER_OK) {
            const board::BoardInitReport *report =
                board::Board_GetInitReport();
            const bool motion_inhibited =
                (report != 0) && report->motion_inhibited;
            if (motion_inhibited) {
                LOG_ERROR("board init failed after retry; motion inhibited");
            } else {
                LOG_WARN("board init degraded after retry");
            }
            if (report != 0) {
                for (uint8_t i = 0U; i < report->count; i++) {
                    if (report->entries[i].status != drivers::DRIVER_OK) {
                        services::DebugUart_WriteString("  failed: ");
                        services::DebugUart_WriteString(
                            report->entries[i].name);
                        services::DebugUart_WriteString("\r\n");
                    }
                }
            }
            if (motion_inhibited) {
                services::Fault_Set(services::FAULT_DRIVER_INIT);
            }
        } else {
            LOG_WARN("board init recovered after retry");
        }
    }

    services::Shell_Init();
    app::AppShell_RegisterCommands();
    app::App_Init();
    board::Board_LateInit();

    while (1) {
        services::Scheduler_Run();
        app::App_Run();
        services::Shell_Process();
        services::DebugUart_TxPump();
    }
}
