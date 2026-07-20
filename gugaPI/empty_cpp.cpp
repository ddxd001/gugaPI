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
#include "config/feature_config.h"
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
#if FEATURE_ENABLE_DEBUG_UART
    DL_UART_Main_reset(DEBUG_UART_INST);
#endif
#if FEATURE_ENABLE_GRAYSCALE
    DL_ADC12_reset(GRAYSCALE_ADC_INST);
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
#if FEATURE_ENABLE_DEBUG_UART
    DL_UART_Main_enablePower(DEBUG_UART_INST);
#endif
#if FEATURE_ENABLE_GRAYSCALE
    DL_ADC12_enablePower(GRAYSCALE_ADC_INST);
#endif
    delay_cycles(POWER_STARTUP_DELAY);
}

void CompetitionSyscfgInitGpio(void)
{
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXOUT_IOMUX);

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
#if FEATURE_ENABLE_DEBUG_UART
    SYSCFG_DL_DEBUG_UART_init();
#endif
#if FEATURE_ENABLE_MOTOR_DRIVER
    SYSCFG_DL_MOTOR_UART_init();
#endif
#if FEATURE_ENABLE_GRAYSCALE
    SYSCFG_DL_GRAYSCALE_ADC_init();
#endif
    SYSCFG_DL_SYSCTL_CLK_init();
}
#endif

int main(void)
{
    SYSCFG_DL_init();

    services::Fault_Init();
    services::Scheduler_Init();
    services::Time_Init();
    board::Board_Init();
    services::DebugUart_Init();
    services::Log_Init();
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
