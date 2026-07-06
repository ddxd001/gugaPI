/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G351X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G351X
#define CONFIG_MSPM0G3519

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_M1_EN */
#define PWM_M1_EN_INST                                                     TIMG0
#define PWM_M1_EN_INST_IRQHandler                               TIMG0_IRQHandler
#define PWM_M1_EN_INST_INT_IRQN                                 (TIMG0_INT_IRQn)
#define PWM_M1_EN_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 1 */
#define GPIO_PWM_M1_EN_C1_PORT                                             GPIOA
#define GPIO_PWM_M1_EN_C1_PIN                                      DL_GPIO_PIN_6
#define GPIO_PWM_M1_EN_C1_IOMUX                                  (IOMUX_PINCM11)
#define GPIO_PWM_M1_EN_C1_IOMUX_FUNC                 IOMUX_PINCM11_PF_TIMG0_CCP1
#define GPIO_PWM_M1_EN_C1_IDX                                DL_TIMER_CC_1_INDEX

/* Defines for PWM_M2_EN */
#define PWM_M2_EN_INST                                                    TIMG12
#define PWM_M2_EN_INST_IRQHandler                              TIMG12_IRQHandler
#define PWM_M2_EN_INST_INT_IRQN                                (TIMG12_INT_IRQn)
#define PWM_M2_EN_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 1 */
#define GPIO_PWM_M2_EN_C1_PORT                                             GPIOA
#define GPIO_PWM_M2_EN_C1_PIN                                     DL_GPIO_PIN_25
#define GPIO_PWM_M2_EN_C1_IOMUX                                  (IOMUX_PINCM55)
#define GPIO_PWM_M2_EN_C1_IOMUX_FUNC                IOMUX_PINCM55_PF_TIMG12_CCP1
#define GPIO_PWM_M2_EN_C1_IDX                                DL_TIMER_CC_1_INDEX




/* Defines for I2C_CONTROL */
#define I2C_CONTROL_INST                                                    I2C1
#define I2C_CONTROL_INST_IRQHandler                              I2C1_IRQHandler
#define I2C_CONTROL_INST_INT_IRQN                                  I2C1_INT_IRQn
#define I2C_CONTROL_TARGET_OWN_ADDR                                         0x20
#define GPIO_I2C_CONTROL_SDA_PORT                                          GPIOA
#define GPIO_I2C_CONTROL_SDA_PIN                                  DL_GPIO_PIN_18
#define GPIO_I2C_CONTROL_IOMUX_SDA                               (IOMUX_PINCM40)
#define GPIO_I2C_CONTROL_IOMUX_SDA_FUNC                IOMUX_PINCM40_PF_I2C1_SDA
#define GPIO_I2C_CONTROL_SCL_PORT                                          GPIOA
#define GPIO_I2C_CONTROL_SCL_PIN                                  DL_GPIO_PIN_17
#define GPIO_I2C_CONTROL_IOMUX_SCL                               (IOMUX_PINCM39)
#define GPIO_I2C_CONTROL_IOMUX_SCL_FUNC                IOMUX_PINCM39_PF_I2C1_SCL


/* Defines for UART_CONTROL */
#define UART_CONTROL_INST                                                  UART4
#define UART_CONTROL_INST_FREQUENCY                                     32000000
#define UART_CONTROL_INST_IRQHandler                            UART4_IRQHandler
#define UART_CONTROL_INST_INT_IRQN                                UART4_INT_IRQn
#define GPIO_UART_CONTROL_RX_PORT                                          GPIOB
#define GPIO_UART_CONTROL_TX_PORT                                          GPIOB
#define GPIO_UART_CONTROL_RX_PIN                                  DL_GPIO_PIN_18
#define GPIO_UART_CONTROL_TX_PIN                                  DL_GPIO_PIN_17
#define GPIO_UART_CONTROL_IOMUX_RX                               (IOMUX_PINCM44)
#define GPIO_UART_CONTROL_IOMUX_TX                               (IOMUX_PINCM43)
#define GPIO_UART_CONTROL_IOMUX_RX_FUNC                IOMUX_PINCM44_PF_UART4_RX
#define GPIO_UART_CONTROL_IOMUX_TX_FUNC                IOMUX_PINCM43_PF_UART4_TX
#define UART_CONTROL_BAUD_RATE                                          (115200)
#define UART_CONTROL_IBRD_32_MHZ_115200_BAUD                                (17)
#define UART_CONTROL_FBRD_32_MHZ_115200_BAUD                                (23)





/* Port definition for Pin Group GPIO_M1_PH */
#define GPIO_M1_PH_PORT                                                  (GPIOA)

/* Defines for M1_PH: GPIOA.5 with pinCMx 10 on package pin 11 */
#define GPIO_M1_PH_M1_PH_PIN                                     (DL_GPIO_PIN_5)
#define GPIO_M1_PH_M1_PH_IOMUX                                   (IOMUX_PINCM10)
/* Port definition for Pin Group GPIO_M2_PH */
#define GPIO_M2_PH_PORT                                                  (GPIOA)

/* Defines for M2_PH: GPIOA.24 with pinCMx 54 on package pin 44 */
#define GPIO_M2_PH_M2_PH_PIN                                    (DL_GPIO_PIN_24)
#define GPIO_M2_PH_M2_PH_IOMUX                                   (IOMUX_PINCM54)
/* Defines for M1_ENC_A: GPIOB.7 with pinCMx 24 on package pin 21 */
#define GPIO_ENCODER_M1_ENC_A_PORT                                       (GPIOB)
#define GPIO_ENCODER_M1_ENC_A_PIN                                (DL_GPIO_PIN_7)
#define GPIO_ENCODER_M1_ENC_A_IOMUX                              (IOMUX_PINCM24)
/* Defines for M1_ENC_B: GPIOB.9 with pinCMx 26 on package pin 23 */
#define GPIO_ENCODER_M1_ENC_B_PORT                                       (GPIOB)
#define GPIO_ENCODER_M1_ENC_B_PIN                                (DL_GPIO_PIN_9)
#define GPIO_ENCODER_M1_ENC_B_IOMUX                              (IOMUX_PINCM26)
/* Defines for M2_ENC_A: GPIOA.22 with pinCMx 47 on package pin 40 */
#define GPIO_ENCODER_M2_ENC_A_PORT                                       (GPIOA)
#define GPIO_ENCODER_M2_ENC_A_PIN                               (DL_GPIO_PIN_22)
#define GPIO_ENCODER_M2_ENC_A_IOMUX                              (IOMUX_PINCM47)
/* Defines for M2_ENC_B: GPIOA.21 with pinCMx 46 on package pin 39 */
#define GPIO_ENCODER_M2_ENC_B_PORT                                       (GPIOA)
#define GPIO_ENCODER_M2_ENC_B_PIN                               (DL_GPIO_PIN_21)
#define GPIO_ENCODER_M2_ENC_B_IOMUX                              (IOMUX_PINCM46)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_M1_EN_init(void);
void SYSCFG_DL_PWM_M2_EN_init(void);
void SYSCFG_DL_I2C_CONTROL_init(void);
void SYSCFG_DL_UART_CONTROL_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
