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




/* Defines for SENSOR_I2C */
#define SENSOR_I2C_INST                                                     I2C2
#define SENSOR_I2C_INST_IRQHandler                               I2C2_IRQHandler
#define SENSOR_I2C_INST_INT_IRQN                                   I2C2_INT_IRQn
#define SENSOR_I2C_BUS_SPEED_HZ                                           400000
#define GPIO_SENSOR_I2C_SDA_PORT                                           GPIOC
#define GPIO_SENSOR_I2C_SDA_PIN                                    DL_GPIO_PIN_3
#define GPIO_SENSOR_I2C_IOMUX_SDA                                (IOMUX_PINCM77)
#define GPIO_SENSOR_I2C_IOMUX_SDA_FUNC                 IOMUX_PINCM77_PF_I2C2_SDA
#define GPIO_SENSOR_I2C_SCL_PORT                                           GPIOC
#define GPIO_SENSOR_I2C_SCL_PIN                                    DL_GPIO_PIN_2
#define GPIO_SENSOR_I2C_IOMUX_SCL                                (IOMUX_PINCM76)
#define GPIO_SENSOR_I2C_IOMUX_SCL_FUNC                 IOMUX_PINCM76_PF_I2C2_SCL

/* Defines for MOTOR_I2C */
#define MOTOR_I2C_INST                                                      I2C1
#define MOTOR_I2C_INST_IRQHandler                                I2C1_IRQHandler
#define MOTOR_I2C_INST_INT_IRQN                                    I2C1_INT_IRQn
#define MOTOR_I2C_BUS_SPEED_HZ                                            100000
#define GPIO_MOTOR_I2C_SDA_PORT                                            GPIOA
#define GPIO_MOTOR_I2C_SDA_PIN                                    DL_GPIO_PIN_10
#define GPIO_MOTOR_I2C_IOMUX_SDA                                 (IOMUX_PINCM21)
#define GPIO_MOTOR_I2C_IOMUX_SDA_FUNC                  IOMUX_PINCM21_PF_I2C1_SDA
#define GPIO_MOTOR_I2C_SCL_PORT                                            GPIOA
#define GPIO_MOTOR_I2C_SCL_PIN                                    DL_GPIO_PIN_11
#define GPIO_MOTOR_I2C_IOMUX_SCL                                 (IOMUX_PINCM22)
#define GPIO_MOTOR_I2C_IOMUX_SCL_FUNC                  IOMUX_PINCM22_PF_I2C1_SCL


/* Defines for DEBUG_UART */
#define DEBUG_UART_INST                                                    UART3
#define DEBUG_UART_INST_FREQUENCY                                       32000000
#define DEBUG_UART_INST_IRQHandler                              UART3_IRQHandler
#define DEBUG_UART_INST_INT_IRQN                                  UART3_INT_IRQn
#define GPIO_DEBUG_UART_RX_PORT                                            GPIOA
#define GPIO_DEBUG_UART_TX_PORT                                            GPIOA
#define GPIO_DEBUG_UART_RX_PIN                                    DL_GPIO_PIN_13
#define GPIO_DEBUG_UART_TX_PIN                                    DL_GPIO_PIN_14
#define GPIO_DEBUG_UART_IOMUX_RX                                 (IOMUX_PINCM35)
#define GPIO_DEBUG_UART_IOMUX_TX                                 (IOMUX_PINCM36)
#define GPIO_DEBUG_UART_IOMUX_RX_FUNC                  IOMUX_PINCM35_PF_UART3_RX
#define GPIO_DEBUG_UART_IOMUX_TX_FUNC                  IOMUX_PINCM36_PF_UART3_TX
#define DEBUG_UART_BAUD_RATE                                            (115200)
#define DEBUG_UART_IBRD_32_MHZ_115200_BAUD                                  (17)
#define DEBUG_UART_FBRD_32_MHZ_115200_BAUD                                  (23)
/* Defines for LORA_UART */
#define LORA_UART_INST                                                     UART0
#define LORA_UART_INST_FREQUENCY                                        32000000
#define LORA_UART_INST_IRQHandler                               UART0_IRQHandler
#define LORA_UART_INST_INT_IRQN                                   UART0_INT_IRQn
#define GPIO_LORA_UART_RX_PORT                                             GPIOB
#define GPIO_LORA_UART_TX_PORT                                             GPIOB
#define GPIO_LORA_UART_RX_PIN                                      DL_GPIO_PIN_1
#define GPIO_LORA_UART_TX_PIN                                      DL_GPIO_PIN_0
#define GPIO_LORA_UART_IOMUX_RX                                  (IOMUX_PINCM13)
#define GPIO_LORA_UART_IOMUX_TX                                  (IOMUX_PINCM12)
#define GPIO_LORA_UART_IOMUX_RX_FUNC                   IOMUX_PINCM13_PF_UART0_RX
#define GPIO_LORA_UART_IOMUX_TX_FUNC                   IOMUX_PINCM12_PF_UART0_TX
#define LORA_UART_BAUD_RATE                                             (115200)
#define LORA_UART_IBRD_32_MHZ_115200_BAUD                                   (17)
#define LORA_UART_FBRD_32_MHZ_115200_BAUD                                   (23)
/* Defines for MOTOR_UART */
#define MOTOR_UART_INST                                                    UART1
#define MOTOR_UART_INST_FREQUENCY                                       32000000
#define MOTOR_UART_INST_IRQHandler                              UART1_IRQHandler
#define MOTOR_UART_INST_INT_IRQN                                  UART1_INT_IRQn
#define GPIO_MOTOR_UART_RX_PORT                                            GPIOA
#define GPIO_MOTOR_UART_TX_PORT                                            GPIOA
#define GPIO_MOTOR_UART_RX_PIN                                     DL_GPIO_PIN_9
#define GPIO_MOTOR_UART_TX_PIN                                     DL_GPIO_PIN_8
#define GPIO_MOTOR_UART_IOMUX_RX                                 (IOMUX_PINCM20)
#define GPIO_MOTOR_UART_IOMUX_TX                                 (IOMUX_PINCM19)
#define GPIO_MOTOR_UART_IOMUX_RX_FUNC                  IOMUX_PINCM20_PF_UART1_RX
#define GPIO_MOTOR_UART_IOMUX_TX_FUNC                  IOMUX_PINCM19_PF_UART1_TX
#define MOTOR_UART_BAUD_RATE                                            (115200)
#define MOTOR_UART_IBRD_32_MHZ_115200_BAUD                                  (17)
#define MOTOR_UART_FBRD_32_MHZ_115200_BAUD                                  (23)




/* Defines for IMU_SPI */
#define IMU_SPI_INST                                                       SPI0
#define IMU_SPI_INST_IRQHandler                                 SPI0_IRQHandler
#define IMU_SPI_INST_INT_IRQN                                     SPI0_INT_IRQn
#define GPIO_IMU_SPI_PICO_PORT                                            GPIOB
#define GPIO_IMU_SPI_PICO_PIN                                    DL_GPIO_PIN_17
#define GPIO_IMU_SPI_IOMUX_PICO                                 (IOMUX_PINCM43)
#define GPIO_IMU_SPI_IOMUX_PICO_FUNC                 IOMUX_PINCM43_PF_SPI0_PICO
#define GPIO_IMU_SPI_POCI_PORT                                            GPIOB
#define GPIO_IMU_SPI_POCI_PIN                                    DL_GPIO_PIN_19
#define GPIO_IMU_SPI_IOMUX_POCI                                 (IOMUX_PINCM45)
#define GPIO_IMU_SPI_IOMUX_POCI_FUNC                 IOMUX_PINCM45_PF_SPI0_POCI
/* GPIO configuration for IMU_SPI */
#define GPIO_IMU_SPI_SCLK_PORT                                            GPIOB
#define GPIO_IMU_SPI_SCLK_PIN                                    DL_GPIO_PIN_18
#define GPIO_IMU_SPI_IOMUX_SCLK                                 (IOMUX_PINCM44)
#define GPIO_IMU_SPI_IOMUX_SCLK_FUNC                 IOMUX_PINCM44_PF_SPI0_SCLK



/* Port definition for Pin Group GPIO_LED_A */
#define GPIO_LED_A_PORT                                                  (GPIOA)

/* Defines for LED1: GPIOA.27 with pinCMx 60 on package pin 99 */
#define GPIO_LED_A_LED1_PIN                                     (DL_GPIO_PIN_27)
#define GPIO_LED_A_LED1_IOMUX                                    (IOMUX_PINCM60)
/* Defines for LED2: GPIOA.26 with pinCMx 59 on package pin 98 */
#define GPIO_LED_A_LED2_PIN                                     (DL_GPIO_PIN_26)
#define GPIO_LED_A_LED2_IOMUX                                    (IOMUX_PINCM59)
/* Port definition for Pin Group GPIO_BUZZER */
#define GPIO_BUZZER_PORT                                                 (GPIOA)

/* Defines for BUZZER: GPIOA.12 with pinCMx 34 on package pin 51 */
#define GPIO_BUZZER_BUZZER_PIN                                  (DL_GPIO_PIN_12)
#define GPIO_BUZZER_BUZZER_IOMUX                                 (IOMUX_PINCM34)
/* Port definition for Pin Group GPIO_BUTTON_C */
#define GPIO_BUTTON_C_PORT                                               (GPIOC)

/* Defines for BUTTON1: GPIOC.9 with pinCMx 87 on package pin 81 */
#define GPIO_BUTTON_C_BUTTON1_PIN                                (DL_GPIO_PIN_9)
#define GPIO_BUTTON_C_BUTTON1_IOMUX                              (IOMUX_PINCM87)
/* Port definition for Pin Group GPIO_BUTTON_B */
#define GPIO_BUTTON_B_PORT                                               (GPIOB)

/* Defines for BUTTON2: GPIOB.20 with pinCMx 48 on package pin 82 */
#define GPIO_BUTTON_B_BUTTON2_PIN                               (DL_GPIO_PIN_20)
#define GPIO_BUTTON_B_BUTTON2_IOMUX                              (IOMUX_PINCM48)
/* Defines for BUTTON3: GPIOB.23 with pinCMx 51 on package pin 85 */
#define GPIO_BUTTON_B_BUTTON3_PIN                               (DL_GPIO_PIN_23)
#define GPIO_BUTTON_B_BUTTON3_IOMUX                              (IOMUX_PINCM51)
/* Port definition for Pin Group GPIO_IMU_C */
#define GPIO_IMU_C_PORT                                                  (GPIOC)

/* Defines for ICM45686_INT1: GPIOC.6 with pinCMx 84 on package pin 78 */
#define GPIO_IMU_C_ICM45686_INT1_PIN                             (DL_GPIO_PIN_6)
#define GPIO_IMU_C_ICM45686_INT1_IOMUX                           (IOMUX_PINCM84)
/* Defines for ICM45686_CS: GPIOC.7 with pinCMx 85 on package pin 79 */
#define GPIO_IMU_C_ICM45686_CS_PIN                               (DL_GPIO_PIN_7)
#define GPIO_IMU_C_ICM45686_CS_IOMUX                             (IOMUX_PINCM85)
/* Defines for LIS3MDL_CS: GPIOC.8 with pinCMx 86 on package pin 80 */
#define GPIO_IMU_C_LIS3MDL_CS_PIN                                (DL_GPIO_PIN_8)
#define GPIO_IMU_C_LIS3MDL_CS_IOMUX                              (IOMUX_PINCM86)
/* Port definition for Pin Group GPIO_IMU_A */
#define GPIO_IMU_A_PORT                                                  (GPIOA)

/* Defines for LIS3MDL_DRDY: GPIOA.22 with pinCMx 47 on package pin 77 */
#define GPIO_IMU_A_LIS3MDL_DRDY_PIN                             (DL_GPIO_PIN_22)
#define GPIO_IMU_A_LIS3MDL_DRDY_IOMUX                            (IOMUX_PINCM47)
/* Defines for ICM45686_INT2_FSYNC: GPIOA.31 with pinCMx 6 on package pin 7 */
#define GPIO_IMU_A_ICM45686_INT2_FSYNC_PIN                      (DL_GPIO_PIN_31)
#define GPIO_IMU_A_ICM45686_INT2_FSYNC_IOMUX                      (IOMUX_PINCM6)
/* Port definition for Pin Group GPIO_LED_B */
#define GPIO_LED_B_PORT                                                  (GPIOB)

/* Defines for LED3: GPIOB.27 with pinCMx 58 on package pin 97 */
#define GPIO_LED_B_LED3_PIN                                     (DL_GPIO_PIN_27)
#define GPIO_LED_B_LED3_IOMUX                                    (IOMUX_PINCM58)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SENSOR_I2C_init(void);
void SYSCFG_DL_MOTOR_I2C_init(void);
void SYSCFG_DL_DEBUG_UART_init(void);
void SYSCFG_DL_LORA_UART_init(void);
void SYSCFG_DL_MOTOR_UART_init(void);
void SYSCFG_DL_IMU_SPI_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
