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

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G351X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_UART_Main_backupConfig gDEBUG_UARTBackup;
DL_SPI_backupConfig gIMU_SPIBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_SENSOR_I2C_init();
    SYSCFG_DL_MOTOR_I2C_init();
    SYSCFG_DL_DEBUG_UART_init();
    SYSCFG_DL_LORA_UART_init();
    SYSCFG_DL_MOTOR_UART_init();
    SYSCFG_DL_IMU_SPI_init();
    /* Ensure backup structures have no valid state */
	gDEBUG_UARTBackup.backupRdy 	= false;
	gIMU_SPIBackup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_UART_Main_saveConfiguration(DEBUG_UART_INST, &gDEBUG_UARTBackup);
	retStatus &= DL_SPI_saveConfiguration(IMU_SPI_INST, &gIMU_SPIBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_UART_Main_restoreConfiguration(DEBUG_UART_INST, &gDEBUG_UARTBackup);
	retStatus &= DL_SPI_restoreConfiguration(IMU_SPI_INST, &gIMU_SPIBackup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_GPIO_reset(GPIOC);
    DL_I2C_reset(SENSOR_I2C_INST);
    DL_I2C_reset(MOTOR_I2C_INST);
    DL_UART_Main_reset(DEBUG_UART_INST);
    DL_UART_Main_reset(LORA_UART_INST);
    DL_UART_Main_reset(MOTOR_UART_INST);
    DL_SPI_reset(IMU_SPI_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_GPIO_enablePower(GPIOC);
    DL_I2C_enablePower(SENSOR_I2C_INST);
    DL_I2C_enablePower(MOTOR_I2C_INST);
    DL_UART_Main_enablePower(DEBUG_UART_INST);
    DL_UART_Main_enablePower(LORA_UART_INST);
    DL_UART_Main_enablePower(MOTOR_UART_INST);
    DL_SPI_enablePower(IMU_SPI_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_SENSOR_I2C_IOMUX_SDA,
        GPIO_SENSOR_I2C_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_SENSOR_I2C_IOMUX_SCL,
        GPIO_SENSOR_I2C_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_SENSOR_I2C_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_SENSOR_I2C_IOMUX_SCL);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_MOTOR_I2C_IOMUX_SDA,
        GPIO_MOTOR_I2C_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_MOTOR_I2C_IOMUX_SCL,
        GPIO_MOTOR_I2C_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_MOTOR_I2C_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_MOTOR_I2C_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_DEBUG_UART_IOMUX_TX, GPIO_DEBUG_UART_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_DEBUG_UART_IOMUX_RX, GPIO_DEBUG_UART_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_LORA_UART_IOMUX_TX, GPIO_LORA_UART_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_LORA_UART_IOMUX_RX, GPIO_LORA_UART_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_MOTOR_UART_IOMUX_TX, GPIO_MOTOR_UART_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_MOTOR_UART_IOMUX_RX, GPIO_MOTOR_UART_IOMUX_RX_FUNC);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_IMU_SPI_IOMUX_SCLK, GPIO_IMU_SPI_IOMUX_SCLK_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_IMU_SPI_IOMUX_PICO, GPIO_IMU_SPI_IOMUX_PICO_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_IMU_SPI_IOMUX_POCI, GPIO_IMU_SPI_IOMUX_POCI_FUNC);

    DL_GPIO_initDigitalOutput(GPIO_LED_A_LED1_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_LED_A_LED2_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_BUZZER_BUZZER_IOMUX);

    DL_GPIO_initDigitalInputFeatures(GPIO_BUTTON_C_BUTTON1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_BUTTON_B_BUTTON2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_BUTTON_B_BUTTON3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_IMU_C_ICM45686_INT1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(GPIO_IMU_C_ICM45686_CS_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_IMU_C_LIS3MDL_CS_IOMUX);

    DL_GPIO_initDigitalInputFeatures(GPIO_IMU_A_LIS3MDL_DRDY_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_IMU_A_ICM45686_INT2_FSYNC_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(GPIO_LED_B_LED3_IOMUX);

    DL_GPIO_initDigitalInputFeatures(GPIO_GY931_I2C_GY931_SCL_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_GY931_I2C_GY931_SDA_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_setPins(GPIOA, GPIO_LED_A_LED1_PIN |
		GPIO_LED_A_LED2_PIN);
    DL_GPIO_enableOutput(GPIOA, GPIO_LED_A_LED1_PIN |
		GPIO_LED_A_LED2_PIN);
    DL_GPIO_setPins(GPIOB, GPIO_LED_B_LED3_PIN);
    DL_GPIO_enableOutput(GPIOB, GPIO_LED_B_LED3_PIN);
    DL_GPIO_clearPins(GPIOC, GPIO_BUZZER_BUZZER_PIN);
    DL_GPIO_setPins(GPIOC, GPIO_IMU_C_ICM45686_CS_PIN |
		GPIO_IMU_C_LIS3MDL_CS_PIN);
    DL_GPIO_enableOutput(GPIOC, GPIO_BUZZER_BUZZER_PIN |
		GPIO_IMU_C_ICM45686_CS_PIN |
		GPIO_IMU_C_LIS3MDL_CS_PIN);

}



SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	/* Set default configuration */
	DL_SYSCTL_disableHFXT();
	DL_SYSCTL_disableSYSPLL();

}


static const DL_I2C_ClockConfig gSENSOR_I2CClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_SENSOR_I2C_init(void) {

    DL_I2C_setClockConfig(SENSOR_I2C_INST,
        (DL_I2C_ClockConfig *) &gSENSOR_I2CClockConfig);
    DL_I2C_disableAnalogGlitchFilter(SENSOR_I2C_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(SENSOR_I2C_INST);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(SENSOR_I2C_INST, 7);
    DL_I2C_setControllerTXFIFOThreshold(SENSOR_I2C_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(SENSOR_I2C_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(SENSOR_I2C_INST);


    /* Enable module */
    DL_I2C_enableController(SENSOR_I2C_INST);


}
static const DL_I2C_ClockConfig gMOTOR_I2CClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_MOTOR_I2C_init(void) {

    DL_I2C_setClockConfig(MOTOR_I2C_INST,
        (DL_I2C_ClockConfig *) &gMOTOR_I2CClockConfig);
    DL_I2C_disableAnalogGlitchFilter(MOTOR_I2C_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(MOTOR_I2C_INST);
    /* Set frequency to 100000 Hz*/
    DL_I2C_setTimerPeriod(MOTOR_I2C_INST, 31);
    DL_I2C_setControllerTXFIFOThreshold(MOTOR_I2C_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(MOTOR_I2C_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(MOTOR_I2C_INST);


    /* Enable module */
    DL_I2C_enableController(MOTOR_I2C_INST);


}

static const DL_UART_Main_ClockConfig gDEBUG_UARTClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gDEBUG_UARTConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_DEBUG_UART_init(void)
{
    DL_UART_Main_setClockConfig(DEBUG_UART_INST, (DL_UART_Main_ClockConfig *) &gDEBUG_UARTClockConfig);

    DL_UART_Main_init(DEBUG_UART_INST, (DL_UART_Main_Config *) &gDEBUG_UARTConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(DEBUG_UART_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(DEBUG_UART_INST, DEBUG_UART_IBRD_32_MHZ_115200_BAUD, DEBUG_UART_FBRD_32_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(DEBUG_UART_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);

    /* Configure FIFOs */
    DL_UART_Main_enableFIFOs(DEBUG_UART_INST);
    DL_UART_Main_setRXFIFOThreshold(DEBUG_UART_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(DEBUG_UART_INST, DL_UART_TX_FIFO_LEVEL_3_4_EMPTY);

    DL_UART_Main_enable(DEBUG_UART_INST);
}
static const DL_UART_Main_ClockConfig gLORA_UARTClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gLORA_UARTConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_LORA_UART_init(void)
{
    DL_UART_Main_setClockConfig(LORA_UART_INST, (DL_UART_Main_ClockConfig *) &gLORA_UARTClockConfig);

    DL_UART_Main_init(LORA_UART_INST, (DL_UART_Main_Config *) &gLORA_UARTConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(LORA_UART_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(LORA_UART_INST, LORA_UART_IBRD_32_MHZ_115200_BAUD, LORA_UART_FBRD_32_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(LORA_UART_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);

    /* Configure FIFOs */
    DL_UART_Main_enableFIFOs(LORA_UART_INST);
    DL_UART_Main_setRXFIFOThreshold(LORA_UART_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(LORA_UART_INST, DL_UART_TX_FIFO_LEVEL_3_4_EMPTY);

    DL_UART_Main_enable(LORA_UART_INST);
}
static const DL_UART_Main_ClockConfig gMOTOR_UARTClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gMOTOR_UARTConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_MOTOR_UART_init(void)
{
    DL_UART_Main_setClockConfig(MOTOR_UART_INST, (DL_UART_Main_ClockConfig *) &gMOTOR_UARTClockConfig);

    DL_UART_Main_init(MOTOR_UART_INST, (DL_UART_Main_Config *) &gMOTOR_UARTConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(MOTOR_UART_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(MOTOR_UART_INST, MOTOR_UART_IBRD_32_MHZ_115200_BAUD, MOTOR_UART_FBRD_32_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(MOTOR_UART_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);

    /* Configure FIFOs */
    DL_UART_Main_enableFIFOs(MOTOR_UART_INST);
    DL_UART_Main_setRXFIFOThreshold(MOTOR_UART_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(MOTOR_UART_INST, DL_UART_TX_FIFO_LEVEL_3_4_EMPTY);

    DL_UART_Main_enable(MOTOR_UART_INST);
}

static const DL_SPI_Config gIMU_SPI_config = {
    .mode        = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL1_PHA1,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = DL_SPI_DATA_SIZE_8,
    .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
};

static const DL_SPI_ClockConfig gIMU_SPI_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

SYSCONFIG_WEAK void SYSCFG_DL_IMU_SPI_init(void) {
    DL_SPI_setClockConfig(IMU_SPI_INST, (DL_SPI_ClockConfig *) &gIMU_SPI_clockConfig);

    DL_SPI_init(IMU_SPI_INST, (DL_SPI_Config *) &gIMU_SPI_config);

    /* Configure Controller mode */
    /*
     * Set the bit rate clock divider to generate the serial output clock
     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
     *     1000000 = (32000000)/((1 + 15) * 2)
     */
    DL_SPI_setBitRateSerialClockDivider(IMU_SPI_INST, 15);
    /* Set RX and TX FIFO threshold levels */
    DL_SPI_setFIFOThreshold(IMU_SPI_INST, DL_SPI_RX_FIFO_LEVEL_1_2_FULL, DL_SPI_TX_FIFO_LEVEL_1_2_EMPTY);

    /* Enable module */
    DL_SPI_enable(IMU_SPI_INST);
}

