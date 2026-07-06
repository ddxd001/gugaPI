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
    SYSCFG_DL_PWM_M1_EN_init();
    SYSCFG_DL_PWM_M2_EN_init();
    SYSCFG_DL_I2C_CONTROL_init();
    SYSCFG_DL_UART_CONTROL_init();
}



SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerG_reset(PWM_M1_EN_INST);
    DL_TimerG_reset(PWM_M2_EN_INST);
    DL_I2C_reset(I2C_CONTROL_INST);
    DL_UART_Main_reset(UART_CONTROL_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerG_enablePower(PWM_M1_EN_INST);
    DL_TimerG_enablePower(PWM_M2_EN_INST);
    DL_I2C_enablePower(I2C_CONTROL_INST);
    DL_UART_Main_enablePower(UART_CONTROL_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_M1_EN_C1_IOMUX,GPIO_PWM_M1_EN_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_M1_EN_C1_PORT, GPIO_PWM_M1_EN_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_M2_EN_C1_IOMUX,GPIO_PWM_M2_EN_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_M2_EN_C1_PORT, GPIO_PWM_M2_EN_C1_PIN);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_CONTROL_IOMUX_SDA,
        GPIO_I2C_CONTROL_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_CONTROL_IOMUX_SCL,
        GPIO_I2C_CONTROL_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_CONTROL_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_CONTROL_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_CONTROL_IOMUX_TX, GPIO_UART_CONTROL_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_CONTROL_IOMUX_RX, GPIO_UART_CONTROL_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(GPIO_M1_PH_M1_PH_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_M2_PH_M2_PH_IOMUX);

    DL_GPIO_initDigitalInputFeatures(GPIO_ENCODER_M1_ENC_A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_ENCODER_M1_ENC_B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_ENCODER_M2_ENC_A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_ENCODER_M2_ENC_B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOA, GPIO_M1_PH_M1_PH_PIN |
		GPIO_M2_PH_M2_PH_PIN);
    DL_GPIO_enableOutput(GPIOA, GPIO_M1_PH_M1_PH_PIN |
		GPIO_M2_PH_M2_PH_PIN);

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


/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gPWM_M1_ENClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gPWM_M1_ENConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 1600,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_M1_EN_init(void) {

    DL_TimerG_setClockConfig(
        PWM_M1_EN_INST, (DL_TimerG_ClockConfig *) &gPWM_M1_ENClockConfig);

    DL_TimerG_initPWMMode(
        PWM_M1_EN_INST, (DL_TimerG_PWMConfig *) &gPWM_M1_ENConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(PWM_M1_EN_INST,DL_TIMER_CZC_CCCTL1_ZCOND,DL_TIMER_CAC_CCCTL1_ACOND,DL_TIMER_CLC_CCCTL1_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(PWM_M1_EN_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_M1_EN_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_M1_EN_INST, 1600, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(PWM_M1_EN_INST);


    
    DL_TimerG_setCCPDirection(PWM_M1_EN_INST , DL_TIMER_CC1_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gPWM_M2_ENClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gPWM_M2_ENConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 1600,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_M2_EN_init(void) {

    DL_TimerG_setClockConfig(
        PWM_M2_EN_INST, (DL_TimerG_ClockConfig *) &gPWM_M2_ENClockConfig);

    DL_TimerG_initPWMMode(
        PWM_M2_EN_INST, (DL_TimerG_PWMConfig *) &gPWM_M2_ENConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(PWM_M2_EN_INST,DL_TIMER_CZC_CCCTL1_ZCOND,DL_TIMER_CAC_CCCTL1_ACOND,DL_TIMER_CLC_CCCTL1_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(PWM_M2_EN_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_M2_EN_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_M2_EN_INST, 1600, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(PWM_M2_EN_INST);


    
    DL_TimerG_setCCPDirection(PWM_M2_EN_INST , DL_TIMER_CC1_OUTPUT );


}


static const DL_I2C_ClockConfig gI2C_CONTROLClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_CONTROL_init(void) {

    DL_I2C_setClockConfig(I2C_CONTROL_INST,
        (DL_I2C_ClockConfig *) &gI2C_CONTROLClockConfig);
    DL_I2C_disableAnalogGlitchFilter(I2C_CONTROL_INST);

    /* Configure Target Mode */
    DL_I2C_setTargetOwnAddress(I2C_CONTROL_INST, I2C_CONTROL_TARGET_OWN_ADDR);
    DL_I2C_setTargetTXFIFOThreshold(I2C_CONTROL_INST, DL_I2C_TX_FIFO_LEVEL_BYTES_1);
    DL_I2C_setTargetRXFIFOThreshold(I2C_CONTROL_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableTargetTXEmptyOnTXRequest(I2C_CONTROL_INST);

    DL_I2C_enableTargetClockStretching(I2C_CONTROL_INST);

    /* Workaround for errata I2C_ERR_04 */
    DL_I2C_disableTargetWakeup(I2C_CONTROL_INST);
    /* Configure Interrupts */
    DL_I2C_enableInterrupt(I2C_CONTROL_INST,
                           DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER |
                           DL_I2C_INTERRUPT_TARGET_START |
                           DL_I2C_INTERRUPT_TARGET_STOP);


    /* Enable module */
    DL_I2C_enableTarget(I2C_CONTROL_INST);


}

static const DL_UART_Main_ClockConfig gUART_CONTROLClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_CONTROLConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_CONTROL_init(void)
{
    DL_UART_Main_setClockConfig(UART_CONTROL_INST, (DL_UART_Main_ClockConfig *) &gUART_CONTROLClockConfig);

    DL_UART_Main_init(UART_CONTROL_INST, (DL_UART_Main_Config *) &gUART_CONTROLConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(UART_CONTROL_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_CONTROL_INST, UART_CONTROL_IBRD_32_MHZ_115200_BAUD, UART_CONTROL_FBRD_32_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_CONTROL_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);

    /* Configure FIFOs */
    DL_UART_Main_enableFIFOs(UART_CONTROL_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_CONTROL_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(UART_CONTROL_INST, DL_UART_TX_FIFO_LEVEL_3_4_EMPTY);

    DL_UART_Main_enable(UART_CONTROL_INST);
}

