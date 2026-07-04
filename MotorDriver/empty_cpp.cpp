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
#include "app/motor_app.h"

namespace {

constexpr bool kEnableRawUartEchoTest = false;

void rawUartEchoLoop(void)
{
#if defined(UART_CONTROL_INST)
#if defined(GPIO_UART_CONTROL_IOMUX_RX)
    DL_GPIO_setDigitalInternalResistor(GPIO_UART_CONTROL_IOMUX_RX, DL_GPIO_RESISTOR_PULL_UP);
#endif

    while (1) {
        while (!DL_UART_Main_isRXFIFOEmpty(UART_CONTROL_INST)) {
            const uint8_t value = DL_UART_Main_receiveData(UART_CONTROL_INST);
            while (DL_UART_Main_isTXFIFOFull(UART_CONTROL_INST)) {
            }
            DL_UART_Main_transmitData(UART_CONTROL_INST, value);
        }
    }
#else
    while (1) {
    }
#endif
}

}  // namespace

int main(void)
{
    SYSCFG_DL_init();

    if (kEnableRawUartEchoTest) {
        rawUartEchoLoop();
    }

    MotorApp_init();

    while (1) {
        MotorApp_process();
    }
}
