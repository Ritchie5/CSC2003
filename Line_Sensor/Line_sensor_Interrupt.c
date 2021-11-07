/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
/******************************************************************************
 * Debugging log via MSP432 UART
 *
 * Description: This demo prints debugging log via a PC serial port (i.e. COM).
 * SMCLK/DCO is used as clock source and auto-clock enable feature is used by eUSCI.
 * COM used is described as "XDS110 Class Application/User UART".
 * Check your device manager to find out which COM port is used, e.g. COM3
 *
 *               MSP432P401
 *             -----------------
 *            |                 |
 *            |             P1.0|----> LED1
 *            |                 |
 *       RST -|     P1.3/UCA0TXD|----> PC
 *            |                 |
 *            |     P1.2/UCA0RXD|<---- PC
 *            |                 |
 *
 *******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

int main(void)
{
    /* Halting WDT  */
    WDT_A_holdTimer();

    /* Line sensor: Configuring Output Light.*/
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);

    /* Line sensor: Configuring P3.7 and P5.5 as Input.*/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN5);

    /* Line sensor: Set interrupt. */
    GPIO_interruptEdgeSelect(GPIO_PORT_P3, GPIO_PIN7,GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN5,GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN5);
    GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN5);
    Interrupt_enableInterrupt(INT_PORT3);
    Interrupt_enableInterrupt(INT_PORT5);

    Interrupt_enableMaster();

    while(1)
    {
        PCM_gotoLPM3InterruptSafe();
    }
}


void PORT3_IRQHandler(void)
{
    uint32_t status_3 = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
    GPIO_clearInterruptFlag(GPIO_PORT_P3, status_3);

    /*When Line Sensor detects change from light to dark*/
    if (status_3 & GPIO_PIN7){
        if(GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN7) == 1){
            GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        }
    }
}

void PORT5_IRQHandler(void)
{
    uint32_t status_5 = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
    GPIO_clearInterruptFlag(GPIO_PORT_P5, status_5);

    /*When Line Sensor detects change from light to dark*/
    if (status_5 & GPIO_PIN5){
        if(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN5) == 1){
            GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
        }
    }
}
