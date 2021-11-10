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
/*******************************************************************************
 * MSP432 Interrupt Controller - Software Pending
 *
 * Description: This simple code example shows users how to pend interrupt
 * from software. This allows users to "simulate" hardware conditions by
 * forcing certain interrupts to fire with the interrupt controller. In this
 * example, a Port 1 interrupt is written to toggle the LED on P1.0. In the
 * main loop for the program however, the pend interrupt function is used
 * in conjunction with a a delay loop to forcibly call this ISR. Additionally,
 * the user can use the switch on P1.1 to trigger an actual hardware interrupt.
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST         P1.0  |---> P1.0 LED
 *            |                  |
 *            |            P2.5  |<--Encoder
 *            |                  |
 *            |                  |
 *
 ******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* PID */
#define TIMER_PERIOD 0x5BAE
#define Wheelcircumference 20.42035

/* PID: Wheel Encoder Variable */
float notchesdetected_left;
float notchesdetected_right;

/* PID: Timer for PID controller to run every 1 second*/
const Timer_A_UpModeConfig speed_timer =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_64,     // 1.5MHz/64 = 23437Hz  period = 1/15625Hz = 0.0000426
        TIMER_PERIOD,                       // When reach 23470, will trigger interrupt.  Delay/0.0000426 = 23470. Delay = 0.99 seconds
        TIMER_A_TAIE_INTERRUPT_DISABLE,     // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                    // Clear value
};

int main(void)
{

    notchesdetected_left = 0;
    notchesdetected_right = 0;

    WDT_A_holdTimer();

    /* PID: Set P4.1 to take in input from Wheel Encoder*/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN1);
    GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN1);

    /* PID: Set P4.3 to take in input from Wheel Encoder*/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN3);
    GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN3);
    GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN3);

    /* Configuring Timer_A1 for Up Mode */
    Timer_A_configureUpMode(TIMER_A1_BASE, &speed_timer);
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableInterrupt(INT_TA1_0);
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    Interrupt_enableInterrupt(INT_PORT4);
    Interrupt_enableMaster();

    while(1);
  {
        PCM_gotoLPM3();
  }
}

//PID: Runs every one second to maintain straightness
void TA1_0_IRQHandler(void)
{
    float speed_difference;

    //Getting Wheel Rotations
    notchesdetected_left = notchesdetected_left/20;
    notchesdetected_right = notchesdetected_right/20;

    //Getting Distance
    notchesdetected_left = notchesdetected_left*Wheelcircumference;
    notchesdetected_left = notchesdetected_left*Wheelcircumference;

    //Getting Speed
    notchesdetected_left = notchesdetected_left/0.5;
    notchesdetected_right = notchesdetected_right/0.5;

    //Compare Speed
    if (notchesdetected_left > notchesdetected_right)
    {
        speed_difference = notchesdetected_left - notchesdetected_right;
        right_wheel.dutyCycle += 100*speed_difference;
    }

    if (notchesdetected_left > notchesdetected_right)
    {
        speed_difference = notchesdetected_right - notchesdetected_left;
        left_wheel.dutyCycle += 100*speed_difference;
    }

    //Keeps PWM speed under 5200
    if (right_wheel.dutyCycle >= 5200 || left_wheel.dutyCycle >= 5200)
    {
        right_wheel.dutyCycle -= 200;
        left_wheel.dutyCycle -= 200;
    }

    //Adjust wheel speed accordingly
    Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
    Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);

    //Reset Notches
    notchesdetected_left = 0;
    notchesdetected_right = 0;
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
                                             TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

//PID: Keeps a count on how many time the wheels spin
void PORT4_IRQHandler(void)
{
    uint32_t status;

    status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
    if (status & GPIO_PIN1)
    {
        notchesdetected_left++;
    }
    if (status & GPIO_PIN3)
    {
        notchesdetected_left++;
    }

    GPIO_clearInterruptFlag(GPIO_PORT_P4, status);
}



