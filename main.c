/*************************************************************************************************************
 *                                          MSP432P401
 *                                      ------------------
 *                               /|\|                      |
 *                                | |                      |
 *                                --|RST                   |
 *                                  |                      |
 *                                  |                      |
 *                                  |                 P2.4 |--> Output PWM (left wheel)
 *                                  |                 P2.6 |--> Output PWM (right wheel)
 *                                  |                      |
 *                                  |                      |
 *                                  |                      |
 *
 **************************************************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>


/* Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig left_wheel =
{
     TIMER_A_CLOCKSOURCE_SMCLK,
     TIMER_A_CLOCKSOURCE_DIVIDER_24,
     10000,
     TIMER_A_CAPTURECOMPARE_REGISTER_1,
     TIMER_A_OUTPUTMODE_RESET_SET,
     5000
};

Timer_A_PWMConfig right_wheel =
{
     TIMER_A_CLOCKSOURCE_SMCLK,
     TIMER_A_CLOCKSOURCE_DIVIDER_24,
     10000,
     TIMER_A_CAPTURECOMPARE_REGISTER_3,
     TIMER_A_OUTPUTMODE_RESET_SET,
     5000
};

int main(void)
{
    /* Halting the watchdog */
    MAP_WDT_A_holdTimer();

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


    /* Configuring P4.4 and P4.5 as Output. P2.4 as peripheral output for PWM (left wheel)*/
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring P4.0 and P4.2 as Output. P2.6 as peripheral output for PWM (right wheel)*/
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring Timer_A to have a period of approximately 80ms and duty cycle of 50% of that (5000 ticks)  */
    Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
    Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);

    /* Enabling interrupts */
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableMaster();

    /* Sleeping when not in use */
    while (1)
    {
        PCM_gotoLPM0();
    }
}

void PORT3_IRQHandler(void)
{
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);

    /*When left sensor detects line*/
    if (status & GPIO_PIN6){
        /* While right sensor is detecting line*/
        /* Move left*/
        if(GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN7) == 1){
            right_wheel.dutycycle = 0;
            left_wheel.dutycycle = 1000;

            Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
            Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
        }

        /* When right sensor no longer detects a line*/
        /* Continue Straight*/
        else{
            right_wheel.dutycycle = 5000;
            left_wheel.dutycycle = 5000;

            Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
            Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
            GPIO_clearInterruptFlag(GPIO_PORT_P3, status);
            }
        }
}


void PORT5_IRQHandler(void)
{
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);

    /*When left sensor detects line*/
    if (status & GPIO_PIN5){
        /* While left sensor is detecting line*/
        /* Move right*/
        if(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN5) == 1){
            right_wheel.dutycycle = 1000;
            left_wheel.dutycycle = 0;

            Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
            Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
        }

        /* When right sensor no longer detects a line*/
        /* Continue Straight*/
        else{
            right_wheel.dutycycle = 5000;
            left_wheel.dutycycle = 5000;

            Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
            Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
            GPIO_clearInterruptFlag(GPIO_PORT_P3, status);
            }
        }
}

