/*************************************************************************************************************
 *                                          MSP432P401
 *                                      ------------------
 *                               /|\|                      |
 *                                | |                      |
 *                                --|RST                   |
 *                                  |                      |
 *                                  |                      |
 *                                  |                 P2.4 |--> Output PWM (right wheel)
 *                                  |                 P2.6 |--> Output PWM (left wheel)
 *                                  |                      |
 *                                  |                      |
 *                                  |                      |
 *
 **************************************************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig left_wheel =
{
     TIMER_A_CLOCKSOURCE_SMCLK,
     TIMER_A_CLOCKSOURCE_DIVIDER_12,
     10000,
     TIMER_A_CAPTURECOMPARE_REGISTER_1,
     TIMER_A_OUTPUTMODE_RESET_SET,
     5000
};

/* Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig right_wheel =
{
     TIMER_A_CLOCKSOURCE_SMCLK,
     TIMER_A_CLOCKSOURCE_DIVIDER_12,
     10000,
     TIMER_A_CAPTURECOMPARE_REGISTER_3,
     TIMER_A_OUTPUTMODE_RESET_SET,
     5000
};

int main(void)
{
    /* Halting the watchdog */
    MAP_WDT_A_holdTimer();

    /* Configuring P4.0 and P4.2 as Output. P2.6 as peripheral output for PWM (left wheel)*/
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring P4.4 and P4.5 as Output. P2.4 as peripheral output for PWM (right wheel)*/
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring Timer_A to have a period of approximately 8 micro sec and duty cycle of 50% of that (5000 ticks)  */
    Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
    Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);

    /* Line sensor: Configuring Output Light.*/
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);;

    /* Configuring P3.7 as Input. Line sensor (right)*/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_interruptEdgeSelect(GPIO_PORT_P3, GPIO_PIN7, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN7);

    /* Configuring P5.5 as Input. Line sensor (left)*/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN5);
    GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN5, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN5);
    GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN5);

    /* Enabling interrupts */
    Interrupt_enableInterrupt(INT_PORT3);
    Interrupt_enableInterrupt(INT_PORT5);
    Interrupt_enableMaster();

    /* Sleeping when not in use */
    while (1)
    {
        PCM_gotoLPM0();
    }
}

void PORT3_IRQHandler(void)
{
    uint32_t status_3 = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);

    /*When Line Sensor detects change from light to dark*/
    // Notice that Interrupt is not cleared till vehicle detects light
    if (status_3 & GPIO_PIN7){
        /*When Line Sensor detects dark*/
        if(GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN7) == 1){
           left_wheel.dutyCycle = 0;
           right_wheel.dutyCycle = 0;
           GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
        }
       /*When Line Sensor detects light*/
       else {
           left_wheel.dutyCycle = 5000;
           right_wheel.dutyCycle = 5000;
           GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
           GPIO_clearInterruptFlag(GPIO_PORT_P3, status_3);
       }
       Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
       Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
    }
}

void PORT5_IRQHandler(void)
{
    uint32_t status_5 = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);

    /*When Line Sensor detects change from light to dark*/
    // Notice that Interrupt is not cleared till vehicle detects light
    if (status_5 & GPIO_PIN5){
        /*When Line Sensor detects dark*/
        if(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN5) == 1){
           left_wheel.dutyCycle = 0;
           right_wheel.dutyCycle = 0;
           GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
       }
       /*When Line Sensor detects light*/
       else{
           left_wheel.dutyCycle = 5000;
           right_wheel.dutyCycle = 5000;
           GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
           GPIO_clearInterruptFlag(GPIO_PORT_P5, status_5);
       }
       Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
       Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
    }
}




