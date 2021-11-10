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

/* PID */
#define TIMER_PERIOD 0x2DC6
#define Wheelcircumference 20.42035

/* PID: Wheel Encoder Variable */
float notchesdetected_left;
float notchesdetected_right;

/* Motor: Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig left_wheel =
{
     TIMER_A_CLOCKSOURCE_SMCLK,
     TIMER_A_CLOCKSOURCE_DIVIDER_12,
     10000,
     TIMER_A_CAPTURECOMPARE_REGISTER_1,
     TIMER_A_OUTPUTMODE_RESET_SET,
     5000
};

/* Motor: Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig right_wheel =
{
     TIMER_A_CLOCKSOURCE_SMCLK,
     TIMER_A_CLOCKSOURCE_DIVIDER_12,
     10000,
     TIMER_A_CAPTURECOMPARE_REGISTER_3,
     TIMER_A_OUTPUTMODE_RESET_SET,
     5000
};

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

void PID_Config(){
    notchesdetected_left = 0;
    notchesdetected_right = 0;

    /* PID: Set P4.1 to take in input from Wheel Encoder*/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN1);
    GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN1);

    /* PID: Set P4.3 to take in input from Wheel Encoder*/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN3);
    GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN3);
    GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN3);
}

void LineSensor_Config(){
    /* Line sensor: Configuring Output Light.*/
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
    ;

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
}

void Motor_Config(){
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
}


int main(void)
{
    /* Halting the watchdog */
    MAP_WDT_A_holdTimer();

    /* PID: Configure PID(Wheel Encoder) controller GPIO Pins*/
    PID_Config();

    /* Line Sensor: Configure Line Sensor GPIO Pins*/
    LineSensor_Config();

    /* Motor: Configuring Motor GPIO Pins*/
    Motor_Config();

    /* Configuring Timer_A to have a period of approximately 8 micro sec and duty cycle of 50% of that (5000 ticks)  */
    Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
    Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);

    /* PID: Configuring Timer*/
    Timer_A_configureUpMode(TIMER_A1_BASE, &speed_timer);
    
    /* Enabling interrupts */
    Interrupt_enableInterrupt(INT_TA1_0);
    Interrupt_enableInterrupt(INT_PORT3);
    Interrupt_enableInterrupt(INT_PORT4);
    Interrupt_enableInterrupt(INT_PORT5);
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableMaster();

    /* Sleeping when not in use */
    while (1)
    {
        PCM_gotoLPM0();
    }
}

//PID: Runs every one second to maintain straightness
void TA1_0_IRQHandler(void)
{
    float speed_difference;

    //Getting Wheel Rotations
    notchesdetected_left = notchesdetected_left / 20;
    notchesdetected_right = notchesdetected_right / 20;

    //Getting Distance
    notchesdetected_left = notchesdetected_left * Wheelcircumference;
    notchesdetected_right = notchesdetected_right * Wheelcircumference;

    //Getting Speed
    //Unnecessary as time is one second. 
    // notchesdetected_left = notchesdetected_left / 1;
    // notchesdetected_right = notchesdetected_right / 1;

    //Compare Speed
    //Adjust Speed accordingly
    if (notchesdetected_left > notchesdetected_right)
    {
        // speed_difference = notchesdetected_left - notchesdetected_right;
        // right_wheel.dutyCycle += 100 * speed_difference;
        right_wheel.dutyCycle = 5100;
        left_wheel.dutyCycle = 5000;
    }

    if (notchesdetected_right > notchesdetected_left)
    {
        // speed_difference = notchesdetected_right - notchesdetected_left;
        // left_wheel.dutyCycle += 100 * speed_difference;
        right_wheel.dutyCycle = 5000;
        left_wheel.dutyCycle = 5100;
    }

    // //Keeps PWM speed under 5200
    // if (right_wheel.dutyCycle >= 5200 || left_wheel.dutyCycle >= 5200)
    // {
    //     right_wheel.dutyCycle -= 200;
    //     left_wheel.dutyCycle -= 200;
    // }

    //Adjust wheel speed accordingly
    Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
    Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);

    //Reset Notches
    notchesdetected_left = 0;
    notchesdetected_right = 0;

    //Resets Timer for 1 Second Interrupt
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
                                             TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

void PORT3_IRQHandler(void)
{
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);

    /*When Line Sensor (right) detects change from light to dark*/
    if (status & GPIO_PIN7)
    {
        /*When Line Sensor detects dark*/
        if (GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN7) == 1)
        {
            left_wheel.dutyCycle = 0;
            right_wheel.dutyCycle = 8000;
            Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
            Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
            Delay(100000);
            left_wheel.dutyCycle = 5000;
            right_wheel.dutyCycle = 5000;
            Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
            Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
        }
        GPIO_clearInterruptFlag(GPIO_PORT_P3, status);
    }
}

//PID: Keeps a count on how many time the wheels spin
void PORT4_IRQHandler(void)
{
    uint32_t status;

    status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
    if (status & GPIO_PIN1) //Left Wheel
    {
        notchesdetected_left++;
    }
    if (status & GPIO_PIN3) //Right Wheel
    {
        notchesdetected_right++;
    }

    GPIO_clearInterruptFlag(GPIO_PORT_P4, status);
}

void PORT5_IRQHandler(void)
{
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);

    /*When Line Sensor (left) detects change from light to dark*/
    if (status & GPIO_PIN1)
    {
        /*When Line Sensor detects dark*/
        if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN1) == 1)
        {
            left_wheel.dutyCycle = 8000;
            right_wheel.dutyCycle = 0;
            Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
            Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
            Delay(100000);
            left_wheel.dutyCycle = 5000;
            right_wheel.dutyCycle = 5000;
            Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
            Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
        }
        GPIO_clearInterruptFlag(GPIO_PORT_P5, status);
    }
}
