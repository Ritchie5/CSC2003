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
#include <stdio.h>

/* Ultra Sonic  */
#define MIN_DISTANCE 10.0f
#define TICKPERIOD 1000

uint32_t SR04IntTimes;

/* PID */
#define TIMER_PERIOD 0x2DC6 //0x5BAE for one second
#define Wheelcircumference 20.42035

/* PID: Wheel Encoder Variable */
uint32_t notchesdetected_left;
uint32_t notchesdetected_right;

/* Motor: Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig left_wheel =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_12,
        10000,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        4000};

/* Motor: Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig right_wheel =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_12,
        10000,
        TIMER_A_CAPTURECOMPARE_REGISTER_3,
        TIMER_A_OUTPUTMODE_RESET_SET,
        4700};

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

static void Delay(uint32_t loop)
{
    volatile uint32_t i;

    for (i = 0; i < loop; i++)
        ;
}

// Ultrasonic ================================================================================================================================================================
void Initalise_HCSR04(void)
{
    /* Timer_A UpMode Configuration Parameter */
    const Timer_A_UpModeConfig upConfig =
        {
            TIMER_A_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
            TIMER_A_CLOCKSOURCE_DIVIDER_3,      // SMCLK/3 = 1MHz
            TICKPERIOD,                         // 1000 tick period
            TIMER_A_TAIE_INTERRUPT_DISABLE,     // Disable Timer interrupt
            TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Enable CCR0 interrupt
            TIMER_A_DO_CLEAR                    // Clear value
        };

    int a = CS_getSMCLK();

    /* Configuring P3.6 as Output */
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);    //
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); //

    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P3, GPIO_PIN7);

    /* Configuring Timer_A0 for Up Mode */
    Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);

    /* Enabling interrupts and starting the timer */
    Interrupt_enableInterrupt(INT_TA0_0);
    //Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    //Timer_A_stopTimer(TIMER_A0_BASE);
    Timer_A_clearTimer(TIMER_A0_BASE);
}

void TA0_0_IRQHandler(void)
{
    /* Increment global variable (count number of interrupt occurred) */
    SR04IntTimes++;

    /* Clear interrupt flag */
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

static uint32_t getHCSR04Time(void)
{
    uint32_t pulsetime = 0;

    /* Number of times the interrupt occurred (1 interrupt = 1000 ticks)    */
    pulsetime = SR04IntTimes * TICKPERIOD;

    /* Number of ticks (between 1 to 999) before the interrupt could occur */
    pulsetime += Timer_A_getCounterValue(TIMER_A0_BASE);

    /* Clear Timer */
    Timer_A_clearTimer(TIMER_A0_BASE);

    Delay(3000);

    return pulsetime;
}

float getHCSR04Distance(void)
{
    uint32_t pulseduration = 0;
    float calculateddistance = 0;

    /* Generate 10us pulse at P3.6 */
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
    Delay(30);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);

    /* Wait for positive-edge */
    while (GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN7) == 0)
        ;

    /* Start Timer */
    SR04IntTimes = 0;
    Timer_A_clearTimer(TIMER_A0_BASE);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    /* Detects negative-edge */
    while (GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN7) == 1)
        ;

    /* Stop Timer */
    Timer_A_stopTimer(TIMER_A0_BASE);

    /* Obtain Pulse Width in microseconds */
    pulseduration = getHCSR04Time();

    /* Calculating distance in cm */
    calculateddistance = (float)pulseduration / 58.0f;

    return calculateddistance;
}
// ================================================================================================================================================================

void PID_Config()
{
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

void LED_Config()
{
    /* LED: Configuring Output Light.*/
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
}

void LineSensor_Config()
{
    /* Configuring P5.4 as Input. Line sensor (right)*/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN4);
    GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN4, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN4);
    GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN4);

    /* Configuring P5.5 as Input. Line sensor (left)*/
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN5);
    GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN5, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN5);
    GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN5);
}

void Motor_Config()
{
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

    /* Motor: Configuring Motor GPIO Pins*/
    Motor_Config();

    /* LED: Debugging purpose*/
    LED_Config();

    /* Line Sensor: Configuring Line Sensor pins*/
    LineSensor_Config();

    Initalise_HCSR04();

    /* Configuring Timer_A to have a period of approximately 8 micro sec and duty cycle of 50% of that (5000 ticks)  */
    Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
    Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);

    /* PID: Configuring Timer*/
    Timer_A_configureUpMode(TIMER_A1_BASE, &speed_timer);

    /* Configuring SysTick to trigger at 300000 (MCLK is 3MHz so this will make
        * it toggle every 0.1s) */
    SysTick_enableModule();
    SysTick_setPeriod(5000000);
    SysTick_enableInterrupt();

    /* Enabling interrupts */
    Interrupt_enableInterrupt(INT_TA1_0);
    // Interrupt_enableInterrupt(INT_PORT3);
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
    // Add Threshold of difference of 60 notches (3 wheels rotation) between right wheel and left wheel before triggering PID controller
    // if (notch_difference >= 100 || notch_difference <= -100 ){
    if (notchesdetected_left > 5 || notchesdetected_right > 5)
    {
        if (notchesdetected_left > notchesdetected_right)
        {
            left_wheel.dutyCycle += 100;
            GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        }

        if (notchesdetected_right > notchesdetected_left)
        {
            right_wheel.dutyCycle += 100;
            GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
        }

        //Keeps PWM speed under 5100
        if (right_wheel.dutyCycle >= 4900 || left_wheel.dutyCycle >= 4900)
        {
            right_wheel.dutyCycle -= 100;
            left_wheel.dutyCycle -= 100;
        }

        //Adjust wheel speed accordingly
        Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
        Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);

        //Reset Notches
        notchesdetected_left = 0;
        notchesdetected_right = 0;
    }

    //Resets Timer for 1 Second Interrupt
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
                                         TIMER_A_CAPTURECOMPARE_REGISTER_0);
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
    uint32_t status_5 = GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
    Interrupt_disableInterrupt(INT_TA1_0);
    Timer_A_stopTimer(TIMER_A1_BASE);

    /*When Line Sensor detects change from light to dark*/
    // Notice that Interrupt is not cleared till vehicle detects light
    if (status_5 & GPIO_PIN4)
    {
        Delay(3000);
        /*When Line Sensor detects dark*/
        if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN4) == 0)
        {

            left_wheel.dutyCycle = 4000;
            right_wheel.dutyCycle = 4700;
            Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
            Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
            Delay(10000);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
            Interrupt_enableInterrupt(INT_TA1_0);
            Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
            GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN4);
            notchesdetected_left = 0;
            notchesdetected_right = 0;
        }
        /*When Line Sensor detects light*/
        else if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN4) == 1)
        {
            left_wheel.dutyCycle = 100;
            right_wheel.dutyCycle = 2000;
            Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
            Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
            Interrupt_disableInterrupt(INT_PORT5);
            Delay(100000);
            Interrupt_enableInterrupt(INT_PORT5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
            notchesdetected_left = 0;
            notchesdetected_right = 0;
        }
    }

    /*When Line Sensor detects change from light to dark*/
    // Notice that Interrupt is not cleared till vehicle detects light
    if (status_5 & GPIO_PIN5)
    {
        Delay(3000);

        /*When Line Sensor detects dark*/
        if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN5) == 0)
        {
            //            left_wheel.dutyCycle = 4000;
            //            right_wheel.dutyCycle = 4700;
            left_wheel.dutyCycle = 4000;
            right_wheel.dutyCycle = 4700;
            Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
            Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
            Delay(10000);
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
            Interrupt_enableInterrupt(INT_TA1_0);
            GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN5);
            notchesdetected_left = 0;
            notchesdetected_right = 0;
        }
        /*When Line Sensor detects light*/
        else if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN5) == 1)
        {
            left_wheel.dutyCycle = 2000;
            right_wheel.dutyCycle = 100;
            Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
            Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
            Interrupt_disableInterrupt(INT_PORT5);
            Delay(100000);
            Interrupt_enableInterrupt(INT_PORT5);
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
            notchesdetected_left = 0;
            notchesdetected_right = 0;
        }
    }
}

void SysTick_Handler(void)
{
    float value = getHCSR04Distance();
    if (value < MIN_DISTANCE)
    {
        right_wheel.dutyCycle = 1000;
        left_wheel.dutyCycle = 1000;

        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        Interrupt_disableInterrupt(INT_TA1_0);

        //Adjust wheel speed accordingly
        Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
        Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
        Delay(10000);
    }
    else
        GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
}
