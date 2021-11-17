/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdio.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* PID */
#define TIMER_PERIOD 0x5BAE
#define Wheelcircumference 20.42035

/* Ultrasonic */
#define MIN_DISTANCE 10.0f
#define TICKPERIOD 1000

/* PID: Wheel Encoder Variable */
uint32_t notchesdetected_left;
uint32_t notchesdetected_right;

/* Ultrasonic variable */
uint32_t SR04IntTimes;

static void Delay(uint32_t loop)
{
    volatile uint32_t i;

    for (i = 0; i < loop; i++)
        ;
}

/* Motor: Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig left_wheel =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_12,
        10000,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        5500};

/* Motor: Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig right_wheel =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_12,
        10000,
        TIMER_A_CAPTURECOMPARE_REGISTER_3,
        TIMER_A_OUTPUTMODE_RESET_SET,
        5000};

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

void Ultrasonic_Config(void)
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

/* Ultrasonic */
void TA0_0_IRQHandler(void)
{
    /* Increment global variable (count number of interrupt occurred) */
    SR04IntTimes++;

    /* Clear interrupt flag */
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

/* Ultrasonic: Get the pulse time */
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

/* Ultrasonic: Get the distance */
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

    /* Ultrasonic: Configuring Ultrasonic sensor GPIO Pins*/
    Ultrasonic_Config();

    /* LED: Debugging purpose*/
    LED_Config();

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

    /* Systick configuration to check on distance calculated by ultrasonic sensor */
    SysTick_enableModule();
    SysTick_setPeriod(300000); //check every 0.1seconds
    SysTick_enableInterrupt();

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
    float notch_difference;
    // notch_difference = notchesdetected_left - notchesdetected_right;
    // Add Threshold of difference of 60 notches (3 wheels rotation) between right wheel and left wheel before triggering PID controller
    // if (notch_difference >= 100 || notch_difference <= -100 ){
    if (notchesdetected_left > 1 && notchesdetected_right > 1)
    {
        if (notchesdetected_left > notchesdetected_right)
        {
            right_wheel.dutyCycle += 300;
            GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        }

        if (notchesdetected_right > notchesdetected_left)
        {
            left_wheel.dutyCycle += 300;
            GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
        }

        //Keeps PWM speed under 5100
        if (right_wheel.dutyCycle >= 5100 || left_wheel.dutyCycle >= 5100)
        {
            right_wheel.dutyCycle -= 300;
            left_wheel.dutyCycle -= 300;
        }

        //Adjust wheel speed accordingly
        Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
        Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);

        //Reset Notches
        notchesdetected_left = 0;
        notchesdetected_right = 0;
    }

    //Resets Timer for 1 Second Interrupt
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
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
    uint32_t status_5 = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);

    /*When Line Sensor detects change from light to dark*/
    // Notice that Interrupt is not cleared till vehicle detects light
    if (status_5 & GPIO_PIN4)
    {
        /*When Line Sensor detects dark*/
        if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN4) == 1)
        {
            left_wheel.dutyCycle = 0;
            right_wheel.dutyCycle = 1000;
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
        }
        /*When Line Sensor detects light*/
        else
        {
            left_wheel.dutyCycle = 5000;
            right_wheel.dutyCycle = 5000;
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
            GPIO_clearInterruptFlag(GPIO_PORT_P5, status_5);
        }
        Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
        Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
    }

    /*When Line Sensor detects change from light to dark*/
    // Notice that Interrupt is not cleared till vehicle detects light
    if (status_5 & GPIO_PIN5)
    {
        /*When Line Sensor detects dark*/
        if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN5) == 1)
        {
            left_wheel.dutyCycle = 1000;
            right_wheel.dutyCycle = 0;
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
        }
        /*When Line Sensor detects light*/
        else
        {
            left_wheel.dutyCycle = 5000;
            right_wheel.dutyCycle = 5000;
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
            GPIO_clearInterruptFlag(GPIO_PORT_P5, status_5);
        }
        Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
        Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
    }
}

/* Get the distance periodically */
void SysTick_Handler(void)
{

    float value = getHCSR04Distance();
    printf("\n%f", value);
    if ((getHCSR04Distance() < MIN_DISTANCE)){
        left_wheel.dutyCycle = 0;
        right_wheel.dutyCycle = 0;
        Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
        Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0); /*TODO: stop the car or ....*/
    }
    else 
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); /*TODO: stop the car or ....*/
}
