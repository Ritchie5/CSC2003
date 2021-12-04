/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* Ultra Sonic  */
#define MIN_DISTANCE 60.0f
#define TICKPERIOD 1000

uint32_t SR04IntTimes;

/* PID */
#define TIMER_PERIOD 0x5BAE //0x5BAE for one second
#define Wheelcircumference 20.42035

/* PID: Wheel Encoder Variable */
volatile uint32_t notchesdetected_left;
volatile uint32_t notchesdetected_right;

volatile uint32_t car_position;
volatile uint32_t ram;

void uPrintf(unsigned char *TxArray);

/* Motor: Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig left_wheel =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_12,
        10000,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        4200};

/* Motor: Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig right_wheel =
    {
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_12,
        10000,
        TIMER_A_CAPTURECOMPARE_REGISTER_3,
        TIMER_A_OUTPUTMODE_RESET_SET,
        4200};

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

//![Simple UART Config]
/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides at:
 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const eUSCI_UART_ConfigV1 uartConfig =
    {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,                // SMCLK Clock Source
        78,                                            // BRDIV = 78
        2,                                             // UCxBRF = 2
        0,                                             // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                        // NO Parity
        EUSCI_A_UART_LSB_FIRST,                        // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,                     // One stop bit
        EUSCI_A_UART_MODE,                             // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION, // Oversampling
        EUSCI_A_UART_8_BIT_LEN                         // 8 bit data length
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

void UART_Config()
{
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    /* Configuring UART Module */
    UART_initModule(EUSCI_A0_BASE, &uartConfig);

    /* Enable UART module */
    UART_enableModule(EUSCI_A0_BASE);

    /* Enabling interrupts (Rx) */
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA0);

    uPrintf("\r\n");
}

int main(void)
{
    car_position = 1;
    ram = 1;

    /* Halting the watchdog */
    MAP_WDT_A_holdTimer();

    /* PID: Configure PID(Wheel Encoder) controller GPIO Pins */
    PID_Config();

    /* Motor: Configuring Motor GPIO Pins */
    Motor_Config();

    /* LED: Debugging purpose */
    LED_Config();

    /* Line Sensor: Configuring Line Sensor pins */
    LineSensor_Config();

    Initalise_HCSR04();

    /* Configuring Timer_A to have a period of approximately 8 micro sec and duty cycle of 50% of that (5000 ticks)  */
    Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
    Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);

    /* PID: Configuring Timer*/
    Timer_A_configureUpMode(TIMER_A1_BASE, &speed_timer);

    /* UART */
    UART_Config();

    /* Configuring SysTick to trigger at 300000 (MCLK is 3MHz so this will make
        * it toggle every 0.1s) */
    SysTick_enableModule();
    SysTick_setPeriod(8000000);
    SysTick_enableInterrupt();

    /* Enabling interrupts */
    //    Interrupt_enableInterrupt(INT_TA1_0); //PID controller interrupts
    //    Interrupt_enableInterrupt(INT_PORT4); // Wheel Encoder interrupts

    //    Interrupt_enableInterrupt(INT_PORT5); // Line sensor interrupts

    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableMaster();

    /* Sleeping when not in use */
    while (1)
    {
        PCM_gotoLPM3InterruptSafe();
    }
}

void uPrintf(unsigned char *TxArray)
{
    unsigned short i = 0;
    while (*(TxArray + i))
    {
        UART_transmitData(EUSCI_A0_BASE, *(TxArray + i)); // Write the character at the location specified by pointer
        i++;                                              // Increment pointer to point to the next character
    }
}

/* EUSCI A0 UART ISR */
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
    unsigned char a;

    a = UART_receiveData(EUSCI_A0_BASE);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        if (a == 'a')
        {
            ram = 0;
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
        }

        if (a == 'b')
        {
            ram = 1;
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
        }

        UART_transmitData(EUSCI_A0_BASE, UART_receiveData(EUSCI_A0_BASE));
    }

    UART_clearInterruptFlag(EUSCI_A0_BASE, status);
}

//PID: Runs every one second to maintain straightness
void TA1_0_IRQHandler(void)
{
    // Add Threshold of difference of 60 notches (3 wheels rotation) between right wheel and left wheel before triggering PID controller
    // if (notch_difference >= 100 || notch_difference <= -100 ){

    if (notchesdetected_left > 20 || notchesdetected_right > 20)
    {
        if (notchesdetected_left > notchesdetected_right)
        {
            right_wheel.dutyCycle += 100;
            GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
            //            printf("right more");
            //            printf("\n%d", left_wheel.dutyCycle);
            //            printf("\n%d", right_wheel.dutyCycle);
        }

        if (notchesdetected_right > notchesdetected_left)
        {
            left_wheel.dutyCycle += 100;
            GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
            //            printf("left more");
            //            printf("\n%d", left_wheel.dutyCycle);
            //            printf("\n%d", right_wheel.dutyCycle);
        }

        //Keeps PWM speed under 5100
        if (right_wheel.dutyCycle >= 4100 || left_wheel.dutyCycle >= 4100)
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
    Timer_A_stopTimer(TIMER_A1_BASE);
    SysTick_disableModule();

    /*When Line Sensor detects change from light to dark*/
    // Notice that Interrupt is not cleared till vehicle detects light
    if (status_5 & GPIO_PIN4)
    {
        /*When Line Sensor detects dark*/
        if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN4) == 0)
        {

            left_wheel.dutyCycle = 4200;
            right_wheel.dutyCycle = 4200;
            Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
            Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

            Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
            notchesdetected_left = 0;
            notchesdetected_right = 0;
            SysTick_enableModule();

            GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN4);
        }
        /*When Line Sensor detects light*/
        else if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN4) == 1)
        {
            if (right_wheel.dutyCycle != 0)
            {
                left_wheel.dutyCycle = 4200;
                right_wheel.dutyCycle = 0;
                Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
                Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
                GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
            }
        }
    }

    /*When Line Sensor detects change from light to dark*/
    // Notice that Interrupt is not cleared till vehicle detects light
    if (status_5 & GPIO_PIN5)
    {
        /*When Line Sensor detects dark*/
        if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN5) == 0)
        {
            left_wheel.dutyCycle = 4200;
            right_wheel.dutyCycle = 4200;
            Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
            Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);

            Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
            notchesdetected_left = 0;
            notchesdetected_right = 0;
            SysTick_enableModule();

            GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN5);
        }
        /*When Line Sensor detects light*/
        else if (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN5) == 1)
        {
            if (left_wheel.dutyCycle != 0)
            {
                left_wheel.dutyCycle = 0;
                right_wheel.dutyCycle = 4200;
                Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
                Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
                GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
            }
        }
    }
}

void SysTick_Handler(void)
{
    Timer_A_stopTimer(TIMER_A1_BASE);
    Interrupt_disableInterrupt(INT_PORT5);

    float value = getHCSR04Distance();
    printf("\n%f", value);

    //char str[] = "AI has detected";

    if (value < MIN_DISTANCE && value > 20)
    {
        if (ram == 1)
        {
        }

        if (ram == 1) // Supposed to be 0
        {
            if (car_position == 1)
            {
                //printf("%s", str);
                right_wheel.dutyCycle += 2000;
                GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
                printf("Move right");
            }
            if (car_position == 0)
            {
                left_wheel.dutyCycle += 2000;
                GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
                printf("Move Left");
            }
            if (car_position == 1)
            {
                car_position = 0;
            }
            else if (car_position == 0)
            {
                car_position = 1;
            }

            //Adjust wheel speed accordingly
            Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
            Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
            SysTick_disableModule();
        }
        //GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
    }
    else if (value < 20)
    {
        left_wheel.dutyCycle = 0;
        right_wheel.dutyCycle = 0;
        Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
        Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
    }

    else
    {
        Timer_A_generatePWM(TIMER_A0_BASE, &right_wheel);
        Timer_A_generatePWM(TIMER_A0_BASE, &left_wheel);
        //        GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN2);
    }

    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    Interrupt_enableInterrupt(INT_PORT5);
    //    notchesdetected_left = 0;
    //    notchesdetected_right = 0;
}
