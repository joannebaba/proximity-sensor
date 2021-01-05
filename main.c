#include "main.h"
#include "driverlib.h"
#include "hal_LCD.h"
#include "timer_a.h"

/*
 *
 * UART: It configures P1.0 and P1.1 to be connected internally to the
 * eSCSI module, which is a serial communications module, and places it
 * in UART mode. This let's you communicate with the PC via a software
 * COM port over the USB cable. You can use a console program, like PuTTY,
 * to type to your LaunchPad. The code in this sample just echos back
 * whatever character was received.
 *
 * ADC:
 *
 * PWM:
 */

char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result
int isRunning = 0; //check if echo interrupt is running
int time = 0; //pulse width for echo in us
int range = 0; // distance from ultrasound sensor

void main(void)
{
    char buttonState = 0; //Current button press state (to allow edge detection)

    /*
     * Functions with two underscores in front are called compiler intrinsics.
     * They are documented in the compiler user guide, not the IDE or MCU guides.
     * They are a shortcut to insert some assembly code that is not really
     * expressible in plain C/C++. Google "MSP430 Optimizing C/C++ Compiler
     * v18.12.0.LTS" and search for the word "intrinsic" if you want to know
     * more.
     * */

    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    Init_US();      //Sets up ultrasound input and output
    Init_ADC();     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display

     /*
     * The MSP430 MCUs have a variety of low power modes. They can be almost
     * completely off and turn back on only when an interrupt occurs. You can
     * look up the power modes in the Family User Guide under the Power Management
     * Module (PMM) section. You can see the available API calls in the DriverLib
     * user guide, or see "pmm.h" in the driverlib directory. Unless you
     * purposefully want to play with the power modes, just leave this command in.
     */
    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.
    __enable_interrupt();

    displayScrollText("ECE 298");

    while(1) //Do this when you want an infinite loop of code
    {
        //Buttons SW1 and SW2 are active low (1 until pressed, then 0)
        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 1) & (buttonState == 0)) //Look for rising edge
        {
            Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal
            buttonState = 1;                //Capture new button state
        }
        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0) & (buttonState == 1)) //Look for falling edge
        {
            Timer_A_outputPWM(TIMER_A0_BASE, &param1);   //Turn on PWM
            buttonState = 0;                            //Capture new button state
        }

        //Start an ADC conversion (if it's not busy) in Single-Channel, Single Conversion Mode
        if (ADCState == 0)
        {
            showHex((int)ADCResult); //Put the previous result on the LCD display
            ADCState = 1; //Set flag to indicate ADC is busy - ADC ISR (interrupt) will clear it
            ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
        }

        range = 0;

        if (time > 0) {
            range = time / 58.0; // calculates distance in cm

            //turn on LEDs depending on range
            if (range <= 100) {
                if (range > 75) {
                    //turn on blue LED
                    GPIO_setOutputHighOnPin(LEDB_PORT, LEDB_PIN);
                    //turn off other LEDs
                    GPIO_setOutputLowOnPin(LEDG_PORT, LEDG_PIN);
                    GPIO_setOutputLowOnPin(LEDY_PORT, LEDY_PIN);
                    GPIO_setOutputLowOnPin(LEDR_PORT, LEDR_PIN);
                }

                if (range > 50 && range <= 75) {
                    //turn on blue LED
                    GPIO_setOutputHighOnPin(LEDB_PORT, LEDB_PIN);
                    //turn on green LED
                    GPIO_setOutputHighOnPin(LEDG_PORT, LEDG_PIN);
                    //turn off other LEDs
                    GPIO_setOutputLowOnPin(LEDY_PORT, LEDY_PIN);
                    GPIO_setOutputLowOnPin(LEDR_PORT, LEDR_PIN);
                }

                if (range > 25 && range <= 50) {
                    //turn on blue LED
                    GPIO_setOutputHighOnPin(LEDB_PORT, LEDB_PIN);
                    //turn on green LED
                    GPIO_setOutputHighOnPin(LEDG_PORT, LEDG_PIN);
                    //turn on yellow LED
                    GPIO_setOutputHighOnPin(LEDY_PORT, LEDY_PIN);
                    //turn off other LEDs
                    GPIO_setOutputLowOnPin(LEDR_PORT, LEDR_PIN);
                }

                if (range > 0 && range <= 25) {
                    //turn on blue LED
                    GPIO_setOutputHighOnPin(LEDB_PORT, LEDB_PIN);
                    //turn on green LED
                    GPIO_setOutputHighOnPin(LEDG_PORT, LEDG_PIN);
                    //turn on yellow LED
                    GPIO_setOutputHighOnPin(LEDY_PORT, LEDY_PIN);
                    //turn on red LED
                    GPIO_setOutputHighOnPin(LEDR_PORT, LEDR_PIN);
                }
            } else {
                //turn off LEDs
                GPIO_setOutputLowOnPin(LEDB_PORT, LEDB_PIN);
                GPIO_setOutputLowOnPin(LEDG_PORT, LEDG_PIN);
                GPIO_setOutputLowOnPin(LEDY_PORT, LEDY_PIN);
                GPIO_setOutputLowOnPin(LEDR_PORT, LEDR_PIN);
            }
        } else {
            time = 0;
            range = 0;

            //turn off LEDs
            GPIO_setOutputLowOnPin(LEDB_PORT, LEDB_PIN);
            GPIO_setOutputLowOnPin(LEDG_PORT, LEDG_PIN);
            GPIO_setOutputLowOnPin(LEDY_PORT, LEDY_PIN);
            GPIO_setOutputLowOnPin(LEDR_PORT, LEDR_PIN);
        }
    }

    /*
     * You can use the following code if you really do plan on only using interrupts
     * to handle all your system events since you don't need any infinite loop of code.
     *
     * //Enter LPM0 - interrupts only
     * __bis_SR_register(LPM0_bits);
     * //For debugger to let it know that you meant for there to be no more code
     * __no_operation();
    */
}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param1 = {0};
        param1.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param1.clockPrescalar    = 6;
        param1.firstModReg       = 8;
        param1.secondModReg      = 17;
        param1.parity            = EUSCI_A_UART_NO_PARITY;
        param1.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param1.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param1.uartMode          = EUSCI_A_UART_MODE;
        param1.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param1))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }
}

/* Ultrasound Initialization */
void Init_US(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param1.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param1.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param1.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param1.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param1.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param1.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //Timer runs in up mode
    param2.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param2.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param2.timerPeriod           = 0xFFFF; //max value since not comparing
    param2.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    param2.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
    param2.timerClear            = TIMER_A_DO_CLEAR; //Defined in main.h
    param2.startTimer            = 0;

    //Configure Timer_A1 module
    Timer_A_initUpMode(TIMER_A1_BASE, &param2);

    //USTRIG (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(USTRIG_PORT, USTRIG_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Ultrasonic distance sensor code
    //2 cm - 400 cm measurement range
    //Requires 10 microsecond pulse -> adjusted to 15 us pulse width (HIGH_COUNT)

    //Set P2.5 as input from echo pin from ultrasonic sensor
    GPIO_setAsPeripheralModuleFunctionInputPin(USECHO_PORT, USECHO_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Interrupt enabled
    GPIO_enableInterrupt(USECHO_PORT, USECHO_PIN);
    //Edge detection
    GPIO_selectInterruptEdge(USECHO_PORT, USECHO_PIN, GPIO_LOW_TO_HIGH_TRANSITION);
    //IFG cleared
    GPIO_clearInterrupt(USECHO_PORT, USECHO_PIN);
}

/* GPIO ISR - Controls echo pulse measurement */
#pragma vector=USECHO_VECTOR
__interrupt
void GPIO_ISR(void)
{
    uint16_t echoStatus = GPIO_getInterruptStatus(USECHO_PORT, USECHO_PIN);

    GPIO_clearInterrupt(USECHO_PORT, USECHO_PIN);

    //count pulse width
    if (echoStatus & USECHO_PIN)
    {
        int edge = GPIO_getInputPinValue(USECHO_PORT, USECHO_PIN);

        if (edge == 1) {
            //Turn on Timer_A1
            Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);

            //Set falling edge detect
            GPIO_selectInterruptEdge(USECHO_PORT, USECHO_PIN, GPIO_HIGH_TO_LOW_TRANSITION);

            isRunning = 1;
            time = 0;
        } else {
            //Stop Timer A1
            Timer_A_stop(TIMER_A1_BASE);

            //Get pulse width
            time = Timer_A_getCounterValue(TIMER_A1_BASE);

            //Set rising edge detect
            GPIO_selectInterruptEdge(USECHO_PORT, USECHO_PIN, GPIO_LOW_TO_HIGH_TRANSITION);

            isRunning = 0;
        }
    }
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}
