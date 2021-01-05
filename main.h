#ifndef MAIN_H_
#define MAIN_H_

#include "driverlib.h"

#define TIMER_A_PERIOD  100000 //T = 1/f = (TIMER_A_PERIOD * 1 us --> in microseconds)
#define HIGH_COUNT      15   //Number of cycles signal is high (Duty Cycle = HIGH_COUNT / TIMER_A_PERIOD) --- pulse width

//Output pin to Blue LED P1.3
#define LEDB_PORT       GPIO_PORT_P1
#define LEDB_PIN        GPIO_PIN3
//Output pin to Green LED P1.4
#define LEDG_PORT       GPIO_PORT_P1
#define LEDG_PIN        GPIO_PIN4
//Output pin to Yellow LED P1.5
#define LEDY_PORT       GPIO_PORT_P1
#define LEDY_PIN        GPIO_PIN5
//Output pin to Red LED P2.7
#define LEDR_PORT       GPIO_PORT_P2
#define LEDR_PIN        GPIO_PIN7
//Output pin to ultrasonic sensor
#define USTRIG_PORT     GPIO_PORT_P1
#define USTRIG_PIN      GPIO_PIN7
//Input pin to ultrasonic sensor
#define USECHO_PORT     GPIO_PORT_P2
#define USECHO_PIN      GPIO_PIN5
#define USECHO_VECTOR   PORT2_VECTOR

//LaunchPad LED1 - note unavailable if UART is used
#define LED1_PORT       GPIO_PORT_P1
#define LED1_PIN        GPIO_PIN0
//LaunchPad LED2
#define LED2_PORT       GPIO_PORT_P4
#define LED2_PIN        GPIO_PIN0
//LaunchPad Pushbutton Switch 1
#define SW1_PORT        GPIO_PORT_P1
#define SW1_PIN         GPIO_PIN2
//LaunchPad Pushbutton Switch 2
#define SW2_PORT        GPIO_PORT_P2
#define SW2_PIN         GPIO_PIN6
//Input to ADC - in this case input A9 maps to pin P8.1
#define ADC_IN_PORT     GPIO_PORT_P8
#define ADC_IN_PIN      GPIO_PIN1
#define ADC_IN_CHANNEL  ADC_INPUT_A9

void Init_GPIO(void);
void Init_Clock(void);
void Init_UART(void);
void Init_US(void);
void Init_ADC(void);

Timer_A_outputPWMParam param1; //Timer configuration data structure for PWM
Timer_A_initUpModeParam param2; //Timer configuration data structure for up mode

#endif /* MAIN_H_ */
