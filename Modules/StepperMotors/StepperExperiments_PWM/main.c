/*
 * main.c
 * Experimenting with PWM and the DRV8825 driver
 *
 * Station 13
 * Lab Team:
 * Emmanuel Ramos
 * Reynaldo Belfort
 */

/* MCU Pin conecction information:
 * Port E: (some times not used)
 * 	1 - STEP pin
 * 	2 - DIR pin
 * Port F:
 * 	0 - SW2 Launchpad Pushbutton
 * 	4 - SW1 Launchpad Pushbutton
 * Port D:
 * 	0 - STEP pin
 *
 * */

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"

//Custom libraries
#include "tivaUtils.h"

//Constants
#define PULSE_DELAY 1
#define PULSE_DELAY_MICROSEC 500 //microseconds 4%-4.8% error This number is not necesarily accurate.
#define STEP_DUTY_CYCLE 0.5
#define STEPPER_SPEED_LIMIT_MAX 1510 //steps/sec
#define STEPPER_SPEED_LIMIT_MIN 0 //steps/sec
//#define SAMPLE_FREQ 50

//Function declarations
void PUSH_ISR();
void LIMIT_SWITCH_ISR();

//Global Variables
short pushButton = 0;
bool pushFlag = false;
bool toggleVar = false;
volatile int currentSpeed = 250; //Initialized to start speed (units: steps/sec or Hz) (20 herz is the minimum)
volatile uint32_t pwmLoadValue = 0;
volatile uint32_t pwmClockFreq = 0;

int main (void){
    //----MCU Initialization----
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //Set-up the clocking of the MCU to 40MHz
    //Enable port E peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //setDelay(2); //1ms delay
   // while( !(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) ); //In order to avoid potential FaultISR

    //GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2); //Driver's STEP pin
    //--------------------------

    //---------Set up Port F push button---------
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    //-------------------------------------------

    //--------Interrupt setup---------
    GPIOIntRegister(GPIO_PORTF_BASE, PUSH_ISR);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_FALLING_EDGE);  // Set PB2/3 to falling edge
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);

    //Register Limit Switch Pin
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntRegister(GPIO_PORTA_BASE, LIMIT_SWITCH_ISR);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_5);
    //--------------------------------

    //---------------PWM setup-------------
    //Divide system clock by 32 to run the PWM at 1.25MHz
    SysCtlPWMClockSet(SYSCTL_PWMDIV_32);
    //Enabling PWM1 module and Port D
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //Port where the PWM pin will be selected

	//Selecting PWM generator 0 and port D pin 0 (PD0) aa a PWM output pin for module one
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0); //Set Port D pin 0 as output //TODO Checkout which ports can be used for PWM functionallity
	GPIOPinConfigure(GPIO_PD0_M1PWM0); //Select PWM Generator 0 from PWM Module 1

	pwmClockFreq = SysCtlClockGet() / 32;
	//Setting initial stepper speed
	pwmLoadValue = (pwmClockFreq / currentSpeed) - 1; //Load Value = (PWMGeneratorClock * DesiredPWMPeriod) - 1

	//PWM Generator initialization
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN); //Set a count-down generator type
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, pwmLoadValue); //Set PWM period determined by the load value
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, pwmLoadValue * STEP_DUTY_CYCLE);
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
	//------------------------------------

	//Enable PWM Generator
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);

	while(1){

		if(pushFlag){
			pushFlag = false;
			//Limit switch code - Just change direction
			if(pushButton == 32){ //Limit switch
				pushButton = -1;
				toggleVar = !toggleVar;
				if(toggleVar)
					PWMGenEnable(PWM1_BASE, PWM_GEN_0);
				else
					PWMGenDisable(PWM1_BASE, PWM_GEN_0);
			}

			//Change direction according to the button pushed
			if(pushButton == 16){ //SW1 button

				//GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 4); //Set DIR pin HIGH

				//------Alternate program------
				//Turn OFF PWM generator
				//PWMGenDisable(PWM1_BASE, PWM_GEN_0);

//				//Decrease speed
//				if(currentSpeed - 150 > STEPPER_SPEED_LIMIT_MIN) {
//					currentSpeed = currentSpeed - 150;
//					//Change PWM Frequency by disabling the generator first
//					//Disable PWM Generator
//					PWMGenEnable(PWM1_BASE, PWM_GEN_0);
//					pwmLoadValue = (pwmClockFreq / currentSpeed) - 1;
//					PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, pwmLoadValue); //Set PWM period determined by the load value
//					//Enable PWM Generator
//					PWMGenEnable(PWM1_BASE, PWM_GEN_0);
//				}

				setDelay(1);
			}
			else{ //SW2 button
				//GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0); //Set DIR pin LOW

				//------Alternate program------
				//Turn ON PWM generator
//				PWMGenEnable(PWM1_BASE, PWM_GEN_0);

//				//Increase speed
//				if(currentSpeed + 150 < STEPPER_SPEED_LIMIT_MAX) {
//					currentSpeed = currentSpeed + 150;
//					//Disable PWM Generator
//					PWMGenEnable(PWM1_BASE, PWM_GEN_0);
//					pwmLoadValue = (pwmClockFreq / currentSpeed) - 1;
//					PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, pwmLoadValue); //Set PWM period determined by the load value
//					//Enable PWM Generator
//					PWMGenEnable(PWM1_BASE, PWM_GEN_0);
//				}

				setDelay(1);
			}
		}

		//****Important! Take into consideration the DRV8825 Timing Diagram!****

	} //End main loop
}

void PUSH_ISR(){
	setDelay(30); //For debouncing
	pushFlag = true;
	pushButton = GPIOIntStatus(GPIO_PORTF_BASE, true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
}

void LIMIT_SWITCH_ISR(){
	setDelay(30); //For debouncing
	pushFlag = true;
	pushButton = GPIOIntStatus(GPIO_PORTA_BASE, true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_5);
}