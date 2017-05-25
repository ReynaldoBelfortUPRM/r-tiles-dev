/*
 * main.c
 * TIVA MCU software for experimenting with the DRV8825 stepper motor driver
 *
 * Station 13
 * Lab Team:
 * Emmanuel Ramos
 * Reynaldo Belfort
 */

/* MCU Pin conecction information:
 * Port E:
 * Movment System
 * 	1 - STEP pin
 * 	2 - DIR pin
 * Dispenser
 * 	3 - STEP
 * 	4 - DIR pin
 * Stack Holder
 *
 * Port A:
 * 	Limit switches:
 * 	2-  Dispenser BOTTOM
 * 	3-  Dispenser TOP
 *
 * Port F:
 * 	0 - SW2 Launchpad Pushbutton
 * 	4 - SW1 Launchpad Pushbutton
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
#include "driverlib/pwm.h"

//Custom libraries
#include "tivaUtils.h"
#include "stepperSWDriver.h"

//Constant definitions
#define PULSE_DELAY 1
#define PULSE_DELAY_MICROSEC 500 //microseconds 4%-4.8% error This number is not necesarily accurate.
#define PWM_FREQUENCY 50

#define DISPENSER_PIN_PARAMS GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_2
#define MOV_SYSTEM_PIN_PARAMS GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_5

//Function declarations
void PUSH_ISR();
void LIMIT_SWITCH_DISPENSER_BOTTOM_ISR();
void LIMIT_SWITCH_DISPENSER_TOP_ISR();
void LIMIT_SWITCH_STACKH_BOTTOM_ISR();
void LIMIT_SWITCH_STACKH_TOP_ISR();
void openArms();
void closeArms();

//Global Variables
short pushButton = 0;
bool pushFlag = false;
bool stopFlag = false;

bool DISPENSER_ACTIVE = false;
bool DISPENSER_STATE = 0;

//For dispenser arms
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint8_t ui8Adjust;
volatile uint8_t ui8AdjustLeft;

int main (void){

    //----MCU Initialization----
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //Set-up the clocking of the MCU to 40MHz
    //Enable port peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //setDelay(2); //1ms delay
    //while( !(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) ); //In order to avoid potential FaultISR

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2); //Driver's STEP pin
    //--------------------------

    //---------Set up Port F push button---------
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    //-------------------------------------------

    //--------Interrupt configuration---------
    GPIOIntRegister(GPIO_PORTF_BASE, PUSH_ISR);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_FALLING_EDGE);  // Set PB2/3 to falling edge
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);

    //----Dispenser - BOTTOM----
    //Register Limit Switch Pin
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntRegister(GPIO_PORTA_BASE, LIMIT_SWITCH_DISPENSER_BOTTOM_ISR);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2);

    //----Dispenser - TOP----
    //Register Limit Switch Pin
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntRegister(GPIO_PORTA_BASE, LIMIT_SWITCH_DISPENSER_TOP_ISR);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_3);
    //--------------------------------


    //------------------Dispenser Arm Servo initialization-----------------
       ui8Adjust = 42;
       ui8AdjustLeft = 120;
       SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
       GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
       GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);

       GPIOPinConfigure(GPIO_PD0_M1PWM0);
       GPIOPinConfigure(GPIO_PD1_M1PWM1);

       ui32PWMClock = SysCtlClockGet() / 64;
       ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
       PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
       PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
       PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
       PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8AdjustLeft* ui32Load / 1000);
       PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
       PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);

       PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    //---------------------------------------------------------------------

    //Temporary variables
    bool runMotor = false;
    bool runDirection = false; //FALSE = downward , TRUE = upward


	while(1){

		//-----------------DISTANCE SPINNING PROGRAM----------------------
//		if(pushFlag){
//			pushFlag = false;
//			spinStepperInches(13.35176878, true);
//			setDelay(10); //ms
//		}

		//-----------CYCLE STEP 1 -Move robot to next tile position-------------

		//----------CYCLE STEP 2 -Move tile from StackHolder to Dispenser-------
//		STACK_STATE = 0; //TODO Maybe we don't need this
//		STACKHOLDER_ACTIVE = true;
//
//		while(STACKHOLDER_ACTIVE){
//			switch(STACK_STATE){
//				case 0:
//					//Mantain moving Stack Holder upward
//					performStep(true); //TODO We have to set what is thw pin to be used first
//					setDelay(35);
//					break;
//				case 1:
//					if(!feederON)
//					{
//						//Turn On Feeder
//						setDelay(350);
//						//TODO Assign GPIO values
//					}
//					break;
//				case 2:
//					//Stop Stack Holder operation
//
//					//Turn OFF feeder
//					if(feederON)
//					{
//						setDelay(200);
//						//TODO Assign GPIO values
//					}
//					STACKHOLDER_ACTIVE = false;
//					break;
//			}
//		}

		if(pushFlag){
			pushFlag = false;
			//-------------CYCLE STEP 3 - Deposit tile to the floor-----------------
					DISPENSER_STATE = 0; //TODO Maybe we don't need this
					DISPENSER_ACTIVE = true;

					while(DISPENSER_ACTIVE){
						switch(DISPENSER_STATE){
							case 0:
								//Mantain moving Dispenser downward n
								//performStep(false, GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_2);
								performStepSpecific(false, DISPENSER_PIN_PARAMS);
								break;
							case 1:
								//Opening & closing dispenser arms
								openArms();
								setDelay(1500); //1.5 secs
								closeArms();
								DISPENSER_STATE = 2; //State transition
								break;
							case 2:
								//Mantain moving Dispenser upward
								performStepSpecific(true, DISPENSER_PIN_PARAMS);
								break;
							case 3:
								//Stop Dispenser operation
								DISPENSER_ACTIVE = false;
								break;
						}
					}
		}

	} //End while
}

//Interrupt Service Routines

void PUSH_ISR(){
	setDelay(30); //For debouncing
	pushFlag = true;
	pushButton = GPIOIntStatus(GPIO_PORTF_BASE, true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
}

void LIMIT_SWITCH_DISPENSER_BOTTOM_ISR(){
	setDelay(30); //For debouncing
//	pushFlag = true;
//	pushButton = GPIOIntStatus(GPIO_PORTA_BASE, true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2);

	DISPENSER_STATE = 1; //State transition - Perform open & close dispenser arms
}

void LIMIT_SWITCH_DISPENSER_TOP_ISR(){
	setDelay(30); //For debouncing
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_3);
	DISPENSER_STATE = 3; //State transition - Stop dispenser operation
}

void LIMIT_SWITCH_STACKH_BOTTOM_ISR(){
}

void LIMIT_SWITCH_STACKH_TOP_ISR(){
//	setDelay(30); //For debouncing
//	GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_7);
//	STACK_STATE = 2; //State transition - Stop stack holder operation
}

//Extra functions
void openArms(){

	//Move servos to one extreme position (at the same time)
	while(ui8Adjust < 119 && ui8AdjustLeft > 41)
	{
	 	ui8Adjust++;
		ui8AdjustLeft--;
		if (ui8Adjust > 120) { ui8Adjust = 120; }
		if (ui8AdjustLeft < 40) { ui8AdjustLeft = 40; }
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000); //Servo Motor 1
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8AdjustLeft * ui32Load / 1000); //Servo Motor 2
	}
}

void closeArms(){

	//Move servos to one extreme position (at the same time)
	while(ui8Adjust > 41 && ui8AdjustLeft < 119)
	{
		ui8Adjust--;
		ui8AdjustLeft++;
		if (ui8Adjust < 40) { ui8Adjust = 40; }
		if (ui8AdjustLeft > 120) { ui8AdjustLeft = 120; }
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000); //Servo Motor 1
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8AdjustLeft * ui32Load / 1000); //Servo Motor 2
	}
}
