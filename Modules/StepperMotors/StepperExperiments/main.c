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
 * 	1 - STEP pin
 * 	2 - DIR pin
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

//Constants
#define PULSE_DELAY 1
#define PULSE_DELAY_MICROSEC 500 //microseconds 4%-4.8% error This number is not necesarily accurate.
#define PWM_FREQUENCY 50

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

//For dispenser arms
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint8_t ui8Adjust;
volatile uint8_t ui8AdjustLeft;

int main (void){
    //----MCU Initialization----
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //Set-up the clocking of the MCU to 40MHz
    //Enable port E peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //setDelay(2); //1ms delay
   // while( !(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) ); //In order to avoid potential FaultISR

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2); //Driver's STEP pin
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

//		if(pushFlag){
//			pushFlag = false;
//			//Change direction according to the button pushed
//			if(pushButton == 16){ //SW1 button
//				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 4); //Set DIR pin HIGH
//				setDelay(1);
//			}
//			else{ //SW2 button
//				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0); //Set STEP pin LOW
//				setDelay(1);
//			}
//		}

		//****Important! Take into consideration the DRV8825 Timing Diagram!****

//		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 2); //Set STEP pin HIGH
////		setDelay(PULSE_DELAY); //ms delay
//		setDelayMicro(PULSE_DELAY_MICROSEC); //us delay
//
//		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0); //Set STEP pin LOW
////		setDelay(PULSE_DELAY); //ms delay
//		setDelayMicro(PULSE_DELAY_MICROSEC); //us delay

		//------------TEST: performStep(direction) function test

//		if(!stopFlag){
//			//Perform many steps
//			int counter = 0;
//			for(; counter < 20; counter++){
//				performStep(false); //just in a single direction
//				setDelay(100); //in ms
//			}
//			stopFlag = true;
//		}

		//-----------------LIMIT SWITCH CODE----------------------
		//Limit switch code - Just change direction
//		if(pushButton == 32){ //Limit switch
//			pushButton = -1;
//			toggleVar = !toggleVar;
//			if(toggleVar)
//				//PWMGenEnable(PWM1_BASE, PWM_GEN_0);
//			else
//				//PWMGenDisable(PWM1_BASE, PWM_GEN_0);
//		}
		//--------------------------------------------------------------------

	//-----------------ROTATE BY PUSH BUTTON PROGRAM----------------------
		//Left and right push buttons are used to turn motor clockWise or coutnerClockwise respectively
//		if(pushFlag){
//			pushFlag = false;
//			//Change direction according to the button pushed
//			if(pushButton == 16){ //SW1 button
//				int counter = 0;
//				for(; counter < 300; counter++){
//					performStep(false); //just in a single direction
//					setDelay(1); //in ms
//				}
//			}
//			else{ //SW2 button
//				int counter = 0;
//				for(; counter < 300; counter++){
//					performStep(true); //just in a single direction
//					setDelay(1); //in ms
//				}
//			}
//		}

	//-----------------RUN STEPPERS WITH LIMIT SWITCH-----------------
		//Left and right push buttons are used to ______________________________
		runMotor = false;
		if(pushFlag){
			pushFlag = false;
			//Change direction according to the button pushed
			if(pushButton == 16){ //SW1 button
				pushButton = -1;
				runMotor = true;
				runDirection = false;
			}
			else if(pushButton == 1){ //SW2 button
				pushButton = -1;
				runMotor = true;
				runDirection = true;
			}
		}


		while(runMotor){
			//Limit switch code
			if(pushFlag && pushButton == 32){ //Limit switch
				pushButton = -1;
				pushFlag = false;
				runMotor = false;

				//Opening & closing dispenser arms
				openArms();
				setDelay(1500);
				closeArms();
			}

			performStep(runDirection);
			//setDelay(30);
		}

	//----------------------------------------------------------------


	} //End while
}

//Interrupt Service Routines

void PUSH_ISR(){
	setDelay(30); //For debouncing
	pushFlag = true;
	pushButton = GPIOIntStatus(GPIO_PORTF_BASE,true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
}

void LIMIT_SWITCH_ISR(){
	setDelay(30); //For debouncing
	pushFlag = true;
	pushButton = GPIOIntStatus(GPIO_PORTA_BASE, true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_5);
}

void LIMIT_SWITCH_DISPENSER_TOP_ISR(){

}

void LIMIT_SWITCH_STACKH_BOTTOM_ISR(){

}

void LIMIT_SWITCH_STACKH_TOP_ISR(){

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
