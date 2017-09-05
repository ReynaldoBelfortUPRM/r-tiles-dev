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

//Global Variables
short pushButton = 0;
short state = 0;
bool pushFlag = false;

//--------Stepper motor global variables-------
bool runMotor = true;
bool runDirectionStack = true; //FALSE = upward , TRUE = downward
bool stackUp1 = false;
bool stackDown = false;
short switchButton = 0;
//short currentState = 0;
short safety = 1;
short currentState = 0;
int stepsPerTile[3] = {0,0,235};
int currentTiles = 3;
//---------------------------------------------

/* Stack Stepper motor Constants */
#define PULSE_DELAY 1
#define PULSE_DELAY_MICROSEC 500 //microseconds 4%-4.8% error This number is not necesarily accurate.
#define DRIVER_STACK_ELE GPIO_PORTE_BASE
#define STEP_STACK_ELE GPIO_PIN_3
#define DIRECTION_STACK_ELE GPIO_PIN_4
#define CLOCK_FREQ 40
#define STEPSTACK 1000

/* Feeder Stepper Motor Constants */
#define DRIVER_FEEDER GPIO_PORTC_BASE		/* Feeder base port */
#define STEP_FEEDER GPIO_PIN_6				/* Step pin feeder */
#define DIRECTION_FEEDER GPIO_PIN_7			/* Direction pin feeder */
//TODO Anadir constantes de steps por losa y steps dependiendo cuantas tiene en el stack

/* General Constants */
#define ALLPINS GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7

/* ISR */
void PUSH_ISR();
void LIMIT_SWITCH_STACK();

/* Stepper motor functions */
void performStepStack(bool clockwise);
void performStepFeeder(bool clockwise);
int computeStepsPerTiles(int number);
/* Delay Functions */
void setDelayMicro(float microSecWaitTime);
void setDelay(uint32_t waitTime);
uint32_t computeDelayCount(uint32_t waitTime, uint8_t clockFreq);
uint32_t computeDelayCountMicrosec(float waitTime, uint8_t clockFreq);

void main(void)
{
	//----MCU Initialization----
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //Set-up the clocking of the MCU to 40MHz

	//----Port Enable----------
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);		//Switches
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); 		//Driver Step's'
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); 		//Driver Step's'
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);		//Tiva buttons
	//--------------------------

	//------Stack Stepper Motor Pin Configuration-------
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_4); //Driver's STEP pin
	//-------------------------------------------
	//------ Feeder Stepper Motor Pin Configuration ----
	GPIOPinTypeGPIOOutput(DRIVER_FEEDER, STEP_FEEDER | DIRECTION_FEEDER); //Feeder Driver's STEP pin
	//--------------------------------------------------

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
	//---------------------------------

	//----------Register Limit Switch Pin--------------
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntRegister(GPIO_PORTA_BASE, LIMIT_SWITCH_STACK);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_FALLING_EDGE);
	//--------------------------------

	while(1){
		while(currentState == 0){
			GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
			if(pushFlag){
				pushFlag = false;
				//If button was pushed start moving motor
				if(pushButton == 16){ //SW1 button
					currentState = 1;
					//GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
				}
			}
		}

		if(currentState == 1){
			//GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3);
			GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2);
			while(currentState ==1){
				//if(stackUp1 == false){
				runMotor = true;
				while(runMotor == true){
					performStepStack(runDirectionStack); 		//just in a single direction
					setDelay(1); //in ms
				}
				//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x80);
				GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_2);

				int counter = 0;
				runMotor = true;
				for(; counter < STEPSTACK; counter++){
					if(runMotor){
						performStepStack(runDirectionStack); //just in a single direction
						setDelay(1); //in ms
					}else{
						counter = STEPSTACK;
					}
				}
				int counter1 = 0;
				runMotor = true;
				for(; counter1 < 235; counter1++){
					performStepStack(runDirectionStack); //just in a single direction
					setDelay(1); //in ms
				}
				currentState = 2;
				//}
			}
		}

		while(currentState == 2){
			if(pushFlag){
				SysCtlDelay(4000000); //For debouncing
				pushFlag = false;
				//If button was pushed start moving motor
				if(pushButton == 16){ //SW1 button
					while(true){
						performStepFeeder(false); //just in a single direction
						setDelay(1); //in ms
					}
					//safety = 0;
				}
			}
		}

		//		while(currentState == 2){
		//			if(safety !=0){
		//				//GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
		//				if(pushFlag){
		//					pushFlag = false;
		//					//If button was pushed start moving motor
		//					if(pushButton == 16){ //SW1 button
		//						int counter = 0;
		//						runMotor = true;
		//						for(; counter < 1100; counter++){
		//							if(runMotor){
		//								performStepStack(runDirectionStack); //just in a single direction
		//								setDelay(1); //in ms
		//							}else{
		//								counter = 1100;
		//							}
		//						}
		//						safety = 0;
		//					}
		//					int counter1 = 0;
		//					runMotor = true;
		//					for(; counter1 < 1200; counter1++){
		//						if(runMotor){
		//							performStepFeeder(false); //just in a single direction
		//							setDelay(1); //in ms
		//						}else{
		//							counter1 = 1200;
		//						}
		//					}
		//					if(stackDown){
		//						stackDown = false;
		//					}
		//				}
		//			}
		//		}
	}
}
//-----------------------Interrupt Service Routines-----------------------
void PUSH_ISR(){
	SysCtlDelay(4000000); //For debouncing
	pushFlag = true;
	pushButton = GPIOIntStatus(GPIO_PORTF_BASE,true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
}

void LIMIT_SWITCH_STACK(){
	SysCtlDelay(4000000); //For debouncing
	switchButton = GPIOIntStatus(GPIO_PORTA_BASE, true); //To read which button interrupted
	if(switchButton == 0x04){ //Stack abajo
		runDirectionStack = false;
		stackDown = true;
		runMotor = false;
		//GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_2);
		//GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_3);
	}
	//	else if(switchButton == 0x08){ //stack arriba //ya no existe
	//		runDirectionStack = true;
	//		stackUp = true;
	//		GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_3);
	//		GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2);
	//	}
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3);
}
//-------------------------------------------------------------------------
//--------------Stepper Motor Functions-----------------------------------
//Send a pulse to the STEP pin
void performStepStack(bool clockwise){ //

	//TODO Having thess if-clauses may cause problems. We constantly keep setting the DIR pin
	if(clockwise == true){ //Clockwise direction
		GPIOPinWrite(DRIVER_STACK_ELE, DIRECTION_STACK_ELE, 16); //Set DIR pin HIGH
		setDelayMicro(PULSE_DELAY_MICROSEC);
	}
	else{
		GPIOPinWrite(DRIVER_STACK_ELE, DIRECTION_STACK_ELE, 0);  //Set DIR pin HIGH
		setDelayMicro(PULSE_DELAY_MICROSEC);
	}

	//****Important! Take into consideration the DRV8825 Timing Diagram!****
	GPIOPinWrite(DRIVER_STACK_ELE, STEP_STACK_ELE, 8); //Set STEP pin HIGH
	setDelayMicro(PULSE_DELAY_MICROSEC); //us delay to comply with DRV8825 timing diagram
	GPIOPinWrite(DRIVER_STACK_ELE, STEP_STACK_ELE, 0); //Set STEP pin LOW
	setDelayMicro(PULSE_DELAY_MICROSEC);//us delay to comply with DRV8825 timing diagram
}

void performStepFeeder(bool clockwise){ //


	//TODO Having thess if-clauses may cause problems. We constantly keep setting the DIR pin
	if(clockwise == true){ //Clockwise direction
		GPIOPinWrite(DRIVER_FEEDER, DIRECTION_FEEDER, 0x80); //Set DIR pin HIGH
		setDelayMicro(PULSE_DELAY_MICROSEC);
	}
	else{
		GPIOPinWrite(DRIVER_FEEDER, DIRECTION_FEEDER, 0);  //Set DIR pin HIGH
		setDelayMicro(PULSE_DELAY_MICROSEC);
	}

	//****Important! Take into consideration the DRV8825 Timing Diagram!****
	GPIOPinWrite(DRIVER_FEEDER, STEP_FEEDER, 0x40); //Set STEP pin HIGH
	setDelayMicro(PULSE_DELAY_MICROSEC); //us delay to comply with DRV8825 timing diagram
	GPIOPinWrite(DRIVER_FEEDER, STEP_FEEDER, 0); //Set STEP pin LOW
	setDelayMicro(PULSE_DELAY_MICROSEC);//us delay to comply with DRV8825 timing diagram
}
int computeStepsPerTiles(int number){
	return STEPSTACK + stepsPerTile[currentTiles];
}
//-----------------------------------------------------------------------------
//---------------------Delay Functions---------------------------------------
//waitTime - time to delay in microsecods
void setDelayMicro(float microSecWaitTime){
	SysCtlDelay(computeDelayCountMicrosec(microSecWaitTime*0.001, CLOCK_FREQ));
}

//Set CPU delay considering the CLOCK_FREQ constant
//waitTime - time to delay in milliseconds
void setDelay(uint32_t waitTime){
	SysCtlDelay(computeDelayCount(waitTime, CLOCK_FREQ));
}

uint32_t computeDelayCount(uint32_t waitTime, uint8_t clockFreq){
	return 0.3333333 * (clockFreq*1000000) * (waitTime * 0.001);
}

//waitTime in milliseconds
uint32_t computeDelayCountMicrosec(float waitTime, uint8_t clockFreq){
	return 0.3333333 * (clockFreq*1000000) * (waitTime * 0.001);
}
//---------------------------------------------------------------------------
