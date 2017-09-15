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
bool stopFlag = false;

//--------Stepper motor global variables-------
bool runMotor = true;
bool runDirectionStack = true; //FALSE = upward , TRUE = downward
bool stackDown = false;
bool stackUp = false;
bool stackUp1 = false;
short switchButton = 0;
short currentState = 0;
short safety = 1;

//---------------------------------------------

/* Stack Stepper motor Constants */
#define PULSE_DELAY 1
#define PULSE_DELAY_MICROSEC 500 //microseconds 4%-4.8% error This number is not necesarily accurate.
#define DRIVER_STACK_ELE GPIO_PORTE_BASE
#define STEP_STACK_ELE GPIO_PIN_3
#define DIRECTION_STACK_ELE GPIO_PIN_4
#define CLOCK_FREQ 40
//TODO Anadir constantes de steps por losa y steps dependiendo cuantas tiene en el stack

/* General Constants */
#define ALLPINS GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7

/* ISR */
void PUSH_ISR();
void LIMIT_SWITCH_ISR();

/* Stepper motor functions */
void performStep(bool clockwise);
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
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); 		//Driver Step's'
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);		//Tiva buttons
	//--------------------------

	//--------- DC motor Pin Configuration --------
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
	//----------------------------------------------

	//------Pin configuration Stepper Motor-------
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_4); //Driver's STEP pin
	//-------------------------------------------

	//------Pin configuration DC motor ----------
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
	//------------------------------------------

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
	GPIOIntRegister(GPIO_PORTA_BASE, LIMIT_SWITCH_ISR);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_FALLING_EDGE);


	//--------------------------------

	while(1){

		//Left and right push buttons are used to turn motor clockWise or coutnerClockwise respectively
		//--------
		//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x80);
		//-------

		while(currentState == 0){
			GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
			if(pushFlag){
				pushFlag = false;
				//If button was pushed start moving motor
				if(pushButton == 16){ //SW1 button
					currentState = 1;
					GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
				}
			}
		}

		if(currentState == 1){
			//GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3);
			GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2);
			while(currentState ==1){
				if(stackUp1 == false){
					runMotor = true;
					while(runMotor == true){
						performStep(runDirectionStack); 		//just in a single direction
						setDelay(1); //in ms
					}
					GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x80);
					currentState = 2;
				}
			}
		}

		while(currentState == 2){
			if(safety !=0){
				GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
				if(pushFlag){
					pushFlag = false;
					//If button was pushed start moving motor
					if(pushButton == 16){ //SW1 button
						int counter = 0;
						runMotor = true;
						for(; counter < 1200; counter++){
							if(runMotor){
								performStep(runDirectionStack); //just in a single direction
								setDelay(1); //in ms
							}else{
								counter = 1200;
							}
						}
						safety = 0;
					}
				}
				if(stackDown){
					stackDown = false;
				}
			}
		}
	}
	//				if(pushFlag){
	//					pushFlag = false;
	//					//If button was pushed start moving motor
	//					if(pushButton == 16){ //SW1 button
	//						int counter = 0;
	//						for(; counter < 300; counter++){
	//							//TODO Change to while(runMotor)
	//							if(runMotor){
	//								performStep(runDirectionStack); //just in a single direction
	//								setDelay(1); //in ms
	//							}else{
	//								counter = 300;
	//							}
	//						}
	//					}
	//				}
	//				if(stackDown){
	//					stackDown = false;
	//				}
	//				else if(stackUp){
	//					stackUp = false;
	//				}
}

//-----------------------Interrupt Service Routines-----------------------
void PUSH_ISR(){
	SysCtlDelay(4000000); //For debouncing
	pushFlag = true;
	pushButton = GPIOIntStatus(GPIO_PORTF_BASE,true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
}

void LIMIT_SWITCH_ISR(){
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
void performStep(bool clockwise){ //

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
