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
bool pushFlag = false;
short currentState = 3;	//Determines the current state of the system 0: User Interface 1:Stack operations 2: dispenser operations 3: positional system operations

//Stepper Motor Constants
#define PULSE_DELAY 1
#define PULSE_DELAY_MICROSEC 500 //microseconds 4%-4.8% error This number is not necesarily accurate.
#define MOTOR_WHEEL_CIRCUM_INCHES 13.35176878
#define MOTOR_STEPS_PER_REV 200
#define DRIVER_POS GPIO_PORTD_BASE
#define DRIVER_POS_1 GPIO_PORTA_BASE
#define STEP_POS_2 GPIO_PIN_3			//Front
#define STEP_POS_1 GPIO_PIN_6			//Back


//#define DIRECTION_STEP_1 GPIO_PIN_2 //Pin D2 da problemas al utilizarlo para los motores
#define DIRECTION_STEP_1 GPIO_PIN_2
#define DIRECTION_STEP_2 GPIO_PIN_4 		//port A
#define CLOCK_FREQ 40


#define ALLPINS GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7

//Interrupt Service Routine
void PUSH_ISR();

//Stepper motor functions
void performStepPositionalSystem(bool clockwise);
void spinStepperInches(double distance, bool clockwise, uint16_t delay);

//Utility
void setDelayMicro(float microSecWaitTime);
void setDelay(uint32_t waitTime);
uint32_t computeDelayCount(uint32_t waitTime, uint8_t clockFreq);
uint32_t computeDelayCountMicrosec(float waitTime, uint8_t clockFreq);

void main(void)
{
	//--------- MCU Initialization -------------
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //Set-up the clocking of the MCU to 40MHz

	//---------Enable port D peripheral---------
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); 		//Driver Step's'
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); 		//Driver Step's'
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);		//Tiva buttons

	//---------- Position System Stepper Motor Pin Configuration ----------
	GPIOPinTypeGPIOOutput(DRIVER_POS_1, DIRECTION_STEP_2); //Driver's STEP pin
	GPIOPinTypeGPIOOutput(DRIVER_POS, DIRECTION_STEP_1 | STEP_POS_1|STEP_POS_2); //Driver's STEP pin
	//---------------------------------------------------------------------

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
	//--------------------------------

	while(1){
		//Left and right push buttons are used to turn motor clockWise or coutnerClockwise respectively
		//		if(currentState = 3){
		//			spinStepperInches(6,false,PULSE_DELAY_MICROSEC); //just in a single direction
		//		}
		if(currentState == 3){
			if(pushFlag){
				pushFlag = false;
				//Change direction according to the button pushed
				if(pushButton == 16){ //SW1 button
//					while(true){
//						performStepPositionalSystem(false);
//						setDelay(1);
//					}
					spinStepperInches(6,false,1); //just in a single direction
				} else{ //TODO PARA ALFRENTE!!!
//					while(true){
//						performStepPositionalSystem(true);
//						setDelay(1);
//					}
					spinStepperInches(6,true,1); //just in a single direction
				}
			}
			//		}
		}
	}
}
//------------------Interrupt Service Routine--------------------
void PUSH_ISR(){
	SysCtlDelay(4000000); //For debouncing
	pushFlag = true;
	pushButton = GPIOIntStatus(GPIO_PORTF_BASE,true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
}

//--------------------------------------------------------------
//---------------------- Functions Stepper motor -----------------
//Send a pulse to the STEP pin
void performStepPositionalSystem(bool clockwise){

	//TODO Having thess if-clauses may cause problems. We constantly keep setting the DIR pin
	if(clockwise == true){ //Clockwise direction
		GPIOPinWrite(DRIVER_POS, DIRECTION_STEP_1, 4); //Set DIR pin HIGH
		GPIOPinWrite(DRIVER_POS_1, DIRECTION_STEP_2, 0); //Set DIR pin HIGH
		setDelayMicro(PULSE_DELAY_MICROSEC);
	}
	else{
		GPIOPinWrite(DRIVER_POS_1, DIRECTION_STEP_2, 0x10); //Set DIR pin HIGH
		GPIOPinWrite(DRIVER_POS, DIRECTION_STEP_1, 0); //Set DIR pin
		setDelayMicro(PULSE_DELAY_MICROSEC);
	}

	//****Important! Take into consideration the DRV8825 Timing Diagram!****
	GPIOPinWrite(DRIVER_POS,STEP_POS_1|STEP_POS_2,0x48);		//Set STEP pin HIGH
	setDelayMicro(PULSE_DELAY_MICROSEC); 		//us delay to comply with DRV8825 timing diagram
	GPIOPinWrite(DRIVER_POS, STEP_POS_1|STEP_POS_2, 0); 		//Set STEP pin LOW
	setDelayMicro(PULSE_DELAY_MICROSEC);		//us delay to comply with DRV8825 timing diagram
}

//distance in inches
void spinStepperInches(double distance, bool clockwise, uint16_t delay){

	uint32_t amountSteps = 0; //Amount of steps per inch
	amountSteps = (MOTOR_WHEEL_CIRCUM_INCHES / distance) * MOTOR_STEPS_PER_REV; //Formula for converting steps to inches
	uint32_t counter = 0;
	for(; counter < amountSteps; counter++){
		performStepPositionalSystem(clockwise); //just in a single direction
		setDelay(delay);
	}
}
//--------------------------------------------------------------------
//------------------------ Delay functions ---------------------------
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
//--------------------------------------------------------------------
