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

//Constants
#define PULSE_DELAY 1
#define PULSE_DELAY_MICROSEC 500 //microseconds 4%-4.8% error This number is not necesarily accurate.
#define DRIVER_DISP_ELE GPIO_PORTE_BASE
#define STEP_DISP_ELE GPIO_PIN_1
#define DIRECTION_DISP_ELE GPIO_PIN_2
#define CLOCK_FREQ 40

#define ALLPINS GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7

//Function declarations
void PUSH_ISR();
void LIMIT_SWITCH_DISPENSER_BOTTOM_ISR();
void LIMIT_SWITCH_DISPENSER_TOP_ISR();

void performStep(bool clockwise);
void setDelayMicro(float microSecWaitTime);
void setDelay(uint32_t waitTime);
uint32_t computeDelayCount(uint32_t waitTime, uint8_t clockFreq);
uint32_t computeDelayCountMicrosec(float waitTime, uint8_t clockFreq);

void main(void)
{
	//----MCU Initialization----
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //Set-up the clocking of the MCU to 40MHz
	//Enable port E peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); 		//Driver Step's'
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);		//Tiva buttons
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);		//Switches

	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2); //Driver's STEP pin

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

	//Temporary variables
	//bool runMotor = false;
	//bool runDirection = false; //FALSE = downward , TRUE = upward

	while(1){
		//-----------------ROTATE BY PUSH BUTTON PROGRAM----------------------
		//Left and right push buttons are used to turn motor clockWise or coutnerClockwise respectively
		if(pushFlag){
			pushFlag = false;
			//Change direction according to the button pushed
			if(pushButton == 16){ //SW1 button
				int counter = 0;
				//for(; counter < 300; counter++){
				//	performStep(false); //just in a single direction
				//	setDelay(1); //in ms
				//}
				while(true){
					performStep(false);
							setDelay(1);
				}
			}
			else{ //SW2 button
				int counter = 0;
				for(; counter < 300; counter++){
					performStep(true); //just in a single direction
					setDelay(1); //in ms
				}
			}
		}
	}
}

void PUSH_ISR(){
	SysCtlDelay(4000000); //For debouncing
	pushFlag = true;
	pushButton = GPIOIntStatus(GPIO_PORTF_BASE,true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
}

//Send a pulse to the STEP pin
void performStep(bool clockwise){ //


	//TODO Having thess if-clauses may cause problems. We constantly keep setting the DIR pin
	if(clockwise == true){ //Clockwise direction
		GPIOPinWrite(DRIVER_DISP_ELE, DIRECTION_DISP_ELE, 4); //Set DIR pin HIGH
		setDelayMicro(PULSE_DELAY_MICROSEC);
	}
	else{
		GPIOPinWrite(DRIVER_DISP_ELE, DIRECTION_DISP_ELE, 0);  //Set DIR pin HIGH
		setDelayMicro(PULSE_DELAY_MICROSEC);
	}

	//****Important! Take into consideration the DRV8825 Timing Diagram!****
	GPIOPinWrite(DRIVER_DISP_ELE, STEP_DISP_ELE, 2); //Set STEP pin HIGH
	setDelayMicro(PULSE_DELAY_MICROSEC); //us delay to comply with DRV8825 timing diagram
	GPIOPinWrite(DRIVER_DISP_ELE, STEP_DISP_ELE, 0); //Set STEP pin LOW
	setDelayMicro(PULSE_DELAY_MICROSEC);//us delay to comply with DRV8825 timing diagram
}


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
