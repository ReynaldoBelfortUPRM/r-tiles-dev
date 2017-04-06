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

//Function declarations
void PUSH_ISR();

//Global Variables
uint8_t pushButton = 0;
bool pushFlag = false;

int main (void){
    //----MCU Initialization----
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //Set-up the clocking of the MCU to 40MHz
    //Enable port E peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

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
    //--------------------------------

    pushFlag = true;

	while(1){

		if(pushFlag){
			pushFlag = false;
			//Change direction according to the button pushed
			if(pushButton == 16){ //SW1 button
				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 4);
				setDelay(1);
			}
			else{ //SW2 button
				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
				setDelay(1);
			}
		}

		//****Important! Take into consideration the DRV8825 Timing Diagram!****

//		//Perform a step
//		if(pushFlag && (pushButton == 16)) {
//			pushFlag = false;
//			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 4);
//			setDelay(1);
//		} //Set DIR pin HIGH

		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 2); //Set STEP pin HIGH
//		setDelay(PULSE_DELAY); //ms delay
		setDelayMicro(PULSE_DELAY_MICROSEC); //us delay

//		if(pushFlag && (pushButton == 0)) {
//			pushFlag = false;
//			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
//			setDelay(1);
//		} //Set DIR pin LOW

		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0); //Set STEP pin LOW
//		setDelay(PULSE_DELAY); //ms delay
		setDelayMicro(PULSE_DELAY_MICROSEC); //us delay

		//if(pushFlag) { pushFlag = false; }
	}
}

void PUSH_ISR(){
	setDelay(30); //For debouncing
	pushFlag = true;
	pushButton = GPIOIntStatus(GPIO_PORTF_BASE,true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
}
