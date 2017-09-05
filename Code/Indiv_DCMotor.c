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
#define ALLPINS GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7

//Function declarations
void PUSH_ISR();

void main(void)
{
	//----MCU Initialization----
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //Set-up the clocking of the MCU to 40MHz

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	 GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
	 GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

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
		if(pushButton != 0 ){
			if(pushButton == 0x10){
				if(state == 0){
					GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x80);
					state = 1;
				}else{
					GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00);
					state = 0;
				}
				pushButton = 0x00;
			}

		}
	}
}

void PUSH_ISR(){
	SysCtlDelay(4000000); //For debouncing
	pushButton = GPIOIntStatus(GPIO_PORTF_BASE,true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
}
