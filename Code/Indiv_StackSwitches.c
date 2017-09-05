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

#define ALLPINS GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7

//Function declarations
void PUSH_ISR();
void LIMIT_SWITCH_ISR();

void main(void)
{
	//----MCU Initialization----
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //Set-up the clocking of the MCU to 40MHz
	//Enable port E peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);		//Switches
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1); //Driver's STEP pin

	//Register Limit Switch Pin
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntRegister(GPIO_PORTA_BASE, LIMIT_SWITCH_ISR);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_FALLING_EDGE);
	GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3);

	//--------------------------------
	while(1){
		if(pushFlag){
			pushFlag = false;
			if(pushButton == 0x04 || pushButton == 0x08){
				if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == 0x00){
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,0x02);
				}else{
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,0x00);
				}
			}
		}
	}
}

void LIMIT_SWITCH_ISR(){
	SysCtlDelay(4000000); //For debouncing
	pushFlag = true;
	pushButton = GPIOIntStatus(GPIO_PORTA_BASE, true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3);
}
