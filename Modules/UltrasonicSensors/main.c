/*
 *
 * ICOM 4217 (EMBEDDED SYSTEM DESIGN)
 * Sensors control.
 *
 *
 *Pin setup:
 *	- Port E
 *		4- TRIGGER
 *		5- ECHO
 */

/* Header files */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "tivaUtils.h"
#include "MIL_LCD_lib.h"


/* Constants */
#define MICROSECPERCENT 29
/* Variable & Function declaration */
void PE5_IntHandler(void);
float calculate(void);
int status = 0;				//Status is the column thats the interrupt occurred.
uint32_t period;			//Timer period
int msc = 0;				//microsecond counter
float distance = 0;			//old distance in cm
int times = 0;
float nextDistance = 0;		//new distance in cm


/* Main Code */
void main (void){

	//------------TODO Code changed to use PORT E for the ultrasonic sensor-------------

	//---------------MCU Initialization------------------
		// Set clock frequency to 40 MHz.
		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

		//****LED Configuration****
		//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);					//Enable the peripheral port to use (F)
		//GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);							//LED

		//--------ULTRASONIC sensor configuration-----------

		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);	//Enable the peripheral port to use (E)

		/* TRIGGER pin configuration */
		GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);	 //Sensor TRIGGER Pin
		//GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

			//-----Timer configuration----
			SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);			//Enable timer
			TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);			//Set timer to count down
			period = (SysCtlClockGet()/1000000);					//timer period
			TimerLoadSet(TIMER0_BASE,TIMER_A, period-1);			//load timer period

		//Interrupt configuration
			/*Setting interrupt for ECHO pin */
			GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_5);		//Input configuration PA_5
			GPIOIntRegister(GPIO_PORTE_BASE, PE5_IntHandler); 				//Registers an interrupt handler for a GPIO port. Ensures that the interrupt handler specified is called.
			GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_BOTH_EDGES);	//Interrupt enable in both rising and falling edge.
			GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_5);						//Enable interrupt in Port A pin 5

			IntEnable(INT_TIMER0A);   								//Enable timer interrupt
			TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);			//Timer interrupt when times out
			IntMasterEnable();										//Enable master interrupt enable

		//--------------------------------------------------

		//--------LCD Setup--------
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);  // Enable, RS and R/W port for LCD Display
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);  // Data port for LCD display
		//Set LCD pins as outputs
		GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5);
		GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, ENTIRE_PORT);
		//-------------------------

	//---------------------------------------------------------------------------

		//LCD configuration
		initializeLCD();
		setCursorPosition(0);
		writeMessage("-Ultrasonic test", 16);
		setCursorPosition(0x40);

		//Send TRIGGER signal to the sensor
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4,16); 					//send one to Sensor trigger Pin
		SysCtlDelay(133);												//10 microsecond delay
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4,0); 				 	//send zero to Sensor trigger Pin

	while(1){
		/* if echo interrupted */
		if(times == 2){								//Do when interrupted twice
			distance = calculate();					//Calculate distance
			if(distance > 15){
//				GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,2);
				writeMessage("Distance: ", 10);
				writeChar(distance);
				setCursorPosition(0x40);
			}
			msc = 0;								//Set counter back to zero
			times =0;								//Reset times sensor interrupted
		}
	}
}

/* Interrupt handler */
void PE5_IntHandler(void)
{
	if(times ==0){								//First time interrupting
		TimerEnable(TIMER0_BASE,TIMER_A);		//Enable Timer
		times++;								//increase times interrupted
	} else if(times == 1){						//Second interrupt
		TimerDisable(TIMER0_BASE,TIMER_A);		//Disable timer
		times++;								//increase times interrupted
	}
	GPIOIntClear(GPIO_PORTE_BASE,GPIO_PIN_5);		// Clear
}

/* Timer Interrupt Handler */
void Timer0IntHandler(void)
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);		//Clear interrupt
	msc++;												//Increment microsecond counter

}

/* Function to calculate the centimeters number of centimeters between the sensor and obstacle */
float calculate(){
	return (msc/2)/MICROSECPERCENT;				//calculations to get Centimeters
}

