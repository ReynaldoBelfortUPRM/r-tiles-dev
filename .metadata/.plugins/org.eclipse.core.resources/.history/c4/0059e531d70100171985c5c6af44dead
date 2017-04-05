/*
 * main.c
 * Main program for TIVA MCU - Experiment 2
 * LCD Module Interfacing
 * Exercise 2.2.3
 *
 * Lab Team:
 * Emmanuel Ramos
 * Reynaldo Belfort
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
//Include our LCDlibrary
#include "customLibs/MIL_LCD_lib.h"

//Display Characters
#define CHAR_M 0x4D //01001101
#define CHAR_O 0x4F //01001111


int main(void) {
	
	//****MCU Initialization****
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //Set-up the clocking of the MCU to 40MHz
	//--Prepare peripherals--
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//Set port output pins
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, ENTIRE_PORT);
	//Set port input pins
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, (GPIO_PIN_2|GPIO_PIN_3));

	// Set Pin 3 to pull up
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPD);
	// Set pin 4 to pull down
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPU);

	//****END of MCU Initialization****

	//----Test LCD subrutines----

		//Setup the LCD
		initializeLCD(); //Contains the writeCommand function

		//Test - Display upper-case character M:
		writeChar(CHAR_M);
		writeChar(CHAR_O);

		//Test - Clear LCD
		SysCtlDelay(5333333); //400ms
		clearLCD();
		SysCtlDelay(5333333); //400ms

		//Test - Display first line
		uint8_t helloStr[9] = "saludosBA";
		writeMessage(helloStr, 9);

		//Test - Jump to second line
		setCursorPosition(0x40);
		//Test - Display second line
		uint8_t string2[7] = "olalala";
		writeMessage(string2, 7);

	//----END Test----

	//Main loop
	while(1){
		SysCtlDelay(5333333); //Stand by
	}
}
