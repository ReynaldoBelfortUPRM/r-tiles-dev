/*****************************************************************************
 *	 		LCD Controller library for Module: 932-MIKROE-55
 *	File: MIL_LCD_lib.h - function prototype and constant definitions
 *
 *  This library contains common routines for interfacing with the
 *  LCD Module Controller: 932-MIKROE-55 from the MIL lab
 *
 *	Developed by Reynaldo Belfort
 *  Lab Team:
 *  Emmanuel Ramos
 *  Reynaldo Belfort
 *
 * ----------------------Notes----------------------
 * This library assumes the following pin configuration (7-0 bits) on the TIVA MCU
 * 7  6  5  4  3  2  1  0
 * D7,D6,D5,D4,D3,D2,D1,D0 - port B GPIOs - Data Port
 * RS,R/W,E 		       - port A GPIOs - Control Port
 *
*****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "MIL_LCD_lib.h"

/*
 * Initializes the LCD with a preset configuration.
 *
 * Preset configuration:
 * Font: 5x8
 * Interface type: 8-bit
 * Display: 2-lines, Incremental cursor, disable display shift
 * */
void initializeLCD(){
		//----Initialize LCD Module (according to the 932-MIKROE-55 datasheet)----
		SysCtlDelay(600000); //Wait more than 40ms (45ms here), CPU @ 40MHz

		//Set the Function Set command and send it to LCD (wait at least 5.77ms)
		writeCommand(COMMAND_FUNCTIONSET_8BIT_2LINE_FONT8, 6);

		//Set Function Set command and send it to LCD (wait at least 5.77ms)
		writeCommand(COMMAND_FUNCTIONSET_8BIT_2LINE_FONT8, 6);

		//Turn ON display with blinking cursor and it send to LCD -  Command: Display ON/OFF Control (wait at least 5.77ms)
		writeCommand(COMMAND_DISPLAY_ON_BLINKING, 6);

		//Clear screen
		clearLCD();

		//Establish the entry mode Command: Entry Mode Set (wait at least 5.77ms)
		writeCommand(COMMAND_ENTRYMODE_INCCUR_NODISPSHIFT, 6);
}


/*
 * Clears display and returns the cursor to home position.
 * */
void clearLCD(){
	writeCommand(COMMAND_CLEARDISPLAY, 227); //226.7ms according to Instruction Table
}

/*
 * Sends a command to set the cursor to a position indicated by the parameter.
 *
 * Parameters:
 * - position: Screen position according to the 932-MIKROE-55 LCD Controller.
 * */
void setCursorPosition(uint8_t position){
//	uint8_t finalPos = position + 128; //128 = 10000000b
	uint8_t finalPos = position + 0x80; //128 = 10000000b
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7|GPIO_PIN_6, 0x00); //Control port: 00b
	GPIOPinWrite(GPIO_PORTB_BASE, ENTIRE_PORT, finalPos); //Data port
	sendSignal(6); //6ms according to Instruction Table
}

/*
 * Writes a single character to the LCD at the current cursor position.
 *
 * Parameters:
 * - targetChar: ASCII value of the character.
 * */
void writeChar(uint8_t targetChar){
	//Set char accordiing to the Character Generator ROM Pattern
//	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7|GPIO_PIN_6, 128); //Control port: Write to RAM Mode - 10000000b
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7|GPIO_PIN_6, 0x80); //Control port: Write to RAM Mode - 10000000b
	GPIOPinWrite(GPIO_PORTB_BASE, ENTIRE_PORT, targetChar); //Data port
	sendSignal(7); //Wait at least 6.37ms delay according to the Instruction Table
}

/*
 * Writes a command to the LCD controller.
 *
 * Parameters:
 * - commad: instruction to be sent.
 * - signalWait: delay to allow the LCD to process the command.n
 * */
void writeCommand(uint8_t command, uint8_t signalWait){
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7|GPIO_PIN_6, 0x00); //Control port: 00b
	GPIOPinWrite(GPIO_PORTB_BASE, ENTIRE_PORT, command); //Data port
	sendSignal(signalWait);
}


/*
 * Writes a sequence of characters to the LCD starting at the current cursor position.
 * Parameters:
 * - strVal: char array of ASCII values to be displayed.
 * - arrSize: length of strVal
 * */
void writeMessage(uint8_t strVal[], uint8_t arrSize){
	//Validation
	if(arrSize <= 0) { return; }

	uint8_t i = 0;
	for(; i < arrSize; i++){
		writeChar(strVal[i]);
	}
}

/*
 * Sets the EN (enable) pin to high during an specified period of time.
 * This sends the current command or character to the LCD.
 *
 * Parameters:
 * - waitTime: time in milliseconds.
 *
 * Delay formula:
 * DelayCount = 1/3 * mcuFreq * waitTime ( DelayCount value to be set to SysCtlDelay() )
 * */
void sendSignal(uint8_t waitTime){
//	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 32); //Set Enable pin high - 00100000b
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x20); //Set Enable pin high - 00100000b
	SysCtlDelay(0.3333333 * (CLOCK_FREQ*1000000) * (waitTime * 0.001) );
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x00); //Set Enable pin low
}

/*
 * Moves the cursor a number of positions to the left or right of the LCD.
 * Parameters:
 * - offset: amount of positions to move.
 * - rightDirection: true for right direction, false for left.
 * */
void moveCursor(uint8_t offset, _Bool rightDirection){
	//Validate parameter
	if(offset == 0) { return; } //Number of positions to move should be indicated

	uint8_t i = 0;
	//Set Cursor/Display Shift command
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7|GPIO_PIN_6, 0x00); //Control port
	if(rightDirection){
		//Move cursor right <<offset>> amount of times
		for(; i < offset; i++){
			writeCommand(COMMAND_MOVECURSOR_RIGHT, 6);
		}
	}
	else{
		//Move cursor left <<offset>> amount of times
		for(; i < offset; i++){
			writeCommand(COMMAND_MOVECURSOR_LEFT, 6);
		}
	}
}
