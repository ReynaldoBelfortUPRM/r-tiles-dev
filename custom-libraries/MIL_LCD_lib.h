/*****************************************************************************
 *	 		LCD Controller library for Module: 932-MIKROE-55
 *	File: MIL_LCD_lib.h - function prototype and constant definitions
 *
 *  This library contains common routines for interfacing with the
 *  LCD Module Controller: 932-MIKROE-55 from the MIL lab
 *
 *  Developed by Reynaldo Belfort
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

#ifndef MIL_LCD_LIB_H_
#define MIL_LCD_LIB_H_

/************************ TIVA related definitions ************************/
//TODO For the following definition: Maybe we will have to add a parameter to each function that uses a delay,
//to make the library universal
#define CLOCK_FREQ 40 //In MHz
#define ENTIRE_PORT GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0

/************************ LCD Controller Commands ************************/

//The following commands are set according to the Instruction Table from the 932-MIKROE-55 datasheet
//These go to the data bus pins. That is, does not include control pins.

#define COMMAND_FUNCTIONSET_8BIT_2LINE_FONT8 0x38 //8-bit interface; 2-line; display font: 5x8;  Binary num: 00111000b
#define COMMAND_DISPLAY_ON_BLINKING 0x0D  //Turn ON display with blinking cursor  -  Command: Display ON/OFF - 00001101b
#define COMMAND_ENTRYMODE_INCCUR_NODISPSHIFT 0x06 //Increment cursor, disable display shift - Command: Entry Mode Set - 00000110b
#define COMMAND_CLEARDISPLAY 0x01 //Command: Clear Display - 000000001b
#define COMMAND_MOVECURSOR_RIGHT 0x14 //Command: Cursor or Display Shift - 00010100b
#define COMMAND_MOVECURSOR_LEFT 0x10  //Command: Cursor or Display Shift - 00010000b

//************************Essential function prototypes************************
extern void clearLCD();
extern void setCursorPosition(uint8_t position);
extern void writeChar(uint8_t targetChar);
extern void writeCommand(uint8_t command, uint8_t signalWait);
extern void writeMessage(uint8_t strVal[], uint8_t arrSize);
//************************Extra function prototypes************************
extern void initializeLCD();
extern void sendSignal(uint8_t waitTime);
extern void moveCursor(uint8_t offset, _Bool rightDirection);

#endif
