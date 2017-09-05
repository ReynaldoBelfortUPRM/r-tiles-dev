/*
 * ICOM 4217 (EMBEDDED SYSTEM DESIGN)
 */

/* include necessary system header files */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

/* LCD function Declarations */
void initializeLCD();//initialize LCD function declaration
void enterCommand(int command,float delayTime);		//enter command function for the LCD
void writeWord(uint8_t string[], uint8_t times);	//Function for writing words on the LCD
void writeCharDisp(unsigned char x);				//Function for writing a single Character on the LCD
short dec2ASCII(short number);						//Functions for converting decimals to ASCII

/* Interrupt service Routine Initialization */
void pushButtons_ISR();

/* Global variables */
short pushButton = 0;
bool pushFlag = false;
short currentState = 0;
short tileNumber = 0;
short currentNumDisplayed=0;

/* Constant label ALLPINS as all the pins in a port */
#define ALLPINS GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7

/* User interface buttons constant variables */
#define BUTTONS_PORT GPIO_PORTC_BASE
#define BUTTON_RIGHT GPIO_PIN_4
#define BUTTON_LEFT GPIO_PIN_5
#define MAXIMUM_NUM_TILES 3

/* LCD Constant Variables */

/* Main code of the program. */
int main(void)
{
	/* Variable declaration */
	uint8_t text[16][16] = {{"ICOM4217:"},{"R-Tile Proyect"},{"Press the right"},{"button to start"},{"Number of tiles"},{"to install: "},{"Error:"},{"Tile Number = 0"}};//Words to be written in LCD
	int timesArray[16] = {9,14,15,15,15,12,6,15}; 				//text[] words lenght
	//int index = 0;// current word pointer

	/* Clock frequency configuration */
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);// Set clock frequency to 40 MHz.

	/* Peripheral Configuration */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //Enable the peripheral port to use. (C)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//Enable the peripheral port to use. (B)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Enable the peripheral port to use. (F)

	/* Input/output configuration */
	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5);//Set pull-up button pin as output
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, ALLPINS);// Set all port B pins as output pins
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);//PIN1 : R/S (R/S: 1 for data ** 0 for instruction) *** PIN2 : R/W (R/S: 1 for Read ** 0 for write) *** PIN3 : E(Chip Enable signal).

	/* Pin current strength configuration */
	GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_STRENGTH_4MA , GPIO_PIN_TYPE_STD_WPU ) ;//Set the pin current strength (default is 2mA) and Pin Type to Pull Down
	GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_STRENGTH_4MA , GPIO_PIN_TYPE_STD_WPU ) ;//Set the pin current strength (default is 2mA) and Pin Type to Pull Down
	GPIOIntRegister(GPIO_PORTC_BASE, pushButtons_ISR);
	GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
	GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4);
	GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_5);

	initializeLCD();// initialize LCD
	writeWord(text[0],timesArray[0]);
	enterCommand(0xC0,493.3);
	writeWord(text[1],timesArray[1]);
	//TODO delay 2 segundos
	SysCtlDelay(40000000);

	while(1){
		//User Interface
		if(currentState == 0){
			//			enterCommand(0x01,20400);			//Clear display command
			//			writeWord(text[2],timesArray[2]);
			//			enterCommand(0xC0,493.3);
			//			writeWord(text[3],timesArray[3]);

			//TODO delay 2 segundos

			enterCommand(0x01,20400);//Clear display command
			writeWord(text[4],timesArray[4]);
			enterCommand(0xC0,493.3);
			writeWord(text[5],timesArray[5]);

			currentNumDisplayed = dec2ASCII(tileNumber);
			writeCharDisp(currentNumDisplayed);

			while(currentState == 0){
				if(pushFlag){
					pushFlag = false;
					if(pushButton == 0x10){
						tileNumber++;
						if(tileNumber > MAXIMUM_NUM_TILES){
							tileNumber = 0;
						}
						currentNumDisplayed = dec2ASCII(tileNumber);
						enterCommand(0xCC,493.3);
						writeCharDisp(currentNumDisplayed);
					}
					if(pushButton ==0x20){
						if(tileNumber == 0){
							//Write error Message
							enterCommand(0x01,20400);				//Clear display command
							writeWord(text[6],timesArray[6]);
							enterCommand(0xC0,493.3);
							writeWord(text[7],timesArray[7]);

							SysCtlDelay(40000000);		//Delay

							enterCommand(0x01,20400);//Clear display command
							writeWord(text[4],timesArray[4]);
							enterCommand(0xC0,493.3);
							writeWord(text[5],timesArray[5]);

							currentNumDisplayed = dec2ASCII(tileNumber);
							writeCharDisp(currentNumDisplayed);

						}else{
							currentState = 1;
						}
					}
				}
			}
		}
	}

}

void pushButtons_ISR(){
	SysCtlDelay(4000000); //For debouncing
	pushFlag = true;
	pushButton = GPIOIntStatus(GPIO_PORTC_BASE, true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5);
}


//------------ LCD Functions ----------------------
/* LCD initialization function */
void initializeLCD()
{
	SysCtlDelay(533333.3333);// Starting delay (40ms).

	enterCommand(0x3C,520);//Function Set command

	enterCommand(0x3C,493.3);//Function Set

	enterCommand(0x0F,493.3);//Display ON

	enterCommand(0x01,20400);//Clear display command

	enterCommand(0x06,20400);//Entry Mode Set

	enterCommand(0x01,20400);//Clear display command
}

/* Enter command to LCD function; command is the desired LCD instruction, delayTime is the required time that the chip enable needs to be on in order to
 * successfully send the instruction */
void enterCommand(int command,float delayTime)
{
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2, 0x00); //R/S & R/W = 0
	GPIOPinWrite(GPIO_PORTB_BASE, ALLPINS, command); //enter command on port
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x08);//Enable On
	SysCtlDelay(delayTime);//value for the desired delay
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);//Enable Off
}

/* Function for writing words in the LCD, string is the word that want to be written, times is the number of characters that the string has */
void writeWord(uint8_t string[], uint8_t times)
{
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x02);//RS enable

	uint8_t i =0;//initialize for loop
	for(; i < times; i++){
		GPIOPinWrite(GPIO_PORTB_BASE, ALLPINS, string[i]);//write character
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x08);//Enable On
		SysCtlDelay(94666);//Delay 7.1ms(Calculated)
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);//Enable Off
	}
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);//RS disable
}
//----------------------------------------------------
void writeCharDisp(unsigned char x){
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x02);//RS enable
	GPIOPinWrite(GPIO_PORTB_BASE, ALLPINS, x  );
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x08);//Enable On
	SysCtlDelay(50000);//Delay 7.1ms(Calculated)
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);//Enable Off
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);//RS disable

}

short dec2ASCII(short number){
	return number + 0x30;
}
