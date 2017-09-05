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

/* Global variables */
short pushButton = 0;		//Switches interrupt variable
short switchButton = 0;
short currentState = 0;
bool pushFlag = false;		//Boolean variable to keep track if the ISR happened or not
bool stopFlag = false;

//------Dispenser Arms Global Variables------
volatile uint32_t load;
volatile uint32_t PWMClock;
volatile uint8_t adjust;
volatile uint8_t adjustLeft;

//-------Dispenser Globlal variables-------
bool dispenserDown = false;
bool dispenserUp = false;
bool dispenserUp1 = true;
bool readyToDispense = false;

//----------- LCD Global variables -----------
/* Variable declaration */
uint8_t text[16][16] = {{"ICOM4217:"},{"R-Tile Proyect"},{"Press the right"},{"button to start"},{"Number of tiles"},{"to install: "},{"Error:"},{"Tile Number = 0"}};//Words to be written in LCD
int timesArray[16] = {9,14,15,15,15,12,6,15}; 				//text[] words lenght
short tileNumber = 0;
short currentNumDisplayed=0;

//-------- Stepper motor Global variables---------
bool runMotor = true;
bool runDirectionDispenser = true; //FALSE = upward , TRUE = downward
//---------------------------------------------

/* Servo Motor Constant Declarations */
#define PWM_FREQUENCY 50
#define PORT_SERVO GPIO_PORTD_BASE
#define PIN_SERVO_1 GPIO_PIN_1
#define PIN_SERVO_2 GPIO_PIN_0

/* LCD Constant Declarations */
#define MAXIMUM_NUM_TILES 3

/* User interface buttons constant variables */
#define BUTTONS_PORT GPIO_PORTC_BASE
#define BUTTON_RIGHT GPIO_PIN_4
#define BUTTON_LEFT GPIO_PIN_5

/* Constant declarations Dispenser Elevator Stepper Motor */
#define PULSE_DELAY 1
#define PULSE_DELAY_MICROSEC 500 		//microseconds 4%-4.8% error This number is not necesarily accurate.
#define DRIVER_DISP_ELE GPIO_PORTE_BASE
#define STEP_DISP_ELE GPIO_PIN_1
#define DIRECTION_DISP_ELE GPIO_PIN_2

/* Constant declarations Positional System Stepper Motor */
#define MOTOR_WHEEL_CIRCUM_INCHES 13.35176878
#define MOTOR_STEPS_PER_REV 200
#define DRIVER_POS GPIO_PORTD_BASE
#define STEP_POS GPIO_PIN_6
#define DIRECTION_STEP_1 GPIO_PIN_2
#define DIRECTION_STEP_2 GPIO_PIN_3

/* General Constant Declarations */
#define CLOCK_FREQ 40
#define ALLPINS GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7

/* LCD function Declarations */
void initializeLCD();//initialize LCD function declaration
void enterCommand(int command,float delayTime);//enter command function declaration
void writeWord(uint8_t string[], uint8_t times);//writeWord function declaration
void writeCharDisp(unsigned char x);				//Function for writing a single Character on the LCD
short dec2ASCII(short number);						//Functions for converting decimals to ASCII

/* Interrupt service Routine Initialization */
void pushButtons_ISR();

/* Interrupt Service Routines */
void LIMIT_SWITCH_ISR();	//Switch interrupt Service Routine

//------------------ Function Declarations -------------------------
/* Stepper motor functions */
void performStep(bool clockwise); 	//Performs single step
void performStepPositionalSystem(bool clockwise);

/* Dispenser arms methods for closing and opening arms */
void openArms();
void closeArms();

/* Delay Functions */
void setDelayMicro(float microSecWaitTime);
void setDelay(uint32_t waitTime);
uint32_t computeDelayCount(uint32_t waitTime, uint8_t clockFreq);
uint32_t computeDelayCountMicrosec(float waitTime, uint8_t clockFreq);
//-------------------------------------------------------------------
void main(void)
{
	//----MCU Initialization----
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //Set-up the clocking of the MCU to 40MHz
	//--------------------------

	//-------Port enable--------
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);		//Switches
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);		//Enable the peripheral port to use. (B)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); 		//Enable the peripheral port to use. (C)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);		//Positional system Steps and Direction(D2,D3,D6)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); 		//Driver Step's'
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);		//Tiva buttons
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	//--------------------------

	//------- Dispenser Elevator Stepper motor driver configuration ------
	GPIOPinTypeGPIOOutput(DRIVER_DISP_ELE, STEP_DISP_ELE | DIRECTION_DISP_ELE); //Driver's STEP pin
	//---------------------------------

	//---------- Position System Stepper Motor Pin Configuration ----------
	GPIOPinTypeGPIOOutput(DRIVER_POS, DIRECTION_STEP_1|DIRECTION_STEP_2 | STEP_POS); //Driver's STEP pin
	//---------------------------------------------------------------------

	//---------- LCD Pin configuration -----------------
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, ALLPINS);// Set all port B pins as output pins
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);//PIN1 : R/S (R/S: 1 for data ** 0 for instruction) *** PIN2 : R/W (R/S: 1 for Read ** 0 for write) *** PIN3 : E(Chip Enable signal).
	//------------------------------------------------

	//------------Buttons Pin Configuration ------------------
	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5);//Set pull-up button pin as output
	//--------------------------------------------------------

	//---------- Buttons User Interface configuration ----------------
	GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_STRENGTH_4MA , GPIO_PIN_TYPE_STD_WPU ) ;//Set the pin current strength (default is 2mA) and Pin Type to Pull Down
	GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_STRENGTH_4MA , GPIO_PIN_TYPE_STD_WPU ) ;//Set the pin current strength (default is 2mA) and Pin Type to Pull Down
	GPIOIntRegister(GPIO_PORTC_BASE, pushButtons_ISR);
	GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);

	//--------------------------------------------------------------
	//------------------ Dispenser Arm Servo configuration -----------------
	adjust = 42;
	adjustLeft = 120;
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	GPIOPinTypePWM(PORT_SERVO, PIN_SERVO_2);
	GPIOPinTypePWM(PORT_SERVO, PIN_SERVO_1);
	GPIOPinConfigure(GPIO_PD0_M1PWM0);
	GPIOPinConfigure(GPIO_PD1_M1PWM1);
	PWMClock = SysCtlClockGet() / 64;

	load = (PWMClock / PWM_FREQUENCY) - 1;

	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);

	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, load);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, adjust * load / 1000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, adjustLeft* load / 1000);

	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
	PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);
	//---------------------------------------------------------------------

	//------------------- Limit Switch Dispenser Configuration --------------
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntRegister(GPIO_PORTA_BASE, LIMIT_SWITCH_ISR);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);
	//GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_5|GPIO_PIN_6);
	//----------------------------------------------------------
	initializeLCD();

	while(1){
		//User Interface
		if(currentState == 0){
			//TODO delay 2 segundos
			GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4);
			GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_5);
			initializeLCD();// initialize LCD
			writeWord(text[0],timesArray[0]);
			enterCommand(0xC0,493.3);
			writeWord(text[1],timesArray[1]);
			//TODO delay 2 segundos
			SysCtlDelay(40000000);

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

		if(currentState ==2){
			GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_5|GPIO_PIN_6);
			while(currentState == 2){
				//TODO si se activa el switch de arriba guardarlo en una variable y chequiar

				if(readyToDispense && dispenserUp1){
					performStep(runDirectionDispenser); //just in a single direction
					runMotor = true;
					while(runMotor == true){
						performStep(runDirectionDispenser); //just in a single direction
						setDelay(1); //in ms
					}
					//TODO wait 1 second & then open arms
					openArms();
					//TODO wait 1 second & then go up again
					runMotor = true;
					while(runMotor == true){
						performStep(runDirectionDispenser); //just in a single direction
						setDelay(1); //in ms
					}
					closeArms();
					//TODO go to next State
					currentState = 3;
					readyToDispense = false;
				}
			}
		}
		//				if(currentState = 3){
		//					spinStepperInches(6,false,1); //just in a single direction
		//				}
	}
}

//----------------- ISR Functions-------------------------------
void pushButtons_ISR(){
	SysCtlDelay(4000000); //For debouncing
	pushFlag = true;
	pushButton = GPIOIntStatus(GPIO_PORTC_BASE, true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5);
}

void LIMIT_SWITCH_ISR(){
	SysCtlDelay(4000000); //For debouncing
	switchButton = GPIOIntStatus(GPIO_PORTA_BASE, true); //To read which button interrupted
	if(switchButton == 0x20){ // Down Dispenser Sensor
		runDirectionDispenser = false;
		runMotor = false;
		//dispenserUp = false;
		dispenserUp1 = false;
		//dispenserDown = true;
		GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_5);
		GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_6);
	}else if(switchButton == 0x40){ //Up Dispenser Sensor
		runDirectionDispenser = true;
		runMotor = false;
		//dispenserUp = true;
		dispenserUp1 = true;
		GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_6);
		GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_5);
	}

	GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_5|GPIO_PIN_6);
}
//--------------------------------------------------------------

//--------------- Stepper Motor Functions ----------------------
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

//Send a pulse to the STEP pin
void performStepPositionalSystem(bool clockwise){

	//TODO Having thess if-clauses may cause problems. We constantly keep setting the DIR pin
	if(clockwise == true){ //Clockwise direction
		GPIOPinWrite(DRIVER_POS, DIRECTION_STEP_1, 4); //Set DIR pin HIGH
		GPIOPinWrite(DRIVER_POS, DIRECTION_STEP_2, 0); //Set DIR pin HIGH
		setDelayMicro(PULSE_DELAY_MICROSEC);
	}
	else{
		GPIOPinWrite(DRIVER_POS, DIRECTION_STEP_1, 0); //Set DIR pin HIGH
		GPIOPinWrite(DRIVER_POS, DIRECTION_STEP_2, 8); //Set DIR pin HIGH

		//GPIOPinWrite(DRIVER_POS, DIRECTION_STEP_1, 0); //Set DIR pin HIGH
		setDelayMicro(PULSE_DELAY_MICROSEC);
	}

	//****Important! Take into consideration the DRV8825 Timing Diagram!****
	GPIOPinWrite(DRIVER_POS, STEP_POS, 0x40); 	//Set STEP pin HIGH
	setDelayMicro(PULSE_DELAY_MICROSEC); 		//us delay to comply with DRV8825 timing diagram
	GPIOPinWrite(DRIVER_POS, STEP_POS, 0); 		//Set STEP pin LOW
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
//-------------------------------------------------------------------
//-------------------------- Dispenser Arms -------------------------
void openArms(){

	//Move servos to one extreme position (at the same time)
	while(adjust < 119 && adjustLeft > 41)
	{
		adjust++;
		adjustLeft--;
		if (adjust > 120) { adjust = 120; }
		if (adjustLeft < 40) { adjustLeft = 40; }
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, adjust * load / 1000); //Servo Motor 2
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, adjustLeft * load / 1000); //Servo Motor 1
	}
}

void closeArms(){
	//Move servos to one extreme position (at the same time)
	while(adjust > 41 && adjustLeft < 119)
	{
		adjust--;
		adjustLeft++;
		if (adjust < 40) { adjust = 40; }
		if (adjustLeft > 120) { adjustLeft = 120; }
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, adjust * load / 1000); //Servo Motor 2
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, adjustLeft * load / 1000); //Servo Motor 1
	}
}
//----------------------------------------------------------------------
//--------------------- DELAY FUNCTIONS -----------------------------------
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
//-------------------------------------------------------------------------
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
//----------------------------------------------------

