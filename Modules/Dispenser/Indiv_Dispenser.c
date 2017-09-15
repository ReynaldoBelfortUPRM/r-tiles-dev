5#include <stdint.h>
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
short currentState = 2;
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

//-------- Stepper motor Global variables---------
bool runMotor = true;
bool runDirectionDispenser = true; //FALSE = upward , TRUE = downward
//---------------------------------------------

/* Constant declarations Servo Motor */
#define PWM_FREQUENCY 50
#define PORT_SERVO GPIO_PORTD_BASE
#define PIN_SERVO_1 GPIO_PIN_1
#define PIN_SERVO_2 GPIO_PIN_0

/* Constant declarations Stepper Motor */
#define PULSE_DELAY 1
#define PULSE_DELAY_MICROSEC 500 //microseconds 4%-4.8% error This number is not necesarily accurate.
#define DRIVER_DISP_ELE GPIO_PORTE_BASE
#define STEP_DISP_ELE GPIO_PIN_1
#define DIRECTION_DISP_ELE GPIO_PIN_2

/* General Constant Declarations */
#define CLOCK_FREQ 40
#define ALLPINS GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7

/* Interrupt Service Routines */
void PUSH_ISR();			//Push Button Interrupt service Routine
void LIMIT_SWITCH_ISR();	//Switch interrupt Service Routine

//------------------ Function Declarations -------------------------
/* Stepper motor functions */
void performStep(bool clockwise); 	//Performs single step

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
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); 		//Driver Step's'
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);		//Tiva buttons
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	//--------------------------

	//------- Stepper motor driver configuration configuration ------
	GPIOPinTypeGPIOOutput(DRIVER_DISP_ELE, STEP_DISP_ELE | DIRECTION_DISP_ELE); //Driver's STEP pin
	//---------------------------------

	//---------Set up Port F microcontroller buttons ---------
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	//-------------------------------------------
	//-------- Interrupt setup Microcontroller buttons ---------
	GPIOIntRegister(GPIO_PORTF_BASE, PUSH_ISR);
	GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_FALLING_EDGE);  // Set PB2/3 to falling edge
	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
	//--------------------------------

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

	//-------------------Register Limit Switch Pin--------------
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntRegister(GPIO_PORTA_BASE, LIMIT_SWITCH_ISR);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);
	//GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_5|GPIO_PIN_6);
	//----------------------------------------------------------

	while(1){
		if(currentState == 2){
			GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_5|GPIO_PIN_6);
			while(currentState == 2){
				//TODO si se activa el switch de arriba guardarlo en una variable y chequiar
				if(pushFlag){
					if(pushButton == 16){
						readyToDispense = true;
						//closeArms();
						//dispenserUp = true;
						//dispenserDown = false;
					}
					//			else if(pushButton == 0){
					//				runMotor = false;
					//				//openArms();
					//				//dispenserDown = true;
					//				//dispenserUp = false;
					//			}
				}
				pushButton = 0;
				if(readyToDispense && dispenserUp1){
					performStep(runDirectionDispenser); //just in a single direction
					runMotor = true;
					while(runMotor == true){
						performStep(runDirectionDispenser); //just in a single direction
						setDelay(1); //in ms
					}
					//TODO wait 1 second & then open arms
					//openArms();
					//TODO wait 1 second & then go up again
					runMotor = true;
					setDelay(100);
					while(runMotor == true){
						performStep(runDirectionDispenser); //just in a single direction
						setDelay(1); //in ms
					}
					//closeArms();
					//TODO go to next State
					currentState = 3;
					readyToDispense = false;
				}
			}
		}

		//Left and right push buttons are used to turn motor clockWise or coutnerClockwise respectively
		//		if(pushFlag){
		//			pushFlag = false;
		//			//If button was pushed start moving motor
		//			if(pushButton == 16){ //SW1 button
		//				int counter = 0;
		//				runMotor = true;
		//				for(; counter < 300; counter++){
		//					//TODO Change to while(runMotor)
		//					if(runMotor){
		//						performStep(runDirectionDispenser); //just in a single direction
		//						setDelay(1); //in ms
		//					}else{
		//						counter = 300;
		//					}
		//				}
		//			}
		//		}
		//		if(dispenserDown){
		//			openArms();
		//			dispenserDown = false;
		//		}else if(dispenserUp){
		//			closeArms();
		//			dispenserUp = false;
		//		}
	}
}

//----------------- ISR Functions-------------------------------
void PUSH_ISR(){
	SysCtlDelay(4000000); //For debouncing
	pushFlag = true;
	pushButton = GPIOIntStatus(GPIO_PORTF_BASE,true); //To read which button interrupted
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0);
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
//-------------------------------------------------------------------
//-------------------------- Dispenser Arms -------------------------
//Extra functions
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
