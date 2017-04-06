/*****************************************************************************
 *	 		Stepper Motor Library for the DRV8825 Stepper driver
 *	HEADER file: stepperSWDriver.h - function prototype and constant definitions
 *
 *  TODO Insert description here
 *
 *  Developed by Reynaldo Belfort
 *  R-Tiles Team:
 *  Emmanuel Ramos
 *  Reynaldo Belfort
 *  Carlos Rodriguez
 *  Osvaldo Ramirez
 *
 * ----------------------Notes----------------------
 *
*****************************************************************************/

#ifndef _STEPPER_SW_DRIVER_H
#define _STEPPER_SW_DRIVER_H

//---------------------- CONSTANT definitions ----------------------
#define PULSE_DELAY_MICROSEC 500 //microseconds 4%-4.8% error This number is not necesarily accurate.
#define STEP_DUTY_CYCLE 0.5

//++++PIN Setup++++
#define DRIVER_PORT GPIO_PORTE_BASE
#define DRIVER_STEP_PIN GPIO_PIN_1
#define DRIVER_DIR_PIN GPIO_PIN_2

//----------------------------------------------------------------------------------------


//----------------------Function prototypes-------------------------

extern void performStep(bool direction);
extern void performAmountSteps(uint32_t amountSteps, bool direction);

//------------------------------------------------------------------

#endif
