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

//DRV8825 Pin Setup
#define DRIVER_PORT GPIO_PORTE_BASE
#define DRIVER_STEP_PIN GPIO_PIN_1
#define DRIVER_DIR_PIN GPIO_PIN_2
//Motor characteristics
#define MOTOR_WHEEL_CIRCUM_METERS 0.339134927012
#define MOTOR_WHEEL_CIRCUM_INCHES 13.35176878
#define MOTOR_STEPS_PER_REV 200
#define MOTOR_STEP_ANGLE 1.800

//----------------------------------------------------------------------------------------


//----------------------Function prototypes-------------------------

extern void performStep(bool direction);
extern void performStepSpecific(bool clockwise, int driverPort, int gpioPin_STEP, int gpioPin_DIR);
extern void performAmountSteps(uint32_t amountSteps, bool direction);
extern void spinStepperMeters(double distance, bool clockwise, uint16_t delay);
extern void spinStepperInches(double distance, bool clockwise, uint16_t delay);
extern void spinStepperAngle(float angle, bool clockwise, uint16_t delay);


//------------------------------------------------------------------

#endif
