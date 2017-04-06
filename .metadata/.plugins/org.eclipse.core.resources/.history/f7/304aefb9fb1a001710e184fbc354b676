/*
 * tivaUtils.c
 *  Developed by: Reynaldo Belfort
 *  Team: Emmanuel Torres, Osvaldo Ramirez, Carlos Rodriguez, Reynaldo Belfort
 */
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "tivaUtils.h"

//TODO Document code

uint32_t computeDelayCount(uint32_t waitTime, uint8_t clockFreq){
	return 0.3333333 * (clockFreq*1000000) * (waitTime * 0.001);
}

//waitTime in milliseconds
uint32_t computeDelayCountMicrosec(float waitTime, uint8_t clockFreq){
    return 0.3333333 * (clockFreq*1000000) * (waitTime * 0.001);
}

//Set CPU delay considering the CLOCK_FREQ constant
//waitTime - time to delay in milliseconds
void setDelay(uint32_t waitTime){
	SysCtlDelay(computeDelayCount(waitTime, CLOCK_FREQ));
}

//waitTime - time to delay in milliseconds
void setDelayMicro(float waitTime){
	SysCtlDelay(computeDelayCountMicrosec(waitTime*0.001, CLOCK_FREQ));
}
