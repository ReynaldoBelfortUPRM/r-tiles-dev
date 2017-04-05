/*
 * tivaUtils.c
 *
 *  Created on: Mar 7, 2017
 *      Team: Emmanuel Torres & Reynaldo Belfort
 */
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "tivaUtils.h"

uint32_t computeDelayCount(uint32_t waitTime, uint8_t clockFreq){
	return 0.3333333 * (clockFreq*1000000) * (waitTime * 0.001);
}

