/*
 * private.c
 *
 *  Created on: Nov 4, 2023
 *      Author: karim
 */


#include "main.h"
#include "MIROS.h"

/* function implementation of Q_onAssert */


void Q_onAssert(char const *module, int loc){
	/*TBD change control */
	(void)module;
	(void)loc;
	HAL_NVIC_SystemReset();
}

void OS_onStartup(void){

	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);

	NVIC_SetPriority(SysTick_IRQn,0U);

}


void OS_onIdle(void){


}
