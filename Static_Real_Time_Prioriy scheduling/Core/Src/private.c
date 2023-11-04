/*
 * MIROS - Minimal Real-Time Operating System
 *
 * MIROS is a compact and efficient real-time operating system
 *
 * designed for resource-constrained embedded systems.
 *
 * It provides a cooperative multitasking environment for managing multiple threads,
 *
 * scheduling, and synchronization.
 *
 * """" Static Priority Based Schedulers """"
 *
 * Author: Karim Tarek
 * Version: 1.0
 * Created on: October 22, 2023
 *
 * private.c
 * source file for MIROS
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
/* stop the CPU and wait for interrupt */

	__WFI();
}


