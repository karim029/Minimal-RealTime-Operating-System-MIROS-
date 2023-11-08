/*
 * MIROS - Minimal Real-Time Operating System
 *
 * MIROS is a compact and efficient real-time operating system
 *
 *  designed for resource-constrained embedded systems.
 *
 *  It provides a cooperative multitasking environment for managing multiple threads,
 *
 *   scheduling, and synchronization.
 *
 * """" Static Priority Based Schedulers """"
 *
 * Author: Karim Tarek
 * Version: 1.0
 * Created on: October 22, 2023
 *
 * MIROS.c
 * Source file for MIROS
 */

#include <stdint.h>
#include "MIROS.h"
#include "stm32f1xx.h"
#include "qassert.h"


Q_DEFINE_THIS_FILE

OSThread *volatile OS_current; /* pointer to the current thread */
OSThread *volatile OS_next;    /* pointer to the next thread to run */

OSThread *OS_thread[32+1]; /* array of threads */
uint32_t OS_readySet; /* bitmak of threads that are ready to run */
uint32_t OS_delayedSet; /* bitmak of threads that are delayed */

#define LOG2(x) (32 - __CLZ(x))

/* The Idle thread */

OSThread idleThread;

void main_idleThread(){

	while(1){

		OS_onIdle();
	}

}


void OS_init(void *stkSto, uint32_t stkSize){

	/* set the PendSV interrupt priority to the lowest level */
	*(uint32_t volatile*)0XE000ED20 |= (0xFFU << 16);
	/* set systick priority to 0 */
	NVIC_SetPriority(SysTick_IRQn,0U);
	NVIC_EnableIRQ(SysTick_IRQn);


	/* start idle Thread */
	OSThread_start(&idleThread,0,&main_idleThread,stkSto, stkSize);


}


void OS_sched(void){
	/* OS_next = ... */
	if(OS_readySet == 0U){ /* idle condition */
		OS_next = OS_thread[0];  /* the idle thread  */
	}
	else{
		// find the highest priority thread that is ready to run using the LOG2 macro
		OS_next = OS_thread[LOG2(OS_readySet)];
		Q_ASSERT(OS_next != (OSThread*)0);
	}

	if(OS_next != OS_current){
		//trigger pendSV
		*(uint32_t volatile *)0XE000ED04 = (1U << 28);

	}

}

void OS_run(void){

	/* callback to configure and start interrupts */
	OS_onStartup();

	__disable_irq();
	OS_sched();
	__enable_irq();

	/* the following code should never excute */
	Q_ERROR();
}

// function will be called from isr (systick handler)
// no need to disable irq because the ISR cannot be preempted by a thread
void OS_tick(void){

	uint32_t workingSet = OS_delayedSet;

	while(workingSet != 0U){
		OSThread* t = OS_thread[LOG2(workingSet)];
		uint32_t bit;
		Q_ASSERT((t != (OSThread*)0) && (t->timeout != 0U));


		bit = (1U << (t->priority - 1U));
		--t->timeout;
		if(t->timeout == 0U){
			OS_readySet |= bit; /* set the corresponding priority bit in readySet */
			OS_delayedSet &= ~bit; /*remove the same bit from the bit mask because the corresponding thread is no longer delayed */
		}

		workingSet &= ~bit; /* remove the same priority bit from workingset bitmask because its now processing */
	}

}

void OS_delay(uint32_t ticks){

	uint32_t bit;

	// entering a critical section IRQ must be disabled
	__disable_irq();

	/* never call OS_delay from the idleThread */
	Q_REQUIRE(OS_current != OS_thread[0]);

	OS_current->timeout = ticks;

	bit = (1U << (OS_current->priority - 1U));
	/* make the thread not ready to run by clearing its corresponding bit in the OS_ready_Set */
	OS_readySet &= ~bit;
	/* set the corresponding priorty bit of the thread inn the delayedset bitmask, because the thread is being delayed */
	OS_delayedSet |= bit;

	/* immediately switch the context away from the thread in order to become blocked */
	OS_sched();
	__enable_irq();

}

void OSThread_start(OSThread *me,uint8_t priorty ,
		OSThreadHandler threadHandler,
		void *stkSto, uint32_t stkSize)
{

	//initialie the sp to build the stack frame
	// stack grows from high to low memory
	// we need to start from the end of the provided stack memory
	// in cortex M the stack needs to be aligned to the 8 byte boundary
	uint32_t *sp = (uint32_t*)((((uint32_t)stkSto + stkSize) / 8)*8);
	uint32_t *stk_limit;


	// assert the number of priority with the available threads
		// and if the thread is not used
		Q_REQUIRE((priorty < Q_DIM(OS_thread))
			&& (OS_thread[priorty] == (OSThread*)0));

	*(--sp) = (1U<<24); /* xPSR */
	*(--sp) = (uint32_t)threadHandler; /* PC */
	*(--sp) = 0x0000000EU; /* LR */
	*(--sp) = 0x0000000CU; /* R12 */
	*(--sp) = 0x00000003U; /* R3 */
	*(--sp) = 0x00000002U; /* R2 */
	*(--sp) = 0x00000001U; /* R1 */
	*(--sp) = 0x00000000U; /* R0 */

	// additionally, corruptable register R4-R11
	*(--sp) = 0x0000000BU; /* R11 */
	*(--sp) = 0x0000000AU; /* R10 */
	*(--sp) = 0x00000009U; /* R9 */
	*(--sp) = 0x00000008U; /* R8 */
	*(--sp) = 0x00000007U; /* R7 */
	*(--sp) = 0x00000006U; /* R6 */
	*(--sp) = 0x00000005U; /* R5 */
	*(--sp) = 0x00000004U; /* R4 */

	// at this point the stack frame is built

	// save the top of the stack into the sp member of the OSThread struct
	me->sp = sp;

	// round the bottom of the stack to the 8 byte boundary
	stk_limit = (uint32_t*)(((((uint32_t)stkSto - 1U)/8) + 1U)* 8);

	// pre-fill the remaining stack with bit pattern (0xDEADBEEF) easier for debugging
	for(sp = sp - 1U;sp >= stk_limit; --sp){
		*sp = 0xDEADBEEFU;
	}


		/* register the thread with the OS */
		OS_thread[priorty] = me;
		/* save priority in the TCB */
		me->priority = priorty;

		/* make the thread ready to run */
		if(priorty > 0U){

			OS_readySet |= (1U << (priorty - 1u));
		}

}


void PendSV_Handler(){

	/*__disable_irq(); */
	__asm volatile(
		  "CPSID		I \n					"
	/* if(OS_current != (OSThread*)0) */
		  "LDR			r1,=OS_current \n		"
		  "LDR			r1,[r1,#0x00]	\n		"
		  "CBZ			r1,PendSV_restore \n 	"
	/*push registers R4-R11 into the stack */
		  "PUSH			{r4-r11}			\n	"
		  "LDR			r1,=OS_current		\n	"
		  "LDR			r1,[r1,#0x00]		\n	"
	//save content of the current thread
			/* OS_current->sp = sp; */
		  "STR			sp,[r1,#0x00]		\n	"
		  "PendSV_restore:					\n	"
	// restore the content of the next thread
			/* sp = OS_next->sp; */
		  "LDR			r1,=OS_next			\n	"
		  "LDR			r1,[r1,#0x00]		\n	"
		  "LDR			sp,[r1,#0x00]		\n	"
			/* OS_current = OS_next; */
		  "LDR			r1,=OS_next			\n	"
		  "LDR			r1,[r1,#0x00]		\n	"
		  "LDR			r2,=OS_current		\n	"
		  "STR			r1,[r2,#0x00]		\n	"
			/* Pop registers R4-R11 from stack */
		  "POP			{r4-r11}			\n	"
			/* __enable_irq();  */
		  "CPSIE		I					\n	"
			/* return to the next thread */
		  "BX			lr 						"





	);




}
