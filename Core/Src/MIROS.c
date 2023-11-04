/*
 * MIROS.C
 *
 *  Created on: Oct 22, 2023
 *      Author: karim
 */

#include <stdint.h>
#include "MIROS.h"
#include "stm32f1xx.h"
#include "qassert.h"

Q_DEFINE_THIS_FILE

OSThread *volatile OS_current; /* pointer to the current thread */
OSThread *volatile OS_next;    /* pointer to the next thread to run */

OSThread *OS_thread[32+1]; /* array of threads */
uint8_t OS_threadNum; /* number of threads started so far */
uint8_t OS_currIdx; /* current thread index for round robin */

void OS_init(void){

	/* set the PendSV interrupt priority to the lowest level */
	*(uint32_t volatile*)0XE000ED20 |= (0xFFU << 16);
	/* set systick priority to 0 */
	NVIC_SetPriority(SysTick_IRQn,0U);
	NVIC_EnableIRQ(SysTick_IRQn);

}


void OS_sched(void){
	/* OS_next = ... */
	// increment the index of the currently running thread
	++OS_currIdx;
	if(OS_currIdx == OS_threadNum){
		OS_currIdx = 0U;
	}
	OS_next = OS_thread[OS_currIdx];

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


void OSThread_start(OSThread *me,
		OSThreadHandler threadHandler,
		void *stkSto, uint32_t stkSize)
{

	//initialie the sp to build the stack frame
	// stack grows from high to low memory
	// we need to start from the end of the provided stack memory
	// in cortex M the stack needs to be aligned to the 8 byte boundary
	uint32_t *sp = (uint32_t*)((((uint32_t)stkSto + stkSize) / 8)*8);
	uint32_t *stk_limit;

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


	Q_ASSERT(OS_threadNum < Q_DIM(OS_thread));
		/* register the thread with the OS */
		OS_thread[OS_threadNum] = me;

		/* increment the number of threads */
		++OS_threadNum;



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
