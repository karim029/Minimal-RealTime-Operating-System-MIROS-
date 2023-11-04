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
 * Author: Karim Tarek
 * Version: 1.0
 * Created on: October 22, 2023
 *
 * MIROS.h
 * Header file for MIROS
 */
#ifndef INC_MIROS_H_
#define INC_MIROS_H_

/* TCB */
typedef struct{

	void* sp; /* stack pointer */
	uint32_t timeout; /* timeout delay down-counter */
}OSThread;

typedef void (*OSThreadHandler)(void);


/* notice ! this function must be called with interrupts DISABLED */
void OS_sched(void);

void OS_init(void *stkSto, uint32_t stkSize);

/* callback to handle the idle condition */
void OS_onIdle(void);


/*
 *  OSThread *me a pointer to the TCB
 *  threadHandler a pointer to the thread function
 *  void* stkSto, uint32_t stkSize memory of the private stack and size
 */

void OSThread_start(OSThread *me,
		OSThreadHandler threadHandler,
		void *stkSto, uint32_t stkSize);

/* transfer control to the RTOS to run the threads */
void OS_run(void);

/* blocking delay  */
void OS_delay(uint32_t ticks);

/* process  timeouts */
void OS_tick(void);

/* callback to configure and start interrupts */
void OS_onStartup(void);

#endif /* INC_MIROS_H_ */
