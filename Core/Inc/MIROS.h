/*
 * MIROS.h
 *
 *  Created on: Oct 22, 2023
 *      Author: karim
 */

#ifndef INC_MIROS_H_
#define INC_MIROS_H_

/* TCB */
typedef struct{

	void* sp; /* stack pointer */

}OSThread;

typedef void (*OSThreadHandler)(void);


/* notice ! this function must be called with interrupts DISABLED */
void OS_sched(void);

void OS_init(void);

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

/* callback to configure and start interrupts */
void OS_onStartup(void);

#endif /* INC_MIROS_H_ */
