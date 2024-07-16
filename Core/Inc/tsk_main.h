#ifndef __TSK_MAIN_H_
#define __TSK_MAIN_H_
#include "stm32g4xx_hal.h"

typedef enum
{
			 INIT = 0,
			SETUP,
	STAYPLACE,
	 ENDPLACE
} BOOTSTAGE_t;

extern BOOTSTAGE_t bootstage;

void tsk_main(void);
void osRtxIdleThread(void *argument);
/* Executes the Medium Frequency Task functions for each drive instance */
void MC_Scheduler(void);

#endif
