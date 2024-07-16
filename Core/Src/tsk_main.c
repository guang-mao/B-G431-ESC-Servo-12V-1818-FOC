#include "main.h"

static uint16_t hMFTaskCounterM1 = 0; //cstat !MISRAC2012-Rule-8.9_a
static volatile uint8_t bMCBootCompleted = ((uint8_t)0);
static uint32_t bootTick = 0;
BOOTSTAGE_t bootstage = INIT;


/**
* tsk_main
**/
void tsk_main(void)
{
	/******************************************************/
  /*   Main motor component initialization       */
  /******************************************************/
	Motor_init(&motor_data, TIM1);
	
	/******************************************************/
  /*   Main speed sensor component initialization       */
  /******************************************************/
  HALL_Init (&HALL_M1);
	
	bMCBootCompleted = 1U;
	
	bootTick = HAL_GetTick();
	return;
}

/**
* osRtxIdleThread
**/
void osRtxIdleThread(void *argument)
{
	uint32_t elapse = HAL_GetTick() - bootTick;
	
	switch ( bootstage )
	{
		case INIT: {
			if ( elapse > 50 )
			{
				bootstage = SETUP;
			}
			break;
		}
		case SETUP: {
			if ( elapse > 150 )
			{
				bootstage = STAYPLACE;
			}
			break;
		}
		case STAYPLACE: {
			if ( elapse > 350 )
			{
				bootstage = ENDPLACE;
			}
			break;
		}
		case ENDPLACE: {
			;	// do nothing...
			break;
		}
	}
	return;
}


/**
 * @brief  Executes the Medium Frequency Task functions for each drive instance.
 *
 * It is to be clocked at the Systick frequency.
 */
__weak void MC_Scheduler(void)
{
  if ( ( (uint8_t) 1 ) == bMCBootCompleted )
  {
    if ( hMFTaskCounterM1 > 0u )
    {
      hMFTaskCounterM1--;
    }
    else
    {
      int16_t wAux = 0;
			bool IsSpeedReliable = HALL_CalcAvrgMecSpeedUnit(&HALL_M1, &wAux);
      hMFTaskCounterM1 = (uint16_t) 1;
    }
  }
  else
  {
    /* Nothing to do */
  }
}