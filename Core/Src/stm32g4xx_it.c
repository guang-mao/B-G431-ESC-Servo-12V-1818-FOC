/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
uint8_t pwm_duty_cycle = 10u;
alphabeta_t Valphabeta;
Vabc_t	Vabc;
uint16_t CCR1, CCR2, CCR3;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

	/* Executes the Medium Frequency Task functions for each drive instance */
	MC_Scheduler();
	
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
	
	if ( LL_TIM_IsActiveFlag_UPDATE(TIM1) && LL_TIM_IsEnabledIT_UPDATE(TIM1) )
	{
		LL_TIM_ClearFlag_UPDATE(TIM1);
		
		qd_t Vqd;
		
		
		
		(void)HALL_CalcElAngle(&HALL_M1);
		
		if ( bootstage >= ENDPLACE )
		{
			Vabc_t Vtmp;
			
			int16_t half_ARR = ( motor_data.M_TIMx->ARR + 1 ) / 2;
			int16_t center = (int16_t) 0; // ( motor_data.M_TIMx->ARR + 1 ) / 2;
			
			#if 0
			uint8_t tmp = ( pwm_duty_cycle > 100 ) ? 100 : pwm_duty_cycle;	
			motor_data.pwm = ( ( motor_data.M_TIMx->ARR + 1 ) * tmp ) / 100;
			motor_data.dir = DIRECTION_FORWARD; //DIRECTION_REVERSE;//DIRECTION_FORWARD;// DIRECTION_UNKNOWN
			BlockCommutate(&motor_data);
			#endif
			
			HAL_GPIO_WritePin(STATUS_GPIO_Port, STATUS_Pin, GPIO_PIN_SET);
			
			Vqd.q = 1024;
			Vqd.d = 0;
			
			Valphabeta = MCM_Rev_Park(Vqd, HALL_M1.MeasuredElAngle);

			int16_t wbeta = (int16_t) ( ( _SQRT3_2 * Valphabeta.beta ) >> 15 );
			
			Vtmp.a =  Valphabeta.alpha;
			Vtmp.b = -Valphabeta.alpha / 2 + wbeta;
			Vtmp.c = -Valphabeta.alpha / 2 - wbeta;
			
			// SpaceVectorPWM
			int16_t Umin = min(Vtmp.a, min(Vtmp.b, Vtmp.c));
			int16_t Umax = max(Vtmp.a, max(Vtmp.b, Vtmp.c));
			
			center -= ( Umax + Umin ) / 2;
			
			Vabc.a = Vtmp.a + center;
			Vabc.b = Vtmp.b + center;
			Vabc.c = Vtmp.c + center;
			
			CCR1 = ( ( Vabc.a * half_ARR ) >> 15 ) + half_ARR;
			CCR2 = ( ( Vabc.b * half_ARR ) >> 15 ) + half_ARR;
			CCR3 = ( ( Vabc.c * half_ARR ) >> 15 ) + half_ARR;
			
			motor_data.M_TIMx->CCR1 = CCR3; //123 132 213 231 321
			motor_data.M_TIMx->CCR2 = CCR1;
			motor_data.M_TIMx->CCR3 = CCR2;
			
			HAL_GPIO_WritePin(STATUS_GPIO_Port, STATUS_Pin, GPIO_PIN_RESET);
		}
	}
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	/* HALL Timer Update IT always enabled, no need to check enable UPDATE state */
  if ( LL_TIM_IsActiveFlag_UPDATE(HALL_M1.TIMx) == 0U )
  {
    /* Nothing to do */
  }
  else
  {
    LL_TIM_ClearFlag_UPDATE(HALL_M1.TIMx);
    (void)HALL_TIMx_UP_IRQHandler(&HALL_M1);
    /* USER CODE BEGIN M1 HALL_Update */

    /* USER CODE END M1 HALL_Update   */
  }

  /* HALL Timer CC1 IT always enabled, no need to check enable CC1 state */
  if ( LL_TIM_IsActiveFlag_CC1 (HALL_M1.TIMx) != 0U )
  {
    LL_TIM_ClearFlag_CC1(HALL_M1.TIMx);
    (void)HALL_TIMx_CC_IRQHandler(&HALL_M1);
//		BlockCommutate(&motor_data);
    /* USER CODE BEGIN M1 HALL_CC1 */

    /* USER CODE END M1 HALL_CC1 */
  }
  else
  {
  /* Nothing to do */
  }
  /* USER CODE END TIM4_IRQn 0 */
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
