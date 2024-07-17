#include "motor.h"
#include "stm32g4xx_ll_system.h"

/*! \brief Motor Handler Definition.
 *
 *  Record motor status.
 */
MOTOR_TYPE_t motor_data = {
	.M_TIMx					= NULL,
	.pwm						= 0,
	.pwmduty_limit	= 0,
	.Hallstep				= 0,
	.dir 						= DIRECTION_UNKNOWN,
	.past_dir				= DIRECTION_UNKNOWN,
};

void Motor_init(MOTOR_TYPE_t *pMotor, TIM_TypeDef *M_TIMx)
{
	pMotor->M_TIMx = M_TIMx;
	
	pMotor->pwmduty_limit = ( pMotor->M_TIMx->ARR + 1 );
	
	pMotor->pwm = 0;
	
	#if !defined(Release)
	LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM1_STOP );
	#endif
	
	LL_TIM_EnableCounter(pMotor->M_TIMx);

	#if 1
	LL_TIM_OC_SetCompareCH1(pMotor->M_TIMx, 0u);
	LL_TIM_OC_SetCompareCH2(pMotor->M_TIMx, 0u);
	LL_TIM_OC_SetCompareCH3(pMotor->M_TIMx, 0u);
	LL_TIM_CC_EnableChannel(pMotor->M_TIMx, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
	#else
	pMotor->M_TIMx->CCER = DISABLE_PWM;
	
	/* Select the Capture Compare preload feature */
  pMotor->M_TIMx->CR2 |= TIM_CR2_CCPC;
	
  /* Select the Commutation event source */
  pMotor->M_TIMx->CR2 &= ~TIM_CR2_CCUS;
  pMotor->M_TIMx->CR2 |= TIM_COMMUTATION_SOFTWARE;

  /* Disable Commutation Interrupt */
	pMotor->M_TIMx->DIER &= ~(TIM_IT_COM);

  /* Disable Commutation DMA request */
	pMotor->M_TIMx->DIER &= ~(TIM_DMA_COM);
	#endif

	LL_TIM_EnableAllOutputs(pMotor->M_TIMx);
	
	LL_TIM_EnableIT_UPDATE(pMotor->M_TIMx);
	
	return;
}

uint8_t GetHall(void)
{
	uint8_t hall;
	uint8_t regU, regV, regW;
	regU = ( LL_GPIO_IsInputPinSet(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin) << 2U );
	regV = ( LL_GPIO_IsInputPinSet(M1_HALL_H2_GPIO_Port, M1_HALL_H2_Pin) << 0U );
	regW = ( LL_GPIO_IsInputPinSet(M1_HALL_H3_GPIO_Port, M1_HALL_H3_Pin) << 1U );
	hall = regU | regV | regW;
	return hall;
}

inline static void BlockCommutationSetDuty(MOTOR_TYPE_t *pMotor)
{
	static const uint8_t pwm_table[4][8] = {
		{0, 0, 0, 0, 0, 0, 0, 0},	//	release table
		{0, 2, 3, 2, 1, 1, 3, 0},	//	forward table
		{0, 3, 1, 1, 2, 3, 2, 0},	 //	backward table
		{0, 0, 0, 0, 0, 0, 0, 0}	//	brake table
	};
	
	uint16_t dutyCycle	= 0;
	
	dutyCycle = ( pMotor->pwm < pMotor->pwmduty_limit ) ? pMotor->pwm : pMotor->pwmduty_limit;
	
	switch( pwm_table[pMotor->dir][pMotor->Hallstep] )
	{
		case 0: {
			TIM1->CCR1	= 0;
			TIM1->CCR2	= 0;
			TIM1->CCR3	= 0;
			break;
		}
		case 1: {
			TIM1->CCR1	= dutyCycle;
			TIM1->CCR2	= 0;
			TIM1->CCR3	= 0;
			break;
		}
		case 2: {
			TIM1->CCR1	= 0;
			TIM1->CCR2	= dutyCycle;
			TIM1->CCR3	= 0;
			break;
		}
		case 3: {
			TIM1->CCR1	= 0;
			TIM1->CCR2	= 0;
			TIM1->CCR3	= dutyCycle;
			break;
		}
		default: {
			TIM1->CCR1	= 0;
			TIM1->CCR2	= 0;
			TIM1->CCR3	= 0;
			break;
		}
	}
	return;
}

void BlockCommutate(MOTOR_TYPE_t *pMotor)
{
//	__disable_irq();
	
	static const uint32_t blockCommutationTable[4][8] = {
		{ DISABLE_PWM, 	DISABLE_PWM, 	DISABLE_PWM, 	DISABLE_PWM, 	DISABLE_PWM, 	DISABLE_PWM, 	DISABLE_PWM, DISABLE_PWM},	//	release table
		{ DISABLE_PWM, ENABLE_VH_WL, ENABLE_WH_UL, ENABLE_VH_UL, ENABLE_UH_VL, ENABLE_UH_WL, ENABLE_WH_VL, DISABLE_PWM},	//	forward table
		{ DISABLE_PWM, ENABLE_WH_VL, ENABLE_UH_WL, ENABLE_UH_VL, ENABLE_VH_UL, ENABLE_WH_UL, ENABLE_VH_WL, DISABLE_PWM},	//	backward table
		{ DISABLE_PWM, 		BRAKE_PWM, 		BRAKE_PWM, 		BRAKE_PWM, 		BRAKE_PWM, 		BRAKE_PWM, 		BRAKE_PWM, DISABLE_PWM}		//	break table
	};
	
	int8_t direction = pMotor->dir;
	
	pMotor->Hallstep = GetHall();
	
	TIM1->CCER = blockCommutationTable[direction][pMotor->Hallstep];

	BlockCommutationSetDuty(pMotor);
	
	LL_TIM_GenerateEvent_COM(TIM1);
	
	pMotor->past_dir = direction;
	
//	__enable_irq();
	return;
}