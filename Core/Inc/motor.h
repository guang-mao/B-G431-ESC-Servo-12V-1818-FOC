#ifndef __MOTOR_H_
#define __MOTOR_H_
#include "main.h"

#define PID_I_TERM_MAX				131072

#if 1
#define ENABLE_UH_VL					0x0155
#define ENABLE_WH_VL					0x0551
#define ENABLE_WH_UL					0x0515
#define ENABLE_VH_UL					0x0155
#define ENABLE_VH_WL					0x0551
#define ENABLE_UH_WL					0x0515
#define DISABLE_PWM						0x0111
#define BRAKE_PWM							0x0555
#else
#define ENABLE_UH_VL					0x0151
#define ENABLE_WH_VL					0x0151
#define ENABLE_WH_UL					0x0115
#define ENABLE_VH_UL					0x0115
#define ENABLE_VH_WL					0x0511
#define ENABLE_UH_WL					0x0511
#define DISABLE_PWM						0x0111
#define BRAKE_PWM							0x0555
#endif

#define LOW_SIDE_BRAKE		do{															\
														motor_data.pwm					= 0;	\
														motor_data.M_TIMx->CCR1	= 0;	\
														motor_data.M_TIMx->CCR2	= 0;	\
														motor_data.M_TIMx->CCR3	= 0;	\
													}while(0)
#define ALL_SIDE_RELEASE	do{															\
														motor_data.pwm					= 0;	\
														motor_data.M_TIMx->CCR1	= 0;	\
														motor_data.M_TIMx->CCR2	= 0;	\
														motor_data.M_TIMx->CCR3	= 0;	\
													}while(0)

//! Unknown direction flag value.
#define DIRECTION_UNKNOWN		 					0
													
//! Forward direction flag value.											
#define	DIRECTION_FORWARD			 				1
													
//! Reverse direction flag value.	
#define	DIRECTION_REVERSE							2
													
//! Brake direction flag value.
#define DIRECTION_BRAKING		 					3

typedef struct{
	TIM_TypeDef *M_TIMx;
	uint16_t		pwm;
	uint16_t		pwmduty_limit;
	uint8_t 		Hallstep;					//! The Hall sensor position feedback.
	int8_t 			dir;							//! The desired direction of rotation.
  int8_t 			past_dir;
} MOTOR_TYPE_t;

void Motor_init(MOTOR_TYPE_t *pMotor, TIM_TypeDef *M_TIMx);
uint8_t GetHall(void);
void BlockCommutate(MOTOR_TYPE_t *pMotor);

extern MOTOR_TYPE_t motor_data;

#endif

