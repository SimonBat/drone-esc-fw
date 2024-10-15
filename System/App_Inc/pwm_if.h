#ifndef __PWM_IF_H
#define __PWM_IF_H

#include "stm32f0xx_ll_tim.h"
#include "mc_cfg.h"

#define PWM_IF_TIM                    	TIM16
#define PWM_IF_TIM_COUNTER_MODE       	TIM_COUNTERMODE_UP
#define PWM_IF_TIM_CLK_ENABLE()       	__HAL_RCC_TIM16_CLK_ENABLE()
/////#define PWM_IF_TIM_CLK_DISABLE()      	__HAL_RCC_TIM16_CLK_DISABLE()
/////#define PWM_IF_TIM_FREEZE_DBGMCU()    	__HAL_DBGMCU_FREEZE_TIM16()
#define PWM_IF_TIM_IRQn               	TIM16_IRQn
#define PWM_IF_TIM_PRIORITY           	2U
#define PWM_IF_TIM_PSC                	1U

#define PWM_IF_COUNTER_CYCLE_TIME      	((uint32_t)(((1<<16)/(MC_CFG_SYSCLOCK_FREQUENCY/1000000))*(PWM_IF_TIM_PSC+1U))) /* (2^-16) us */
#define PWM_IF_TIM_ARR                	0xFFFF

/////#define PWM_IF_TIM_STOP_MS            	1500U
/////#define PWM_IF_TIM_PERIOD_US          	(((BSP_BOARD_IF_TIMx_ARR+1)*BSP_BOARD_IF_COUNTER_CYCLE_TIME)>>16)
/////#define PWM_IF_TIM_STOP_PERIODS       	((BSP_BOARD_IF_TIMx_STOP_MS*1000)/(BSP_BOARD_IF_TIMx_PERIOD_US))
/////#define PWM_IF_TIM_ARMING_VALID_TON   	10U
#define PWM_IF_TIM_START_VALID_TON    	10U
#define PWM_IF_TIM_STOP_VALID_TON     	10U
#define PWM_IF_TIM_MIN_MAX_BITS       	10U
#define PWM_IF_TIM_MIN_SPEED_TON_US   	1060U
#define PWM_IF_TIM_MAX_SPEED_TON_US   	(PWM_IF_TIM_MIN_SPEED_TON_US+(1<<PWM_IF_TIM_MIN_MAX_BITS))
#define PWM_IF_TIM_MIN_SPEED_TON      	((uint32_t)(PWM_IF_TIM_MIN_SPEED_TON_US<<16)) /* (2^-16) us */
#define PWM_IF_TIM_MAX_SPEED_TON      	((uint32_t)(PWM_IF_TIM_MAX_SPEED_TON_US<<16)) /* (2^-16) us */
#define PWM_IF_SPEED_RANGE				((uint32_t)((PWM_IF_TIM_MAX_SPEED_TON-PWM_IF_TIM_MIN_SPEED_TON)>>16))

#define PWM_IF_TIM_CH1_PIN       		BSP_IF_TIM_CH1_PIN
#define PWM_IF_TIM_CH1_PORT      		BSP_IF_TIM_CH1_PORT
#define PWM_IF_TIM_AF            		GPIO_AF5_TIM16
#define PWM_IF_GPIO_CLK_ENABLE()  		__HAL_RCC_GPIOA_CLK_ENABLE()

typedef struct{
	TIM_HandleTypeDef timIF;
	uint16_t pwmInput1stRisingEdge;
	uint16_t pwmInputFallingEdge;
	/////uint16_t pwmInput2ndRisingEdge;
	uint32_t pwmInputTonCapture;
	/////uint32_t pwmInputToffCapture;
	/////uint16_t pwmInputCaptureIndex;
	uint32_t pwmInputTon;
	/////uint32_t pwmInputPeriod;
	/////uint16_t armingCnt;
	uint16_t startCnt;
	uint16_t stopCnt;
	uint16_t blankStopCnt;
	/////uint16_t speedCnt;
	int32_t speedCommand;
	/////int32_t speedCommandOld;
}pwm_if_ts;

/* Global functions declarations */
void PWM_IF_Init(void);

#endif
