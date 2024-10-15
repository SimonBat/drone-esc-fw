/**
  ***************************************************************************************************************************************
  * @file     pwm_if.c
  * @owner    SimonBat
  * @version  V0.0.1
  * @date     2024.01.31
  * @update   2024.01.31
  * @brief    drone_esc_fw
  ***************************************************************************************************************************************
  * @attention
  *
  * (Where to use)
  *
  ***************************************************************************************************************************************
  */

#include "pwm_if.h"
#include "system_it.h"
#include "mc.h"

static pwm_if_ts H_PWM_IF;

/**
  ***************************************************************************************************************************************
  * @brief  PWM input interface initialization
  * @param  None
  * @retval None
  ***************************************************************************************************************************************
  */
void  PWM_IF_Init(void)
{
	TIM_IC_InitTypeDef _sIcConfig;

	H_PWM_IF.timIF.Instance=PWM_IF_TIM;
	H_PWM_IF.timIF.Init.Prescaler=PWM_IF_TIM_PSC;
	H_PWM_IF.timIF.Init.CounterMode=PWM_IF_TIM_COUNTER_MODE;
	H_PWM_IF.timIF.Init.Period=PWM_IF_TIM_ARR;
	H_PWM_IF.timIF.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	H_PWM_IF.timIF.Init.RepetitionCounter=0U;
	H_PWM_IF.timIF.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
	if(HAL_OK!=HAL_TIM_IC_Init(&H_PWM_IF.timIF)){BSP_Error_Handler(BSP_CFG_ERR_PWM_IF);}

	_sIcConfig.ICPolarity=TIM_ICPOLARITY_RISING;
	_sIcConfig.ICSelection=TIM_ICSELECTION_DIRECTTI;
	_sIcConfig.ICPrescaler=TIM_ICPSC_DIV1;
	_sIcConfig.ICFilter=2U;
	if(HAL_OK!=HAL_TIM_IC_ConfigChannel(&H_PWM_IF.timIF,&_sIcConfig,TIM_CHANNEL_1)){BSP_Error_Handler(BSP_CFG_ERR_PWM_IF);}
	if(HAL_OK!=HAL_TIM_IC_Start_IT(&H_PWM_IF.timIF,TIM_CHANNEL_1)){BSP_Error_Handler(BSP_CFG_ERR_PWM_IF);}
	__HAL_TIM_ENABLE_IT(&H_PWM_IF.timIF,TIM_IT_UPDATE);
}

/**
  ***************************************************************************************************************************************
  * @brief  HAL timer input capture MSP initialization
  * @param  Timer handle (mc_ts*)
  * @retval None
  ***************************************************************************************************************************************
  */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *_hTim)
{
	UNUSED(_hTim);
	GPIO_InitTypeDef _gpioInitStruct={0};

	PWM_IF_TIM_CLK_ENABLE();
	PWM_IF_GPIO_CLK_ENABLE();

	_gpioInitStruct.Pin=PWM_IF_TIM_CH1_PIN;
	_gpioInitStruct.Mode=GPIO_MODE_AF_PP;
	_gpioInitStruct.Pull=GPIO_PULLUP;
	_gpioInitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
	_gpioInitStruct.Alternate=PWM_IF_TIM_AF;
	HAL_GPIO_Init(PWM_IF_TIM_CH1_PORT,&_gpioInitStruct);

	HAL_NVIC_SetPriority(PWM_IF_TIM_IRQn,PWM_IF_TIM_PRIORITY,0U);
	/* Enable the TIM global Interrupt */
	HAL_NVIC_EnableIRQ(PWM_IF_TIM_IRQn);
}

/**
  ***************************************************************************************************************************************
  * @brief  PWM input interface IRQ handler
  * @param  None
  * @retval None
  ***************************************************************************************************************************************
  */
void __attribute__((section(".RamFunc"))) TIM16_IRQHandler(void)
{
	if(__HAL_TIM_GET_FLAG(&H_PWM_IF.timIF,TIM_FLAG_CC1)&&__HAL_TIM_GET_IT_SOURCE(&H_PWM_IF.timIF,TIM_IT_CC1))
	{
		__HAL_TIM_CLEAR_IT(&H_PWM_IF.timIF,TIM_IT_CC1);

		if(LL_TIM_IC_POLARITY_RISING==LL_TIM_IC_GetPolarity(H_PWM_IF.timIF.Instance,LL_TIM_CHANNEL_CH1))
		{
			/* Get the 1st Input Capture value */
			H_PWM_IF.pwmInput1stRisingEdge=HAL_TIM_ReadCapturedValue(&H_PWM_IF.timIF,TIM_CHANNEL_1);
			LL_TIM_IC_SetPolarity(H_PWM_IF.timIF.Instance,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_FALLING);
			H_PWM_IF.blankStopCnt=0U;
		}
		else if(LL_TIM_IC_POLARITY_RISING!=LL_TIM_IC_GetPolarity(H_PWM_IF.timIF.Instance,LL_TIM_CHANNEL_CH1))
		{
			/* Get the 2nd Input Capture value */
			H_PWM_IF.pwmInputFallingEdge=HAL_TIM_ReadCapturedValue(&H_PWM_IF.timIF,TIM_CHANNEL_1);

			/* Capture computation */
			if(H_PWM_IF.pwmInputFallingEdge>H_PWM_IF.pwmInput1stRisingEdge)
			{H_PWM_IF.pwmInputTonCapture=(H_PWM_IF.pwmInputFallingEdge-H_PWM_IF.pwmInput1stRisingEdge);}
			else if(H_PWM_IF.pwmInputFallingEdge<H_PWM_IF.pwmInput1stRisingEdge)
			{H_PWM_IF.pwmInputTonCapture=((0xFFFF-H_PWM_IF.pwmInput1stRisingEdge)+H_PWM_IF.pwmInputFallingEdge)+1U;}
			else{BSP_Error_Handler(BSP_CFG_ERR_PWM_IF);}

			/* Period computation */
			H_PWM_IF.pwmInputTon=H_PWM_IF.pwmInputTonCapture*PWM_IF_COUNTER_CYCLE_TIME;

			if(MC_STATUS_STOP!=MC_Get_Status())
			{
				if(H_PWM_IF.pwmInputTon<PWM_IF_TIM_MIN_SPEED_TON)
				{
					if(H_PWM_IF.stopCnt>PWM_IF_TIM_STOP_VALID_TON){/* H_PWM_IF.armingCnt=1U; */ MC_Stop_Motor();}
					else{H_PWM_IF.stopCnt++;}
				}
				else
				{
					H_PWM_IF.speedCommand=(H_PWM_IF.pwmInputTon-PWM_IF_TIM_MIN_SPEED_TON)>>16U;

					if((H_PWM_IF.speedCommand>=0)&&(H_PWM_IF.speedCommand<=PWM_IF_SPEED_RANGE))
					{MC_Set_PWM_Pulse_Command(H_PWM_IF.speedCommand,PWM_IF_TIM_MIN_MAX_BITS);}

					H_PWM_IF.stopCnt=0U;
				}
				H_PWM_IF.startCnt=0U;
			}
			else if(MC_STATUS_RUN!=MC_Get_Status())
			{
				if((H_PWM_IF.pwmInputTon>=PWM_IF_TIM_MIN_SPEED_TON)&&(H_PWM_IF.pwmInputTon<=PWM_IF_TIM_MAX_SPEED_TON))
				{
					if(H_PWM_IF.startCnt>PWM_IF_TIM_START_VALID_TON){MC_Start_Motor();}
					else{H_PWM_IF.startCnt++;}
				}else{H_PWM_IF.startCnt=0U;}
				H_PWM_IF.stopCnt=0U;
			}

			LL_TIM_IC_SetPolarity(H_PWM_IF.timIF.Instance,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_RISING);
		}
	}
	else if(__HAL_TIM_GET_FLAG(&H_PWM_IF.timIF,TIM_FLAG_UPDATE)&&__HAL_TIM_GET_IT_SOURCE(&H_PWM_IF.timIF,TIM_FLAG_UPDATE))
	{
		__HAL_TIM_CLEAR_IT(&H_PWM_IF.timIF,TIM_FLAG_UPDATE);

		if(MC_STATUS_STOP!=MC_Get_Status())
		{
			if(H_PWM_IF.blankStopCnt>PWM_IF_TIM_STOP_VALID_TON){MC_Stop_Motor();}
			else{H_PWM_IF.blankStopCnt++;}
		}
	}
}
