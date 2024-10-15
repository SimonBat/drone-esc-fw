/**
  ***************************************************************************************************************************************
  * @file     mc.c
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

#include "mc.h"
#include "system_it.h"

static mc_ts H_MC;

/* Local functions declarations */
static void MC_Init_Data(mc_ts *_hMc);
static void MC_Reset_Data(mc_ts *_hMc);
static inline void MC_Get_Next_Step(mc_ts *_hMc);
static inline void MC_Prepare_Next_Step(mc_ts *_hMc);
static inline void MC_Alignment_Handler(mc_ts *_hMc);
static void MC_HF_TIM_Init(mc_ts *_hMc);
static void MC_LF_TIM_Init(mc_ts *_hMc);
static void MC_ZC_TIM_Init(mc_ts *_hMc);

/**
  ***************************************************************************************************************************************
  * @brief  MC system initialization
  * @param  None
  * @retval None
  ***************************************************************************************************************************************
  */
void MC_System_Init(void)
{
	MC_HF_TIM_Init(&H_MC);
	MC_LF_TIM_Init(&H_MC);
	MC_ZC_TIM_Init(&H_MC);
	MC_Init_Data(&H_MC);
	MC_Reset_Data(&H_MC);
}

/**
  ***************************************************************************************************************************************
  * @brief  MC start motor
  * @param  None
  * @retval None
  ***************************************************************************************************************************************
  */
void MC_Start_Motor(void)
{
	H_MC.hfn=0U;
	H_MC.zcn=0U;
	H_MC.zcuIndexLh=0U;
	H_MC.zcvIndexLh=0U;
	H_MC.zcwIndexLh=0U;
	H_MC.zcuIndexHl=0U;
	H_MC.zcvIndexHl=0U;
	H_MC.zcwIndexHl=0U;
	H_MC.zcuLh[0]=0U;
	H_MC.zcvLh[0]=0U;
	H_MC.zcwLh[0]=0U;
	H_MC.zcuHl[0]=1U;
	H_MC.zcvHl[0]=1U;
	H_MC.zcwHl[0]=1U;
	H_MC.stepReadyFlag=0U;
	H_MC.status=MC_STATUS_START;
	H_MC.pulseCommand=H_MC.startupReference;
	H_MC.pulseCommandRef=H_MC.startupReference;
	H_MC.timHF.Instance->CCR4=H_MC.timHF.Init.Period-H_MC.startupReference;
	H_MC.guardTime=MC_CFG_ZC_READ_TO_PWM_EDGE_GUARD_TIME_CYC;

	BSP_Fault_LED_Disable();
	HAL_TIMEx_ConfigCommutationEvent(&H_MC.timHF,TIM_TS_NONE,TIM_COMMUTATION_SOFTWARE);
	H_MC.alignmentFlag=1U;
}

/**
  ***************************************************************************************************************************************
  * @brief  MC stop motor
  * @param  None
  * @retval None
  ***************************************************************************************************************************************
  */
void MC_Stop_Motor(void)
{
	uint32_t _priMask=__get_PRIMASK();
	__disable_irq();
	HAL_TIM_Base_Stop_IT(&H_MC.timLF);
	__HAL_TIM_CLEAR_IT(&H_MC.timLF, TIM_IT_UPDATE);
	__HAL_TIM_CLEAR_IT(&H_MC.timHF, TIM_IT_UPDATE);

	/* Stop PWM driving */
	HAL_TIM_PWM_Stop(&H_MC.timHF,TIM_CHANNEL_1); /* TIM1_CH1 DISABLE */
	HAL_TIMEx_PWMN_Stop(&H_MC.timHF,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&H_MC.timHF,TIM_CHANNEL_2); /* TIM1_CH2  DISABLE */
	HAL_TIMEx_PWMN_Stop(&H_MC.timHF,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&H_MC.timHF,TIM_CHANNEL_3); /* TIM1_CH3 DISABLE */
	HAL_TIMEx_PWMN_Stop(&H_MC.timHF,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&H_MC.timHF,TIM_CHANNEL_4); /* TIM1_CH4 DISABLE */

	__HAL_TIM_DISABLE(&H_MC.timHF);
	__HAL_TIM_SET_COUNTER(&H_MC.timHF,0U);
	__HAL_TIM_SET_COUNTER(&H_MC.timLF,0U);
	HAL_TIM_Base_Stop_IT(&H_MC.timZC);
	__HAL_TIM_SET_COUNTER(&H_MC.timZC,0U);
	HAL_TIM_Base_Stop_IT(&H_MC.timLF);

	MC_Reset_Data(&H_MC);
	if(!_priMask){__enable_irq();}
}

/**
  ***************************************************************************************************************************************
  * @brief  MC get status
  * @param  None
  * @retval Status (mc_status_te)
  ***************************************************************************************************************************************
  */
mc_status_te MC_Get_Status(void)
{
	return H_MC.status;
}

/**
  ***************************************************************************************************************************************
  * @brief  MC set PWM pulse command
  * @param  Command (uint32_t), timer min to max bits (uint8_t)
  * @retval None
  ***************************************************************************************************************************************
  */
void MC_Set_PWM_Pulse_Command(uint32_t _pulseCmd, uint8_t _timMinMaxBits)
{
	H_MC.pulseCommandRef=((_pulseCmd*(H_MC.timHF.Init.Period-H_MC.startupReference))>>_timMinMaxBits)+H_MC.startupReference;
}

/**
  ***************************************************************************************************************************************
  * @brief  MC data initialization
  * @param  MC handle (mc_ts*)
  * @retval None
  ***************************************************************************************************************************************
  */
static void MC_Init_Data(mc_ts *_hMc)
{
	_hMc->startupReference=(MC_CFG_STARTUP_DUTY_CYCLE*_hMc->timHF.Init.Period)/1000U;
	_hMc->direction=(mc_direction_te)MC_CFG_DIRECTION;
	_hMc->ocFlag=0U;
}

/**
  ***************************************************************************************************************************************
  * @brief  MC data reset
  * @param  MC handle (mc_ts*)
  * @retval None
  ***************************************************************************************************************************************
  */
static void MC_Reset_Data(mc_ts *_hMc)
{
	_hMc->arrLF=0U;
	_hMc->alignFlag=0U;
	_hMc->indexAlign=0U;
	_hMc->lfErrorFlag=0U;
	_hMc->bemfTdownCount=0U;
	_hMc->speedErrorFlag=0U;
	_hMc->alignmentFlag=0U;
	_hMc->stepPosition=0U;
	_hMc->validationFlag=0U;
	_hMc->status=MC_STATUS_STOP;
	_hMc->pulseCommand=_hMc->startupReference;
	_hMc->pulseCommandRef=_hMc->startupReference;
	_hMc->timHF.Instance->ARR=_hMc->timHF.Init.Period;
	_hMc->timLF.Instance->ARR=_hMc->timLF.Init.Period;
	_hMc->timHF.Instance->CCR1=_hMc->timHF.Init.Period;
	_hMc->timHF.Instance->CCR2=_hMc->timHF.Init.Period;
	_hMc->timHF.Instance->CCR3=_hMc->timHF.Init.Period;
	_hMc->timHF.Instance->PSC=_hMc->timHF.Init.Prescaler;
	_hMc->pulseValue=_hMc->timHF.Init.Period-_hMc->startupReference;
}

/**
  ***************************************************************************************************************************************
  * @brief  MC get next step
  * @param  MC handle (mc_ts*)
  * @retval None
  ***************************************************************************************************************************************
  */
static inline void MC_Get_Next_Step(mc_ts *_hMc)
{
	_hMc->arrLF=_hMc->timLF.Instance->ARR;
	_hMc->timLF.Instance->ARR=0xFFFF;

	if(!_hMc->stepReadyFlag){MC_Prepare_Next_Step(_hMc);}

	_hMc->hfn=0U;
	/* Enable TIM update event */
    _hMc->timHF.Instance->CR1&=~(TIM_CR1_UDIS);
    /* Generate TIM COM event */
    _hMc->timHF.Instance->EGR|=(TIM_EGR_COMG);

    if((0xFFFF==_hMc->arrLF)&&(MC_STATUS_RUN==_hMc->status)){_hMc->lfErrorFlag=1U;}

    if(_hMc->validationFlag)
    {
    	/* Motor Stall condition detection and Speed-Feedback error generation */
    	if(++_hMc->bemfTdownCount>MC_CFG_BEMF_CONSEC_DOWN_MAX){_hMc->speedErrorFlag=1U;}
    }

    _hMc->stepReadyFlag=0U;
}

/**
  ***************************************************************************************************************************************
  * @brief  MC prepare next step
  * @param  MC handle (mc_ts*)
  * @retval None
  ***************************************************************************************************************************************
  */
static inline void MC_Prepare_Next_Step(mc_ts *_hMc)
{
	_hMc->pulseValue=_hMc->pulseCommand;
	uint16_t _arrPulse=_hMc->timHF.Init.Period-_hMc->pulseValue;

	/* Disable TIM update event */
	_hMc->timHF.Instance->CR1|=(TIM_CR1_UDIS);
	_hMc->stepReadyFlag=1U;
	_hMc->zcn=0U;

    if(MC_DIRECTION_CW==_hMc->direction)
    {
    	if(_hMc->stepPosition>=6U){_hMc->stepPosition=1U;}
    	else{_hMc->stepPosition++;}
    }
    else
    {
    	if(_hMc->stepPosition<=1U){_hMc->stepPosition=6U;}
    	else{_hMc->stepPosition--;}
    }


    switch(_hMc->stepPosition)
    {
       	case 1U:
       		if(MC_DIRECTION_CW==_hMc->direction)
       		{
       			_hMc->timHF.Instance->CCR1=0U;
       			_hMc->timHF.Instance->CCR2=_hMc->pulseValue;
       		}
       		else
       		{
       			_hMc->timHF.Instance->CCR1=_hMc->pulseValue;
       			_hMc->timHF.Instance->CCR2=0U;
       		}

       		_hMc->timHF.Instance->CCR3=0U;
       		_hMc->timHF.Instance->CCR4=_hMc->pulseValue;
       		_hMc->timHF.Instance->CCER=MC_TABLE_STEP_1_CCER;
       	break;

       	case 2U:
       		if(MC_DIRECTION_CW==_hMc->direction)
       		{
       			_hMc->timHF.Instance->CCR1=_arrPulse;
       			_hMc->timHF.Instance->CCR3=_hMc->timHF.Init.Period;
       		}
        	else
        	{
        		_hMc->timHF.Instance->CCR1=_hMc->timHF.Init.Period;
        		_hMc->timHF.Instance->CCR3=_arrPulse;
        	}

        	_hMc->timHF.Instance->CCR2=_hMc->timHF.Init.Period;
        	_hMc->timHF.Instance->CCR4=_arrPulse;
        	_hMc->timHF.Instance->CCER=MC_TABLE_STEP_2_CCER;
       	break;

       	case 3U:
       		if(MC_DIRECTION_CW==_hMc->direction)
       		{
       			_hMc->timHF.Instance->CCR2=0U;
       			_hMc->timHF.Instance->CCR3=_hMc->pulseValue;
       		}
       		else
       		{
       			_hMc->timHF.Instance->CCR2=_hMc->pulseValue;
       			_hMc->timHF.Instance->CCR3=0U;
       		}

        	_hMc->timHF.Instance->CCR1=0U;
        	_hMc->timHF.Instance->CCR4=_hMc->pulseValue;
        	_hMc->timHF.Instance->CCER=MC_TABLE_STEP_3_CCER;
       	break;

       	case 4U:
       		if(MC_DIRECTION_CW==_hMc->direction)
       		{
       			_hMc->timHF.Instance->CCR1=_hMc->timHF.Init.Period;
       			_hMc->timHF.Instance->CCR2=_arrPulse;
       		}
       		else
       		{
       			_hMc->timHF.Instance->CCR1=_arrPulse;
       			_hMc->timHF.Instance->CCR2=_hMc->timHF.Init.Period;
       		}

       		_hMc->timHF.Instance->CCR3=_hMc->timHF.Init.Period;
       		_hMc->timHF.Instance->CCR4=_arrPulse;
       		_hMc->timHF.Instance->CCER=MC_TABLE_STEP_4_CCER;
       	break;

       	case 5U:
       		if(MC_DIRECTION_CW==_hMc->direction)
       		{
       			_hMc->timHF.Instance->CCR1=_hMc->pulseValue;
       			_hMc->timHF.Instance->CCR3=0U;
       		}
       		else
       		{
       			_hMc->timHF.Instance->CCR1=0U;
       			_hMc->timHF.Instance->CCR3=_hMc->pulseValue;
       		}

       		_hMc->timHF.Instance->CCR2=0U;
       		_hMc->timHF.Instance->CCR4=_hMc->pulseValue;
       		_hMc->timHF.Instance->CCER=MC_TABLE_STEP_5_CCER;
       	break;

       	case 6U:
       		if(MC_DIRECTION_CW==_hMc->direction)
       		{
       			_hMc->timHF.Instance->CCR2=_hMc->timHF.Init.Period;
       			_hMc->timHF.Instance->CCR3=_arrPulse;
       		}
       		else
       		{
       			_hMc->timHF.Instance->CCR2=_arrPulse;
       			_hMc->timHF.Instance->CCR3=_hMc->timHF.Init.Period;
       		}

        	_hMc->timHF.Instance->CCR1=_hMc->timHF.Init.Period;
        	_hMc->timHF.Instance->CCR4=_arrPulse;
        	_hMc->timHF.Instance->CCER=MC_TABLE_STEP_6_CCER;
        break;
    }
}

/**
  ***************************************************************************************************************************************
  * @brief  MC alignment handler
  * @param  MC handle (mc_ts*)
  * @retval None
  ***************************************************************************************************************************************
  */
static inline void MC_Alignment_Handler(mc_ts *_hMc)
{
	if(!_hMc->indexAlign)
	{
		uint32_t _priMask=__get_PRIMASK();
		__disable_irq();

		/* Enable counter */
		_hMc->timHF.Instance->CR1|=(TIM_CR1_CEN);
		/* Enable update event */
		_hMc->timHF.Instance->CR1|=(TIM_CR1_CEN);
		_hMc->status=MC_STATUS_ALIGNMENT;
		/* Start PWM driving */
		HAL_TIM_PWM_Start(&_hMc->timHF,TIM_CHANNEL_1); /* TIM1_CH1 ENABLE */
		HAL_TIMEx_PWMN_Start(&_hMc->timHF,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&_hMc->timHF,TIM_CHANNEL_2); /* TIM1_CH2 ENABLE */
		HAL_TIMEx_PWMN_Start(&_hMc->timHF,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&_hMc->timHF,TIM_CHANNEL_3); /* TIM1_CH3 ENABLE */
		HAL_TIMEx_PWMN_Start(&_hMc->timHF,TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&_hMc->timHF,TIM_CHANNEL_4); /* TIM1_CH4 ENABLE */
		MC_Get_Next_Step(_hMc);
		if(!_priMask){__enable_irq();}
	}

	if(++_hMc->indexAlign>=MC_CFG_TIME_FOR_ALIGN)
	{
		_hMc->alignFlag=1U;
		/* Enable TIM counter */
		_hMc->timZC.Instance->CR1|=(TIM_CR1_CEN);
		/* Enable the TIM Update interrupt */
		_hMc->timLF.Instance->DIER|=(TIM_IT_UPDATE);
		/* Enable the Peripheral */
		_hMc->timLF.Instance->CR1|=(TIM_CR1_CEN);
		/* Disable TIM update event */
		_hMc->timHF.Instance->CR1|=(TIM_CR1_UDIS);
		/* Enable TIM interupt */
		_hMc->timZC.Instance->DIER|=(TIM_IT_CC1);
		MC_Get_Next_Step(_hMc);
		_hMc->validationFlag=1U;
		_hMc->status=MC_STATUS_RUN;
		_hMc->indexAlign=0U;
	}
}

/**
  ***************************************************************************************************************************************
  * @brief  MC HF timer initialization
  * @param  MC handle (mc_ts*)
  * @retval None
  ***************************************************************************************************************************************
  */
static void MC_HF_TIM_Init(mc_ts *_hMc)
{
	TIM_OC_InitTypeDef _sConfigOC={0};;
	TIM_SlaveConfigTypeDef _sSlaveConfig={0};
	TIM_MasterConfigTypeDef _sMasterConfig={0};
	TIM_BreakDeadTimeConfigTypeDef _sBreakDeadTimeConfig={0};

	_hMc->timHF.Instance=MC_HF_TIM;
	_hMc->timHF.Init.Prescaler=MC_CFG_HF_TIM_PSC;
	_hMc->timHF.Init.CounterMode=MC_HF_TIM_COUNTER_MODE;
	_hMc->timHF.Init.Period=MC_CFG_HF_TIM_ARR;
	_hMc->timHF.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	_hMc->timHF.Init.RepetitionCounter=0;
	if(HAL_OK!=HAL_TIM_PWM_Init(&_hMc->timHF)){BSP_Error_Handler(BSP_CFG_ERR_MC);}

	_sSlaveConfig.SlaveMode=MC_HF_TIM_SLAVE_MODE;
	_sSlaveConfig.InputTrigger=MC_HF_TIM_TS_ITR;
	if(HAL_OK!=HAL_TIM_SlaveConfigSynchronization(&_hMc->timHF,&_sSlaveConfig)){BSP_Error_Handler(BSP_CFG_ERR_MC);}

	_sMasterConfig.MasterOutputTrigger=MC_HF_TIM_TRGO;
	_sMasterConfig.MasterSlaveMode=TIM_MASTERSLAVEMODE_ENABLE;
	if(HAL_OK!=HAL_TIMEx_MasterConfigSynchronization(&_hMc->timHF,&_sMasterConfig)){BSP_Error_Handler(BSP_CFG_ERR_MC);}

	_sBreakDeadTimeConfig.OffStateRunMode=TIM_OSSR_DISABLE;
	_sBreakDeadTimeConfig.OffStateIDLEMode=TIM_OSSI_DISABLE;
	_sBreakDeadTimeConfig.LockLevel=TIM_LOCKLEVEL_OFF;
	_sBreakDeadTimeConfig.DeadTime=MC_CFG_DEAD_TIME;
	_sBreakDeadTimeConfig.BreakState=MC_HF_TIM_BREAK_STATE;
	_sBreakDeadTimeConfig.BreakPolarity=MC_HF_TIM_BREAK_POL;
	_sBreakDeadTimeConfig.AutomaticOutput=MC_HF_TIM_BREAK_OUT;
	if(HAL_OK!=HAL_TIMEx_ConfigBreakDeadTime(&_hMc->timHF,&_sBreakDeadTimeConfig)){BSP_Error_Handler(BSP_CFG_ERR_MC);}

	_sConfigOC.Pulse=MC_CFG_HF_TIM_ARR;
	_sConfigOC.OCMode=TIM_OCMODE_PWM1;
	_sConfigOC.OCFastMode=TIM_OCFAST_DISABLE;
	_sConfigOC.OCPolarity=TIM_OCPOLARITY_LOW;
	_sConfigOC.OCNPolarity=TIM_OCNPOLARITY_LOW;
	_sConfigOC.OCIdleState=TIM_OCIDLESTATE_RESET;
	_sConfigOC.OCNIdleState=TIM_OCNIDLESTATE_RESET;

	if(HAL_OK!=HAL_TIM_PWM_ConfigChannel(&_hMc->timHF,&_sConfigOC,TIM_CHANNEL_1)){BSP_Error_Handler(BSP_CFG_ERR_MC);}
	if(HAL_OK!=HAL_TIM_PWM_ConfigChannel(&_hMc->timHF,&_sConfigOC,TIM_CHANNEL_2)){BSP_Error_Handler(BSP_CFG_ERR_MC);}
	if(HAL_OK!=HAL_TIM_PWM_ConfigChannel(&_hMc->timHF,&_sConfigOC,TIM_CHANNEL_3)){BSP_Error_Handler(BSP_CFG_ERR_MC);}
	if(HAL_OK!=HAL_TIM_PWM_ConfigChannel(&_hMc->timHF,&_sConfigOC,TIM_CHANNEL_4)){BSP_Error_Handler(BSP_CFG_ERR_MC);}
}

/**
  ***************************************************************************************************************************************
  * @brief  HAL timer PWM MSP initialization
  * @param  TIM handle (TIM_HandleTypeDef*)
  * @retval None
  ***************************************************************************************************************************************
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *_hTim)
{
	GPIO_InitTypeDef _gpioInitStruct={0};

	if(MC_HF_TIM==_hTim->Instance)
	{
		/* Peripheral clock enable */
		MC_HF_TIM_CLK_ENABLE();
		/* GPIOs Clocks Enable */
		MC_HF_GPIOA_CLK_ENABLE();
		MC_HF_GPIOB_CLK_ENABLE();

		/* HF_TIMx Break Input Configuration */
		_gpioInitStruct.Pin=MC_HF_TIM_BKIN_PIN;
		_gpioInitStruct.Mode=GPIO_MODE_AF_PP;
		_gpioInitStruct.Pull=GPIO_NOPULL;
		_gpioInitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
		_gpioInitStruct.Alternate=MC_HF_TIM_BKIN_AF;
		HAL_GPIO_Init(MC_HF_TIM_BKIN_PORT,&_gpioInitStruct);

		/* HF_TIMx OUTPUTs Configuration */
		_gpioInitStruct.Pin=MC_HF_TIM_CH1_PIN|MC_HF_TIM_CH2_PIN|MC_HF_TIM_CH3_PIN;
		_gpioInitStruct.Mode=MC_HF_TIM_MODE;
		_gpioInitStruct.Pull=MC_HF_TIM_PULL;
		_gpioInitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
		_gpioInitStruct.Alternate=MC_HF_TIM_AF;
		HAL_GPIO_Init(MC_HF_TIM_CH_PORT,&_gpioInitStruct);

		_gpioInitStruct.Pin=MC_HF_TIM_CH1N_PIN|MC_HF_TIM_CH2N_PIN|MC_HF_TIM_CH3N_PIN;
		_gpioInitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
		_gpioInitStruct.Mode=MC_HF_TIM_MODE;
		_gpioInitStruct.Pull=MC_HF_TIM_PULL;
		_gpioInitStruct.Alternate=MC_HF_TIM_AF;
		HAL_GPIO_Init(MC_HF_TIM_CHN_PORT,&_gpioInitStruct);

		/* Enable and set priority for the HF_TIMx interrupt */
		HAL_NVIC_SetPriority(MC_HF_TIM_IRQn,MC_HF_TIM_PRIORITY,0U);
		HAL_NVIC_EnableIRQ(MC_HF_TIM_IRQn);

		/* Stop TIM during Breakpoint */
		MC_HF_TIM_FREEZE_DBGMCU();
		/* Enable TIM break interrupt */
		__HAL_TIM_ENABLE_IT(_hTim,TIM_IT_BREAK);
	}
	else if(MC_ZC_TIM==_hTim->Instance)
	{
		/* TIMx Peripheral clock enable */
		MC_ZC_TIM_CLK_ENABLE();
		/* Set the TIMx priority */
		HAL_NVIC_SetPriority(MC_ZC_TIM_IRQn,MC_ZC_TIM_PRIORITY,MC_ZC_TIM_SUB_PRIORITY);
		/* Enable the TIMx global Interrupt */
		HAL_NVIC_EnableIRQ(MC_ZC_TIM_IRQn);
	}
}

/**
  ***************************************************************************************************************************************
  * @brief  MC LF timer initialization
  * @param  MC handle (mc_ts*)
  * @retval None
  ***************************************************************************************************************************************
  */
static void MC_LF_TIM_Init(mc_ts *_hMc)
{
	TIM_MasterConfigTypeDef _sMasterConfig={0};
	TIM_ClockConfigTypeDef _sClockSourceConfig={0};

	_hMc->timLF.Instance=MC_LF_TIM;
	_hMc->timLF.Init.Prescaler=MC_CFG_LF_TIM_PSC;
	_hMc->timLF.Init.CounterMode=MC_LF_TIM_COUNTER_MODE ;
	_hMc->timLF.Init.Period=MC_CFG_LF_TIM_ARR;
	_hMc->timLF.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	if(HAL_OK!=HAL_TIM_Base_Init(&_hMc->timLF)){BSP_Error_Handler(BSP_CFG_ERR_MC);}

	_sClockSourceConfig.ClockSource=TIM_CLOCKSOURCE_INTERNAL;
	if(HAL_OK!=HAL_TIM_ConfigClockSource(&_hMc->timLF,&_sClockSourceConfig)){BSP_Error_Handler(BSP_CFG_ERR_MC);}

	_sMasterConfig.MasterOutputTrigger=MC_LF_TIM_TRGO;
	_sMasterConfig.MasterSlaveMode=TIM_MASTERSLAVEMODE_DISABLE;
	if(HAL_OK!=HAL_TIMEx_MasterConfigSynchronization(&_hMc->timLF,&_sMasterConfig)){BSP_Error_Handler(BSP_CFG_ERR_MC);}
}

/**
  ***************************************************************************************************************************************
  * @brief  MC ZC timer initialization
  * @param  MC handle (mc_ts*)
  * @retval None
  ***************************************************************************************************************************************
  */
static void MC_ZC_TIM_Init(mc_ts *_hMc)
{
	TIM_OC_InitTypeDef _sConfig={0};
	TIM_SlaveConfigTypeDef _sSlaveConfig={0};

	_hMc->timZC.Instance=MC_ZC_TIM;
	_hMc->timZC.Init.Prescaler=MC_CFG_ZC_TIM_PSC;
	_hMc->timZC.Init.Period=(uint32_t)(SystemCoreClock/(MC_CFG_ZC_TIM_FREQUENCY_HZ*(MC_CFG_ZC_TIM_PSC+1U)))-1U;
	_hMc->timZC.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	_hMc->timZC.Init.CounterMode=MC_ZC_TIM_COUNTER_MODE ;
	_hMc->timZC.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
	if(HAL_OK!=HAL_TIM_PWM_Init(&_hMc->timZC)){BSP_Error_Handler(BSP_CFG_ERR_MC);}

	_sSlaveConfig.SlaveMode=MC_ZC_TIM_SLAVE_MODE;
	_sSlaveConfig.InputTrigger=MC_ZC_TIM_TS_ITR;
	if(HAL_OK!=HAL_TIM_SlaveConfigSynchronization(&_hMc->timZC,&_sSlaveConfig)){BSP_Error_Handler(BSP_CFG_ERR_MC);}

	_sConfig.OCMode=TIM_OCMODE_PWM1;
	_sConfig.OCPolarity=TIM_OCPOLARITY_HIGH;
	_sConfig.OCFastMode=TIM_OCFAST_DISABLE;
	_sConfig.OCNPolarity=TIM_OCNPOLARITY_HIGH;
	_sConfig.OCNIdleState=TIM_OCNIDLESTATE_RESET;
	_sConfig.OCIdleState=TIM_OCIDLESTATE_RESET;
	_sConfig.Pulse=MC_CFG_PWM_EDGE_TO_ZC_READ_EXTRA_DELAY_CYC;
	if(HAL_OK!=HAL_TIM_PWM_ConfigChannel(&_hMc->timZC,&_sConfig,TIM_CHANNEL_1)){BSP_Error_Handler(BSP_CFG_ERR_MC);}

	__HAL_TIM_CLEAR_IT(&_hMc->timZC,TIM_IT_CC1);
	__HAL_TIM_ENABLE_IT(&_hMc->timZC,TIM_IT_CC1);
}

/**
  ***************************************************************************************************************************************
  * @brief  HAL timer base MSP initialization
  * @param  TIM handle (TIM_HandleTypeDef*)
  * @retval None
  ***************************************************************************************************************************************
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* _hTim)
{
	if(MC_LF_TIM==_hTim->Instance)
	{
		/* Peripheral clock enable */
		MC_LF_TIM_CLK_ENABLE();
		/* System interrupt init*/
		HAL_NVIC_SetPriority(MC_LF_TIM_IRQn,MC_LF_TIM_PRIORITY,MC_LF_TIM_SUB_PRIORITY);
		HAL_NVIC_EnableIRQ(MC_LF_TIM_IRQn);
		/* Stop TIM during Breakpoint */
		MC_LF_TIM_FREEZE_DBGMCU();
	}
}

/**
  ***************************************************************************************************************************************
  * @brief  MC ZC timer IRQ handler
  * @param  None
  * @retval None
  ***************************************************************************************************************************************
  */
void __attribute__((section(".RamFunc"))) TIM2_IRQHandler(void)
{
	int32_t _cyclesToPwmEdge=H_MC.timHF.Instance->CCR4-H_MC.timHF.Instance->CNT;

	if(((TIM_FLAG_CC1)==(H_MC.timZC.Instance->SR&(TIM_FLAG_CC1)))&&((TIM_IT_CC1)==(H_MC.timZC.Instance->DIER&(TIM_IT_CC1))))
	{
		H_MC.timZC.Instance->SR=~(TIM_IT_CC1);

		if(_cyclesToPwmEdge<0){_cyclesToPwmEdge*=-1;}

		if((H_MC.stepReadyFlag)||(MC_STATUS_RUN!=H_MC.status)||(_cyclesToPwmEdge<H_MC.guardTime)||(!H_MC.hfn))
		{
			H_MC.hfn=1U;
			return;
		}

		switch(H_MC.stepPosition)
		{
			case 1U:
			{
				uint16_t _zc=BSP_ZCW_PORT->IDR&(BSP_ZCW_PIN);

				if(H_MC.zcn)
				{
					if((_zc)&&(!H_MC.zcwLh[H_MC.zcwIndexLh]))
					{
						H_MC.timLF.Instance->ARR=H_MC.timLF.Instance->CNT+((((H_MC.timHF.Init.Period-H_MC.pulseValue)>>1)*H_MC.arrLF)>>9);
						MC_Prepare_Next_Step(&H_MC);
					}
				}
				else{H_MC.zcn++;}
				if(H_MC.zcwIndexLh<(MC_ZC_ARRAY_SIZE-1U)){H_MC.zcwIndexLh++;}
				else{H_MC.zcwIndexLh=0U;}
				H_MC.zcwLh[H_MC.zcwIndexLh]=_zc;
			}
			break;

			case 2U:
			{
				uint16_t _zc=BSP_ZCV_PORT->IDR&(BSP_ZCV_PIN);

				if(H_MC.zcn)
				{
					if ((!_zc)&&(H_MC.zcvHl[H_MC.zcvIndexHl]))
					{
						H_MC.timLF.Instance->ARR=H_MC.timLF.Instance->CNT+((((H_MC.timHF.Init.Period-H_MC.pulseValue)>>1)*H_MC.arrLF)>>9);
						MC_Prepare_Next_Step(&H_MC);
						H_MC.bemfTdownCount=0U;
					}
				}
				else{H_MC.zcn++;}
				if(H_MC.zcvIndexHl<(MC_ZC_ARRAY_SIZE-1U)){H_MC.zcvIndexHl++;}
				else{H_MC.zcvIndexHl=0U;}
				H_MC.zcvHl[H_MC.zcvIndexHl]=_zc;
			}
			break;

			case 3U:
			{
				uint16_t _zc=BSP_ZCU_PORT->IDR&(BSP_ZCU_PIN);

				if(H_MC.zcn)
				{
					if ((_zc)&&(!H_MC.zcuLh[H_MC.zcuIndexLh]))
					{
						H_MC.timLF.Instance->ARR=H_MC.timLF.Instance->CNT+((((H_MC.timHF.Init.Period-H_MC.pulseValue)>>1)*H_MC.arrLF)>>9);
						MC_Prepare_Next_Step(&H_MC);
					}
				}
				else{H_MC.zcn++;}
				if(H_MC.zcuIndexLh<(MC_ZC_ARRAY_SIZE-1U)){H_MC.zcuIndexLh++;}
				else{H_MC.zcuIndexLh=0U;}
				H_MC.zcuLh[H_MC.zcuIndexLh]=_zc;
			}
			break;

			case 4U:
			{
				uint16_t _zc=BSP_ZCW_PORT->IDR&(BSP_ZCW_PIN);

				if(H_MC.zcn)
				{
					if((!_zc)&&(H_MC.zcwHl[H_MC.zcwIndexHl]))
					{
						H_MC.timLF.Instance->ARR=H_MC.timLF.Instance->CNT+((((H_MC.timHF.Init.Period-H_MC.pulseValue)>>1)*H_MC.arrLF)>>9);
						MC_Prepare_Next_Step(&H_MC);
						H_MC.bemfTdownCount=0U;
					}
				}
				else{H_MC.zcn++;}
				if(H_MC.zcwIndexHl<(MC_ZC_ARRAY_SIZE-1U)){H_MC.zcwIndexHl++;}
				else{H_MC.zcwIndexHl=0U;}
				H_MC.zcwHl[H_MC.zcwIndexHl]=_zc;
			}
			break;

			case 5U:
			{
				uint16_t _zc=BSP_ZCV_PORT->IDR&(BSP_ZCV_PIN);

				if(H_MC.zcn)
				{
					if((_zc)&&(!H_MC.zcvLh[H_MC.zcvIndexLh]))
					{
						H_MC.timLF.Instance->ARR=H_MC.timLF.Instance->CNT+((((H_MC.timHF.Init.Period-H_MC.pulseValue)>>1)*H_MC.arrLF)>>9);
						MC_Prepare_Next_Step(&H_MC);
					}
				}
				else{H_MC.zcn++;}
				if(H_MC.zcvIndexLh<(MC_ZC_ARRAY_SIZE-1U)){H_MC.zcvIndexLh++;}
				else{H_MC.zcvIndexLh=0U;}
				H_MC.zcvLh[H_MC.zcvIndexLh]=_zc;
			}
			break;

			case 6U:
			{
				uint16_t _zc=BSP_ZCU_PORT->IDR&(BSP_ZCU_PIN);

				if(H_MC.zcn)
				{
					if((!_zc)&&(H_MC.zcuHl[H_MC.zcuIndexHl]))
					{
						H_MC.timLF.Instance->ARR=H_MC.timLF.Instance->CNT+((((H_MC.timHF.Init.Period-H_MC.pulseValue)>>1)*H_MC.arrLF)>>9);
						MC_Prepare_Next_Step(&H_MC);
						H_MC.bemfTdownCount=0U;
					}
				}
				else{H_MC.zcn++;}

				if(H_MC.zcuIndexHl<(MC_ZC_ARRAY_SIZE-1U)){H_MC.zcuIndexHl++;}
				else{H_MC.zcuIndexHl=0U;}
				H_MC.zcuHl[H_MC.zcuIndexHl]=_zc;
			}
			break;
		}
	}
}

/**
  ***************************************************************************************************************************************
  * @brief  MC HF timer break input event IRQ handler
  * @param  None
  * @retval None
  ***************************************************************************************************************************************
  */
void __attribute__((section(".RamFunc"))) TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
	if(((TIM_FLAG_BREAK)==(H_MC.timHF.Instance->SR&(TIM_FLAG_BREAK)))&&((TIM_IT_BREAK)==(H_MC.timHF.Instance->DIER&(TIM_IT_BREAK))))
	{
		H_MC.timHF.Instance->SR=~(TIM_IT_BREAK);
		H_MC.timHF.Instance->CR1&=~(TIM_CR1_CEN);
		MC_Stop_Motor();
		H_MC.ocFlag=1U;
	}
}

/**
  ***************************************************************************************************************************************
  * @brief  MC LF timer IRQ handler
  * @param  None
  * @retval None
  ***************************************************************************************************************************************
  */
void __attribute__((section(".RamFunc"))) TIM3_IRQHandler(void)
{
	if(((TIM_FLAG_UPDATE)==(H_MC.timLF.Instance->SR&(TIM_FLAG_UPDATE)))&& \
	   ((TIM_IT_UPDATE)==(H_MC.timLF.Instance->DIER&(TIM_IT_UPDATE))))
	{
		H_MC.timLF.Instance->SR=~(TIM_IT_UPDATE);
		MC_Get_Next_Step(&H_MC);
	}
}

/**
  ***************************************************************************************************************************************
  * @brief  MC system tick IRQ handler
  * @param  None
  * @retval None
  ***************************************************************************************************************************************
  */
void MC_Tick_IRQ_Handler(void)
{
	if(H_MC.alignmentFlag)
	{
		if(!H_MC.alignFlag){MC_Alignment_Handler(&H_MC);}
		else
		{
			if(H_MC.pulseCommand<H_MC.pulseCommandRef){H_MC.pulseCommand++;}
			else if(H_MC.pulseCommand>H_MC.pulseCommandRef){H_MC.pulseCommand--;}

			if(H_MC.speedErrorFlag)
			{
				MC_Stop_Motor();
				H_MC.status=MC_STATUS_SPEED_ERROR;
				BSP_Fault_LED_Enable();
			}
		}
	}

	if(H_MC.lfErrorFlag)
	{
		MC_Stop_Motor();
		H_MC.status=MC_STATUS_LF_ERROR;
		BSP_Fault_LED_Enable();
	}

	if(H_MC.ocFlag)
	{
		MC_Stop_Motor();
		H_MC.status=MC_STATUS_OC_ERROR;
		BSP_Fault_LED_Enable();
		H_MC.status=MC_STATUS_STOP;
		H_MC.ocFlag=0U;
	}
}
