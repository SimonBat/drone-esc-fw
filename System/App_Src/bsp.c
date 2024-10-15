/**
  ***************************************************************************************************************************************
  * @file     bsp.c
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

#include "bsp.h"

/**
  ***************************************************************************************************************************************
  * @brief  BSP system clock initialization
  * @param  None
  * @retval None
  ***************************************************************************************************************************************
  */
void BSP_System_Clock_Init(void)
{
	RCC_OscInitTypeDef _rccOscInitStruct={0};
	RCC_ClkInitTypeDef _rccClkInitStruct={0};

	HAL_Init();
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

	_rccOscInitStruct.OscillatorType=RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
	_rccOscInitStruct.HSIState=RCC_HSI_ON;
	_rccOscInitStruct.HSI14State=RCC_HSI14_ON;
	_rccOscInitStruct.HSICalibrationValue=16U;
	_rccOscInitStruct.HSI14CalibrationValue=16U;
	_rccOscInitStruct.PLL.PLLState=RCC_PLL_ON;
	_rccOscInitStruct.PLL.PLLSource=RCC_PLLSOURCE_HSI;
	_rccOscInitStruct.PLL.PLLMUL=RCC_PLL_MUL12;
	_rccOscInitStruct.PLL.PREDIV=RCC_PREDIV_DIV1;
	if(HAL_OK!=HAL_RCC_OscConfig(&_rccOscInitStruct)){BSP_Error_Handler(BSP_CFG_ERR_LL);}

	_rccClkInitStruct.ClockType=RCC_CLOCKTYPE_SYSCLK;
	_rccClkInitStruct.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;
	_rccClkInitStruct.AHBCLKDivider=RCC_SYSCLK_DIV1;
	_rccClkInitStruct.APB1CLKDivider=RCC_HCLK_DIV1;
	if(HAL_OK!=HAL_RCC_ClockConfig(&_rccClkInitStruct,FLASH_LATENCY_1)){BSP_Error_Handler(BSP_CFG_ERR_LL);}

	HAL_NVIC_SetPriority(SysTick_IRQn,BSP_SYSTICK_PRIORITY,0U);

	/* GPIO ports clock enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
}

/**
  ***************************************************************************************************************************************
  * @brief  BSP GPIO initialization
  * @param  None
  * @retval None
  ***************************************************************************************************************************************
  */
void BSP_GPIO_Init(void)
{
	GPIO_InitTypeDef _gpioInitStruct;

	/* Configure the FAULT LED GPIO pin */
	_gpioInitStruct.Pin=BSP_FAULT_LED_PIN;
	_gpioInitStruct.Mode=GPIO_MODE_OUTPUT_PP;
	_gpioInitStruct.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(BSP_FAULT_LED_PORT,&_gpioInitStruct);
	BSP_Fault_LED_Disable();

	/* Configure over-current threshold GPIO pins */
	_gpioInitStruct.Pin=BSP_OC_TH_STBY_PIN_1|BSP_OC_TH_STBY_PIN_2;
	_gpioInitStruct.Mode=GPIO_MODE_OUTPUT_PP;
	_gpioInitStruct.Pull=GPIO_PULLUP;
	_gpioInitStruct.Speed=GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BSP_OC_TH_STBY_PORT,&_gpioInitStruct);

	/* Set the over-current threshold */
	BSP_OC_Set_Threshold(BSP_OC_TH_500mV);

	/* Configure the over-current selection GPIO pin  */
	_gpioInitStruct.Pin=BSP_OC_SEL_PIN;
	_gpioInitStruct.Mode=GPIO_MODE_OUTPUT_PP;
	_gpioInitStruct.Pull=GPIO_NOPULL;
	_gpioInitStruct.Speed=GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BSP_OC_SEL_PORT,&_gpioInitStruct);

	/* Set the over-current selection */
	BSP_OC_Set_Visibility(BSP_SEL_VIS_FROM_MCU_AND_GATE_LOGIC);

	/* Configure ZCU, ZCV and ZCW pins */
	_gpioInitStruct.Mode=GPIO_MODE_INPUT;
	_gpioInitStruct.Pull=GPIO_NOPULL;
	_gpioInitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
	_gpioInitStruct.Pin=BSP_ZCU_PIN;
	HAL_GPIO_Init(BSP_ZCU_PORT,&_gpioInitStruct);
	_gpioInitStruct.Pin=BSP_ZCV_PIN;
	HAL_GPIO_Init(BSP_ZCV_PORT,&_gpioInitStruct);
	_gpioInitStruct.Pin=BSP_ZCW_PIN;
	HAL_GPIO_Init(BSP_ZCW_PORT,&_gpioInitStruct);
}

/**
  ***************************************************************************************************************************************
  * @brief  BSP over-current set visibility mode
  * @param  Mode (bsp_oc_sel_vis_te)
  * @retval None
  ***************************************************************************************************************************************
  */
void BSP_OC_Set_Visibility(bsp_oc_sel_vis_te _ocSelVis)
{
	HAL_GPIO_WritePin(BSP_OC_SEL_PORT,BSP_OC_SEL_PIN,(GPIO_PinState)_ocSelVis);
}

/**
  ***************************************************************************************************************************************
  * @brief  BSP over-current set threshold
  * @param  Threshold (bsp_oc_th_te)
  * @retval None
  ***************************************************************************************************************************************
  */
void BSP_OC_Set_Threshold(bsp_oc_th_te _ocTh)
{
	switch(_ocTh)
	{
    	case BSP_OC_TH_STBY:
    		HAL_GPIO_WritePin(BSP_OC_TH_STBY_PORT,BSP_OC_TH_STBY_PIN_1,GPIO_PIN_RESET);
    		HAL_GPIO_WritePin(BSP_OC_TH_STBY_PORT,BSP_OC_TH_STBY_PIN_2,GPIO_PIN_RESET);
    	break;

    	case BSP_OC_TH_100mV:
    		HAL_GPIO_WritePin(BSP_OC_TH_STBY_PORT,BSP_OC_TH_STBY_PIN_1,GPIO_PIN_RESET);
    		HAL_GPIO_WritePin(BSP_OC_TH_STBY_PORT,BSP_OC_TH_STBY_PIN_2,GPIO_PIN_SET);
    	break;

    	case  BSP_OC_TH_250mV:
    		HAL_GPIO_WritePin(BSP_OC_TH_STBY_PORT,BSP_OC_TH_STBY_PIN_1,GPIO_PIN_SET);
    		HAL_GPIO_WritePin(BSP_OC_TH_STBY_PORT,BSP_OC_TH_STBY_PIN_2,GPIO_PIN_RESET);
    	break;

    	case  BSP_OC_TH_500mV:
    		HAL_GPIO_WritePin(BSP_OC_TH_STBY_PORT,BSP_OC_TH_STBY_PIN_1,GPIO_PIN_SET);
    		HAL_GPIO_WritePin(BSP_OC_TH_STBY_PORT,BSP_OC_TH_STBY_PIN_2,GPIO_PIN_SET);
    	break;

    	default:
    	break;
	}
}

/**
  ***************************************************************************************************************************************
  * @brief  BSP fault LED enable
  * @param  None
  * @retval None
  ***************************************************************************************************************************************
  */
void BSP_Fault_LED_Enable(void)
{
	HAL_GPIO_WritePin(BSP_FAULT_LED_PORT,BSP_FAULT_LED_PIN,GPIO_PIN_SET);
}

/**
  ***************************************************************************************************************************************
  * @brief  BSP fault LED disable
  * @param  None
  * @retval None
  ***************************************************************************************************************************************
  */
void BSP_Fault_LED_Disable(void)
{
	HAL_GPIO_WritePin(BSP_FAULT_LED_PORT,BSP_FAULT_LED_PIN,GPIO_PIN_RESET);
}

/**
  ***************************************************************************************************************************************
  * @brief  BSP error handler
  * @param  Error source (bsp_cfg_error_te)
  * @retval None
  ***************************************************************************************************************************************
  */
void BSP_Error_Handler(bsp_cfg_error_te _errorSource)
{
	UNUSED(_errorSource);
	BSP_Fault_LED_Disable();

  	while(1U)
  	{
  		__disable_irq();
  		NVIC_SystemReset();
  	}
}

#ifdef USE_FULL_ASSERT
/**
  ***************************************************************************************************************************************
  * @brief  Reports the name of the source file and the source line number where the assert_param error has occurred
  * @param  File pointer (uint8_t*), line (uint32_t)
  * @retval None
  ***************************************************************************************************************************************
  */
void BSP_Assert_Failed(uint8_t *_file, uint32_t _line)
{
	UNUSED(_file);
	UNUSED(_line);
}
#endif
