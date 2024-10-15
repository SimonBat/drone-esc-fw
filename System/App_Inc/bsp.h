#ifndef __BSP_H
#define __BSP_H

#include "stm32f0xx_hal.h"

#define BSP_HF_TIM_CH1_PIN				GPIO_PIN_8
#define BSP_HF_TIM_CH2_PIN         		GPIO_PIN_9
#define BSP_HF_TIM_CH3_PIN         		GPIO_PIN_10
#define BSP_HF_TIM_CH_PORT        		GPIOA
#define BSP_HF_TIM_CH1N_PIN        		GPIO_PIN_13
#define BSP_HF_TIM_CH2N_PIN        		GPIO_PIN_14
#define BSP_HF_TIM_CH3N_PIN        		GPIO_PIN_15
#define BSP_HF_TIM_CHN_PORT       		GPIOB
#define BSP_HF_TIM_BKIN_PIN        		GPIO_PIN_12
#define BSP_HF_TIM_BKIN_PORT       		GPIOB
#define BSP_IF_TIM_CH1_PIN       		GPIO_PIN_6
#define BSP_IF_TIM_CH1_PORT      		GPIOA
#define BSP_OC_TH_STBY_PIN_1        	GPIO_PIN_6
#define BSP_OC_TH_STBY_PIN_2        	GPIO_PIN_7
#define BSP_OC_TH_STBY_PORT         	GPIOF
#define BSP_OC_SEL_PIN              	GPIO_PIN_11
#define BSP_OC_SEL_PORT             	GPIOA
#define BSP_ZCU_PIN               		GPIO_PIN_1
#define BSP_ZCU_PORT              		GPIOF
#define BSP_ZCV_PIN               		GPIO_PIN_0
#define BSP_ZCV_PORT              		GPIOF
#define BSP_ZCW_PIN               		GPIO_PIN_1
#define BSP_ZCW_PORT              		GPIOB
#define BSP_FAULT_LED_PIN    			GPIO_PIN_0
#define BSP_FAULT_LED_PORT   			GPIOA

#define BSP_SYSTICK_PRIORITY        	3U

typedef enum{
	BSP_CFG_ERR_LL,
	BSP_CFG_ERR_MC,
	BSP_CFG_ERR_PWM_IF
}bsp_cfg_error_te;

typedef enum{
	BSP_SEL_VIS_FROM_MCU,
	BSP_SEL_VIS_FROM_MCU_AND_GATE_LOGIC,
}bsp_oc_sel_vis_te;

typedef enum{
	BSP_OC_TH_STBY,
	BSP_OC_TH_100mV,
	BSP_OC_TH_250mV,
	BSP_OC_TH_500mV
}bsp_oc_th_te;

/* Global functions declarations */
void BSP_System_Clock_Init(void);
void BSP_GPIO_Init(void);
void BSP_OC_Set_Visibility(bsp_oc_sel_vis_te _ocSelVis);
void BSP_OC_Set_Threshold(bsp_oc_th_te _ocTh);
void BSP_Fault_LED_Enable(void);
void BSP_Fault_LED_Disable(void);
void BSP_Error_Handler(bsp_cfg_error_te _errorSource);

#endif
