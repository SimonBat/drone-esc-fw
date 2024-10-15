#ifndef __MC_H
#define __MC_H

#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_tim.h"
#include "mc_cfg.h"
#include "bsp.h"

#define MC_HF_TIM                 		TIM1
#define MC_HF_GPIOA_CLK_ENABLE()    	__HAL_RCC_GPIOA_CLK_ENABLE();
#define MC_HF_GPIOB_CLK_ENABLE()    	__HAL_RCC_GPIOB_CLK_ENABLE();
#define MC_HF_TIM_CLK_ENABLE()    		__HAL_RCC_TIM1_CLK_ENABLE()
#define MC_HF_TIM_CLK_DISABLE()   		__HAL_RCC_TIM1_CLK_DISABLE()
#define MC_HF_TIM_FREEZE_DBGMCU() 		__HAL_DBGMCU_FREEZE_TIM1()
#define MC_HF_TIM_CH1_PIN         		BSP_HF_TIM_CH1_PIN
#define MC_HF_TIM_CH2_PIN         		BSP_HF_TIM_CH2_PIN
#define MC_HF_TIM_CH3_PIN         		BSP_HF_TIM_CH3_PIN
#define MC_HF_TIM_CH_PORT        		BSP_HF_TIM_CH_PORT
#define MC_HF_TIM_CH1N_PIN        		BSP_HF_TIM_CH1N_PIN
#define MC_HF_TIM_CH2N_PIN        		BSP_HF_TIM_CH2N_PIN
#define MC_HF_TIM_CH3N_PIN        		BSP_HF_TIM_CH3N_PIN
#define MC_HF_TIM_CHN_PORT       		BSP_HF_TIM_CHN_PORT
#define MC_HF_TIM_MODE            		GPIO_MODE_AF_PP
#define MC_HF_TIM_PULL            		GPIO_PULLDOWN
#define MC_HF_TIM_AF              		GPIO_AF2_TIM1
#define MC_HF_TIM_COUNTER_MODE    		TIM_COUNTERMODE_CENTERALIGNED1

#define MC_HF_TIM_BKIN_PIN        		BSP_HF_TIM_BKIN_PIN
#define MC_HF_TIM_BKIN_PORT       		BSP_HF_TIM_BKIN_PORT
#define MC_HF_TIM_BKIN_AF         		GPIO_AF2_TIM1
#define MC_HF_TIM_IRQn            		TIM1_BRK_UP_TRG_COM_IRQn
#define MC_HF_TIM_PRIORITY        		1U

#define MC_HF_TIM_SLAVE_MODE         	TIM_SLAVEMODE_TRIGGER
#define MC_HF_TIM_TS_ITR             	TIM_TS_ITR2
#define MC_HF_TIM_TRGO               	TIM_TRGO_OC4REF
#define MC_HF_TIM_BREAK_STATE        	TIM_BREAK_ENABLE
#define MC_HF_TIM_BREAK_POL          	TIM_BREAKPOLARITY_HIGH
#define MC_HF_TIM_BREAK_OUT          	TIM_AUTOMATICOUTPUT_DISABLE

#define MC_LF_TIM                    	TIM3
#define MC_LF_TIM_COUNTER_MODE       	TIM_COUNTERMODE_UP
#define MC_LF_TIM_CLK_ENABLE()       	__HAL_RCC_TIM3_CLK_ENABLE()
#define MC_LF_TIM_CLK_DISABLE()      	__HAL_RCC_TIM3_CLK_DISABLE()
#define MC_LF_TIM_FREEZE_DBGMCU()    	__HAL_DBGMCU_FREEZE_TIM3()
#define MC_LF_TIM_IRQn               	TIM3_IRQn
#define MC_LF_TIM_PRIORITY           	0U
#define MC_LF_TIM_SUB_PRIORITY       	1U
#define MC_LF_TIM_TRGO               	TIM_TRGO_UPDATE

#define MC_ZC_TIM                    	TIM2
#define MC_ZC_TIM_COUNTER_MODE       	TIM_COUNTERMODE_UP
#define MC_ZC_TIM_CLK_ENABLE()       	__HAL_RCC_TIM2_CLK_ENABLE()
#define MC_ZC_TIM_CLK_DISABLE()      	__HAL_RCC_TIM2_CLK_DISABLE()
#define MC_ZC_TIM_FREEZE_DBGMCU()    	__HAL_DBGMCU_FREEZE_TIM2()
#define MC_ZC_TIM_IRQn               	TIM2_IRQn
#define MC_ZC_TIM_PRIORITY           	0U
#define MC_ZC_TIM_SUB_PRIORITY       	0U
#define MC_ZC_TIM_SLAVE_MODE         	TIM_SLAVEMODE_RESET
#define MC_ZC_TIM_TS_ITR             	TIM_TS_ITR0
#define MC_ZC_ARRAY_SIZE 				4U

#define MC_TABLE_STEP_1_CCER			(uint32_t)(LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|LL_TIM_CHANNEL_CH4| \
    		    					   			  ((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<0)|((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<4)| \
												  ((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<8)|(TIM_OCPOLARITY_LOW<<12))
#define MC_TABLE_STEP_2_CCER			(uint32_t)(LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N|LL_TIM_CHANNEL_CH4| \
    		    	              	   			  ((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<0)|((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<4)| \
												  ((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<8)|(TIM_OCPOLARITY_LOW<<12))
#define MC_TABLE_STEP_3_CCER			(uint32_t)(LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N|LL_TIM_CHANNEL_CH4| \
    		    	  				    		  ((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<0)|((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<4)| \
												  ((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<8)|(TIM_OCPOLARITY_LOW<<12))
#define MC_TABLE_STEP_4_CCER		 	(uint32_t)(LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|LL_TIM_CHANNEL_CH4| \
    		    	      	  	  	    		  ((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<0)|((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<4)| \
												  ((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<8)|(TIM_OCPOLARITY_LOW<<12))
#define MC_TABLE_STEP_5_CCER			(uint32_t)(LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N|LL_TIM_CHANNEL_CH4| \
    		    	         	 	    		  ((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<0)|((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<4)| \
												  ((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<8)|(TIM_OCPOLARITY_LOW<<12))
#define MC_TABLE_STEP_6_CCER			(uint32_t)(LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N|LL_TIM_CHANNEL_CH4| \
    	    						    		  ((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<0)|((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<4)| \
												  ((TIM_OCPOLARITY_LOW|TIM_OCNPOLARITY_LOW)<<8)|(TIM_OCPOLARITY_LOW<<12))

typedef enum{
	MC_DIRECTION_CW,
	MC_DIRECTION_CCW
}mc_direction_te;

typedef enum{
	MC_STATUS_STOP,
    MC_STATUS_START,
    MC_STATUS_RUN,
    MC_STATUS_ALIGNMENT,
    MC_STATUS_SPEED_ERROR,
    MC_STATUS_OC_ERROR,
    MC_STATUS_LF_ERROR
}mc_status_te;

typedef struct{
	TIM_HandleTypeDef timHF;
	TIM_HandleTypeDef timLF;
	TIM_HandleTypeDef timZC;

	uint32_t arrLF;
	uint16_t guardTime;
	uint8_t ocFlag;
	uint8_t lfErrorFlag;
	uint8_t speedErrorFlag;
	uint8_t alignFlag;
	uint8_t alignmentFlag;
	uint8_t validationFlag;
	uint8_t bemfTdownCount;
	uint16_t indexAlign;
	mc_status_te status;
	uint8_t stepPosition;
	uint16_t pulseCommandRef;
	uint16_t pulseCommand;
	__IO  uint16_t pulseValue;
	uint16_t startupReference;
	__IO uint8_t stepReadyFlag;
	mc_direction_te direction;

	__IO uint8_t hfn;
	__IO uint8_t zcn;
	uint8_t zcuIndexLh;
	uint8_t zcvIndexLh;
	uint8_t zcwIndexLh;
	uint8_t zcuIndexHl;
	uint8_t zcvIndexHl;
	uint8_t zcwIndexHl;
	uint16_t zcuLh[MC_ZC_ARRAY_SIZE];
	uint16_t zcvLh[MC_ZC_ARRAY_SIZE];
	uint16_t zcwLh[MC_ZC_ARRAY_SIZE];
	uint16_t zcuHl[MC_ZC_ARRAY_SIZE];
	uint16_t zcvHl[MC_ZC_ARRAY_SIZE];
	uint16_t zcwHl[MC_ZC_ARRAY_SIZE];
}mc_ts;

/* Global functions declarations */
void MC_System_Init(void);
void MC_Start_Motor(void);
void MC_Stop_Motor(void);
mc_status_te MC_Get_Status(void);
void MC_Set_PWM_Pulse_Command(uint32_t _pulseCmd, uint8_t _timMinMaxBits);
void MC_ZC_IRQ_Handler(void);
void MC_TIM_BRK_UP_TRG_COM_IRQ_Handler(void);
void MC_TIM_Timebase_IRQ_Handler(void);
void MC_Tick_IRQ_Handler(void);

#endif
