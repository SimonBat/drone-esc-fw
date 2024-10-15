/**
  ***************************************************************************************************************************************
  * @file     system.c
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

#include "system.h"
#include "pwm_if.h"
#include "mc.h"

static system_ts H_SYSTEM;

/**
  ***************************************************************************************************************************************
  * @brief  System main entry point
  * @param  None
  * @retval None
  ***************************************************************************************************************************************
  */
int main(void)
{
	BSP_System_Clock_Init();
	BSP_GPIO_Init();
	PWM_IF_Init();
	MC_System_Init();

	while(1)
	{
		if(MC_STATUS_RUN==MC_Get_Status())
		{
			if(!H_SYSTEM.ledTmo)
			{
				if(SYSTEM_LED_STATE_DISABLED==H_SYSTEM.ledState)
				{
					H_SYSTEM.ledTmo=SYSTEM_LED_ENABLE_TMO;
					H_SYSTEM.ledState=SYSTEM_LED_STATE_ENABLED;
					BSP_Fault_LED_Enable();
				}
				else if(SYSTEM_LED_STATE_ENABLED==H_SYSTEM.ledState)
				{
					H_SYSTEM.ledTmo=SYSTEM_LED_DISABLE_TMO;
					H_SYSTEM.ledState=SYSTEM_LED_STATE_DISABLED;
					BSP_Fault_LED_Disable();
				}
			}
		}
		else
		{
			H_SYSTEM.ledTmo=0U;
			H_SYSTEM.ledState=SYSTEM_LED_STATE_DISABLED;
			BSP_Fault_LED_Disable();
		}
	}
}

/**
  ***************************************************************************************************************************************
  * @brief  Update system TMO values
  * @param  None
  * @retval None
  ***************************************************************************************************************************************
  */
void SYSTEM_Update_TMO(void)
{
	if(H_SYSTEM.ledTmo){H_SYSTEM.ledTmo--;}
}
