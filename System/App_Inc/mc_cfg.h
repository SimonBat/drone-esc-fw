#ifndef __MC_CFG_H
#define __MC_CFG_H

#define MC_CFG_DIRECTION                            	0U /* Set motor direction CW=0U and CCW=1U*/
#define MC_CFG_STARTUP_DUTY_CYCLE                   	100U  /* Tenths of percentage of PWM on time */
#define MC_CFG_GATE_DRIVING_PWM_FREQUENCY      			64000U /* Hz */
#define MC_CFG_SYSCLOCK_FREQUENCY            			48000000U /* Hz */
#define MC_CFG_HF_TIM_PSC                          		0U
#define MC_CFG_HF_TIM_ARR 								(MC_CFG_SYSCLOCK_FREQUENCY/(MC_CFG_GATE_DRIVING_PWM_FREQUENCY*2U*(MC_CFG_HF_TIM_PSC+1U))-1U)
#define MC_CFG_HF_COUNTER_CYCLE_TIME_NS            		((1000000000U/(MC_CFG_SYSCLOCK_FREQUENCY))*(MC_CFG_HF_TIM_PSC+1U))
#define MC_CFG_DEAD_TIME_NS        						400U
#define MC_CFG_DEAD_TIME           						((MC_CFG_DEAD_TIME_NS/MC_CFG_HF_COUNTER_CYCLE_TIME_NS)+1U)
#define MC_CFG_LF_TIM_PSC                      			2U
#define MC_CFG_LF_TIM_ARR                      			65535U
#define MC_CFG_TIME_FOR_ALIGN                    		100U /* Time for alignment (ms) */
#define MC_CFG_ZC_TIM_FREQUENCY_HZ                		350000U
#define MC_CFG_ZC_TIM_PSC                         		0U
#define MC_CFG_ZC_COUNTER_CYCLE_TIME_NS            		((1000000000U/(MC_CFG_SYSCLOCK_FREQUENCY))*(MC_CFG_ZC_TIM_PSC+1U))
#define MC_CFG_ZC_READ_TO_PWM_EDGE_GUARD_TIME_NS   		900U
#define MC_CFG_ZC_READ_TO_PWM_EDGE_GUARD_TIME_CYC  		(MC_CFG_ZC_READ_TO_PWM_EDGE_GUARD_TIME_NS/MC_CFG_HF_COUNTER_CYCLE_TIME_NS)
#define MC_CFG_PWM_EDGE_TO_ZC_READ_EXTRA_DELAY_NS  		25U /* Additional delay between the gate driver PWM edge and the Zero Crossing reading in ns */
#define MC_CFG_PWM_EDGE_TO_ZC_READ_EXTRA_DELAY_CYC 		(MC_CFG_PWM_EDGE_TO_ZC_READ_EXTRA_DELAY_NS/MC_CFG_ZC_COUNTER_CYCLE_TIME_NS)
#define MC_CFG_BEMF_CONSEC_DOWN_MAX                		10U /* Maximum value of BEMF Consecutive Threshold Falling Crossings Counter in closed loop */

#endif
