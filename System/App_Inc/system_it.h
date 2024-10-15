#ifndef __SYSTEM_IT_H
#define __SYSTEM_IT_H

#include "stm32f0xx_hal.h"

void NMI_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void HardFault_Handler(void);
void __attribute__((section(".RamFunc"))) SysTick_Handler(void);
void __attribute__((section(".RamFunc"))) TIM1_BRK_UP_TRG_COM_IRQHandler(void);
void __attribute__((section(".RamFunc"))) TIM2_IRQHandler(void);
void __attribute__((section(".RamFunc"))) TIM3_IRQHandler(void);
void __attribute__((section(".RamFunc"))) TIM16_IRQHandler(void);

#endif
