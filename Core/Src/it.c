/*
 * it.c
 *
 *  Created on: Dec 13, 2021
 *      Author: suleyman oflaz
 */
#include "main.h"

extern UART_HandleTypeDef Usart3Handle;
extern TIM_HandleTypeDef Tim6Handle;
extern TIM_HandleTypeDef Tim7Handle;

void HardFault_Handler(void)
{
	while(1);
}
void MemManage_Handler(void)
{
	while(1);
}
void BusFault_Handler(void)
{
	while(1);
}
void SysTick_Handler(void)
{
	HAL_IncTick();
}

void USART3_IRQHandler(void)
{
	HAL_UART_IRQHandler(&Usart3Handle);
}

void EXTI3_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

void TIM6_DAC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&Tim6Handle);
}

void TIM7_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&Tim7Handle);
}

void EXTI0_IRQHandler(void)
{
	HAL_Delay(500);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
