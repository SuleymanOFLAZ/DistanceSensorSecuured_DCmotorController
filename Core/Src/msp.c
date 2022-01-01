/*
 * msp.c
 *
 *  Created on: Dec 13, 2021
 *      Author: suleyman oflaz
 */

#include "main.h"

void HAL_MspInit(void)
{
	__NVIC_EnableIRQ(MemoryManagement_IRQn);  	// Enable Cortex-M4's MemFault
	__NVIC_EnableIRQ(BusFault_IRQn);			// Enable Cortex-M4's BusFault
	__NVIC_EnableIRQ(UsageFault_IRQn);			// Enable Cortex-M4's UsageFault
	__NVIC_EnableIRQ(SysTick_IRQn);

	// Setting Priority grouping
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	// Enable USART3 CLK
	__HAL_RCC_USART3_CLK_ENABLE();

	// Enable NVIC IT for UART
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);

	// Configure GPIOs
	__HAL_RCC_GPIOD_CLK_ENABLE();

	GPIO_InitTypeDef GPIOInitValues = {0};
	GPIOInitValues.Pin = GPIO_PIN_8;
	GPIOInitValues.Mode = GPIO_MODE_AF_PP;
	GPIOInitValues.Pull = GPIO_NOPULL;
	GPIOInitValues.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIOInitValues.Alternate = GPIO_AF7_USART3;

	HAL_GPIO_Init(GPIOD, &GPIOInitValues);

	GPIOInitValues.Pin = GPIO_PIN_9;

	HAL_GPIO_Init(GPIOD, &GPIOInitValues);
}

void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{
	// Enable TIM9 CLK
	__HAL_RCC_TIM9_CLK_ENABLE();

	// Enable TIM9 ITs
	HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
	HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 2, 0);

	// Configure GPIOs
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIOInitValues = {0};
	GPIOInitValues.Pin = GPIO_PIN_2;
	GPIOInitValues.Mode = GPIO_MODE_AF_PP;
	GPIOInitValues.Pull = GPIO_NOPULL;
	GPIOInitValues.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIOInitValues.Alternate = GPIO_AF3_TIM9;

	HAL_GPIO_Init(GPIOA, &GPIOInitValues);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	// Enable TIM6 CLK
	__HAL_RCC_TIM6_CLK_ENABLE();

	// Enable ITs for TIM6
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3, 0);

	// Enable TIM7 CLK
	__HAL_RCC_TIM7_CLK_ENABLE();

	// Enable ITs for TIM7
	HAL_NVIC_EnableIRQ(TIM7_IRQn);
	HAL_NVIC_SetPriority(TIM7_IRQn, 3, 0);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	// Enable TIM10 CLK
	__HAL_RCC_TIM4_CLK_ENABLE();

	// Enable TIM10 ITs
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);

	// Configure GPIOs
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIOInitValues = {0};
	GPIOInitValues.Pin = GPIO_PIN_7;
	GPIOInitValues.Mode = GPIO_MODE_AF_PP;
	GPIOInitValues.Pull = GPIO_NOPULL;
	GPIOInitValues.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIOInitValues.Alternate = GPIO_AF2_TIM4;

	HAL_GPIO_Init(GPIOB, &GPIOInitValues);

	GPIOInitValues.Pin = GPIO_PIN_8;

	HAL_GPIO_Init(GPIOB, &GPIOInitValues);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	// Enable ADC CLK
	__HAL_RCC_ADC1_CLK_ENABLE();

	// Configure GPIOs
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIOInitValues = {0};
	GPIOInitValues.Pin = GPIO_PIN_0;
	GPIOInitValues.Mode = GPIO_MODE_ANALOG;
	GPIOInitValues.Pull = GPIO_NOPULL;
	GPIOInitValues.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIOInitValues.Alternate = GPIO_AF3_TIM10;

	HAL_GPIO_Init(GPIOB, &GPIOInitValues);

}
