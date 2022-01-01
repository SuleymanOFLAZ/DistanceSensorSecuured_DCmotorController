/*
 * main.c
 *
 *  Created on: Dec 13, 2021
 *      Author: Suleyman Oflaz
 *  Used Board: STM32F407G-DISC1
 *
 *  Description: 	Distance Auto-Stop secured buzzer and led alerted,
 *  				two direction brushed DC motor controller with speed control.
 *  				For more information: ReadMe.md
 *
 * 	Used Pins:
 * 		PWM Signal Outputs: -- Brushed DC Motor Drive
 * 			* PB7
 * 			* PB8
 * 		Square Signal Output: -- Buzzer Driven
 * 			* PA2
 * 		Sensor I/Os: -- HC-SR04
 * 			* PA1 -- Output (Trigger)
 * 			* PA3 -- Input (Echo)
 * 		ADC Input:
 * 			* PB0 (10k Pot)
 * 		LED Outputs:
 * 			* PB11 -- Sensor Light
 * 			* PB12 -- Emergency Light
 * 			* PB13 -- Motor Direction Light 1
 * 			* PB14 -- Motor Direction Light 2
 * 			* PB5  -- Motor Status Light
 * 		UART I/Os:
 * 			* PD8 -- TX
 * 			* PD9 -- RX
 * 		Button/Switch Inputs:
 * 			* PA0  -- Motor Direction Change Button
 * 			* PB15 -- Motor-On Switch (Active Low)
 *
 */

#include "main.h"
#include <stdio.h>
#include <string.h>

#define AlertDistance 20	// in cm, must be between 2cm - 200cm

void ConfigureSystemClock(void);
void UART_Init(void);
void TIM_Init(void);
void buton_init(void);
void led_init(void);
void prnt_over_uart(uint8_t *ptr, uint16_t size);
void HCSR04Init(void);
void handle_distance_sensor(void);
void HCSR04_Trigger(void);
uint32_t HCSR04_Measure_Distance(void);
void TIM7_PeriodElapsedHandle(TIM_HandleTypeDef *htim);
void ADC_Init(void);
void motor_speed_handle(void);
void Motor_Change_Direction(void);
uint32_t  read_adc(void);
void Emergency_Handle(void);

UART_HandleTypeDef Usart3Handle = {0};
TIM_HandleTypeDef Tim9OCHandle = {0};
TIM_HandleTypeDef Tim10PWMHandle = {0};
TIM_HandleTypeDef Tim6Handle = {0};
TIM_HandleTypeDef Tim7Handle = {0};
ADC_HandleTypeDef ADCHandle = {0};
char message[50];
uint32_t tim6_global_tick = 0;
double distance = 0;
uint8_t buzzer_state = 0;
uint8_t buzzer_status = 0;
uint8_t emergency_state = 0;
uint8_t motor_direction = 0;
uint8_t motor_status = 1;

int main(void)
{
	HAL_Init();
	ConfigureSystemClock();

	UART_Init();

	TIM_Init();

	ADC_Init();

	buton_init();

	led_init();

	HCSR04Init();


	if(HAL_TIM_PWM_Start(&Tim10PWMHandle, TIM_CHANNEL_2) != HAL_OK)
		while(1);


	strcpy(message,"Initializations are done!\n\r");
	prnt_over_uart((uint8_t *)message, strlen(message));

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

	while(1)
	{
		handle_distance_sensor();

		motor_speed_handle();
	}

	while(1);
}

void ConfigureSystemClock(void)
{
	/* System Clock configured as 25Mhz  */

	RCC_OscInitTypeDef OscInitVariables = {0};
	RCC_PLLInitTypeDef PLLInitVariables = {0};

	PLLInitVariables.PLLState = RCC_PLL_ON;
	PLLInitVariables.PLLSource = RCC_PLLSOURCE_HSE;
	PLLInitVariables.PLLM = 8;
	PLLInitVariables.PLLN = 50;
	PLLInitVariables.PLLP = RCC_PLLP_DIV2;
	PLLInitVariables.PLLQ = 2		;

	OscInitVariables.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	OscInitVariables.HSEState = RCC_HSE_ON;
	OscInitVariables.LSEState = RCC_LSE_OFF;
	OscInitVariables.HSIState = RCC_HSI_ON;
	OscInitVariables.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	OscInitVariables.PLL = PLLInitVariables;

	if(HAL_RCC_OscConfig(&OscInitVariables) != HAL_OK)
		while(1);

	RCC_ClkInitTypeDef CLKInitVariables = {0};
	CLKInitVariables.ClockType = RCC_CLOCKTYPE_SYSCLK || RCC_CLOCKTYPE_HCLK || RCC_CLOCKTYPE_PCLK1 || RCC_CLOCKTYPE_PCLK2;
	CLKInitVariables.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	CLKInitVariables.AHBCLKDivider = RCC_SYSCLK_DIV1;
	CLKInitVariables.APB1CLKDivider = RCC_HCLK_DIV1;
	CLKInitVariables.APB2CLKDivider = RCC_HCLK_DIV1;

	if(HAL_RCC_ClockConfig(&CLKInitVariables, FLASH_ACR_LATENCY_0WS) != HAL_OK)
		while(1);

	__HAL_RCC_HSI_DISABLE();
}

void UART_Init(void)
{
	UART_InitTypeDef UartInitVariables = {0};

	UartInitVariables.BaudRate = 115200;
	UartInitVariables.WordLength = UART_WORDLENGTH_8B;
	UartInitVariables.StopBits = UART_STOPBITS_1;
	UartInitVariables.Parity = UART_PARITY_NONE;
	UartInitVariables.Mode = UART_MODE_TX;
	UartInitVariables.HwFlowCtl = UART_HWCONTROL_NONE;
	UartInitVariables.OverSampling = UART_OVERSAMPLING_8;

	Usart3Handle.Instance = USART3;
	Usart3Handle.Init = UartInitVariables;


	if(HAL_UART_Init(&Usart3Handle) != HAL_OK)
		while(1);
}

void TIM_Init(void)
{
	/* TIM9 Init for buzzer signal ------------------------------------------------*/
	TIM_Base_InitTypeDef TimInitVariables = {0};

	TimInitVariables.Prescaler = 25;
	TimInitVariables.CounterMode = TIM_COUNTERMODE_DOWN;
	TimInitVariables.Period = 400;
	TimInitVariables.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TimInitVariables.RepetitionCounter = 0;
	TimInitVariables.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	Tim9OCHandle.Instance = TIM9;
	Tim9OCHandle.Init = TimInitVariables;

	if(HAL_TIM_OC_Init(&Tim9OCHandle) != HAL_OK)
		while(1);

	TIM_OC_InitTypeDef Tim9CHConfigVariables = {0};

	Tim9CHConfigVariables.OCMode = TIM_OCMODE_TOGGLE;
	Tim9CHConfigVariables.Pulse = 0;
	Tim9CHConfigVariables.OCPolarity = TIM_OCPOLARITY_HIGH;
	Tim9CHConfigVariables.OCNPolarity = TIM_OCPOLARITY_LOW;
	Tim9CHConfigVariables.OCFastMode = TIM_OCFAST_DISABLE;
	Tim9CHConfigVariables.OCIdleState = TIM_OCIDLESTATE_RESET;
	Tim9CHConfigVariables.OCNIdleState = TIM_OCIDLESTATE_RESET;

	if(HAL_TIM_OC_ConfigChannel(&Tim9OCHandle, &Tim9CHConfigVariables, TIM_CHANNEL_1) != HAL_OK)
		while(1);

	/*----------------------------------------------------------------------------*/

	/* TIM6 Init for time counting -----------------------------------------------*/

	TimInitVariables.Prescaler = 25;
	TimInitVariables.CounterMode = TIM_COUNTERMODE_UP;
	TimInitVariables.Period = 0xFFFF;
	TimInitVariables.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TimInitVariables.RepetitionCounter = 0;
	TimInitVariables.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	Tim6Handle.Instance = TIM6;
	Tim6Handle.Init = TimInitVariables;

	if(HAL_TIM_Base_Init(&Tim6Handle) != HAL_OK)
		while(1);

	/*----------------------------------------------------------------------------*/

	/* TIM7 Init for buzzer delay ------------------------------------------------*/

	TimInitVariables.Prescaler = 100;
	TimInitVariables.CounterMode = TIM_COUNTERMODE_UP;
	TimInitVariables.Period = 100000;
	TimInitVariables.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TimInitVariables.RepetitionCounter = 0;
	TimInitVariables.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	Tim7Handle.Instance = TIM7;
	Tim7Handle.Init = TimInitVariables;

	if(HAL_TIM_Base_Init(&Tim7Handle) != HAL_OK)
		while(1);

	/*----------------------------------------------------------------------------*/

	/* TIM10 Init for motor driven PWM signal ------------------------------------*/

	TimInitVariables.Prescaler = 25;
	TimInitVariables.CounterMode = TIM_COUNTERMODE_UP;
	TimInitVariables.Period = 500;
	TimInitVariables.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TimInitVariables.RepetitionCounter = 0;
	TimInitVariables.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	Tim10PWMHandle.Instance = TIM4;
	Tim10PWMHandle.Init = TimInitVariables;

	if(HAL_TIM_PWM_Init(&Tim10PWMHandle) != HAL_OK)
		while(1);

	TIM_OC_InitTypeDef Tim10CHConfigVariables = {0};

	Tim10CHConfigVariables.OCMode = TIM_OCMODE_PWM1;
	Tim10CHConfigVariables.Pulse = 15;
	Tim10CHConfigVariables.OCPolarity = TIM_OCPOLARITY_HIGH;
	Tim10CHConfigVariables.OCNPolarity = TIM_OCPOLARITY_LOW;
	Tim10CHConfigVariables.OCFastMode = TIM_OCFAST_DISABLE;
	Tim10CHConfigVariables.OCIdleState = TIM_OCIDLESTATE_RESET;
	Tim10CHConfigVariables.OCNIdleState = TIM_OCIDLESTATE_RESET;

	if(HAL_TIM_PWM_ConfigChannel(&Tim10PWMHandle, &Tim10CHConfigVariables, TIM_CHANNEL_2) != HAL_OK)
		while(1);

	Tim10CHConfigVariables.OCMode = TIM_OCMODE_PWM1;
	Tim10CHConfigVariables.Pulse = 15;
	Tim10CHConfigVariables.OCPolarity = TIM_OCPOLARITY_HIGH;
	Tim10CHConfigVariables.OCNPolarity = TIM_OCPOLARITY_LOW;
	Tim10CHConfigVariables.OCFastMode = TIM_OCFAST_DISABLE;
	Tim10CHConfigVariables.OCIdleState = TIM_OCIDLESTATE_RESET;
	Tim10CHConfigVariables.OCNIdleState = TIM_OCIDLESTATE_RESET;

	if(HAL_TIM_PWM_ConfigChannel(&Tim10PWMHandle, &Tim10CHConfigVariables, TIM_CHANNEL_3) != HAL_OK)
		while(1);

	/*----------------------------------------------------------------------------*/
}

void buton_init(void)
{
	// Configure GPIOs
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIOInitValues = {0};
	GPIOInitValues.Pin = GPIO_PIN_0;
	GPIOInitValues.Mode = GPIO_MODE_IT_RISING;
	GPIOInitValues.Pull = GPIO_NOPULL;
	GPIOInitValues.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIOInitValues.Alternate = 0;

	HAL_GPIO_Init(GPIOA, &GPIOInitValues);

	// Enable IRQs
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIOInitValues.Pin = GPIO_PIN_15;
	GPIOInitValues.Mode = GPIO_MODE_INPUT;
	GPIOInitValues.Pull = GPIO_PULLUP ;
	GPIOInitValues.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIOInitValues.Alternate = 0;

	HAL_GPIO_Init(GPIOB, &GPIOInitValues);
}

void led_init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIOInitValues = {0};
	GPIOInitValues.Pin = GPIO_PIN_11;
	GPIOInitValues.Mode = GPIO_MODE_OUTPUT_PP;
	GPIOInitValues.Pull = GPIO_NOPULL;
	GPIOInitValues.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIOInitValues.Alternate = 0;

	HAL_GPIO_Init(GPIOB, &GPIOInitValues);

	GPIOInitValues.Pin = GPIO_PIN_12;
	HAL_GPIO_Init(GPIOB, &GPIOInitValues);

	GPIOInitValues.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOB, &GPIOInitValues);

	GPIOInitValues.Pin = GPIO_PIN_14;
	HAL_GPIO_Init(GPIOB, &GPIOInitValues);

	GPIOInitValues.Pin = GPIO_PIN_5;
	HAL_GPIO_Init(GPIOB, &GPIOInitValues);


}

void prnt_over_uart(uint8_t *ptr, uint16_t size)
{
	while( Usart3Handle.gState != HAL_UART_STATE_READY);
	if(HAL_UART_Transmit_IT(&Usart3Handle, ptr, size) != HAL_OK)
		while(1);
}

void HCSR04Init(void)
{
	// Configure GPIOs
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIOInitValues = {0};
	GPIOInitValues.Pin = GPIO_PIN_1;
	GPIOInitValues.Mode = GPIO_MODE_OUTPUT_PP;
	GPIOInitValues.Pull = GPIO_PULLDOWN;
	GPIOInitValues.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIOInitValues.Alternate = 0;

	HAL_GPIO_Init(GPIOA, &GPIOInitValues);

	GPIOInitValues.Pin = GPIO_PIN_3;
	GPIOInitValues.Mode = GPIO_MODE_INPUT;

	HAL_GPIO_Init(GPIOA, &GPIOInitValues);
}

void ADC_Init(void)
{
	ADC_InitTypeDef ADCInitVariables = {0};

	ADCInitVariables.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	ADCInitVariables.Resolution = ADC_RESOLUTION_12B;
	ADCInitVariables.DataAlign = ADC_DATAALIGN_RIGHT;
	ADCInitVariables.ScanConvMode = ENABLE;
	ADCInitVariables.EOCSelection = ADC_EOC_SINGLE_SEQ_CONV;
	ADCInitVariables.ContinuousConvMode = ENABLE;
	ADCInitVariables.NbrOfConversion = 4;
	ADCInitVariables.DiscontinuousConvMode = DISABLE;
	ADCInitVariables.NbrOfDiscConversion = 0;
	ADCInitVariables.ExternalTrigConv = ADC_SOFTWARE_START;
	ADCInitVariables.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	ADCInitVariables.DMAContinuousRequests = DISABLE;

	ADCHandle.Instance = ADC1;
	ADCHandle.Init = ADCInitVariables;

	if(HAL_ADC_Init(&ADCHandle) != HAL_OK)
		while(1);

	ADC_ChannelConfTypeDef ADC_CH_ConfgVariables = {0};
	ADC_CH_ConfgVariables.Channel = ADC_CHANNEL_8;
	ADC_CH_ConfgVariables.Rank = 1;
	ADC_CH_ConfgVariables.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	ADC_CH_ConfgVariables.Offset = 0;


	if(HAL_ADC_ConfigChannel(&ADCHandle, &ADC_CH_ConfgVariables) != HAL_OK)
		while(1);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
	{
		// Motor direction switch button triggered
		Motor_Change_Direction();
	}

}

void HCSR04_Trigger(void)
{
	// Set the trigger pin
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

	// Reset the trigger pin
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

uint32_t HCSR04_Measure_Distance(void)
{
	uint32_t total_tick = 0, readings[5] = {0};

	for(uint8_t cnt = 0; cnt < 5; cnt++)
	{
		// Disable All Interrupts for avoiding the pin read problem (with polling methode) from external signal
		__disable_irq();

		// Trigger (PA1)
		HCSR04_Trigger();

		// Wait until echo pin gets high (PA3)
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET);

		// Start TIM6
		if(HAL_TIM_Base_Start_IT(&Tim6Handle) != HAL_OK)
			while(1);

		// Wait until echo pin gets low (PA3)
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET);

		// Enable all intterupts
		__enable_irq();

		// Calculate total timer tick
		readings[cnt] = TIM6->CNT + (tim6_global_tick * 0xFFFF);
		tim6_global_tick = 0;

		// Stop and Reset the TIM6
		if(HAL_TIM_Base_Stop_IT(&Tim6Handle) != HAL_OK)
			while(1);
		TIM6->CNT = 0;
	}

	total_tick = (readings[0] + readings[1] + readings[2] + readings[3] + readings[4])/5;


	// eject if the value is not in the sensing range
	if(total_tick < 100 || total_tick > 23200)  // if range is not between 1.7cm - 4000cm
	{
		total_tick = 0;

	}

	return total_tick/58; // in cm
}

void handle_distance_sensor(void)
{
	distance = HCSR04_Measure_Distance();

	emergency_state = 0;
	buzzer_state = 0;

	// eject if the value is not in the sensing range
	if(distance > 2 && distance < 200)  // if range between 2cm - 200cm
	{
		buzzer_state = 1;

		if(Tim7Handle.State == HAL_TIM_STATE_READY)
		{
		if(HAL_TIM_Base_Start_IT(&Tim7Handle) != HAL_OK)
			while(1);
		}
	}
	if(distance > 2 && AlertDistance < 20)  // if range is between 2cm - 20cm
	{
		emergency_state = 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
		tim6_global_tick++;
	else if(htim->Instance == TIM7)
	{
		TIM7_PeriodElapsedHandle(htim);
	}
}

void TIM7_PeriodElapsedHandle(TIM_HandleTypeDef *htim)
{
	uint32_t buzzer_time_gap_count = 0;

	if(emergency_state == 1)
	{
		// Turn on buzzer
		if((*Tim9OCHandle.ChannelState == HAL_TIM_CHANNEL_STATE_READY)  && (buzzer_state == 1) && (motor_status == 1))
		{
			if(HAL_TIM_OC_Start(&Tim9OCHandle, TIM_CHANNEL_1) != HAL_OK)
				while(1);
		}

		// Turn on emergency light
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	}
	else if(buzzer_state == 1 && emergency_state == 0)
	{
		// Update buzzer time gap
		if(distance > 0)
			{
			if(Tim7Handle.State == HAL_TIM_STATE_BUSY)
			{
				if(HAL_TIM_Base_Stop_IT(&Tim7Handle) != HAL_OK)
					while(1);
			}

			buzzer_time_gap_count = 10000+(500*((distance/200)*100));
			TIM7->CNT = 0;
			TIM7->ARR = buzzer_time_gap_count;

			if(Tim7Handle.State == HAL_TIM_STATE_READY)
			{
				if(HAL_TIM_Base_Start_IT(&Tim7Handle) != HAL_OK)
					while(1);
			}
		}

		// Toggle buzzer status
		if((*Tim9OCHandle.ChannelState == HAL_TIM_CHANNEL_STATE_READY) && (buzzer_state == 1) && (motor_status == 1))
		{
			if(motor_status == 1)
			{
				if(HAL_TIM_OC_Start(&Tim9OCHandle, TIM_CHANNEL_1) != HAL_OK)
					while(1);
			}
			buzzer_status = 1;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
		}
		else if((*Tim9OCHandle.ChannelState == HAL_TIM_CHANNEL_STATE_BUSY) && (buzzer_state == 1) && (motor_status == 1))
		{

			if(motor_status == 1)
			{
				if(HAL_TIM_OC_Stop(&Tim9OCHandle, TIM_CHANNEL_1) != HAL_OK)
					while(1);
			}
			buzzer_status = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
		}

		if(buzzer_state == 1 && motor_status == 0)
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
		}

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	}
	else
	{
		// shut down buzzer
		if((*Tim9OCHandle.ChannelState == HAL_TIM_CHANNEL_STATE_BUSY)  && (buzzer_state == 0))
		{
			if(HAL_TIM_OC_Stop(&Tim9OCHandle, TIM_CHANNEL_1) != HAL_OK)
				while(1);
		}

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

		if(Tim7Handle.State == HAL_TIM_STATE_BUSY)
		{
		if(HAL_TIM_Base_Stop_IT(&Tim7Handle) != HAL_OK)
			while(1);
		}
	}
}

void motor_speed_handle(void)
{
	uint32_t value = 0;
	float calculated = 0;

	// Update motor status (external switch)
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == 1)
	{
		motor_status = 0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == 0)
	{
		motor_status = 1;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	}

	if((emergency_state == 1 && motor_status == 1) || motor_status == 0)
	{
		if(motor_direction == 0)
		{
			if(Tim10PWMHandle.ChannelState[1] == HAL_TIM_CHANNEL_STATE_BUSY)
			{
				if(HAL_TIM_PWM_Stop(&Tim10PWMHandle, TIM_CHANNEL_2) != HAL_OK)
					while(1);
			}
		}
		else if(motor_direction == 1)
		{
			if(Tim10PWMHandle.ChannelState[2] == HAL_TIM_CHANNEL_STATE_BUSY)
			{
				if(HAL_TIM_PWM_Stop(&Tim10PWMHandle, TIM_CHANNEL_3) != HAL_OK)
					while(1);
			}
		}

		if(emergency_state == 1)
			Emergency_Handle();

	}
	else if(emergency_state == 0 && motor_status == 1)
	{
		// Update motor speed

		// Get analog input value
		value = read_adc();

		calculated = ((float)value-2000)/45000 *100; // % input

		value = (calculated*4) +100;

		if(TIM4->CCR2 < value-20 || TIM4->CCR2 > value+20) // Eliminate small reading differences that cause non-stable motor speed problem
			TIM4->CCR2 = value; // Load to PWM Channel

		if(TIM4->CCR3 < value-20 || TIM4->CCR3 > value+20) // Eliminate small reading differences that cause non-stable motor speed problem
			TIM4->CCR3 = value; // Load to PWM Channel

		// start motors
		if(motor_direction == 0)
		{
			if(Tim10PWMHandle.ChannelState[1] == HAL_TIM_CHANNEL_STATE_READY)
			{
				if(HAL_TIM_PWM_Start(&Tim10PWMHandle, TIM_CHANNEL_2) != HAL_OK)
					while(1);
			}
		}
		else if(motor_direction == 1)
		{
			if(Tim10PWMHandle.ChannelState[2] == HAL_TIM_CHANNEL_STATE_READY)
			{
				if(HAL_TIM_PWM_Start(&Tim10PWMHandle, TIM_CHANNEL_3) != HAL_OK)
					while(1);
			}
		}
	}


}

uint32_t  read_adc(void)
{
	uint32_t read_value = 0;

	if(HAL_ADC_Start(&ADCHandle) != HAL_OK)
		while(1);

	if(HAL_ADC_PollForConversion(&ADCHandle, 5) != HAL_OK)
		while(1);

	read_value = HAL_ADC_GetValue(&ADCHandle);

	if(HAL_ADC_Stop(&ADCHandle) != HAL_OK)
		while(1);

	return read_value;
}

void Emergency_Handle(void)
{
	if(motor_status == 0)
	{

	}
	else if(motor_status == 1)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // Turn on emergency light

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // Turn off motor light

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

		if((Tim9OCHandle.ChannelState[0] == HAL_TIM_CHANNEL_STATE_READY))
		{
			if(HAL_TIM_OC_Start(&Tim9OCHandle, TIM_CHANNEL_1) != HAL_OK)
				while(1);
			buzzer_status = 1;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
		}

		// Disable All Interrupts for avoiding the pin read problem (with polling methode) from external signal
		__disable_irq();

		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET)
			while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET);

		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET);

		// Enable all interrupts
		__enable_irq();

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	}
}

void Motor_Change_Direction(void)
{
	// Stop the motor if it is on
	if(motor_status == 1)
	{
		if(motor_direction == 0)
		{
			if(Tim10PWMHandle.ChannelState[1] == HAL_TIM_CHANNEL_STATE_BUSY)
			{
				if(HAL_TIM_PWM_Stop(&Tim10PWMHandle, TIM_CHANNEL_2) != HAL_OK)
					while(1);
			}
		}
		else if(motor_direction == 1)
		{
			if(Tim10PWMHandle.ChannelState[2] == HAL_TIM_CHANNEL_STATE_BUSY)
			{
				if(HAL_TIM_PWM_Stop(&Tim10PWMHandle, TIM_CHANNEL_3) != HAL_OK)
					while(1);
			}
		}

	}
	// Wait some
	HAL_Delay(100); // 10 ms

	if(motor_direction == 0)
	{
		motor_direction = 1;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	}
	else if(motor_direction == 1)
	{
		motor_direction = 0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	}


	// Start the motor if it is on
	if(motor_direction == 0 && motor_status == 1 && emergency_state ==  0)
	{
		if(Tim10PWMHandle.ChannelState[1] == HAL_TIM_CHANNEL_STATE_READY)
		{
			if(HAL_TIM_PWM_Start(&Tim10PWMHandle, TIM_CHANNEL_2) != HAL_OK)
				while(1);
		}
	}
	else if(motor_direction == 1 && motor_status == 1 && emergency_state ==  0)
	{
		if(Tim10PWMHandle.ChannelState[2] == HAL_TIM_CHANNEL_STATE_READY)
		{
			if(HAL_TIM_PWM_Start(&Tim10PWMHandle, TIM_CHANNEL_3) != HAL_OK)
				while(1);
		}
	}
}
