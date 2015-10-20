//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "diag/Trace.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_cortex.h"
// ----------------------------------------------------------------------------
//
// Standalone STM32F4 empty sample (trace via DEBUG).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).


// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"
void PWM_Init(TIM_HandleTypeDef * TimHandle, TIM_OC_InitTypeDef * OCHandle);
void TIM9_Init(TIM_HandleTypeDef * TimHandle);
void ADC_Init(ADC_HandleTypeDef * ADC_Handle);
void DMA_Init(__DMA_HandleTypeDef * DMAHandle);
static TIM_HandleTypeDef Tim1Handle;	//STM HAL variable containing TIM config
static int index = 0;
TIM_HandleTypeDef Tim4Handle;
//NVIC_InitTypeDef NVIC_InitStructure;
volatile const static uint16_t sin_table[360] = {
	0, 293, 586, 879, 1171, 1464, 1756, 2047, 2338, 2628,
	2917, 3205, 3492, 3779, 4064, 4348, 4630, 4911, 5191, 5469,
	5745, 6020, 6293, 6564, 6833, 7099, 7364, 7627, 7887, 8144,
	8400, 8652, 8902, 9149, 9394, 9636, 9874, 10110, 10343, 10572,
	10798, 11021, 11241, 11457, 11670, 11879, 12084, 12286, 12484, 12679,
	12869, 13056, 13238, 13417, 13591, 13761, 13927, 14089, 14247, 14400,
	14549, 14693, 14833, 14968, 15099, 15225, 15347, 15464, 15576, 15684,
	15786, 15884, 15977, 16065, 16149, 16227, 16300, 16369, 16432, 16491,
	16544, 16593, 16636, 16674, 16707, 16736, 16759, 16776, 16789, 16797,
	16800, 16797, 16789, 16776, 16759, 16736, 16707, 16674, 16636, 16593,
	16544, 16491, 16432, 16369, 16300, 16227, 16149, 16065, 15977, 15884,
	15786, 15684, 15576, 15464, 15347, 15225, 15099, 14968, 14833, 14693,
	14549, 14400, 14247, 14089, 13927, 13761, 13591, 13417, 13238, 13056,
	12869, 12679, 12484, 12286, 12084, 11879, 11670, 11457, 11241, 11021,
	10798, 10572, 10343, 10110, 9874, 9636, 9394, 9149, 8902, 8652,
	8399, 8144, 7887, 7627, 7364, 7099, 6833, 6564, 6293, 6020,
	5745, 5469, 5191, 4911, 4630, 4348, 4064, 3779, 3492, 3205,
	2917, 2628, 2338, 2047, 1756, 1464, 1171, 879, 586, 293,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int main(int argc, char* argv[])
{

	TIM_OC_InitTypeDef OCHandle;

	HAL_Init(); //Initialize Hardware Abstraction Layer
	__HAL_RCC_TIM1_CLK_ENABLE();
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE(); //Used for pwm output
	PWM_Init(&Tim1Handle,&OCHandle);
/*	//	uint32_t ADCval, ARRval;

	//ADC_HandleTypeDef ADC_Handle;

	//__DMA_HandleTypeDef DMAHandle;


	//__HAL_RCC_ADC1_CLK_ENABLE();

	//__HAL_RCC_GPIOC_CLK_ENABLE(); //Used for ADC input


	//ADC_Init(&ADC_Handle); //ADC is connected to PC2
	//DMA_Init(&DMAHandle);
	while(1){ //endless loop
*/
/*
		// Start the conversion
		if (HAL_ADC_Start(&ADC_Handle) != HAL_OK) {
			// TODO: Error_Handler();
			while(1);
		}

		// Wait for end of conversion
		if (HAL_ADC_PollForConversion(&ADC_Handle, 1000) != HAL_OK) {
			// TODO: Error_Handler();
			while(1);
		}

		// Get the conversion value
		ADCval = HAL_ADC_GetValue(&ADC_Handle);

		// Stop the ADC
		HAL_ADC_Stop(&ADC_Handle);

		ARRval = (ADCval - 0x0000) * ((65535-655)/0xFFF) + 655;
		//__HAL_TIM_SET_PRESCALER(&TimHandle,ARRval);
		__HAL_TIM_SET_AUTORELOAD(&TimHandle,ARRval);
		__HAL_TIM_SET_COMPARE(&TimHandle,TIM_CHANNEL_1,ARRval/2);

	//	HAL_TIM_PWM_ConfigChannel(&TimHandle, &OCHandle, TIM_CHANNEL_1);
	//	HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1);

	}
*/

	 while(1);
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
void TIM4_IRQHandler(void)
{
    __HAL_TIM_CLEAR_FLAG(&Tim4Handle, TIM_FLAG_UPDATE);
    /*put your code here */
}

void PWM_Init(TIM_HandleTypeDef * TimHandle, TIM_OC_InitTypeDef * OCHandle)
{
	GPIO_InitTypeDef GPIO_BaseStruct;
	//set up all the parameter in the GPIO struct
	GPIO_BaseStruct.Pin = GPIO_PIN_8;
	GPIO_BaseStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_BaseStruct.Pull = GPIO_NOPULL;
	GPIO_BaseStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_BaseStruct.Alternate = GPIO_AF1_TIM1;

	HAL_GPIO_Init(GPIOA, &GPIO_BaseStruct); //Initalize the struct


	Tim1Handle.Instance = TIM1;
	TimHandle->Init.Period = 16800;
	TimHandle->Init.Prescaler = 0;
	TimHandle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TimHandle->Init.CounterMode = TIM_COUNTERMODE_UP;
	TimHandle->State = HAL_TIM_STATE_RESET;

	if (HAL_TIM_PWM_Init(TimHandle) != HAL_OK) {
		// TODO: Error_Handler();
		while(1);
	}

	OCHandle->OCMode = TIM_OCMODE_PWM1;
	OCHandle->Pulse = 16800/2;
	OCHandle->OCPolarity = TIM_OCPOLARITY_HIGH;
	OCHandle->OCNPolarity = TIM_OCPOLARITY_HIGH;
	OCHandle->OCIdleState = TIM_OCIDLESTATE_RESET;
	OCHandle->OCNIdleState = TIM_OCNIDLESTATE_RESET;
	OCHandle->OCFastMode = TIM_OCFAST_ENABLE;

	HAL_TIM_PWM_ConfigChannel(TimHandle, OCHandle, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(TimHandle, TIM_CHANNEL_1);
}

// ----------------------------------------------------------------------------

void ADC_Init(ADC_HandleTypeDef * ADC_Handle)
{
	GPIO_InitTypeDef GPIO_ADC_InitStruct;

	GPIO_ADC_InitStruct.Pin = GPIO_PIN_2;
	GPIO_ADC_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_ADC_InitStruct.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOC, &GPIO_ADC_InitStruct);


	// Configure the ADC
	ADC_Handle->Instance = ADC1;
	ADC_Handle->Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	ADC_Handle->Init.Resolution = ADC_RESOLUTION_12B;
	ADC_Handle->Init.DataAlign = ADC_DATAALIGN_RIGHT;
	ADC_Handle->Init.ScanConvMode = DISABLE;
	ADC_Handle->Init.EOCSelection = DISABLE;
	ADC_Handle->Init.ContinuousConvMode = DISABLE;
	ADC_Handle->Init.DMAContinuousRequests = DISABLE;
	ADC_Handle->Init.NbrOfConversion = 1;
	ADC_Handle->Init.DiscontinuousConvMode = ENABLE;
	ADC_Handle->Init.NbrOfDiscConversion = 1;
	ADC_Handle->Init.ExternalTrigConv = ADC_SOFTWARE_START;
	ADC_Handle->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;

	if (HAL_ADC_Init(ADC_Handle) != HAL_OK) {
		// TODO: Error_Handler();
		while(1);
	}
	ADC_ChannelConfTypeDef sConfig;

	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	sConfig.Offset = 0;

	if (HAL_ADC_ConfigChannel(ADC_Handle, &sConfig) != HAL_OK) {
		// TODO: Error_Handler();
		while(1);
	}
}

// ----------------------------------------------------------------------------

void DMA_Init(__DMA_HandleTypeDef * DMAHandle)
{
	//DMAHandle->
}

// ----------------------------------------------------------------------------

void TIM9_Init(TIM_HandleTypeDef * TimHandle)
{
	Tim4Handle.Instance = TIM4;
	Tim4Handle.Init.Period = 50000;
	Tim4Handle.Init.Prescaler = 1679;
	Tim4Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Tim4Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim4Handle.State = HAL_TIM_STATE_RESET;

	if (HAL_TIM_Base_Init(&Tim4Handle) != HAL_OK) {
		// TODO: Error_Handler();
		while(1);
	}
/*	HAL_TIM_Base_Start_IT(&Tim4Handle); // start timer interrupts
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/
	HAL_NVIC_SetPriority(TIM4_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
}
