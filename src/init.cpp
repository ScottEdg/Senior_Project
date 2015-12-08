/*
 * init.cpp
 *
 *  Created on: Dec 7, 2015
 *      Author: Scott Edgerly and Rigel Paradise
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_i2c.h"
#include "init.h"


void CLK_Init(void){
	__HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
	__HAL_RCC_TIM8_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
}

// ----------------------------------------------------------------------------

void PWM_Init(TIM_HandleTypeDef * htim1, TIM_HandleTypeDef * htim2){
	// --Set pins PA8 & PA10 for Phase U---------------------------------------
	TIM_OC_InitTypeDef OCHandle;
	GPIO_InitTypeDef GPIO_BaseStruct;
	GPIO_BaseStruct.Pin = GPIO_PIN_8 | GPIO_PIN_10;
	GPIO_BaseStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_BaseStruct.Pull = GPIO_NOPULL;
	GPIO_BaseStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_BaseStruct.Alternate = GPIO_AF1_TIM1;

	HAL_GPIO_Init(GPIOA, &GPIO_BaseStruct);
	// ------------------------------------------------------------------------

	// --Set TIM1 to 20kHz switching frequency---------------------------------
	htim1->Instance = TIM1;
	htim1->Init.Period = 8399;
	htim1->Init.Prescaler = 0;
	htim1->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1->Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1->State = HAL_TIM_STATE_RESET;

	if (HAL_TIM_PWM_Init(htim1) != HAL_OK) {
		// TODO: Error_Handler();
		while(1);
	}
	// ------------------------------------------------------------------------

	// --Set pins PC6 & PC7 for Phase V and PC8 & PC9 for Phase W--------------
	GPIO_BaseStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_BaseStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_BaseStruct.Pull = GPIO_NOPULL;
	GPIO_BaseStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_BaseStruct.Alternate = GPIO_AF3_TIM8;

	HAL_GPIO_Init(GPIOC, &GPIO_BaseStruct); //Initalize the struct
	// ------------------------------------------------------------------------

	// --Set TIM8 to 20kHz switching frequency---------------------------------
	htim2->Instance = TIM8;
	htim2->Init.Period = 8399;
	htim2->Init.Prescaler = 0;
	htim2->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2->Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2->State = HAL_TIM_STATE_RESET;

	if (HAL_TIM_PWM_Init(htim2) != HAL_OK) {
		// TODO: Error_Handler();
		while(1);
	}
	// ------------------------------------------------------------------------

	// --Initialize OCR register for both TIM1 and TIM8 to 0-------------------
	OCHandle.OCMode = TIM_OCMODE_PWM1;
	OCHandle.Pulse = 0;
	OCHandle.OCPolarity = TIM_OCPOLARITY_HIGH;
	OCHandle.OCNPolarity = TIM_OCPOLARITY_HIGH;
	OCHandle.OCIdleState = TIM_OCIDLESTATE_RESET;
	OCHandle.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	OCHandle.OCFastMode = TIM_OCFAST_ENABLE;
	// ------------------------------------------------------------------------

	// --Configure and Start all timers and channels---------------------------

	HAL_TIM_PWM_ConfigChannel(htim1, &OCHandle, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(htim1, &OCHandle, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(htim2, &OCHandle, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(htim2, &OCHandle, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(htim2, &OCHandle, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(htim2, &OCHandle, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_4);
}

// ----------------------------------------------------------------------------

void TIM2_Init(TIM_HandleTypeDef * htim, TIM_IC_InitTypeDef * sConfig){
	// --Set up TIM2-----------------------------------------------------------
	htim->Instance = TIM2;
	htim->Init.Period = 0xFFFFFFFF; //This changes how fast to go through the sine LUT
	htim->Init.Prescaler = 0;
	htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim->Init.CounterMode = TIM_COUNTERMODE_UP;
	htim->State = HAL_TIM_STATE_RESET;

	if (HAL_TIM_IC_Init(htim) != HAL_OK) {
		// Error
	}

	sConfig->ICPrescaler = TIM_ICPSC_DIV1;
	sConfig->ICFilter = 0;
	sConfig->ICPolarity = TIM_ICPOLARITY_RISING;
	sConfig->ICSelection = TIM_ICSELECTION_INDIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(htim, sConfig, TIM_CHANNEL_1) != HAL_OK) {
		// Error
	}

	sConfig->ICPolarity = TIM_ICPOLARITY_FALLING;
	sConfig->ICSelection = TIM_ICSELECTION_DIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(htim, sConfig, TIM_CHANNEL_2) != HAL_OK) {
		// Error
	}

	if (HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1) != HAL_OK) {
		// Error
	}

	if (HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2) != HAL_OK) {
		// Error
	}
}

// ----------------------------------------------------------------------------

void TIM4_Init(TIM_HandleTypeDef * htim, uint32_t current){
	// --Set up TIM4-----------------------------------------------------------
	htim->Instance = TIM4;
	htim->Init.Period = current; //This changes how fast to go through the sine LUT
	htim->Init.Prescaler = 209;
	htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim->Init.CounterMode = TIM_COUNTERMODE_UP;
	htim->State = HAL_TIM_STATE_RESET;

	if (HAL_TIM_Base_Init(htim) != HAL_OK) {
		while(1);
	}

	htim->Instance->CR1 |= 1<<7;	//Set Auto-Reload Pre-Load Enable

	// --Priority set up-------------------------------------------------------
	HAL_TIM_Base_Start_IT(htim); // start timer interrupts

	HAL_NVIC_SetPriority(TIM4_IRQn, 0, 1); //set priority to highest
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	// ------------------------------------------------------------------------
}


void TIM5_Init(TIM_HandleTypeDef * htim, TIM_IC_InitTypeDef * sConfig){
	// --Set up Tim5-----------------------------------------------------------
	htim->Instance = TIM5;
	htim->Init.Period = 0xFFFFFFFF; //This changes how fast to go through the sine LUT
	htim->Init.Prescaler = 0;
	htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim->Init.CounterMode = TIM_COUNTERMODE_UP;
	htim->State = HAL_TIM_STATE_RESET;

	if (HAL_TIM_IC_Init(htim) != HAL_OK) {
		// Error
	}

	sConfig->ICPrescaler = TIM_ICPSC_DIV1;
	sConfig->ICFilter = 0;
	sConfig->ICPolarity = TIM_ICPOLARITY_FALLING;
	sConfig->ICSelection = TIM_ICSELECTION_DIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(htim, sConfig, TIM_CHANNEL_2) != HAL_OK) {
		// Error
	}

	if (HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2) != HAL_OK) {
		// Error
	}
}



void I2C_INIT(I2C_HandleTypeDef *hi2c){
	GPIO_InitTypeDef GPIO_BaseStruct;
	GPIO_BaseStruct.Pin = GPIO_PIN_6;
	GPIO_BaseStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_BaseStruct.Pull = GPIO_PULLUP;
	GPIO_BaseStruct.Speed = GPIO_SPEED_FAST;
	GPIO_BaseStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_BaseStruct);

	GPIO_BaseStruct.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOB, &GPIO_BaseStruct);

	hi2c->Instance = I2C1;
	hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c->Init.ClockSpeed = 50000;
	hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c->Init.DutyCycle = I2C_DUTYCYCLE_16_9;
	hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	hi2c->Init.OwnAddress1 = 0xFE;
	hi2c->Init.OwnAddress2 = 0xFE;

	if (HAL_I2C_Init(hi2c) != HAL_OK) {
		// TODO: ErrorHandler();
		while(1);
	}
}


void USER_Init(void){
	GPIO_InitTypeDef GPIO_BaseStruct;
	GPIO_BaseStruct.Pin = GPIO_PIN_0;
	GPIO_BaseStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_BaseStruct.Pull = GPIO_NOPULL;
	GPIO_BaseStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_BaseStruct);

	HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}


void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim){

	if (htim->Instance == TIM2) {

		GPIO_InitTypeDef GPIO_BaseStruct;

		GPIO_BaseStruct.Pin = GPIO_PIN_3;
		GPIO_BaseStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_BaseStruct.Pull = GPIO_PULLUP;
		GPIO_BaseStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_BaseStruct.Alternate = GPIO_AF1_TIM2;

		HAL_GPIO_Init(GPIOB, &GPIO_BaseStruct);

		HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
	}

	if (htim->Instance == TIM5){

		GPIO_InitTypeDef GPIO_BaseStruct;

		GPIO_BaseStruct.Pin = GPIO_PIN_1;
		GPIO_BaseStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_BaseStruct.Pull = GPIO_PULLUP;
		GPIO_BaseStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_BaseStruct.Alternate = GPIO_AF2_TIM5;

		HAL_GPIO_Init(GPIOA, &GPIO_BaseStruct);

		HAL_NVIC_SetPriority(TIM5_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(TIM5_IRQn);
	}
}


void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c){
		__HAL_RCC_I2C1_CLK_ENABLE();

		HAL_NVIC_SetPriority(I2C1_EV_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
		HAL_NVIC_SetPriority(I2C1_ER_IRQn, 4, 0);
		HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}
