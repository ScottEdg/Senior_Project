//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

/*
 * main.cpp
 *      Author: Scott Edgerly & Rigel Paradise
 *      Description: This code is a work in progress and anything within this
 *      file is subject to change.  This code will eventually be used to spin a
 *      quadcopter motor up to 2000 RPM as stated in our Senior Project
 *      contract.
 */
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
#include "sin_table.h"

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

void PWM_Init(void);
void TIM4_Init(void);

static TIM_HandleTypeDef Tim1Handle; //used for Phase U
TIM_HandleTypeDef Tim4Handle; //Used to "count" through the sine LUT
static TIM_HandleTypeDef Tim8Handle; //Used for Phase V and W
static int index = 0;

// ----- main() ---------------------------------------------------------------
int main(int argc, char* argv[])
{

// --Initialization------------------------------------------------------------
	HAL_Init();
	__HAL_RCC_TIM1_CLK_ENABLE();
	__HAL_RCC_TIM8_CLK_ENABLE();
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE(); //Used for pwm output
	__HAL_RCC_GPIOC_CLK_ENABLE();

	PWM_Init();
	TIM4_Init();
// ----------------------------------------------------------------------------
	while(1);
}

#pragma GCC diagnostic pop


// ----------------------------------------------------------------------------

extern "C" void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&Tim4Handle, TIM_FLAG_UPDATE) != RESET)      //In case other interrupts are also running
    {
        if (__HAL_TIM_GET_ITSTATUS(&Tim4Handle, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_FLAG(&Tim4Handle, TIM_FLAG_UPDATE);								//(These pin assignments should be double checked before use
            __HAL_TIM_SET_COMPARE(&Tim1Handle,TIM_CHANNEL_1, sin_table[index]);				//Phase U+ on PA8
            __HAL_TIM_SET_COMPARE(&Tim1Handle,TIM_CHANNEL_3, sin_table[(index+45)%90]);	//Phase U- on PA10
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_1, sin_table[(index+30)%90]);	//Phase V+ on PC6
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_2, sin_table[(index+75)%90]);	//Phase V- on PC7
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_3, sin_table[(index+60)%90]);	//Phase W+ on PC8
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_4, sin_table[(index+15)%90]); 	//Phase W- on PC9
            index++;
            index = index % 90; //reset index to 0 if we get to 360
        }
    }
}

// ----------------------------------------------------------------------------

void PWM_Init(void)
{
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

	// --Set TIM1 to 10kHz switching frequency---------------------------------
	Tim1Handle.Instance = TIM1;
	Tim1Handle.Init.Period = 8399;
	Tim1Handle.Init.Prescaler = 0;
	Tim1Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Tim1Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim1Handle.State = HAL_TIM_STATE_RESET;

	if (HAL_TIM_PWM_Init(&Tim1Handle) != HAL_OK) {
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

	// --Set TIM8 to 10kHz switching frequency---------------------------------
	Tim8Handle.Instance = TIM8;
	Tim8Handle.Init.Period = 8399;
	Tim8Handle.Init.Prescaler = 0;
	Tim8Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Tim8Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim8Handle.State = HAL_TIM_STATE_RESET;

	if (HAL_TIM_PWM_Init(&Tim8Handle) != HAL_OK) {
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
//TODO: Perhaps find a better way to start all of these at once
	HAL_TIM_PWM_ConfigChannel(&Tim1Handle, &OCHandle, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&Tim1Handle, &OCHandle, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&Tim8Handle, &OCHandle, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&Tim8Handle, &OCHandle, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&Tim8Handle, &OCHandle, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&Tim8Handle, &OCHandle, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&Tim1Handle, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&Tim1Handle, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&Tim8Handle, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&Tim8Handle, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&Tim8Handle, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&Tim8Handle, TIM_CHANNEL_4);
	// ------------------------------------------------------------------------
}

// ----------------------------------------------------------------------------

void TIM4_Init()
{
	// --Set up TIM4-----------------------------------------------------------
	Tim4Handle.Instance = TIM4;
//TODO: Recalculate the Prescaler and create a range for the period to change the speed using ADC
	Tim4Handle.Init.Period = 95; //This changes how fast to go through the sine LUT
	Tim4Handle.Init.Prescaler = 104;
	Tim4Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Tim4Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim4Handle.State = HAL_TIM_STATE_RESET;

	if (HAL_TIM_Base_Init(&Tim4Handle) != HAL_OK) {
		// TODO: Error_Handler();
		while(1);
	}
	// ------------------------------------------------------------------------

	// --Priority set up-------------------------------------------------------
	HAL_TIM_Base_Start_IT(&Tim4Handle); // start timer interrupts

	HAL_NVIC_SetPriority(TIM4_IRQn, 0, 1); //set priority to highest
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	// ------------------------------------------------------------------------
}
