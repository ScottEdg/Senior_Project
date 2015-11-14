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
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_tim_ex.h"
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
void TIM2_Init(void);
void TIM3_Init(void);
void TIM4_Init(void);
void I2C_INIT(void);
void USER_Init(void);
void Display_Init(void);

extern "C"{
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void EXTI0_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
};

#define SPEEDSAMPLES 25
#define SENSORSAMPLES 10
static volatile uint32_t buffer[SENSORSAMPLES];
static volatile uint32_t speed[SPEEDSAMPLES];

static TIM_HandleTypeDef Tim1Handle; //used for Phase U
static TIM_HandleTypeDef Tim2Handle;
static TIM_HandleTypeDef Tim3Handle;
static TIM_HandleTypeDef Tim4Handle; //Used to "count" through the sine LUT
static TIM_HandleTypeDef Tim8Handle; //Used for Phase V and W
static I2C_HandleTypeDef I2CHandle;
static TIM_IC_InitTypeDef SysConf;

static int index = 0;
uint32_t desired = 1143;
uint32_t current = desired;

static HAL_LockTypeDef speedLock = HAL_UNLOCKED;
static HAL_LockTypeDef sensorLock = HAL_UNLOCKED;

#define _GET_LOCK_NORETURN(__LOCK__)            \
	do{                                        	\
		if((__LOCK__) == HAL_LOCKED){           \
			return;                    			\
		}                                      	\
		else{                                   \
			(__LOCK__) = HAL_LOCKED;    		\
		}                                      	\
	}while (0)

#define _UNLOCK(__LOCK__)						\
	do{											\
		(__LOCK__) = HAL_UNLOCKED;				\
	}while (0)

// ----- main() ---------------------------------------------------------------
int main(int argc, char* argv[])
{

// --Initialization------------------------------------------------------------
	__HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM8_CLK_ENABLE();
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	PWM_Init();
	TIM4_Init();
	TIM2_Init();
	TIM3_Init();
	I2C_INIT();
	HAL_TIM_IC_Init(&Tim2Handle);
	HAL_TIM_IC_Init(&Tim3Handle);
	HAL_I2C_Init(&I2CHandle);
	USER_Init();

	HAL_Delay(100);

	Display_Init();

// ----------------------------------------------------------------------------
	while(1);
}

#pragma GCC diagnostic pop


// ----------------------------------------------------------------------------

void PWM_Init(void){
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

	// --Set TIM8 to 20kHz switching frequency---------------------------------
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

void TIM2_Init(){
	// --Set up TIM2-----------------------------------------------------------
	Tim2Handle.Instance = TIM2;
	Tim2Handle.Init.Period = 0xFFFFFFFF; 										//This changes how fast to go through the sine LUT
	Tim2Handle.Init.Prescaler = 0;
	Tim2Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Tim2Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim2Handle.State = HAL_TIM_STATE_RESET;

	if (HAL_TIM_IC_Init(&Tim2Handle) != HAL_OK) {
		// Error
	}

	SysConf.ICPrescaler = TIM_ICPSC_DIV1;
	SysConf.ICFilter = 0;
	SysConf.ICPolarity = TIM_ICPOLARITY_RISING;
	SysConf.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&Tim2Handle, &SysConf, TIM_CHANNEL_1) != HAL_OK) {
		// Error
	}

	SysConf.ICPolarity = TIM_ICPOLARITY_FALLING;
	SysConf.ICSelection = TIM_ICSELECTION_DIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&Tim2Handle, &SysConf, TIM_CHANNEL_2) != HAL_OK) {
		// Error
	}

	if (HAL_TIM_IC_Start_IT(&Tim2Handle, TIM_CHANNEL_1) != HAL_OK) {
		// Error
	}

	if (HAL_TIM_IC_Start_IT(&Tim2Handle, TIM_CHANNEL_2) != HAL_OK) {
		// Error
	}
}

// ----------------------------------------------------------------------------

void TIM3_Init(){
	// --Set up TIM3-----------------------------------------------------------
	Tim3Handle.Instance = TIM3;
	Tim3Handle.Init.Period = 0xFFFF; 												//This changes how fast to go through the sine LUT
	Tim3Handle.Init.Prescaler = 0;
	Tim3Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Tim3Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim3Handle.State = HAL_TIM_STATE_RESET;

	if (HAL_TIM_IC_Init(&Tim3Handle) != HAL_OK) {
		// Error
	}

	SysConf.ICPrescaler = TIM_ICPSC_DIV1;
	SysConf.ICFilter = 0;
	SysConf.ICPolarity = TIM_ICPOLARITY_FALLING;
	SysConf.ICSelection = TIM_ICSELECTION_DIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&Tim3Handle, &SysConf, TIM_CHANNEL_1) != HAL_OK) {
		// Error
	}

	if (HAL_TIM_IC_Start_IT(&Tim3Handle, TIM_CHANNEL_1) != HAL_OK) {
		// Error
	}
}

// ----------------------------------------------------------------------------

void TIM4_Init(){
	// --Set up TIM4-----------------------------------------------------------
	Tim4Handle.Instance = TIM4;
	Tim4Handle.Init.Period = current; //This changes how fast to go through the sine LUT
	Tim4Handle.Init.Prescaler = 209;
	Tim4Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Tim4Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim4Handle.State = HAL_TIM_STATE_RESET;

	if (HAL_TIM_Base_Init(&Tim4Handle) != HAL_OK) {
		while(1);
	}
	// ------------------------------------------------------------------------

	// --Priority set up-------------------------------------------------------
	HAL_TIM_Base_Start_IT(&Tim4Handle); // start timer interrupts

	HAL_NVIC_SetPriority(TIM4_IRQn, 0, 1); //set priority to highest
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	// ------------------------------------------------------------------------
}

void I2C_INIT(){
	GPIO_InitTypeDef GPIO_BaseStruct;
	GPIO_BaseStruct.Pin = GPIO_PIN_6;
	GPIO_BaseStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_BaseStruct.Pull = GPIO_PULLUP;
	GPIO_BaseStruct.Speed = GPIO_SPEED_FAST;
	GPIO_BaseStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_BaseStruct);

	GPIO_BaseStruct.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOB, &GPIO_BaseStruct);

	I2CHandle.Instance = I2C1;
	I2CHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	I2CHandle.Init.ClockSpeed = 50000;
	I2CHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2CHandle.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
	I2CHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2CHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	I2CHandle.Init.OwnAddress1 = 0xFE;
	I2CHandle.Init.OwnAddress2 = 0xFE;

	if (HAL_I2C_Init(&I2CHandle) != HAL_OK) {
		// TODO: ErrorHandler();
		while(1);
	}
}

void I2C1_EV_IRQHandler(void){
	HAL_I2C_EV_IRQHandler(&I2CHandle);
}

void I2C1_ER_IRQHandler(void){
	HAL_I2C_ER_IRQHandler(&I2CHandle);
}

// ----------------------------------------------------------------------------

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

// ----------------------------------------------------------------------------

void Display_Init(void){
	uint8_t startup10[3] = {0xFE, 0x45, 0x4D};
	uint8_t startup9[4] = "DIE";
	uint8_t startup5[2] = {0xFE, 0x51};
	uint8_t startup1[39] = "DESIRED:----RPM";
	uint8_t startup4[3] = {0xFE, 0x45, 0x40};
	uint8_t startup2[42] = "CURRENT:-------";

	HAL_I2C_Master_Transmit_IT(&I2CHandle, 0x50, startup10, 3);
	HAL_Delay(5);
	HAL_I2C_Master_Transmit_IT(&I2CHandle, 0x50, startup9, 3);
	HAL_Delay(5);
	//HAL_I2C_Master_Transmit_IT(&I2CHandle, 0x50, startup5, 2);
	//HAL_Delay(2);
	//HAL_I2C_Master_Transmit_IT(&I2CHandle, 0x50, startup1, 15);
	//HAL_Delay(5);
	//HAL_I2C_Master_Transmit_IT(&I2CHandle, 0x50, startup4, 3);
	//HAL_Delay(5);
	//HAL_I2C_Master_Transmit_IT(&I2CHandle, 0x50, startup2, 15);
//	HAL_Delay(5);
//	HAL_I2C_Master_Transmit_IT(&I2CHandle, 0x50, (uint8_t *)"RPM",3);
}

// ----------------------------------------------------------------------------

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

	if (htim->Instance == TIM3){

		GPIO_InitTypeDef GPIO_BaseStruct;

		GPIO_BaseStruct.Pin = GPIO_PIN_4;
		GPIO_BaseStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_BaseStruct.Pull = GPIO_PULLUP;
		GPIO_BaseStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_BaseStruct.Alternate = GPIO_AF1_TIM2;

		HAL_GPIO_Init(GPIOB, &GPIO_BaseStruct);

		HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
	}
}

// ----------------------------------------------------------------------------

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c){
		__HAL_RCC_I2C1_CLK_ENABLE();

		HAL_NVIC_SetPriority(I2C1_EV_IRQn, 3, 0);
		HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
		HAL_NVIC_SetPriority(I2C1_ER_IRQn, 4, 0);
		HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

// ----------------------------------------------------------------------------

void TIM2_IRQHandler(void){
	HAL_TIM_IRQHandler(&Tim2Handle);
}

// ----------------------------------------------------------------------------

void TIM3_IRQHandler(void){
	HAL_TIM_IRQHandler(&Tim3Handle);
}

// ----------------------------------------------------------------------------

void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&Tim4Handle, TIM_FLAG_UPDATE) != RESET)      //In case other interrupts are also running
    {
        if (__HAL_TIM_GET_ITSTATUS(&Tim4Handle, TIM_IT_UPDATE) != RESET)
        {
        	double dooty = 0.3;
        	if((desired > current) && (index == 0)){
        		current+=2;
        		__HAL_TIM_SET_AUTORELOAD(&Tim4Handle,current);
        	}
        	if((desired < current) && (index == 0)){
        		current -=2;
        		__HAL_TIM_SET_AUTORELOAD(&Tim4Handle,current);
        	}
            __HAL_TIM_CLEAR_FLAG(&Tim4Handle, TIM_FLAG_UPDATE);								//(These pin assignments should be double checked before use
            __HAL_TIM_SET_COMPARE(&Tim1Handle,TIM_CHANNEL_1, uint32_t(dooty * sin_table[index]));				//Phase U+ on PA8
            __HAL_TIM_SET_COMPARE(&Tim1Handle,TIM_CHANNEL_3, uint32_t(dooty * sin_table[(index+15)%30]));	//Phase U- on PA10
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_1, uint32_t(dooty * sin_table[(index+10)%30]));	//Phase V+ on PC6
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_2, uint32_t(dooty * sin_table[(index+25)%30]));	//Phase V- on PC7
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_3, uint32_t(dooty * sin_table[(index+20)%30]));	//Phase W+ on PC8
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_4, uint32_t(dooty * sin_table[(index+5)%30])); 	//Phase W- on PC9

           /* __HAL_TIM_CLEAR_FLAG(&Tim4Handle, TIM_FLAG_UPDATE);								//(These pin assignments should be double checked before use
            __HAL_TIM_SET_COMPARE(&Tim1Handle,TIM_CHANNEL_1, uint32_t(dooty * sin_table2[index]));				//Phase U+ on PA8
            __HAL_TIM_SET_COMPARE(&Tim1Handle,TIM_CHANNEL_3, uint32_t(dooty * sin_table2[(index+45)%90]));	//Phase U- on PA10
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_1, uint32_t(dooty * sin_table2[(index+30)%90]));	//Phase V+ on PC6
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_2, uint32_t(dooty * sin_table2[(index+75)%90]));	//Phase V- on PC7
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_3, uint32_t(dooty * sin_table2[(index+60)%90]));	//Phase W+ on PC8
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_4, uint32_t(dooty * sin_table2[(index+15)%90])); 	//Phase W- on PC9
            */
            /*
            __HAL_TIM_CLEAR_FLAG(&Tim4Handle, TIM_FLAG_UPDATE);								//(These pin assignments should be double checked before use
            __HAL_TIM_SET_COMPARE(&Tim1Handle,TIM_CHANNEL_1, uint32_t(dooty * sin_table2[index]));				//Phase U+ on PA8
            __HAL_TIM_SET_COMPARE(&Tim1Handle,TIM_CHANNEL_3, uint32_t(dooty * sin_table2[(index+15)%30]));	//Phase U- on PA10
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_1, uint32_t(dooty * sin_table2[(index+10)%30]));	//Phase V+ on PC6
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_2, uint32_t(dooty * sin_table2[(index+25)%30]));	//Phase V- on PC7
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_3, uint32_t(dooty * sin_table2[(index+20)%30]));	//Phase W+ on PC8
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_4, uint32_t(dooty * sin_table2[(index+5)%30])); 	//Phase W- on PC9
            */
            index++;
            index = index % 30; //reset index to 0 if we get to 360
        }
    }
}

// ----------------------------------------------------------------------------

void EXTI0_IRQHandler(void){
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

// ----------------------------------------------------------------------------

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	desired /= 2;
}

// ----------------------------------------------------------------------------

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	static uint8_t speedIndex;

	static uint32_t old;
	static uint8_t bufferIndex;

	if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) && (htim->Instance == TIM2)){
		_GET_LOCK_NORETURN(speedLock);

		speed[speedIndex++] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) - HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

		if(speedIndex == SPEEDSAMPLES){
			speedIndex = 0;
		}

		_UNLOCK(speedLock);
	}

	if (htim->Instance == TIM3){
		_GET_LOCK_NORETURN(sensorLock);

		buffer[bufferIndex++] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4) - old;
		old = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

		if(bufferIndex == SENSORSAMPLES){
			bufferIndex = 0;
		}

		_UNLOCK(sensorLock);
	}
}
