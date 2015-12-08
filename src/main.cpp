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
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_tim_ex.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_cortex.h"
#include "sin_table.h"
#include "init.h"

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

void I2C_INIT(void);
void USER_Init(void);
void Display_Init(void);
void Display(void);

extern "C"{
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void TIM2_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM5_IRQHandler(void);
void EXTI0_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
};

#define SPEEDSAMPLES 3
#define SENSORSAMPLES 12
#define MAXSPEED 2000
#define CLOCKSPEED 84000000

//list of LCD Commands
#define PREFIX 0xFE
#define DISPLAY_ON 0x41
#define DISPLAY_OFF 0x42
#define SET_CURSOR 0x45
#define CURSOR_HOME 0x46
#define UNDERLINE_CURSOR_ON 0x47
#define UNDERLINE_CURSOR_OFF 0x48
#define MOVE_CURSOR_LEFT 0x49
#define MOVE_CURSOR_RIGHT 0x4A
#define BLINKING_CURSOR_ON 0x4B
#define BLINKING_CURSOR_OFF 0x4C
#define BACKSPASE 0x4E
#define CLEAR_SCREEN 0x51
#define SET_CONTRAST 0x52
#define MOVE_DISPLAY_LEFT 0x55
#define MOVE_DISPLAY_RIGHT 0x56
#define CHANGE_I2C_ADDR 0x62
#define DISPLAY_FW_VER 0x70
#define DISPLAY_I2C_ADDR 0x72

static volatile uint32_t sensor[SENSORSAMPLES];
static volatile uint32_t speed[SPEEDSAMPLES];

static TIM_HandleTypeDef Tim1Handle; //used for Phase U
static TIM_HandleTypeDef Tim2Handle;
static TIM_HandleTypeDef Tim4Handle; //Used to "count" through the sine LUT
static TIM_HandleTypeDef Tim5Handle;
static TIM_HandleTypeDef Tim8Handle; //Used for Phase V and W
static I2C_HandleTypeDef I2CHandle;
static TIM_IC_InitTypeDef SysConf;

static int index = 0;
uint32_t desired = 1203;
uint32_t current = desired;
uint32_t desiredRPM = 100;
uint32_t currentRPM = 0;

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
	CLK_Init();
	PWM_Init(&Tim1Handle, &Tim8Handle); //initalizes two timers and sets up PWM to drive motor
	TIM2_Init(&Tim2Handle, &SysConf);
	TIM4_Init(&Tim4Handle, current);
	TIM5_Init(&Tim5Handle, &SysConf);
	I2C_INIT();
	HAL_TIM_IC_Init(&Tim2Handle);
	HAL_TIM_IC_Init(&Tim5Handle);
	HAL_I2C_Init(&I2CHandle);
	USER_Init();

	HAL_Delay(100);

	Display_Init();

// ----------------------------------------------------------------------------
	while(1){
	HAL_Delay(250);
	Display();
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

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

// ----------------------------------------------------------------------------

void I2C1_EV_IRQHandler(void){
	HAL_I2C_EV_IRQHandler(&I2CHandle);
}

// ----------------------------------------------------------------------------

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
	uint8_t clear[2] = {PREFIX, CLEAR_SCREEN};
	uint8_t line1[17] = "DESIRED:---- RPM";
	uint8_t setcursor[3] = {PREFIX, SET_CURSOR, 0x40};
	uint8_t line2[17] = "CURRENT:---- RPM";

	HAL_I2C_Master_Transmit_IT(&I2CHandle, 0x50, clear, 2);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit_IT(&I2CHandle, 0x50, line1, 16);
	HAL_Delay(5);
	HAL_I2C_Master_Transmit_IT(&I2CHandle, 0x50, setcursor, 3);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit_IT(&I2CHandle, 0x50, line2, 16);
	HAL_Delay(5);
}

// ----------------------------------------------------------------------------

void Display(void){
	uint8_t DESIRED[5];
	uint8_t CURRENT[5];
	uint8_t setcursor[3] = {PREFIX, SET_CURSOR, 0x08};
	uint8_t i;
	float average = 0;

	for(i=0 ; i < SENSORSAMPLES ; i++){
		average += sensor[i];
	}
	average /= SENSORSAMPLES;
	average = (average/CLOCKSPEED)*3;
	average = 1.0f / average;
	currentRPM = (int)(average*60);
	average = 0;

	itoa(desiredRPM, (char*)DESIRED, 10);
	itoa(currentRPM, (char*)CURRENT, 10);

	for(i=0 ; i<5 ; i++){
		if(DESIRED[i]== NULL){
			while(i<5)
				DESIRED[i++] = ' ';
		}
	}
	for(i=0 ; i<5 ; i++){
		if(CURRENT[i]== NULL){
			while(i<5){
				CURRENT[i++] = ' ';
			}
		}
	}
	HAL_I2C_Master_Transmit_IT(&I2CHandle, 0x50, setcursor, 3);
	HAL_Delay(1);
	HAL_I2C_Master_Transmit_IT(&I2CHandle, 0x50, DESIRED, 4);
	HAL_Delay(1);
	setcursor[2] = 0x48;
	HAL_I2C_Master_Transmit_IT(&I2CHandle, 0x50, setcursor, 3);
	HAL_Delay(1);
	HAL_I2C_Master_Transmit_IT(&I2CHandle, 0x50, CURRENT, 4);
	HAL_Delay(1);
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

void TIM5_IRQHandler(void){
	HAL_TIM_IRQHandler(&Tim5Handle);
}

// ----------------------------------------------------------------------------

void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&Tim4Handle, TIM_FLAG_UPDATE) != RESET)      //In case other interrupts are also running
    {
        if (__HAL_TIM_GET_ITSTATUS(&Tim4Handle, TIM_IT_UPDATE) != RESET)
        {
        	float dooty = 0.35;

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
	static uint32_t temp1;
	static uint32_t temp2;
	static uint32_t temp3;
	static uint8_t flag = 0;
	static uint8_t flag2 = 0;
	float average = 0;
	static int i;

	static uint32_t old;
	static uint8_t sensorIndex;

	if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) && (htim->Instance == TIM2)){
		_GET_LOCK_NORETURN(speedLock);

		temp1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		temp2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

		if (temp2 >= temp1) {
			speed[speedIndex++] = temp2 - temp1;
		} else {
			speed[speedIndex++] = 0xFFFFFFFF - temp1 + temp2;
		}

		if(sensorIndex == SENSORSAMPLES){
			sensorIndex = 0;
		}

		if(speedIndex == SPEEDSAMPLES){
			speedIndex = 0;
			flag = 1;
		}

		if(flag == 1){
			for(i=0 ; i < SPEEDSAMPLES ; i++){
				average += speed[i];
			}
			average /= SPEEDSAMPLES;
			average = (average/CLOCKSPEED)*1000;
			average -= 1;
			if(average < 0.023) average = 0.023;
			if(average > .5) average = .5;
			desiredRPM = average * 4200;
			average *= 29400;
			desired = 800000/average;
        	if((desired > current) && (current < 1203)){
        		if((desired - current) > 100) current+=20;
        		else current+=2;
        		__HAL_TIM_SET_AUTORELOAD(&Tim4Handle,current);
        	}
        	if((desired < current) && current > 26){
        		if((current - desired) > 100) current -=20;
        		else current-=2;
        		__HAL_TIM_SET_AUTORELOAD(&Tim4Handle,current);
        	}
			average = 0;
			flag = 0;
		}

		_UNLOCK(speedLock);
	}

	if (htim->Instance == TIM5){
		_GET_LOCK_NORETURN(sensorLock);
		if (flag2!=0){
			temp3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			if (temp3 >= old) {
				sensor[sensorIndex++] = temp3 - old;
			} else {
				sensor[sensorIndex++] = 0xFFFFFFFF - old + temp3;
			}

			old = temp3;

			if(sensorIndex == SENSORSAMPLES){
				sensorIndex = 0;
			}
		}
		else{
			old = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			flag2 = 1;
		}
		_UNLOCK(sensorLock);
	}
}
