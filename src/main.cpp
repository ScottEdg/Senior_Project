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
#include "display.h"

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


extern "C"{
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
	I2C_INIT(&I2CHandle);
	HAL_TIM_IC_Init(&Tim2Handle);
	HAL_TIM_IC_Init(&Tim5Handle);
	HAL_I2C_Init(&I2CHandle);
	USER_Init();

	HAL_Delay(100);

	Display_Init(&I2CHandle);

// ----------------------------------------------------------------------------
	while(1){ //update the display every 250ms
	HAL_Delay(250);
	Display(&I2CHandle, sensor, SENSORSAMPLES, CLOCKSPEED, desiredRPM, currentRPM);
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

void I2C1_EV_IRQHandler(void){
	HAL_I2C_EV_IRQHandler(&I2CHandle);
}

// ----------------------------------------------------------------------------

void I2C1_ER_IRQHandler(void){
	HAL_I2C_ER_IRQHandler(&I2CHandle);
}

// ----------------------------------------------------------------------------

void TIM2_IRQHandler(void){
	HAL_TIM_IRQHandler(&Tim2Handle);
}

// ----------------------------------------------------------------------------

void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&Tim4Handle, TIM_FLAG_UPDATE) != RESET)      //In case other interrupts are also running
    {
        if (__HAL_TIM_GET_ITSTATUS(&Tim4Handle, TIM_IT_UPDATE) != RESET)
        {
        	float duty = 0.35;

            __HAL_TIM_CLEAR_FLAG(&Tim4Handle, TIM_FLAG_UPDATE);								//(These pin assignments should be double checked before use
            __HAL_TIM_SET_COMPARE(&Tim1Handle,TIM_CHANNEL_1, uint32_t(duty * sin_table[index]));				//Phase U+ on PA8
            __HAL_TIM_SET_COMPARE(&Tim1Handle,TIM_CHANNEL_3, uint32_t(duty * sin_table[(index+15)%30]));	//Phase U- on PA10
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_1, uint32_t(duty * sin_table[(index+10)%30]));	//Phase V+ on PC6
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_2, uint32_t(duty * sin_table[(index+25)%30]));	//Phase V- on PC7
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_3, uint32_t(duty * sin_table[(index+20)%30]));	//Phase W+ on PC8
            __HAL_TIM_SET_COMPARE(&Tim8Handle,TIM_CHANNEL_4, uint32_t(duty * sin_table[(index+5)%30])); 	//Phase W- on PC9

            index++;
            index = index % 30; //reset index to 0 if we get to 360
        }
    }
}

// ----------------------------------------------------------------------------

void TIM5_IRQHandler(void){
	HAL_TIM_IRQHandler(&Tim5Handle);
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
