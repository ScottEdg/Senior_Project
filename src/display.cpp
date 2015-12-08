/*
 * display.cpp
 *
 *  Created on: Dec 8, 2015
 *      Author: T100
 */


#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include "display.h"

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
I2C_HandleTypeDef *hi2c;
//void Display_Init(I2C_HandleTypeDef *hi2c){
void Display_Init(I2C_HandleTypeDef *hi2c){
	uint8_t clear[2] = {PREFIX, CLEAR_SCREEN};
	uint8_t line1[17] = "DESIRED:---- RPM";
	uint8_t setcursor[3] = {PREFIX, SET_CURSOR, 0x40};
	uint8_t line2[17] = "CURRENT:---- RPM";

	HAL_I2C_Master_Transmit_IT(hi2c, 0x50, clear, 2);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit_IT(hi2c, 0x50, line1, 16);
	HAL_Delay(5);
	HAL_I2C_Master_Transmit_IT(hi2c, 0x50, setcursor, 3);
	HAL_Delay(2);
	HAL_I2C_Master_Transmit_IT(hi2c, 0x50, line2, 16);
	HAL_Delay(5);
}

// ----------------------------------------------------------------------------

void Display(I2C_HandleTypeDef *hi2c, volatile uint32_t sensor[], uint32_t sensorsamples, uint32_t clockspeed, uint32_t desiredrpm, uint32_t currentrpm){
	uint8_t DESIRED[5];
	uint8_t CURRENT[5];
	uint8_t setcursor[3] = {PREFIX, SET_CURSOR, 0x08};
	uint8_t i;
	float average = 0;

	for(i=0 ; i < sensorsamples ; i++){
		average += sensor[i];
	}
	average /= sensorsamples;
	average = (average/clockspeed)*3;
	average = 1.0f / average;
	currentrpm = (int)(average*60);
	average = 0;

	itoa(desiredrpm, (char*)DESIRED, 10);
	itoa(currentrpm, (char*)CURRENT, 10);

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
	HAL_I2C_Master_Transmit_IT(hi2c, 0x50, setcursor, 3);
	HAL_Delay(1);
	HAL_I2C_Master_Transmit_IT(hi2c, 0x50, DESIRED, 4);
	HAL_Delay(1);
	setcursor[2] = 0x48;
	HAL_I2C_Master_Transmit_IT(hi2c, 0x50, setcursor, 3);
	HAL_Delay(1);
	HAL_I2C_Master_Transmit_IT(hi2c, 0x50, CURRENT, 4);
	HAL_Delay(1);
}

