/*
 * display.h
 *
 *  Created on: Dec 8, 2015
 *      Author: T100
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

void Display_Init(I2C_HandleTypeDef *hi2c);
void Display(I2C_HandleTypeDef *hi2c, volatile uint32_t sensor[], uint32_t sensorsamples, uint32_t clockspeed, uint32_t desiredrpm, uint32_t currentrpm);
#endif /* DISPLAY_H_ */
