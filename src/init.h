/*
 * init.h
 *
 *  Created on: Dec 7, 2015
 *      Author: Scott Edgerly and Rigel Paradise
 */

#ifndef INIT_H_
#define INIT_H_

void CLK_Init(void);
void PWM_Init(TIM_HandleTypeDef * htim1, TIM_HandleTypeDef * htim2);
void TIM2_Init(TIM_HandleTypeDef * htim, TIM_IC_InitTypeDef * sConfig);
void TIM4_Init(TIM_HandleTypeDef * htim, uint32_t current);
void TIM5_Init(TIM_HandleTypeDef * htim, TIM_IC_InitTypeDef * sConfig);
#endif /* INIT_H_ */
