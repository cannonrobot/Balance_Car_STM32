#ifndef _function_H
#define _function_H
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"


void Encoder_Init(void);
void Get_Speed(void);
void Steer_Pwm_Init(void);
void Motor_Pwm_Init(void);
HAL_StatusTypeDef HAL_TIM_PWM_Pulse(TIM_HandleTypeDef *htim,  uint32_t Channel,uint32_t Pulse);
#endif
