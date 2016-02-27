#include "function.h"

TIM_HandleTypeDef        TimHandleT1;//电机
TIM_HandleTypeDef        TimHandleT2;//电机
TIM_HandleTypeDef        TimHandleT3;//舵机
TIM_HandleTypeDef        TimHandleT4;//编码器
TIM_HandleTypeDef        TimHandleT5;//编码器

TIM_Encoder_InitTypeDef  encoderConfig;
TIM_OC_InitTypeDef       pwmConfig;

void Encoder_Init(void){
TimHandleT3.Instance = TIM3;
  TimHandleT3.Init.Period =  0xFFFF;
  TimHandleT3.Init.Prescaler = 0;
  TimHandleT3.Init.ClockDivision = 0;
  TimHandleT3.Init.CounterMode = TIM_COUNTERMODE_UP;  
  
  encoderConfig.EncoderMode =TIM_ENCODERMODE_TI1;
  encoderConfig.IC1Polarity =TIM_ICPOLARITY_RISING;
  encoderConfig.IC1Selection=TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC1Prescaler=0;
  encoderConfig.IC1Filter   =6;
  encoderConfig.IC2Polarity =TIM_ICPOLARITY_RISING;
  encoderConfig.IC2Selection=TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC2Prescaler=0;
  encoderConfig.IC2Filter   =6;
HAL_TIM_Encoder_Init(&TimHandleT3,  &encoderConfig);
HAL_TIM_Encoder_Start(&TimHandleT3,TIM_CHANNEL_1);

  TimHandleT4.Instance = TIM4;
  TimHandleT4.Init.Period =  0xFFFF;
  TimHandleT4.Init.Prescaler = 0;
  TimHandleT4.Init.ClockDivision = 0;
  TimHandleT4.Init.CounterMode = TIM_COUNTERMODE_UP;  
HAL_TIM_Encoder_Init(&TimHandleT4,  &encoderConfig);
HAL_TIM_Encoder_Start(&TimHandleT4,TIM_CHANNEL_1);

}
void Motor_Pwm_Init(void){
   
  
  TimHandleT2.Instance = TIM2;
  TimHandleT2.Init.Period =  1000 - 1;;
  TimHandleT2.Init.Prescaler = 20-1;
  TimHandleT2.Init.ClockDivision = 0;
  TimHandleT2.Init.CounterMode = TIM_COUNTERMODE_UP;  
  HAL_TIM_PWM_Init(&TimHandleT2);

  pwmConfig.OCMode=TIM_OCMODE_PWM1;
  pwmConfig.Pulse=700;
  HAL_TIM_PWM_ConfigChannel(&TimHandleT2, &pwmConfig, TIM_CHANNEL_3);
   HAL_TIM_PWM_ConfigChannel(&TimHandleT2, &pwmConfig, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&TimHandleT2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&TimHandleT2, TIM_CHANNEL_1);
  
  
  //电机控制引脚
     __HAL_RCC_GPIOA_CLK_ENABLE();
   GPIO_InitTypeDef GPIO_InitStruct;
     GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;

  GPIO_InitStruct.Pin     =GPIO_PIN_13|GPIO_PIN_14;

 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   GPIO_InitStruct.Pin     =GPIO_PIN_2|GPIO_PIN_12;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  }

void Steer_Pwm_Init(void){
  
    GPIO_InitTypeDef GPIO_InitStruct;
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
     __HAL_RCC_TIM1_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;

     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  TimHandleT1.Instance = TIM1;
  TimHandleT1.Init.Period =  1000 - 1;;
  TimHandleT1.Init.Prescaler = 2000-1;
  TimHandleT1.Init.ClockDivision = 0;
  TimHandleT1.Init.CounterMode = TIM_COUNTERMODE_UP;  
  HAL_TIM_PWM_Init(&TimHandleT1);

  pwmConfig.OCMode=TIM_OCMODE_PWM1;
  pwmConfig.Pulse=79;
  HAL_TIM_PWM_ConfigChannel(&TimHandleT1, &pwmConfig, TIM_CHANNEL_2);
   HAL_TIM_PWM_ConfigChannel(&TimHandleT1, &pwmConfig, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&TimHandleT1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&TimHandleT1, TIM_CHANNEL_2);
  
  

     __HAL_RCC_GPIOA_CLK_ENABLE();
      __HAL_RCC_TIM1_CLK_ENABLE();
  
  
  }
