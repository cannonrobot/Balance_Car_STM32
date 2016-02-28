#include "function.h"


int32_t Speed_L=0;
int32_t Speed_R=0;
TIM_HandleTypeDef        TimHandleT1;//������
TIM_HandleTypeDef        TimHandleT2;//���
TIM_HandleTypeDef        TimHandleT3;//���
TIM_HandleTypeDef        TimHandleT4;//���
TIM_HandleTypeDef        TimHandleT5;//������

TIM_Encoder_InitTypeDef  encoderConfig;
TIM_OC_InitTypeDef       pwmConfig;

void Encoder_Init(void){
	GPIO_InitTypeDef   GPIO_InitStruct;
   __HAL_RCC_TIM1_CLK_ENABLE();
   __HAL_RCC_GPIOA_CLK_ENABLE();
   GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 
 
  
   __HAL_RCC_TIM5_CLK_ENABLE();
   __HAL_RCC_GPIOA_CLK_ENABLE();
   GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
	
TimHandleT1.Instance = TIM1;
  TimHandleT1.Init.Period =  0xFFFF;
  TimHandleT1.Init.Prescaler = 0;
  TimHandleT1.Init.ClockDivision = 0;
  TimHandleT1.Init.CounterMode = TIM_COUNTERMODE_UP;  
  
  encoderConfig.EncoderMode =TIM_ENCODERMODE_TI12;
  encoderConfig.IC1Polarity =TIM_ICPOLARITY_RISING;
  encoderConfig.IC1Selection=TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC1Prescaler=0;
  encoderConfig.IC1Filter   =6;
  encoderConfig.IC2Polarity =TIM_ICPOLARITY_RISING;
  encoderConfig.IC2Selection=TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC2Prescaler=0;
  encoderConfig.IC2Filter   =6;
HAL_TIM_Encoder_Init(&TimHandleT1,  &encoderConfig);
HAL_TIM_Encoder_Start(&TimHandleT1,TIM_CHANNEL_1);

  TimHandleT5.Instance = TIM5;
  TimHandleT5.Init.Period =  0xFFFF;
  TimHandleT5.Init.Prescaler = 0;
  TimHandleT5.Init.ClockDivision = 0;
  TimHandleT5.Init.CounterMode = TIM_COUNTERMODE_UP;  
HAL_TIM_Encoder_Init(&TimHandleT5,  &encoderConfig);
HAL_TIM_Encoder_Start(&TimHandleT5,TIM_CHANNEL_1);

}


void Motor_Pwm_Init(void){
   
	//PWM����
  GPIO_InitTypeDef   GPIO_InitStruct;
  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
    
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	
  //TIM4
  TimHandleT4.Instance = TIM4;
  TimHandleT4.Init.Period =  1000 - 1;;
  TimHandleT4.Init.Prescaler = 20-1;
  TimHandleT4.Init.ClockDivision = 0;
  TimHandleT4.Init.CounterMode = TIM_COUNTERMODE_UP;  
  HAL_TIM_PWM_Init(&TimHandleT4);

  pwmConfig.OCMode=TIM_OCMODE_PWM1;
  pwmConfig.Pulse=700;
  HAL_TIM_PWM_ConfigChannel(&TimHandleT4, &pwmConfig, TIM_CHANNEL_3);
   HAL_TIM_PWM_ConfigChannel(&TimHandleT4, &pwmConfig, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&TimHandleT4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&TimHandleT4, TIM_CHANNEL_4);
  
  
  //�����������
	/*
     __HAL_RCC_GPIOB_CLK_ENABLE();
   GPIO_InitTypeDef GPIO_InitStruct;
     GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;

  GPIO_InitStruct.Pin     =GPIO_PIN_13|GPIO_PIN_14;

 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   GPIO_InitStruct.Pin     =GPIO_PIN_2|GPIO_PIN_12;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  */
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
void Get_Speed(void){
  uint32_t TempL,TempR;
    TempL=HAL_TIM_ReadCapturedValue(&TimHandleT1, TIM_CHANNEL_1);//��������ȡ
	Speed_L=TempL;
 /*
      Speed_L=TempL-Pre_Speed_L;
     Pre_Speed_L=TempL;  
     if(Speed_L<-20000){
        Speed_L+=65535;}
        else   if(Speed_L>20000){
        Speed_L-=65535;} 
	*/
    TempR=HAL_TIM_ReadCapturedValue(&TimHandleT5, TIM_CHANNEL_1);//��������ȡ
		Speed_R=TempR;
  /*
       Speed_R=TempR-Pre_Speed_R;
 Pre_Speed_R=TempR;  
       if(Speed_R<-20000){
        Speed_R+=65535;}
          else   if(Speed_R>20000){
        Speed_R-=65535;} 
 
        Speed_A_Last=(Speed_L-Speed_R)/2;
  Speed_A*=0.7;
    Speed_A+=Speed_A_Last*0.3;  
      */
 
}
