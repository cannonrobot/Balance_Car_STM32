#include "function.h"

FATFS SDFatFs;  /* File system object for SD disk logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD disk logical drive path */ 
extern int16_t steer_out[2][5];
int steer[3];
TIM_HandleTypeDef        TimHandleT1;//编码器
TIM_HandleTypeDef        TimHandleT2;//未用
TIM_HandleTypeDef        TimHandleT3;//舵机
TIM_HandleTypeDef        TimHandleT4;//电机
TIM_HandleTypeDef        TimHandleT5;//编码器
TIM_HandleTypeDef        TimHandleT9;//舵机
	TIM_OC_InitTypeDef       pwmConfig;//PWM控制
	/* ADC handler declaration */
ADC_HandleTypeDef    AdcHandle;

	SD_HandleTypeDef hsd;

extern	void Error_Handler(void);
	
void Adc_Init(){
	 ADC_ChannelConfTypeDef sConfig;
	GPIO_InitTypeDef          GPIO_InitStruct;
  
  /*##-- Enable peripherals and GPIO Clocks #################################*/
  /* ADC3 Periph clock enable */
	__HAL_RCC_ADC1_CLK_ENABLE();
  /* Enable GPIO clock ****************************************/
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /*##-- Configure peripheral GPIO ##########################################*/ 
  /* ADC3 Channel8 GPIO pin configuration */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
 AdcHandle.Instance          = ADC1;
  
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
  AdcHandle.Init.ScanConvMode          = DISABLE;
  AdcHandle.Init.ContinuousConvMode    = DISABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.NbrOfDiscConversion   = 0;
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.NbrOfConversion       = 1;
  AdcHandle.Init.DMAContinuousRequests = DISABLE;
  AdcHandle.Init.EOCSelection          = DISABLE;
      
  if(HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-- Configure ADC regular channel ######################################*/  
  sConfig.Channel      = ADC_CHANNEL_13;
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfig.Offset       = 0;
  
  if(HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
 
}	
	void Get_Adc(uint32_t *Adc){
		static uint32_t pre_Adc;
		 uint32_t now_Adc;
		 /*##-- Start the conversion process #######################################*/  
  if(HAL_ADC_Start(&AdcHandle) != HAL_OK)
  {
    /* Start Conversation Error */
    Error_Handler();
  }
		/*##- Wait for the end of conversion #####################################*/  
  HAL_ADC_PollForConversion(&AdcHandle, 10);
  /* Check if the continuous conversion of regular channel is finished */
  if((HAL_ADC_GetState(&AdcHandle) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG)
  {
    /*##-5- Get the converted value of regular channel  ######################*/
		  now_Adc = HAL_ADC_GetValue(&AdcHandle);
		
 now_Adc=now_Adc*0.1+pre_Adc*0.9;
		pre_Adc=now_Adc;
		*Adc =now_Adc;
  }
	}
	
void SD_Init(void){	

if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0) 
  {
    if(f_mount(&SDFatFs,(TCHAR const*)SDPath, 0) != FR_OK)
    {
      Error_Handler();
    }
    else
    {
        if(f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS  | FA_WRITE) != FR_OK) 
        {
          Error_Handler();
        }
        else
        {
					f_puts("First string in my file\n", &MyFile);
					/*
          res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);
          if((byteswritten == 0) || (res != FR_OK))
          {
            Error_Handler();
          }
          else
          {
						*/
            f_close(&MyFile);
       
       //   }
        }
		}
	}
		 FATFS_UnLinkDriver(SDPath);
}
	
/**
  * @brief  编码器初始化.
  * @param  None
  * @retval None
  */	
void Encoder_Init(void){
	//A8 A9引脚定义
	GPIO_InitTypeDef   GPIO_InitStruct;
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
 //A0 A1引脚定义
  __HAL_RCC_TIM5_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//设置TIM1为编码器读数功能
	TIM_Encoder_InitTypeDef  encoderConfig;
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
//设置TIM5为编码器读数功能
  TimHandleT5.Instance = TIM5;
  TimHandleT5.Init.Period =  0xFFFF;
  TimHandleT5.Init.Prescaler = 0;
  TimHandleT5.Init.ClockDivision = 0;
  TimHandleT5.Init.CounterMode = TIM_COUNTERMODE_UP;  
  HAL_TIM_Encoder_Init(&TimHandleT5,  &encoderConfig);
  HAL_TIM_Encoder_Start(&TimHandleT5,TIM_CHANNEL_1);
}

/**
  * @brief  初始化电机PWM.
  * @param  None
  * @retval None
  */	
void Motor_Pwm_Init(void){
	//PWM引脚
  GPIO_InitTypeDef   GPIO_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct2;
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
  TimHandleT4.Init.Prescaler = 10-1;
  TimHandleT4.Init.ClockDivision = 0;
  TimHandleT4.Init.CounterMode = TIM_COUNTERMODE_UP;  
  HAL_TIM_PWM_Init(&TimHandleT4);

  pwmConfig.OCMode=TIM_OCMODE_PWM1;
  pwmConfig.Pulse=0;
  HAL_TIM_PWM_ConfigChannel(&TimHandleT4, &pwmConfig, TIM_CHANNEL_3);
   HAL_TIM_PWM_ConfigChannel(&TimHandleT4, &pwmConfig, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&TimHandleT4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&TimHandleT4, TIM_CHANNEL_4);
  
  //电机控制引脚/*
     __HAL_RCC_GPIOC_CLK_ENABLE();
   
     GPIO_InitStruct2.Mode    = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct2.Pull      = GPIO_NOPULL;
      GPIO_InitStruct2.Speed     = GPIO_SPEED_FAST;

  GPIO_InitStruct2.Pin     =GPIO_PIN_5|GPIO_PIN_13;

 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct2);

  }

	
	void Motor_Control_1(int16_t Pulse){
  if(Pulse>=0){
		if(Pulse>1000)Pulse=1000;
    GPIOC->BSRR =GPIO_PIN_5;	
    HAL_TIM_PWM_Pulse(&TimHandleT4,TIM_CHANNEL_4,Pulse);
  }
  else{
		if(Pulse<-1000)Pulse=-1000;
        GPIOC->BSRR =GPIO_PIN_5<<16;	
    HAL_TIM_PWM_Pulse(&TimHandleT4,TIM_CHANNEL_4,-Pulse);
  }
}
	
	void Motor_Control_2(int16_t Pulse){
  if(Pulse>=0){
			if(Pulse>1000)Pulse=1000;
    GPIOC->BSRR =GPIO_PIN_13;	
   
    HAL_TIM_PWM_Pulse(&TimHandleT4,TIM_CHANNEL_3,Pulse);
  }
  else{
			if(Pulse<-1000)Pulse=-1000;
        GPIOC->BSRR =GPIO_PIN_13<<16;	
    HAL_TIM_PWM_Pulse(&TimHandleT4,TIM_CHANNEL_3,-Pulse);
  }
}
/**
  * @brief  初始化舵机PWM.
  * @param  None
  * @retval None
  */	
void Steer_Pwm_Init(void){
  //B0引脚定义
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	//C7引脚定义
	 __HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	 GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  //TIM3
	
  TimHandleT3.Instance = TIM3;
  TimHandleT3.Init.Period =  1000 - 1;
  TimHandleT3.Init.Prescaler = 420-1;
  TimHandleT3.Init.ClockDivision = 0;
  TimHandleT3.Init.CounterMode = TIM_COUNTERMODE_UP;  
  HAL_TIM_PWM_Init(&TimHandleT3);

  pwmConfig.OCMode=TIM_OCMODE_PWM1;
  pwmConfig.Pulse=0;
  HAL_TIM_PWM_ConfigChannel(&TimHandleT3, &pwmConfig, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&TimHandleT3, &pwmConfig, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&TimHandleT3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&TimHandleT3, TIM_CHANNEL_2);
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	 __HAL_RCC_TIM9_CLK_ENABLE();
	  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	 TimHandleT9.Instance = TIM9;
  TimHandleT9.Init.Period =  1000 - 1;
  TimHandleT9.Init.Prescaler = 420-1;
  TimHandleT9.Init.ClockDivision = 0;
  TimHandleT9.Init.CounterMode = TIM_COUNTERMODE_UP;  
  HAL_TIM_PWM_Init(&TimHandleT9);

  pwmConfig.OCMode=TIM_OCMODE_PWM1;
  pwmConfig.Pulse=0;
  HAL_TIM_PWM_ConfigChannel(&TimHandleT9, &pwmConfig, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&TimHandleT9, TIM_CHANNEL_2);
	
	/*
	// __HAL_RCC_GPIOB_CLK_ENABLE();
	 __HAL_RCC_TIM2_CLK_ENABLE();
	  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	 TimHandleT2.Instance = TIM2;
  TimHandleT2.Init.Period =  1000 - 1;
  TimHandleT2.Init.Prescaler = 840-1;
  TimHandleT2.Init.ClockDivision = 0;
  TimHandleT2.Init.CounterMode = TIM_COUNTERMODE_UP;  
  HAL_TIM_PWM_Init(&TimHandleT2);

  pwmConfig.OCMode=TIM_OCMODE_PWM1;
  pwmConfig.Pulse=450;
  HAL_TIM_PWM_ConfigChannel(&TimHandleT2, &pwmConfig, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&TimHandleT2, TIM_CHANNEL_3);
	*/
  }

/**
  * @brief  获得两次时间间隔中的编码器读数差.
  * @param  None
  * @retval None
  */	
void Get_Speed(int32_t *speedL,int32_t *speedR,float *speedA){
static	int32_t Pre_Speed_L;
static int32_t Pre_Speed_R;
static	float Pre_Speed_A;
  uint32_t TempL,TempR;
	int32_t Speed_L,Speed_R,Speed_A;
    TempL=HAL_TIM_ReadCapturedValue(&TimHandleT1, TIM_CHANNEL_1);//编码器读取
  //  Speed_L=TempL;
	
	Speed_L=TempL-Pre_Speed_L;
    Pre_Speed_L=TempL;  
    if(Speed_L<-20000){
       Speed_L+=65535;}
    else   if(Speed_L>20000){
    Speed_L-=65535;} 
	
    TempR=HAL_TIM_ReadCapturedValue(&TimHandleT5, TIM_CHANNEL_1);//编码器读取
		
	//	Speed_R=TempR;
		
    Speed_R=TempR-Pre_Speed_R;
    Pre_Speed_R=TempR;  
    if(Speed_R<-20000){
       Speed_R+=65535;}
    else   if(Speed_R>20000){
    Speed_R-=65535;} 
		//一阶低通滤波
		
		Speed_A=(Speed_R-Speed_L)/2.0f*0.7f+Pre_Speed_A*0.3f;
		Pre_Speed_A=Speed_A;
		
		
		*speedL=-Speed_L;
		*speedR=Speed_R;
		*speedA=Speed_A;
		
}
/**
  * @brief  set PWM duty cycle.
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @param  Channel: TIM Channels to be enabled.
  *          This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_TIM_PWM_Pulse(TIM_HandleTypeDef *htim,  uint32_t Channel,uint32_t Pulse){
  switch (Channel)
  {
    case TIM_CHANNEL_1:
    {
   htim->Instance->CCR1 =Pulse;
    }break;
    case TIM_CHANNEL_2:
    {
       htim->Instance->CCR2 =Pulse;
    }break;
    case TIM_CHANNEL_3:
    {
       htim->Instance->CCR3 =Pulse;
    }break;
    case TIM_CHANNEL_4:
    {
       htim->Instance->CCR4 =Pulse;
    }break;
    default:
    break;  
  }
   return HAL_OK;
}


extern float Encoder_Integral;
uint8_t Fall_Detect(float Angle,float Target){
	static uint8_t Falled_Flag=0;
	float E_Angle;
	E_Angle=Angle-Target;
	if(Falled_Flag==0){
		if(E_Angle>50||E_Angle<-50)
		Falled_Flag=1;
	}
	else{
		if(E_Angle>-5&&E_Angle<5){
		Falled_Flag=0;
		Encoder_Integral=0;
		}
	}
		return Falled_Flag;
}
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
函数功能：检测小车是否被拿起

**************************************************************************/
int Pick_Up_Detect(float Angle,float Car_Angle_Center,int encoder_left,int encoder_right)
{ 	static uint16_t Pick_Up_flag=0;	   
	 static uint16_t flag,count,count0,count1,count2;
	
	if(Pick_Up_flag==0){
		if(flag==0)                                                                   //第一步
		{
	      if(myabs(encoder_left)+myabs(encoder_right)<30)                         //条件1，小车接近静止
				count0++;
        else 
        count0=0;		
        if(count0>10)				
		    flag=1,count0=0; 
		} 
		if(flag==1)                                                                  //进入第二步
		{
		    if(++count1>200)       count1=0,flag=0;                                 //超时不再等待2000ms
	      if((Angle>(-20+Car_Angle_Center))&&(Angle<(20+Car_Angle_Center)))   //条件2，小车是在0度附近被拿起
		    flag=2; 
		} 
		if(flag==2)                                                                  //第三步
		{
		  if(++count2>100)       count2=0,flag=0;                                   //超时不再等待1000ms
	    if(myabs(encoder_left+encoder_right)>200)                                 //条件3，小车的轮胎因为正反馈达到最大的转速   
      {
				flag=0;                                                                                     
				Pick_Up_flag= 1;       
         return   Pick_Up_flag;                                             //检测到小车被拿起
			}
		}
	}
	else{
		 if(flag==0)                                               
	 {
	      if(Angle>(-10+Car_Angle_Center)&&Angle<(10+Car_Angle_Center)&&encoder_left==0&&encoder_right==0)         //条件1，小车是在0度附近的
		    flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                                          //超时不再等待 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left<-3&&encoder_right<-3&&encoder_left>-30&&encoder_right>-30)                //条件2，小车的轮胎在未上电的时候被人为转动  
      {
				flag=0;
				flag=0;
				Pick_Up_flag= 0;       
				Encoder_Integral=0;
				return Pick_Up_flag;                                             //检测到小车被放下
			}
	 }
	}
	return Pick_Up_flag;  
}



void Steer_Control(int16_t steer_out[2][5]){//舵机控制200HZ，30%是中间值，50%和10%是最小值
	
	static int pre_steer[3];
		for(int i=0;i<2;i++){
			for(int j=0;j<3;j++){
			if(steer_out[i][j]!=0)
				steer[j]=steer_out[i][j];
			}
		}
		if(pre_steer[0]!=steer[0]){
			if(steer[0]>100)steer[0]=100;
			else if(steer[0]<-100)steer[0]=-100;
				HAL_TIM_PWM_Pulse(&TimHandleT9,TIM_CHANNEL_2,(steer[0]*2+300));
		pre_steer[0]=steer[0];}
		
		if(pre_steer[1]!=steer[1]){
						if(steer[1]>100)steer[1]=100;
			else if(steer[1]<-100)steer[1]=-100;
				HAL_TIM_PWM_Pulse(&TimHandleT3,TIM_CHANNEL_2,(steer[1]*2+300));
			pre_steer[1]=steer[1];}
		
		if(pre_steer[2]!=steer[2]){
				if(steer[2]>100)steer[2]=100;
			else if(steer[2]<-100)steer[2]=-100;
				HAL_TIM_PWM_Pulse(&TimHandleT3,TIM_CHANNEL_3,(steer[2]*2+300));	
			pre_steer[2]=steer[2];}
}

void Stand_Up(float Angle,float Car_Angle_Center,int8_t Flag_Fall){
	static uint16_t Fall_Cnt_Step1=0;
	//static uint16_t	Fall_Cnt_Step2=0;
	static int8_t Pre_Flag_Fall;
	if((Flag_Fall==1)&&(Fall_Cnt_Step1<15)){
		
		if(Angle-Car_Angle_Center>0){
					steer_out[1][1]=-100;
					steer_out[1][2]=100;
				}
				else {
					steer_out[1][1]=100;
					steer_out[1][2]=-100;
				}
				
		Fall_Cnt_Step1++;
	}
	//else if((Flag_Fall==1)&&(Fall_Cnt_Step2<15)){
	//	steer_out[1][1]=-8;//不能设置为0
	//	steer_out[1][2]=1;
	//		Fall_Cnt_Step2++;
	//}
	else if(Flag_Fall==1){
		
		Fall_Cnt_Step1=0;
	//	Fall_Cnt_Step2=0;
	}
	else if(Flag_Fall==0&&Pre_Flag_Fall==1){
		
		steer_out[1][1]=0;
		steer_out[1][2]=0;
		Fall_Cnt_Step1=0;
	//	Fall_Cnt_Step2=0;
	}
	Pre_Flag_Fall=Flag_Fall;
}
