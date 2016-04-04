
#include "app.h"
#include "cmsis_os.h"
/*start adv*/

#if 0 //NO_PRINTF
#define printf(...)
#endif




static void adv_name_generate(uint8_t* uni_name);


#ifdef CANNON_V2
char name[20] = "CANNON_V2";
#endif
extern float speed_A;//速度和
extern int32_t speed_L;//左边电机速度
extern int32_t speed_R;//右边电机速度
uint32_t adc_value;//ADC值，代表电压
extern imu_sensor_raw_data_t sensor_saw_data;//IMU和磁力计原始值
extern imu_sensor_data_t sensor_data;//校准转换后的值，Offset见MyOffset参数
extern imu_euler_data_t sensor_euler_angle;//欧拉角
extern float r_pitch;//pitch的反转值
extern int16_t motor1_output;//电机1的输出值，-1000~1000
extern int16_t motor2_output;//电机2的输出值，-1000~1000
extern float Speed_Kp,Speed_Ki;	
extern float Angle_Kp,Angle_Kd;	
extern float Turn_Kp;							//转向控制P
extern float Car_Angle_Center;			//平衡点角度
extern int8_t remote_control_X,remote_control_Y;
extern int16_t motor_output_Speed;
extern int16_t motor_output_temp;
extern int16_t motor_output_Angle;

extern float speed_target;
extern float 	turn_target_speed;
extern float 	turn_target_orientaion;
extern int8_t	trun_mode;
	
	extern int fifo_length;
	
	extern TIM_HandleTypeDef        TimHandleT2;//舵机
extern   TIM_HandleTypeDef        TimHandleT3;//舵机
int temp=450;//程序调试使用的全局变量
int temp2=0;
 xQueueHandle  TXQueue;
typedef struct 
 {
	uint8_t type;
	uint8_t length;
	uint8_t value[18];
 }BLEMessage;

/* 
static void osTimerCallback (void const *argument)
{
  BSP_LED_Toggle(LED0);
	
}
*/
static void ToggleLEDsThread(void const *argument)
{
  
  for(;;)
  {
    BSP_LED_Toggle(LED0);
    osDelay(300);
  }
}
static void BLEThread(void const *argument)
{
	for(;;){
	 HCI_Process();
   if(Ble_conn_state) {
   Ble_conn_state = BLE_NOCONNECTABLE;
     }
	}
}

uint16_t	nowTick=0;
	uint16_t preTick=0;
	uint16_t cha;
extern  int my_cnt;
static void mainThread(const void *argument){
		TickType_t xLastWakeTime;
		const TickType_t xFrequency = 5;
    xLastWakeTime = xTaskGetTickCount ();
	
		for( ;; )
		{
		
			vTaskDelayUntil( &xLastWakeTime, xFrequency );
	//		   preTick = HAL_GetTick();
			Get_Adc(&adc_value);	
			
			temp++;
			if(temp>=750)temp=150;
		//	if(temp>=550)temp=0;
			 HAL_TIM_PWM_Pulse(&TimHandleT2,TIM_CHANNEL_3,temp);
			HAL_TIM_PWM_Pulse(&TimHandleT3,TIM_CHANNEL_2,temp);
			HAL_TIM_PWM_Pulse(&TimHandleT3,TIM_CHANNEL_3,temp);
			
	//		nowTick = HAL_GetTick();
	//		cha=nowTick-preTick;				
				
				
//OutData[2] = fifo_length;
//OutData[3] = cha;	
//OutData[0] = sensor_euler_angle.pitch;
OutData[0] =sensor_euler_angle.pitch*100;
		//	OutData[1] =my_cnt;
		OutData[1] =	sensor_saw_data.gyro[0];
		//OutData[3] =	sensor_data.gyro[0]*100;
				OutData[2] =	sensor_saw_data.gyro[1];
						OutData[3] =	sensor_saw_data.gyro[2];
//OutData[0] =motor_output_temp;
//OutData[1] = turn_target_speed;
//OutData[2] =motor_output_Speed;
//OutData[1] = adc_value;
//OutData[2] = speed_R;
//OutData[3] = speed_target;
//OutData[3] = cha;
//	OutPut_Data();		
		//BSP_IMU_6AXES_X_GetAxesRaw(&XData);
	//BSP_IMU_6AXES_G_GetAxesRaw(&GData);
	// LSM303AGR_MAG_Get_Raw_Magnetic((u8_t*)MData);

//OutData[0] = sensor_saw_data.acc[0];
//OutData[1] = sensor_saw_data.acc[1];
//OutData[2] = sensor_saw_data.gyro[0];
//OutData[3] = sensor_saw_data.gyro[0];	

	OutPut_Data();
		}
	
	
}
static void BLEMessageQueueConsumer (const void *argument)
{
  BLEMessage pxMessage;
  for(;;)
  {	
		
   if( TXQueue != 0 )
		{
		if( xQueueReceive( TXQueue,  ( void* )&pxMessage, 10 ) )
			{
				ble_device_send(pxMessage.type,pxMessage.length,pxMessage.value);
				
			}
		}
		osDelay(50);	
  }
}

void on_ready(void)
{
    uint8_t tx_power_level = 7;
    uint16_t adv_interval = 100;
    uint8_t bdAddr[6];
    uint32_t data_rate = 400;

    HCI_get_bdAddr(bdAddr);
    adv_name_generate(bdAddr+4);

    ble_set_adv_param(name, bdAddr, tx_power_level, adv_interval);
    ble_device_start_advertising();

    imu_sensor_select_features(ALL_ENABLE);

    imu_sensor_reset();

    imu_sensor_set_data_rate(&data_rate, LSM6DS3_XG_FIFO_MODE_CONTINUOUS_OVERWRITE);

	  //imu_sensor_filter();
	
    imu_sensor_start();

	
	 //HAL_Delay(100);
		Motor_Pwm_Init();
		Encoder_Init();         
	Steer_Pwm_Init();
	Adc_Init();
//SD_Init();
 
  TXQueue=xQueueCreate( 5 , 20 );
	
 /* Create Timer */
  //osTimerDef(LEDTimer, osTimerCallback);
 // osTimerId osTimer = osTimerCreate (osTimer(LEDTimer), osTimerPeriodic, NULL);
  
  /* Start Timer */
  //osTimerStart(osTimer, 200);		
osThreadDef(BLE, BLEThread, osPriorityIdle, 0, 2*configMINIMAL_STACK_SIZE);//蓝牙接收，最低的优先级
  osThreadCreate(osThread(BLE), NULL);
	
	osThreadDef(QCons, BLEMessageQueueConsumer, osPriorityAboveNormal, 0, 3*configMINIMAL_STACK_SIZE);//蓝牙发送，结合列队
  osThreadCreate(osThread(QCons), NULL);
	
osThreadDef(uLEDThread, ToggleLEDsThread, osPriorityNormal, 0,configMINIMAL_STACK_SIZE);//指示灯，周期的闪烁
 osThreadCreate(osThread(uLEDThread), NULL);
	
	osThreadDef(mainThread, mainThread, osPriorityRealtime, 0,2*configMINIMAL_STACK_SIZE);//主定时，最高优先级
  osThreadCreate(osThread(mainThread), NULL);
	
		 osKernelStart();
}




static void adv_name_generate(uint8_t* uni_name) {
    char temp[3] = "_";
    /*adv name aplice*/
    sprintf(temp+1,"%01d%01d",*uni_name,*(uni_name+1));
    strcat(name, temp);
}
extern  float invSqrt(float x) ;
/* Device On Message */

/*
校验sum
length:需要校验字节的长度
* value：需要校验的数组的指针
return:0成功，1错误
*/


uint8_t Checksum(uint8_t length, uint8_t* value){
	int  check=0;
	for(int i=0;i<length;i++)	check+=*(value+i);
			if(*(value+length)==(uint8_t)check)return 0;
			else return 1;	
}

void ble_device_on_message(uint8_t type, uint16_t length, uint8_t* value)
{
	BLEMessage RXMessage;
	
	
	int16_t p16Data[8] = {0};//发送或接收的temp
	int8_t send_temp[18]={0};//发送数据
	int8_t p8Data[4]={0};//用来接收4个字节的遥控数据
	
	if(type!=1)return;
	
		
		switch (*value){
		case 0x01://遥控ID
			if(Checksum(5,value))return;
			p8Data[0]=*(value+1);//X轴
			p8Data[1]=*(value+2);//Y轴
			p8Data[2]=*(value+3);//模式控制字1
			p8Data[3]=*(value+4);//模式控制字2
			if(p8Data[2]==0){
				speed_target=p8Data[0];
				turn_target_speed=p8Data[1];
				trun_mode=0;
			}
			else if(p8Data[2]==1){
				speed_target=invSqrt(p8Data[0]*p8Data[0]+p8Data[1]*p8Data[1]);
				turn_target_orientaion=atan2((double)p8Data[0],(double)p8Data[1]);
				trun_mode=1;
			}
			
			break;
		case 0x09://修改参数ID
			if(Checksum(17,value))return;
			for (int i = 0; i < 8; i++) {
				p16Data[i] = (((int16_t)*(value +i* 2 + 1)) << 8) + (int16_t)*(value+i*2+2);
			}
			Angle_Kp=					(float)p16Data[0];
			Angle_Kd=					(float)p16Data[1];
			Speed_Kp=					(float)p16Data[2];
			Speed_Ki=					(float)p16Data[3];
			Turn_Kp= 					(float)p16Data[4];
			Car_Angle_Center=	(float)p16Data[5]-20;
			//ble_device_send((uint8_t)0x01,1,(uint8_t*)"Y");
			RXMessage.type=0x01;
			RXMessage.length=2;
			memcpy(RXMessage.value, (uint8_t*)"YY", RXMessage.length);
			if( TXQueue != 0 )
			{
			xQueueSend( TXQueue, ( void* )&RXMessage, 0 );  
			}
			
			break;
		case 0x08://请求原始数据
		
		
			p16Data[0]=Angle_Kp;
			p16Data[1]=Angle_Kd;
			p16Data[2]=Speed_Kp;
			p16Data[3]=Speed_Ki;
			p16Data[4]=Turn_Kp;
			p16Data[5]=Car_Angle_Center+20;
			p16Data[6]=0;
			p16Data[7]=0;
			
			for(int i=0;i<16;i++){
       send_temp[i+1]=(uint8_t)(p16Data[i/2]>>(8-(i%2)*8));
				send_temp[1+16]+=send_temp[i+1];
       }
			send_temp[1+16]+=send_temp[0];
			send_temp[0]=7;//APP接收原始数据的ID
			//ble_device_send((uint8_t)0x01,18,(uint8_t*)send_temp);
			
			 RXMessage.type=(uint8_t)0x01;
			RXMessage.length=(uint8_t)18;
			memcpy(RXMessage.value, (uint8_t*)send_temp, RXMessage.length);
			if( TXQueue != 0 )
			{
			xQueueSend( TXQueue, ( void* )&RXMessage, 0 );  
			}
			
			break;
		}
}
/* Device on connect */
void ble_device_on_connect(void)
{


}
/* Device on disconnect */
void ble_device_on_disconnect(uint8_t reason)
{
    /* Make the device connectable again. */
    Ble_conn_state = BLE_CONNECTABLE;
    ble_device_start_advertising();
}
