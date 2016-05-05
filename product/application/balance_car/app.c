
#include "app.h"
#include "cmsis_os.h"
#include "stm32_bluenrg_ble.h"
#include "motion.h"
/*start adv*/

#if 0 //NO_PRINTF
#define printf(...)
#endif

/*
遥控数据数据
0     ID    01
1     X轴
2     Y轴
3     模式控制字1,朝向和普通
4     模式控制字2，姿态和摇杆
5			手机朝向高8字节
6     手机朝向低8字节
7			舵机1
8			舵机2
9			舵机3
10		舵机4
11		舵机5
12		CheckSum
*/
#define REMOTE_CONTROL_ID 					0x01
#define REMOTE_CONTROL_LONG 				12

/*
修改参数
0     ID    09
1~16  参数1~8高8位与低8位
17		CheckSum
*/
#define PARAMETER_MODIFY_ID 				0x09
#define PARAMETER_MODIFY_LONG 			17

/*
获取原始数据
0     ID    08
1			CheckSum
*/
#define PARAMETER_GET_ORIGIN_ID   	0x08
#define PARAMETER_GET_ORIGIN_LONG 	1

/*
发送原始数据
0     ID    07
1~16  参数1~8高8位与低8位
17			CheckSum
*/
#define PARAMETER_PUT_ORIGIN_ID   	0x07
#define PARAMETER_PUT_ORIGIN_LONG 	17

/*
音乐播放模式
0     ID    02
1			模式
2			播放曲目
3			CheckSum
*/
#define MUSIC_PLAYER_MODE_ID     0x02
#define MUSIC_PLAYER_MODE_LONG   3
/*
音量大小
0     ID    03
1			音量大小
2			CheckSum
*/
#define MUSIC_PLAYER_VOLUME_ID     0x03
#define MUSIC_PLAYER_VOLUME_LONG   2
/*
电压大小
0     ID    04
1~2		电压高8位与低8位
3			CheckSum
*/
#define VOLTAGE_ID     0x04
#define VOLTAGE_LONG   3


#define ADC_to_VOLTAGE   0.004462f//ADC转化为电压值

static void adv_name_generate(uint8_t* uni_name);

extern uint8_t Volume;
uint8_t Volume_Pre;
uint8_t Music_Mode;
uint8_t Music_Mode_List_Ptr;

AxesRaw_TypeDef XData,GData;
#ifdef CANNON_V2
char name[20] = "CannonRobot";
#endif
extern float speed_A;//速度和
extern int32_t speed_L;//左边电机速度
extern int32_t speed_R;//右边电机速度
uint32_t adc_value;//ADC值，代表电压
extern imu_sensor_raw_data_t sensor_saw_data;//IMU和磁力计原始值
extern imu_sensor_data_t sensor_data;//校准转换后的值，Offset见MyOffset参数
extern imu_euler_data_t sensor_euler_angle;//欧拉角
extern uint16_t MData[3];
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
extern  int8_t Flag_Fall;
extern float speed_target;
extern float 	turn_target_speed;
extern float 	turn_target_orientaion;
extern int8_t	trun_mode;
extern int16_t motor_output_Turn;	
extern int fifo_length;
	
extern   TIM_HandleTypeDef        TimHandleT3;//舵机
extern   TIM_HandleTypeDef        TimHandleT9;//舵机
extern __IO uint32_t PauseResumeStatus;
extern char USBDISKPath[4];  
extern FATFS USBDISKFatFs;   
extern  MSC_ApplicationTypeDef AppliState;
extern __IO uint32_t RepeatState;
extern __IO uint32_t AudioPlayStart ;
extern 	float heading;
extern IMU_Offset MyOffset;
extern int16_t steer_out[2][5];
extern AUDIO_PLAYBACK_StateTypeDef AudioState;
extern Music_List MusicList;//固定播放列表
uint8_t Dance_Mode_Flag=0;
uint8_t Dance_Mode_Flag_Pre;
uint8_t Read_Temp_Flag=0;
extern int16_t Temperature;
int temp=0;//程序调试使用的全局变量
int temp2=0;
 xQueueHandle  TXQueue;				//用于蓝牙发送的列队
  osMutexId osMutexSPI; 		//用于SPI不可重入函数的互斥量
  osMutexId osMutexIIC; 					//用于IIC不可重入函数的互斥量
  osSemaphoreId osSemaphore_MWMS_EXTI;		//用于中断的信号量
  osSemaphoreId osSemaphore_SPI;		//用于中断的信号量
	
extern	int steer[3];
typedef struct 
 {
	uint8_t type;
	uint8_t length;
	uint8_t value[18];
 }BLEMessage;
 
   static void InitThread(void const *argument)
{    
		uint8_t tx_power_level = 7;
    uint16_t adv_interval = 100;
    uint8_t bdAddr[6];
  
		 /* Initialize the BlueNRG SPI driver */
    BNRG_SPI_Init();
    /* Initialize the BlueNRG HCI */
    HCI_Init();
    /* Reset BlueNRG hardware */
    BlueNRG_RST();
    /*Gatt And Gap Init*/
    ble_init_bluenrg();
    HCI_get_bdAddr(bdAddr);
    adv_name_generate(bdAddr+4);

    ble_set_adv_param(name, bdAddr, tx_power_level, adv_interval);
    ble_device_start_advertising();
		vTaskDelete(NULL);
}
 
 
  static void SPIISRhread(void const *argument)
{  
  for(;;)
  {
		if(xSemaphoreTake( osSemaphore_SPI, portMAX_DELAY )==pdTRUE)
      {
				 if(osMutexWait(osMutexSPI, osWaitForever) == osOK)
				   HCI_Isr();
				 osMutexRelease(osMutexSPI);
      }
  }
}
/*
 static void CarControlhread(void const *argument)
{ 
  for(;;)
  {
		if(xSemaphoreTake( osSemaphore_MWMS_EXTI, portMAX_DELAY )==pdTRUE)
      {
				 if(osMutexWait(osMutexIIC, osWaitForever) == osOK)
				imu_sensor_read_data_from_fifo_DMA();
				   osMutexRelease(osMutexIIC);
      }
  }
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


static void HeartBeatthread(void const *argument)
{
	BLEMessage RXMessage;
	int8_t send_temp[4]={0};//发送数据
	int16_t p16Data[1] = {0};//发送或接收的temp
	for(;;){
		 RXMessage.type=(uint8_t)0x01;
			RXMessage.length=VOLTAGE_LONG+1;
		send_temp[0]=VOLTAGE_ID;
		p16Data[0]=(int16_t)adc_value*ADC_to_VOLTAGE*100.0f;
		temp2=p16Data[0];
			for(int i=0;i<2;i++){
       send_temp[i+1]=(int8_t)(p16Data[i/2]>>(8-(i%2)*8));
       }
			send_temp[3]=send_temp[0]+send_temp[1]+send_temp[2];
			memcpy(RXMessage.value, (uint8_t*)send_temp, RXMessage.length);
			if( TXQueue != 0 &&Ble_conn_state==BLE_NOCONNECTABLE)
			{
			xQueueSend( TXQueue, ( void* )&RXMessage, 0 );  
			}
		 	osDelay(2200);	
	}
}
static void BLEThread(void const *argument)
{
	for(;;){
		 if(osMutexWait(osMutexSPI, osWaitForever) == osOK){
			HCI_Process();
			}
		  osMutexRelease(osMutexSPI);
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
			Get_Adc(&adc_value);	
			nowTick = HAL_GetTick();
			cha=nowTick-preTick;	
			if (cha>100)cha=5;
			getYaw(cha);
		//	get_heading();
			preTick=nowTick; 
			
			if(Dance_Mode_Flag){
				steer_out[1][0]=sensor_euler_angle.yaw/1.8f;
				steer_out[1][1]=speed_A;
				steer_out[1][2]=motor_output_Turn;
				
			}
			else if((Dance_Mode_Flag==0)&&(Dance_Mode_Flag_Pre==1)){
				
				steer_out[1][0]=0;
				steer_out[1][1]=0;
				steer_out[1][2]=0;
			}
			Dance_Mode_Flag_Pre=Dance_Mode_Flag;
			Steer_Control(steer_out);
				
OutData[0] = sensor_saw_data.gyro[0];
OutData[1] = sensor_saw_data.gyro[1];
OutData[2] = sensor_saw_data.gyro[2];	

//OutData[2] =MyOffset.G_X;
//OutData[3] = MyOffset.G_Y;

//OutData[0]=MData[0];
//OutData[1]=MData[1];
//OutData[2]=MData[2];				
//OutData[0]=steer_out[0][0];
//OutData[1]=steer_out[0][1];
//OutData[2]=steer_out[0][2];

//OutData[2] = fifo_length;
//OutData[3] = cha;	
//OutData[0] = sensor_euler_angle.pitch;
//OutData[0] =sensor_euler_angle.pitch*100;
		//	OutData[1] =my_cnt;
	//	OutData[1] =	sensor_saw_data.gyro[0];
		//OutData[3] =	sensor_data.gyro[0]*100;
		//		OutData[2] =	sensor_saw_data.acc[1];
		//	OutData[3] =	sensor_saw_data.acc[0];
//OutData[0] =motor_output_temp;
//OutData[1] = turn_target_speed;
//OutData[2] =motor_output_Speed;
//OutData[1] = adc_value;
//OutData[2] = speed_R;
//OutData[3] = speed_target;
//OutData[3] = cha;
//	OutPut_Data();		
	//	BSP_IMU_6AXES_X_GetAxesRaw(&XData);
//	BSP_IMU_6AXES_G_GetAxesRaw(&GData);
 //LSM303AGR_MAG_Get_Raw_Magnetic((u8_t*)MData);

//OutData[0] = sensor_saw_data.gyro[0];
//OutData[1] = sensor_saw_data.gyro[1];
//OutData[2] = sensor_saw_data.gyro[2];
//OutData[3] = sensor_saw_data.gyro[1];	
//OutData[0]=motor1_output;
//OutData[1]=motor_output_Angle;
OutData[2]=sensor_euler_angle.yaw*100;
OutData[3]=sensor_euler_angle.pitch*100;
//OutData[3]=motor_output_Speed;

//OutData[0]=speed_target;
//OutData[1]=turn_target_speed;
//OutData[2]=sensor_data.mag[0];
//OutData[0]=my_raw;
//OutData[1]=sensor_data.mag[1];
//OutData[2]=adc_value;

//OutData[0]=my_cnt;

//OutData[2]= heading*10;
//OutData[3]= sensor_euler_angle.pitch*10;
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
				 if(osMutexWait(osMutexSPI, osWaitForever) == osOK)
				ble_device_send(pxMessage.type,pxMessage.length,pxMessage.value);				 
				 osMutexRelease(osMutexSPI);
			}
		}
		osDelay(50);	
  }
}

static void MusicPlayThread (const void *argument)
{
  for(;;)
  {	
		AUDIO_PLAYER_Process();
	}
}


static void MusicControlThread (const void *argument)
{
  for(;;)
  {	
		if(Volume!=Volume_Pre){
			BSP_AUDIO_OUT_SetVolume(SOUNDTERMINAL_DEV1,  STA350BW_CHANNEL_MASTER ,Volume);
			Volume_Pre=Volume;
		}
		if(Music_Mode!=0){
			switch (Music_Mode){
			case 1:   //播放按钮
					if(AudioState ==AUDIO_STATE_WAIT){
					
					AudioState = AUDIO_STATE_RESUME;
				}		
					break;
			case 2:
				if(AudioState == AUDIO_STATE_PLAY)
				{
					AudioState = AUDIO_STATE_PAUSE;
				}		
					break;
			case 4:
				WavePlayerStop();
				if(Music_Mode_List_Ptr!=4){
				MusicList.Music_List_All=1;
			  MusicList.Music_List_Ptr=0;
				MusicList.list[MusicList.Music_List_Ptr]=Music_Mode_List_Ptr;//
			  AUDIO_PLAYER_Start(MusicList.list[MusicList.Music_List_Ptr]);
			
				}
				else{
					if(Dance_Mode_Flag==0){
						Dance_Mode_Flag=1;
							MusicList.Music_List_All=1;
							MusicList.Music_List_Ptr=0;
							MusicList.list[MusicList.Music_List_Ptr]=Music_Mode_List_Ptr;//
							AUDIO_PLAYER_Start(MusicList.list[MusicList.Music_List_Ptr]);
					}
					else {
						Dance_Mode_Flag=0;
							AudioState = AUDIO_STATE_STOP;			
					}
				}
			
				break;
			case 5:  //向左按钮
				if(AudioState == AUDIO_STATE_PLAY){
				AudioState = AUDIO_STATE_STOP;//停止
				}
				else{
					//上一首
				}	
					break;
			case 6:  //向右按钮
				//下一首
				break;
			case 7:  //循环模式按钮
				
				break;
			}	
		Music_Mode=0;;
		}
		osDelay(50);	
	}
}
void on_ready(void)
{
    uint32_t data_rate = 400;
	
//HAL_Delay(100);
	/* Initialize STA350BW */
 // Init_AudioOut_Device();
  
  /* Start Audio Streaming*/
 // Start_AudioOut_Device();  
	FATFS_LinkDriver(&SD_Driver, USBDISKPath);
	 /* FatFs initialization fails */
  if(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 1 ) == FR_OK )  {
       AppliState = APPLICATION_START;
			AUDIO_PLAYER_Start(0);//播放欢迎音乐
	}
    imu_sensor_select_features(ALL_ENABLE);
	  jsensor_app_set_sensor(JSENSOR_TYPE_HUMITY_TEMP);

   imu_sensor_reset();

    imu_sensor_set_data_rate(&data_rate, LSM6DS3_XG_FIFO_MODE_CONTINUOUS_OVERWRITE);

	 // imu_sensor_filter();
	
    imu_sensor_start();

		Motor_Pwm_Init();
		Encoder_Init();         
		Steer_Pwm_Init();
		Adc_Init();
	
		osMutexDef(osMutexSPI);//由于SPI是非可重入函数，这里需要使用互斥信号
		osMutexSPI = osMutexCreate(osMutex(osMutexSPI));
	
	  osMutexDef(osMutexIIC);//由于IIC是非可重入函数，这里需要使用互斥信号
		osMutexIIC = osMutexCreate(osMutex(osMutexIIC));
	
		TXQueue=xQueueCreate( 5 , 20 );//蓝牙发送列队，方便统一管理

		osSemaphoreDef(SEM);
		osSemaphore_MWMS_EXTI = osSemaphoreCreate(osSemaphore(SEM) , 1);
		osSemaphoreDef(SEM1);
		osSemaphore_SPI = osSemaphoreCreate(osSemaphore(SEM1) , 1);
	
		osThreadDef(BLE, BLEThread, osPriorityIdle, 0, 2*configMINIMAL_STACK_SIZE);//蓝牙接收，最低的优先级
		osThreadCreate(osThread(BLE), NULL);
	
		osThreadDef(QCons, BLEMessageQueueConsumer, osPriorityAboveNormal, 0, 3*configMINIMAL_STACK_SIZE);//蓝牙发送，结合列队
		osThreadCreate(osThread(QCons), NULL);
	
		if( AppliState == APPLICATION_START){
			osThreadDef(MusicPlayThread, MusicPlayThread, osPriorityIdle, 0, 3*configMINIMAL_STACK_SIZE);//音乐播放器播放线程
			osThreadCreate(osThread(MusicPlayThread), NULL);
	  
			osThreadDef(MusicControlThread, MusicControlThread, osPriorityAboveNormal, 0, 2*configMINIMAL_STACK_SIZE);//音乐播放器控制线程,比播放线程优先级高
			osThreadCreate(osThread(MusicControlThread), NULL);
		}
		else{//这里如果使用了音乐播放那么就不能开LED了，PB3被用作IIC2中的的CLK了
			osThreadDef(uLEDThread, ToggleLEDsThread, osPriorityNormal, 0,configMINIMAL_STACK_SIZE);//指示灯，周期的闪烁
			osThreadCreate(osThread(uLEDThread), NULL);
		}
			osThreadDef(HeartBeatThread, HeartBeatthread, osPriorityLow, 0,configMINIMAL_STACK_SIZE);//心跳任务，低频率的发送相关信息到手机
			osThreadCreate(osThread(HeartBeatThread), NULL);
	
			osThreadDef(mainThread, mainThread, osPriorityHigh, 0,2*configMINIMAL_STACK_SIZE);//主函数定时，高优先级
			osThreadCreate(osThread(mainThread), NULL);
	
			//osThreadDef(CarControlhread, CarControlhread, osPriorityRealtime, 0,3*configMINIMAL_STACK_SIZE);//MEMS中断，最高优先级
			//osThreadCreate(osThread(CarControlhread), NULL);
	
			osThreadDef(SPIISRhread, SPIISRhread, osPriorityRealtime,0,3*configMINIMAL_STACK_SIZE);//SPI中断，最高优先级
			osThreadCreate(osThread(SPIISRhread), NULL);
	
			osThreadDef(InitThread, InitThread, osPriorityHigh, 0,3*configMINIMAL_STACK_SIZE);//BLE初始化任务,要比SPI中断低，要比发送和接收数据任务高，完成任务后删除自己
			osThreadCreate(osThread(InitThread), NULL);
		
		//	osThreadDef(MotionThread, MotionThread, osPriorityAboveNormal, 0,configMINIMAL_STACK_SIZE);//运动控制任务
		//	osThreadCreate(osThread(MotionThread), NULL);
	
			osKernelStart();
}

static void adv_name_generate(uint8_t* uni_name) {
    char temp[3] = "_";
    /*adv name aplice*/
    sprintf(temp+1,"%01d%01d",*uni_name,*(uni_name+1));
    strcat(name, temp);
}
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
	int8_t p8Data[13]={0};//用来接收13个字节的遥控数据
	if(type!=1)return;
	
		switch (*value){
		case REMOTE_CONTROL_ID://遥控ID
			if(Checksum(REMOTE_CONTROL_LONG,value))return;
			p8Data[0]=*(value+1);//X轴
			p8Data[1]=*(value+2);//Y轴
			p8Data[2]=*(value+3);//模式控制字1,朝向和普通
			p8Data[3]=*(value+4);//模式控制字2，姿态和摇杆
		  p16Data[0]=(((int16_t)*(value +5)) << 8) + (int16_t)*(value+6);//手机朝向
			p8Data[7]=*(value+7);//舵机1
		  p8Data[8]=*(value+8);//舵机2
		  p8Data[9]=*(value+9);//舵机3
		  p8Data[10]=*(value+10);//舵机4
		  p8Data[11]=*(value+11);//舵机5
		
			if(p8Data[2]==0&&p8Data[3]==0){
				speed_target=p8Data[0];
				turn_target_speed=p8Data[1];
				trun_mode=0;
			}
			else if(p8Data[2]==1&&p8Data[3]==0){		
				speed_target=-sqrt(p8Data[0]*p8Data[0]+p8Data[1]*p8Data[1]);
				if(p8Data[0]&&p8Data[1])
				turn_target_orientaion=-atan2((double)p8Data[0],(double)p8Data[1])*57.295646f;
				//else
				//	turn_target_orientaion_offset=
				trun_mode=1;
			}
			else if(p8Data[2]==0&&p8Data[3]==1){
				speed_target=p8Data[0];
				turn_target_speed=p16Data[0];
				trun_mode=0;
			}
			else if(p8Data[2]==1&&p8Data[3]==1){
				speed_target=-sqrt(p8Data[0]*p8Data[0]+p8Data[1]*p8Data[1]);
				turn_target_orientaion=-p16Data[0];
				trun_mode=1;
			}
			steer_out[0][0]=(p8Data[7]);
			steer_out[0][1]=-(p8Data[8]);
			steer_out[0][2]=(p8Data[9]);
			Stand_Up( sensor_euler_angle.pitch, Car_Angle_Center, Flag_Fall);
			
			break;
		case PARAMETER_MODIFY_ID://修改参数ID
			if(Checksum(PARAMETER_MODIFY_LONG,value))return;
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
			
		case PARAMETER_GET_ORIGIN_ID://请求原始数据，这里每一个必须和修改参数中一一对应,所有参数必须全部为正整数
			p16Data[0]=Angle_Kp;
			p16Data[1]=Angle_Kd;
			p16Data[2]=Speed_Kp;
			p16Data[3]=Speed_Ki;
			p16Data[4]=Turn_Kp;
			p16Data[5]=Car_Angle_Center+20;
			p16Data[6]=sensor_euler_angle.pitch+20;//这个参数不用来修改，在首次运行可以App查看平衡点处的值然后修改Car_Angle_Center
			p16Data[7]=0;
			for(int i=0;i<16;i++){
       send_temp[i+1]=(int8_t)(p16Data[i/2]>>(8-(i%2)*8));
				send_temp[1+16]+=send_temp[i+1];
       }
			send_temp[1+16]+=send_temp[0];
			send_temp[0]=PARAMETER_PUT_ORIGIN_ID;//APP发送原始数据的ID
			//ble_device_send((uint8_t)0x01,18,(uint8_t*)send_temp);

			 RXMessage.type=(uint8_t)0x01;
			RXMessage.length=(uint8_t)PARAMETER_PUT_ORIGIN_LONG+1;
			memcpy(RXMessage.value, (uint8_t*)send_temp, RXMessage.length);
			if( TXQueue != 0 )
			{
			xQueueSend( TXQueue, ( void* )&RXMessage, 0 );  
			}
			break;
		case MUSIC_PLAYER_MODE_ID:
		if(Checksum(MUSIC_PLAYER_MODE_LONG,value))return;
		Music_Mode=  *(value+1);
		Music_Mode_List_Ptr= *(value+2);
		break;
		case MUSIC_PLAYER_VOLUME_ID:
				if(Checksum(MUSIC_PLAYER_VOLUME_LONG,value))return;
		Volume=*(value+1);
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
