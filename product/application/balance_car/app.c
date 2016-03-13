
#include "app.h"
#include "imu_sensor_fusion.h"
/*start adv*/

#if 0 //NO_PRINTF
#define printf(...)
#endif

static void adv_name_generate(uint8_t* uni_name);
void print_message(void* args);

#ifdef CANNON_V2
char name[20] = "CANNON_V2";
#endif
#ifdef CANNON_V1
char name[20] = "CANNON_V1";
#endif

gravity_filter_context_t gravity_filter_context;

int temp=79;

void on_ready(void)
{
    uint8_t tx_power_level = 5;
    uint16_t adv_interval = 100;
    uint8_t bdAddr[6];
    uint32_t data_rate = 800;

    HCI_get_bdAddr(bdAddr);
    adv_name_generate(bdAddr+4);
    /*Config Adv Parameter And Ready to Adv*/
    ble_set_adv_param(name, bdAddr, tx_power_level, adv_interval);
    ble_device_start_advertising();

    BSP_LED_On(LED0);
    imu_sensor_select_features(ALL_ENABLE);

    imu_sensor_reset();

    imu_sensor_set_data_rate(&data_rate, LSM6DS3_XG_FIFO_MODE_FIFO);

    gravity_filter_init(&gravity_filter_context);

    imu_sensor_start();

  //  run_after_delay(print_message, NULL, 0);
		//Motor_Pwm_Init();
	//	Encoder_Init();         
	//	Steer_Pwm_Init();
		SD_Init();
}


static void adv_name_generate(uint8_t* uni_name) {
    char temp[3] = "_";
    /*adv name aplice*/
    sprintf(temp+1,"%01d%01d",*uni_name,*(uni_name+1));
    strcat(name, temp);
}

/*received data callback*/
void on_imu_sensor_data(imu_sensor_data_t* data)
{
    gravity_filter_run(&gravity_filter_context, data);
}
extern float OutData[4];
extern void 	OutPut_Data(void);
extern UART_HandleTypeDef UartHandle;
extern TIM_HandleTypeDef        TimHandleT3;//¶æ»ú
extern int32_t Speed_R;
void print_message(void* args)
{
    gravity_filter_context_t* cx = &gravity_filter_context;
    uint8_t tmp_buf[6];
    int32_t x,y,z;
    extern float test[3];

    run_after_delay(print_message, NULL, 50);
/*
    x = (int32_t) cx->gravity.x;
    y = (int32_t) cx->gravity.y;
    z = (int32_t) cx->gravity.z;

    tmp_buf[0] = (x >> 8) & 0xFF;
    tmp_buf[1] = (x) & 0xFF;
    tmp_buf[2] = (y >> 8) & 0xFF;
    tmp_buf[3] = (y) & 0xFF;
    tmp_buf[4] = (z >> 8) & 0xFF;
    tmp_buf[5] = (z) & 0xFF;
	OutData[0]=x;
	OutData[1]=y;
	OutData[2]=z;
	OutData[3]=40;
	*/
	Get_Speed();
	OutData[0]=Speed_R;
	OutPut_Data();//use"Visual Scope" to check the waveform
    ble_device_send(0x01, 6, tmp_buf);
		Get_Speed();
		temp++;
		if(temp>100)temp=60;
	//	HAL_TIM_PWM_Pulse(&TimHandleT3,TIM_CHANNEL_3,temp);
	//	HAL_TIM_PWM_Pulse(&TimHandleT3,TIM_CHANNEL_2,temp);
}

/* Device On Message */
void ble_device_on_message(uint8_t type, uint16_t length, uint8_t* value)
{


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
