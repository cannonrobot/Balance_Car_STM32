#ifndef _IMU_SENSOR_FUSION_9_AXIS_H_
#define _IMU_SENSOR_FUSION_9_AXIS_H_
#include "cube_hal.h"
#include "imu_sensor.h"


typedef struct _sensor_fusion_angle_t{
 
   float pitch;
   
   float roll;

   float yaw;
 
} sensor_fusion_angle_t;

typedef struct _imu_sensor_fusion_1_context_t{
 
   float k_acc_1;
 
   float k_acc_2;
  
   float k_gyr_1;
 
   float k_gyr_2;
  
   float k_mag_1;
 
   float k_mag_2;
 
   float k_offset;
 
   float gyro_offset_x;
 
   float gyro_offset_y;
  
   float gyro_offset_z;

} imu_sensor_fusion_1_context_t;

struct _vector{
		 int16_t x;
		int16_t y;
		int16_t z;
	};
	
	struct _vector32{
		int32_t x;	
		int32_t y;
		int32_t z;
	};
	
	struct _float{
		float x;
		float y;
		float z;
	};
	
	
	void vector_normalize_E(void);
	void vector_normalize_N(void);
	float vector_dot_E(void);
	float vector_dot_N(void);
	void get_heading(void);//Actualiza el heading

void complementary_filter(float acc_raw[3], float gyr_raw[3], float mag_raw[3], float *pitch, float *roll, float *yaw);


void MadgwickAHRSupdate(float* quat, float deltaT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdate(float* quat, float deltaT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Kalman_Filter(float Gyro,float Accel);
void MahonyAHRSupdateIMU(float* quat, float deltaT, float gx, float gy, float gz, float ax, float ay, float az);
void count_Acc_angle(void);
#endif /*_IMU_SENSOR_FUSION_H_*/

