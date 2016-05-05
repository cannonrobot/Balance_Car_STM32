#include "imu_sensor_fusion_9-axis.h"

#include "math.h"
#include "app.h"

#define BETADEF		0.4f		// 2 * proportional gain
extern imu_sensor_data_t sensor_data;
extern imu_sensor_raw_data_t sensor_saw_data;
extern uint16_t MData[3];
 float  Acc_angle,Gry_vivi;
 float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

///////madgwick algorithm
void MadgwickAHRSupdate(float* quat, float deltaT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float q0=quat[0];
	float q1=quat[1];
	float q2=quat[2];
	float q3=quat[3];
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	/*// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}*/

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= BETADEF * s0;
		qDot2 -= BETADEF * s1;
		qDot3 -= BETADEF * s2;
		qDot4 -= BETADEF * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f * deltaT);
	q1 += qDot2 * (1.0f * deltaT);
	q2 += qDot3 * (1.0f * deltaT);
	q3 += qDot4 * (1.0f * deltaT);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	quat[0]=q0;
	quat[1]=q1;
	quat[2]=q2;
	quat[3]=q3;
}

#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain


static float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
static float twoKi = twoKiDef;											// 2 * integral gain (Ki)
static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(float* quat, float deltaT, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float q0=quat[0];
	float q1=quat[1];
	float q2=quat[2];
	float q3=quat[3];
	float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f * deltaT);;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f * deltaT);
			integralFBz += twoKi * halfez * (1.0f * deltaT);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f * deltaT));		// pre-multiply common factors
	gy *= (0.5f * (1.0f * deltaT));
	gz *= (0.5f * (1.0f * deltaT));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	quat[0]=q0;
	quat[1]=q1;
	quat[2]=q2;
	quat[3]=q3;
}
void MahonyAHRSupdateIMU(float* quat, float deltaT, float gx, float gy, float gz, float ax, float ay, float az) {
	float q0=quat[0];
	float q1=quat[1];
	float q2=quat[2];
	float q3=quat[3];
float recipNorm;
float halfvx, halfvy, halfvz;
float halfex, halfey, halfez;
float qa, qb, qc;
 
// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
 
 
// Normalise accelerometer measurement
recipNorm = invSqrt(ax * ax + ay * ay + az * az);
ax *= recipNorm;
ay *= recipNorm;
az *= recipNorm;       
 
// Estimated direction of gravity and vector perpendicular to magnetic flux
halfvx = q1 * q3 - q0 * q2;
halfvy = q0 * q1 + q2 * q3;
halfvz = q0 * q0 - 0.5f + q3 * q3;
 
// Error is sum of cross product between estimated and measured direction of gravity
halfex = (ay * halfvz - az * halfvy);
halfey = (az * halfvx - ax * halfvz);
halfez = (ax * halfvy - ay * halfvx);
 
// Compute and apply integral feedback if enabled
if(twoKi > 0.0f) {
 
integralFBx += twoKi * halfex * (1.0f * deltaT);// integral error scaled by Ki
integralFBy += twoKi * halfey * (1.0f * deltaT);
integralFBz += twoKi * halfez * (1.0f * deltaT);
gx += integralFBx;// apply integral feedback
gy += integralFBy;
gz += integralFBz;
 
}
else {
 
integralFBx = 0.0f;// prevent integral windup
integralFBy = 0.0f;
integralFBz = 0.0f;
 
}
 
// Apply proportional feedback
gx += twoKp * halfex;
gy += twoKp * halfey;
gz += twoKp * halfez;
 
}
 
// Integrate rate of change of quaternion
gx *= (0.5f * (1.0f * deltaT));// pre-multiply common factors
gy *= (0.5f * (1.0f * deltaT));
gz *= (0.5f * (1.0f * deltaT));
qa = q0;
qb = q1;
qc = q2;
q0 += (-qb * gx - qc * gy - q3 * gz);
q1 += (qa * gx + qc * gz - q3 * gy);
q2 += (qa * gy - qb * gz + q3 * gx);
q3 += (qa * gz + qb * gy - qc * gx);
 
// Normalise quaternion
recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	quat[0]=q0;
	quat[1]=q1;
	quat[2]=q2;
	quat[3]=q3;
 
}

//反正切计算倾角值
void count_Acc_angle(void)																									//计算倾角
{
  	if(sensor_data.acc[1]>0)
	{
		Acc_angle = atan2((float) sensor_data.acc[1],(float)sensor_data.acc[2])*(180/3.14159265);		   			//反正切计算

	}
		else
		{
	   	Acc_angle = atan2((float)sensor_data.acc[2],(float)sensor_data.acc[1])*(180/3.14159265)-90;				//反正切计算
			Acc_angle = -Acc_angle;

		}
}


float Angle=0,Gyro_x=0;         //小车滤波后倾斜角度/角速度	
//******卡尔曼参数************
		
float  Q_angle=0.001;  
float  Q_gyro=0.003;
float  R_angle=0.5;
float  dt=0.0024;	                  //dt为kalman滤波器采样时间;
char   C_0 = 1;
float  Q_bias=0, Angle_err=0;
//float  PCt_0=0, PCt_1=0, E=0;
float  K_0=0, K_1=0, t_0=0, t_1=0;
float  Pdot[4] ={0,0,0,0};
float  PP[2][2] = { { 1, 0 },{ 0, 1 } };

//*********************************************************
// 卡尔曼滤波
//*********************************************************



void Kalman_Filter(float Gyro,float Accel)	
{
	
	/*
	
	Angle+=(Gyro - Q_bias) * dt;           //先验估计

	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle;
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	Gyro_x   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
	//calculate_PWM(Gyro_x,Angle);
	*/
}
struct _float E, N;
	struct _vector32 temp_m;
	struct _vector a,m,from;
	float heading;//En esta variable almacenamos el heading cada vez que se actualiza. Si heading es igual a 500 significa error
	
void vector_normalize_E(void){
		float ax;
		ax=sqrt((E.x*E.x) + (E.y*E.y) + (E.z*E.z));
		E.x /= ax;
		E.y /= ax;
		E.z /= ax;
	}
	
	void vector_normalize_N(void){
		float ax;
		ax=sqrt((N.x*N.x) + (N.y*N.y) + (N.z*N.z));
		N.x /= ax;
		N.y /= ax;
		N.z /= ax;
	}
	float vector_dot_E(void){
		return (E.x * from.x) + (E.y * from.y) + (E.z * from.z);
	}
	
	float vector_dot_N(void){
		return (N.x * from.x) + (N.y * from.y) + (N.z * from.z);
	}

	void get_heading(void ){
		static	float Pre_heading=0;;
		a.x=sensor_saw_data.acc[0];
		a.y=sensor_saw_data.acc[1];
		a.z=sensor_saw_data.acc[2];
		
		m.x=MData[0];
		m.y=MData[1];
		m.z=MData[2];
		//Calculamos el heading
		 temp_m.x=m.x;
		 temp_m.y=m.y;
		 temp_m.z=m.z;
		 
		 //producto cruz de E = temp_m X a
		 E.x = (temp_m.y * a.z) - (temp_m.z * a.y);
		 E.y = (temp_m.z * a.x) - (temp_m.x * a.z);
		 E.z = (temp_m.x * a.y) - (temp_m.y * a.x);
		 
		 //Normalizaci髇 de E
		 vector_normalize_E();
		 
		//producto cruz de N = a X E
		 N.x = (a.y * E.z) - (a.z * E.y);
		 N.y = (a.z * E.x) - (a.x * E.z);
		 N.z = (a.x * E.y) - (a.y * E.x);
		 
		 //Normalizaci髇 de N
		 vector_normalize_N();
		 
		 //Estas variables se usan depende la ortientacion de la tarjeta respecto a la horizontal
		 from.x=1;
		 from.y=0;
		 from.z=0;
		 
		 heading = Pre_heading*0.9f+(atan2(vector_dot_E(),vector_dot_N()) * 180.0f) / 3.1416f*0.1f ;
		 
		Pre_heading=heading;
		 
		 
		// *Heading=heading;
	
}
