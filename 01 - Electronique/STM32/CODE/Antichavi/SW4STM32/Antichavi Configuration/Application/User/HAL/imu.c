/*
 * imu.c
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#include "imu.h"
#include "stm32f4xx_hal.h"
#include "delay.h"
#include "math.h"
#include "MadgwickAHRS.h"

/// ADD to linker : -specs=rdimon.specs
/// ADD to linker : -specs=rdimon.specs
/// ADD to linker : -specs=rdimon.specs

extern I2C_HandleTypeDef hi2c1;

/*****************************************************************************/

#define M_PI 3.141592f
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define ToRad(x) ((x) * 0.01745329252)  // *pi/180
#define ToDeg(x) ((x) * 57.2957795131)  // *180/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f

float fastAtan2( float y, float x)
{
  static float atan;
  static float z;
  if ( x == 0.0f )
  {
    if ( y > 0.0f ) return PIBY2_FLOAT;
    if ( y == 0.0f ) return 0.0f;
    return -PIBY2_FLOAT;
  }
  z = y / x;
  if ( fabs( z ) < 1.0f )
  {
    atan = z/(1.0f + 0.28f*z*z);
    if ( x < 0.0f )
    {
      if ( y < 0.0f ) return atan - PI_FLOAT;
      return atan + PI_FLOAT;
    }
  }
  else
  {
    atan = PIBY2_FLOAT - z/(z*z + 0.28f);
    if ( y < 0.0f ) return atan - PI_FLOAT;
  }
  return atan;
}

/*****************************************************************************/
float pitch,roll,yaw;

void GetEuler(void){
  roll = ToDeg(fastAtan2(2 * (q0 * q1 + q2 * q3),1 - 2 * (q1 * q1 + q2 * q2)));
  pitch = ToDeg(asin(2 * (q0 * q2 - q3 * q1)));
  yaw = ToDeg(fastAtan2(2 * (q0 * q3 + q1 * q2) , 1 - 2* (q2 * q2 + q3 * q3)));
  if (yaw < 0){
    yaw +=360;
  }
}

float HAL_Imu_Pitch()
{
	return pitch;
}

float HAL_Imu_Roll()
{
	return roll;
}

float HAL_Imu_Yaw()
{
	return yaw;
}

float HAL_Imu_Angle(int angle)
{
	switch(angle)
	{
	case IMU_ANGLE_PITCH:
		return pitch;
	case IMU_ANGLE_ROLL:
		return roll;
	case IMU_ANGLE_YAW:
		return yaw;

	}
	return 0.0;
}

/*****************************************************************************/

enum IMU_ADDR
{
	IMU_L3GD20H_ADDR = 0xD6, //6B<<1, // Gyro
	IMU_LSM303D_ADDR = 0x3A // 1D<<1, // Acc
};

enum IMU_WHO_AM_I
{
	IMU_L3GD20H_WHO_AM_I = 0xD7, // Gyro
	IMU_LSM303D_WHO_AM_I = 0x49 // Acc
};

enum IMU_L3GD20H_REGISTER
{
	IMU_L3GD20H_REGISTER_WHO_AM_I 		= 0x0F,
	IMU_L3GD20H_REGISTER_CTRL1          = 0x20, // D20H
	IMU_L3GD20H_REGISTER_CTRL2          = 0x21, // D20H
	IMU_L3GD20H_REGISTER_CTRL3          = 0x22, // D20H
	IMU_L3GD20H_REGISTER_CTRL4          = 0x23, // D20H
	IMU_L3GD20H_REGISTER_CTRL5          = 0x24, // D20H
	IMU_L3GD20H_REGISTER_REFERENCE      = 0x25,
	IMU_L3GD20H_REGISTER_OUT_TEMP       = 0x26,
	IMU_L3GD20H_REGISTER_STATUS         = 0x27, // D20H
	IMU_L3GD20H_REGISTER_OUT_X_L        = 0x28,
	IMU_L3GD20H_REGISTER_OUT_X_H        = 0x29,
	IMU_L3GD20H_REGISTER_OUT_Y_L        = 0x2A,
	IMU_L3GD20H_REGISTER_OUT_Y_H        = 0x2B,
	IMU_L3GD20H_REGISTER_OUT_Z_L        = 0x2C,
	IMU_L3GD20H_REGISTER_OUT_Z_H        = 0x2D,
	IMU_L3GD20H_REGISTER_LOW_ODR        = 0x39  // D20H
};

enum IMU_LSM303D_REGISTER
{
	IMU_LSM303D_REGISTER_WHO_AM_I 			= 0x0F,
	IMU_LSM303D_REGISTER_TEMP_OUT_L        	= 0x05, // D
	IMU_LSM303D_REGISTER_TEMP_OUT_H        	= 0x06, // D
	IMU_LSM303D_REGISTER_STATUS_M          	= 0x07, // D
	IMU_LSM303D_REGISTER_OUT_X_L_M      	= 0x08, // D
	IMU_LSM303D_REGISTER_OUT_X_H_M      	= 0x09, // D
	IMU_LSM303D_REGISTER_OUT_Y_L_M      	= 0x0A, // D
	IMU_LSM303D_REGISTER_OUT_Y_H_M      	= 0x0B, // D
	IMU_LSM303D_REGISTER_OUT_Z_L_M      	= 0x0C, // D
	IMU_LSM303D_REGISTER_OUT_Z_H_M      	= 0x0D, // D
	IMU_LSM303D_REGISTER_INT_CTRL_M        	= 0x12, // D
	IMU_LSM303D_REGISTER_INT_SRC_M         	= 0x13, // D
	IMU_LSM303D_REGISTER_INT_THS_L_M       	= 0x14, // D
	IMU_LSM303D_REGISTER_INT_THS_H_M       	= 0x15, // D
	IMU_LSM303D_REGISTER_OFFSET_X_L_M      	= 0x16, // D
	IMU_LSM303D_REGISTER_OFFSET_X_H_M      	= 0x17, // D
	IMU_LSM303D_REGISTER_OFFSET_Y_L_M      	= 0x18, // D
	IMU_LSM303D_REGISTER_OFFSET_Y_H_M      	= 0x19, // D
	IMU_LSM303D_REGISTER_OFFSET_Z_L_M      	= 0x1A, // D
	IMU_LSM303D_REGISTER_OFFSET_Z_H_M      	= 0x1B, // D
	IMU_LSM303D_REGISTER_REFERENCE_X       	= 0x1C, // D
	IMU_LSM303D_REGISTER_REFERENCE_Y       	= 0x1D, // D
	IMU_LSM303D_REGISTER_REFERENCE_Z       	= 0x1E, // D
	IMU_LSM303D_REGISTER_CTRL0             	= 0x1F, // D
	IMU_LSM303D_REGISTER_CTRL1             	= 0x20, // D
	IMU_LSM303D_REGISTER_CTRL2             	= 0x21, // D
	IMU_LSM303D_REGISTER_CTRL3             	= 0x22, // D
	IMU_LSM303D_REGISTER_CTRL4             	= 0x23, // D
	IMU_LSM303D_REGISTER_CTRL5             	= 0x24, // D
	IMU_LSM303D_REGISTER_CTRL6             	= 0x25, // D
	IMU_LSM303D_REGISTER_CTRL7             	= 0x26, // D
	IMU_LSM303D_REGISTER_STATUS_A          	= 0x27, // D
	IMU_LSM303D_REGISTER_OUT_X_L_A         	= 0x28,
	IMU_LSM303D_REGISTER_OUT_X_H_A         	= 0x29,
	IMU_LSM303D_REGISTER_OUT_Y_L_A         	= 0x2A,
	IMU_LSM303D_REGISTER_OUT_Y_H_A         	= 0x2B,
	IMU_LSM303D_REGISTER_OUT_Z_L_A         	= 0x2C,
	IMU_LSM303D_REGISTER_OUT_Z_H_A         	= 0x2D
};

struct t_imu_config_element
{
	int addr;
	int reg;
	uint8_t value;
};

struct t_imu_config_element imu_config[] = {
		/// Order L3GD20 AN
		/// 1. Write CTRL2
		/// 2. Write CTRL3
		/// 3. Write CTRL4
		/// 4. Write CTRL6
		/// 5. Write Reference
		/// 9. Write CTRL5
		/// 10. Write CTRL1
		{IMU_L3GD20H_ADDR,IMU_L3GD20H_REGISTER_CTRL4,0x80},   // GYR: BDU enabled, BLE Data LSB @ lower address, ± 245 dps Full scale
		{IMU_L3GD20H_ADDR,IMU_L3GD20H_REGISTER_LOW_ODR,0x01}, // GYR: Low Speed ODR enabled (<100Hz)
		{IMU_L3GD20H_ADDR,IMU_L3GD20H_REGISTER_CTRL1,0xFF},   // GYR: ODR 50Hz, cut off 16Hz, Normal mode, Everything enabled

		{IMU_LSM303D_ADDR,IMU_LSM303D_REGISTER_CTRL1,0x5F},   // ACC: 50Hz, BDU enabled, Everything enabled
		{IMU_LSM303D_ADDR,IMU_LSM303D_REGISTER_CTRL2,0xC0},   // ACC: anti alias filter 50Hz, ± 2g

		{IMU_LSM303D_ADDR,IMU_LSM303D_REGISTER_CTRL5,0x70},   // MAG: ODR 50Hz, High Re11solution
		{IMU_LSM303D_ADDR,IMU_LSM303D_REGISTER_CTRL6,0x00},   // MAG: ± 2 gauss
		{IMU_LSM303D_ADDR,IMU_LSM303D_REGISTER_OFFSET_X_L_M,0x00},
		{IMU_LSM303D_ADDR,IMU_LSM303D_REGISTER_CTRL7,0x00}    // MAG: Continious mode

};

/*****************************************************************************/

enum e_data
{
	GYR_X = 0,
	GYR_Y,
	GYR_Z,
	ACC_X,
	ACC_Y,
	ACC_Z,
	MAG_X,
	MAG_Y,
	MAG_Z,
	SENSOR_COUNT
};
static int16_t raw_sensor_data[SENSOR_COUNT] = {0,0,0,0,0,0,0,0,0};
static int16_t raw_sensor_data_min[SENSOR_COUNT] = {0,0,0,0,0,0,-5735,5365,6074};
static int16_t raw_sensor_data_max[SENSOR_COUNT] = {0,0,0,0,0,0,4462,5430,4195};
static int16_t raw_sensor_data_offset[SENSOR_COUNT] = {0,0,0,0,0,0,0,0,0};

int16_t * HAL_Imu_Raw_Sensor_Data_Offset(void)
{
	return raw_sensor_data_offset;
}

/*****************************************************************************/

static float scaled_sensor_data[SENSOR_COUNT];
static const float sensor_scale_factor[SENSOR_COUNT] = {
		245.0/32768.0*M_PI/180, // ± 245dps 16bits => rad/s
		245.0/32768.0*M_PI/180, // ± 245dps => rad/s
		245.0/32768.0*M_PI/180, // ± 245dps => rad/s
		2.0/32768.0, // ± 2g 16bits
		2.0/32768.0, // ± 2g
		2.0/32768.0, // ± 2g
		2.0/32768.0, // ± 2gauss 16bits
		2.0/32768.0, // ± 2gauss
		2.0/32768.0  // ± 2gauss
};
static const float  sensor_sign[SENSOR_COUNT] = {1,1,1,1,1,1,1,1,1};
static const int  sensor_order[SENSOR_COUNT] = {0,1,2,3,4,5,6,7,8};

void scale(void)
{
	for(short index=0;index<SENSOR_COUNT;++index)
	{
		scaled_sensor_data[sensor_order[index]]=(float)(raw_sensor_data[index]-raw_sensor_data_offset[index])*sensor_scale_factor[index]*sensor_sign[index];
	}
}

/*****************************************************************************/

static const int16_t calibration_default_value = 12000;

void HAL_Imu_Begin_Gyr_Calibration()
{
	for(short index=0;index<3;++index)
	{
		raw_sensor_data_min[GYR_X+index]=calibration_default_value;
		raw_sensor_data_max[GYR_X+index]=-calibration_default_value;
	}
}

void HAL_Imu_Do_Gyr_Calibration(void)
{
	for(short index=0;index<3;++index)
	{
		raw_sensor_data_min[GYR_X+index]= MIN(raw_sensor_data_min[GYR_X+index],raw_sensor_data[GYR_X+index]);
		raw_sensor_data_max[GYR_X+index]= MAX(raw_sensor_data_max[GYR_X+index],raw_sensor_data[GYR_X+index]);
	}
}

void HAL_Imu_End_Gyr_Calibration(void)
{
	for(short index=0;index<3;++index)
	{
		raw_sensor_data_offset[GYR_X+index]= (raw_sensor_data_max[GYR_X+index]+raw_sensor_data_min[GYR_X+index])/2;
	}
}

void HAL_Imu_Begin_Mag_Calibration()
{
	for(short index=0;index<3;++index)
	{
		raw_sensor_data_min[MAG_X+index]=calibration_default_value;
		raw_sensor_data_max[MAG_X+index]=-calibration_default_value;
	}
}

void HAL_Imu_Do_Mag_Calibration(void)
{
	for(short index=0;index<3;++index)
	{
		raw_sensor_data_min[MAG_X+index]= MIN(raw_sensor_data_min[MAG_X+index],raw_sensor_data[MAG_X+index]);
		raw_sensor_data_max[MAG_X+index]= MAX(raw_sensor_data_max[MAG_X+index],raw_sensor_data[MAG_X+index]);
	}
}

void HAL_Imu_End_Mag_Calibration(void)
{
	for(short index=0;index<3;++index)
	{
		raw_sensor_data_offset[MAG_X+index]= (raw_sensor_data_max[MAG_X+index]+raw_sensor_data_min[MAG_X+index])/2;
	}
}

/*****************************************************************************/

void HAL_Imu_Process_Failure(void)
{
	while(1);
}

void HAL_Imu_Read_Sensors(void)
{
	HAL_StatusTypeDef result = HAL_OK;
	uint8_t donnee = 0x5A;

	/// query GYR
	result = HAL_I2C_Mem_Read(&hi2c1, IMU_L3GD20H_ADDR, IMU_L3GD20H_REGISTER_STATUS, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 10);
	if(result!=HAL_OK)
	{
		HAL_Imu_Process_Failure();
	}
	if((donnee&0x08)!=0) // GYR: X, Y, Z -axis new data set available.
	{
		result = HAL_I2C_Mem_Read(&hi2c1, IMU_L3GD20H_ADDR, IMU_L3GD20H_REGISTER_OUT_X_L | (1<<7), I2C_MEMADD_SIZE_8BIT, (uint8_t*)&raw_sensor_data[GYR_X], 6, 10);
		if(result!=HAL_OK)
		{
			HAL_Imu_Process_Failure();
		}
	}

	/// query ACC
	result = HAL_I2C_Mem_Read(&hi2c1, IMU_LSM303D_ADDR, IMU_LSM303D_REGISTER_STATUS_A, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 10);
	if(result!=HAL_OK)
	{
		HAL_Imu_Process_Failure();
	}
	if((donnee&0x08)!=0) // GYR: X, Y, Z -axis new data set available.
	{
		result = HAL_I2C_Mem_Read(&hi2c1, IMU_LSM303D_ADDR, IMU_LSM303D_REGISTER_OUT_X_L_A | (1<<7), I2C_MEMADD_SIZE_8BIT, (uint8_t*)&raw_sensor_data[ACC_X], 6, 10);
		if(result!=HAL_OK)
		{
			HAL_Imu_Process_Failure();
		}
	}
	/// query MAG
	result = HAL_I2C_Mem_Read(&hi2c1, IMU_LSM303D_ADDR, IMU_LSM303D_REGISTER_STATUS_M, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 10);
	if(result!=HAL_OK)
	{
		HAL_Imu_Process_Failure();
	}
	if((donnee&0x08)!=0) // GYR: X, Y, Z -axis new data set available.
	{
		result = HAL_I2C_Mem_Read(&hi2c1, IMU_LSM303D_ADDR, IMU_LSM303D_REGISTER_OUT_X_L_M | (1<<7), I2C_MEMADD_SIZE_8BIT, (uint8_t*)&raw_sensor_data[MAG_X], 6, 10);
		if(result!=HAL_OK)
		{
			HAL_Imu_Process_Failure();
		}
	}
	scale();
}

/*****************************************************************************/
enum imu_states
{
	STATE_IMU_NORMAL,
	STATE_IMU_GYR_DRIFT_CALIBRATION_START,
	STATE_IMU_GYR_DRIFT_CALIBRATION,
	STATE_IMU_MAG_CALIBRATION_START,
	STATE_IMU_MAG_CALIBRATION
};

uint32_t state = STATE_IMU_NORMAL;

void HAL_Imu_Gyr_Drift_Calibration(void)
{
	if(state == STATE_IMU_NORMAL)
	{
		state = STATE_IMU_GYR_DRIFT_CALIBRATION_START;
	}

}

void HAL_Imu_Mag_Calibration(void)
{
	if(state == STATE_IMU_NORMAL)
	{
		state = STATE_IMU_MAG_CALIBRATION_START;
	}

}

/*****************************************************************************/

void HAL_Imu_Init(void)
{
	HAL_StatusTypeDef result = HAL_OK;
	uint8_t donnee = 0x5A;

	// wait for IMU boot and check presence/identification
	HAL_Delay(50);
	result = HAL_I2C_Mem_Read(&hi2c1, IMU_L3GD20H_ADDR, IMU_L3GD20H_REGISTER_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 100);
	if(result!=HAL_OK)
	{
		do
		{
			HAL_Delay(500);
			result = HAL_I2C_Mem_Read(&hi2c1, IMU_L3GD20H_ADDR, IMU_L3GD20H_REGISTER_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 100);
		}
		while(result!=HAL_OK);
	}
	if(donnee!=IMU_L3GD20H_WHO_AM_I)
		while(1);
	result = HAL_I2C_Mem_Read(&hi2c1, IMU_LSM303D_ADDR, IMU_LSM303D_REGISTER_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 10);
	if(result!=HAL_OK)
	{
		do
		{
			HAL_Delay(500);
			result = HAL_I2C_Mem_Read(&hi2c1, IMU_LSM303D_ADDR, IMU_LSM303D_REGISTER_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 100);
		}
		while(result!=HAL_OK);
	}
	if(donnee!=IMU_LSM303D_WHO_AM_I)
		while(1);

	/// configure
	int index = 0;
	for(index=0;index<sizeof(imu_config)/sizeof(struct t_imu_config_element);++index)
	{
		result = HAL_I2C_Mem_Write(&hi2c1, imu_config[index].addr, imu_config[index].reg, I2C_MEMADD_SIZE_8BIT, &imu_config[index].value, 1, 100);
		if(result!=HAL_OK)
			while(1);
	}
	for(index=0;index<sizeof(imu_config)/sizeof(struct t_imu_config_element);++index)
	{
		result = HAL_I2C_Mem_Read(&hi2c1, imu_config[index].addr, imu_config[index].reg, I2C_MEMADD_SIZE_8BIT, &donnee, 1, 100);
		if(result!=HAL_OK)
			while(1);
		if(donnee != imu_config[index].value)
			while(1);
	}

	//calculate the initial quaternion
	HAL_Imu_Read_Sensors();
	//these are rough values. This calibration works a lot better if the device is kept as flat as possible
	//find the initial pitch and roll
	pitch = ToDeg(fastAtan2(scaled_sensor_data[ACC_X],sqrt(scaled_sensor_data[ACC_Y] * scaled_sensor_data[ACC_Y] + scaled_sensor_data[ACC_Z] * scaled_sensor_data[ACC_Z])));
	roll = ToDeg(fastAtan2(-1*scaled_sensor_data[ACC_Y],sqrt(scaled_sensor_data[ACC_X] * scaled_sensor_data[ACC_X] + scaled_sensor_data[ACC_Z] * scaled_sensor_data[ACC_Z])));

//	if(scaled_sensor_data[ACC_Z] > 0)
//	{
//		if(scaled_sensor_data[ACC_X] > 0)
//		{
//			pitch = 180.0 - pitch;
//		}
//		else
//		{
//			pitch = -180.0 - pitch;
//		}
//		if(scaled_sensor_data[ACC_Y] > 0)
//		{
//			roll = -180.0 - roll;
//		}
//		else
//		{
//			roll = 180.0 - roll;
//		}
//	}

	float floatMagX = scaled_sensor_data[MAG_X];
	float floatMagY = scaled_sensor_data[MAG_Y];
	float floatMagZ = scaled_sensor_data[MAG_Z];
	//tilt compensate the compass
	float xMag = (floatMagX * cos(ToRad(pitch))) + (floatMagZ * sin(ToRad(pitch)));
	float yMag = -1 * ((floatMagX * sin(ToRad(roll))  * sin(ToRad(pitch))) + (floatMagY * cos(ToRad(roll))) - (floatMagZ * sin(ToRad(roll)) * cos(ToRad(pitch))));

	yaw = ToDeg(fastAtan2(yMag,xMag));

	if(yaw < 0)
	{
		yaw += 360;
	}

	//calculate the rotation matrix
	float cosPitch = cos(ToRad(pitch));
	float sinPitch = sin(ToRad(pitch));

	float cosRoll = cos(ToRad(roll));
	float sinRoll = sin(ToRad(roll));

	float cosYaw = cos(ToRad(yaw));
	float sinYaw = sin(ToRad(yaw));

	//need the transpose of the rotation matrix
	float r11 = cosPitch * cosYaw;
	float r21 = cosPitch * sinYaw;
	float r31 = -1.0 * sinPitch;

	float r12 = -1.0 * (cosRoll * sinYaw) + (sinRoll * sinPitch * cosYaw);
	float r22 = (cosRoll * cosYaw) + (sinRoll * sinPitch * sinYaw);
	float r32 = sinRoll * cosPitch;

	float r13 = (sinRoll * sinYaw) + (cosRoll * sinPitch * cosYaw);
	float r23 = -1.0 * (sinRoll * cosYaw) + (cosRoll * sinPitch * sinYaw);
	float r33 = cosRoll * cosPitch;

	//convert to quaternion
	q0 = 0.5 * sqrt(1 + r11 + r22 + r33);
	q1 = (r32 - r23)/(4 * q0);
	q2 = (r13 - r31)/(4 * q0);
	q3 = (r21 - r12)/(4 * q0);

	MadgwickAHRSupdate(
			scaled_sensor_data[GYR_X],
			scaled_sensor_data[GYR_Y],
			scaled_sensor_data[GYR_Z],
			scaled_sensor_data[ACC_X],
			scaled_sensor_data[ACC_Y],
			scaled_sensor_data[ACC_Z],
			scaled_sensor_data[MAG_X],
			scaled_sensor_data[MAG_Y],
			scaled_sensor_data[MAG_Z]);
	GetEuler();
}

uint32_t calibration_iteration = 0;

void HAL_Imu_Process(void)
{
	static uint32_t last_time = 0;
	HAL_Imu_Read_Sensors();
	uint32_t current_time = HAL_GetTick();
	if(current_time>=last_time+20)
	{
		last_time = current_time;
		switch(state)
		{
		case STATE_IMU_NORMAL:
			{
				MadgwickAHRSupdate(
						scaled_sensor_data[GYR_X],
						scaled_sensor_data[GYR_Y],
						scaled_sensor_data[GYR_Z],
						scaled_sensor_data[ACC_X],
						scaled_sensor_data[ACC_Y],
						scaled_sensor_data[ACC_Z],
						scaled_sensor_data[MAG_X],
						scaled_sensor_data[MAG_Y],
						scaled_sensor_data[MAG_Z]);
				GetEuler();
			}
			break;
		case STATE_IMU_GYR_DRIFT_CALIBRATION_START:
			{
				HAL_Imu_Begin_Gyr_Calibration();
				calibration_iteration = 0;
				state = STATE_IMU_GYR_DRIFT_CALIBRATION;
			}
			break;
		case STATE_IMU_GYR_DRIFT_CALIBRATION:
			{
				HAL_Imu_Do_Gyr_Calibration();
				++calibration_iteration;
				if(calibration_iteration>=50)
				{
					HAL_Imu_End_Gyr_Calibration();
					state = STATE_IMU_NORMAL;
				}
			}
			break;
		case STATE_IMU_MAG_CALIBRATION_START:
			{
				HAL_Imu_Begin_Mag_Calibration();
				calibration_iteration = 0;
				state = STATE_IMU_MAG_CALIBRATION;
			}
			break;
		case STATE_IMU_MAG_CALIBRATION:
			{
				HAL_Imu_Do_Mag_Calibration();
				++calibration_iteration;
				if(calibration_iteration>=600)
				{
					HAL_Imu_End_Mag_Calibration();
					state = STATE_IMU_NORMAL;
				}
			}
			break;
		}
	}
}
