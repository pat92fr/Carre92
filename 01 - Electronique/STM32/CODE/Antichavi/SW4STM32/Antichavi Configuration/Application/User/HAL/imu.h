/*
 * imu.h
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#ifndef APPLICATION_USER_HAL_IMU_H_
#define APPLICATION_USER_HAL_IMU_H_

#include "stm32f4xx_hal.h"

void HAL_Imu_Init(void);
void HAL_Imu_Process(void);

float HAL_Imu_Pitch(void);
float HAL_Imu_Roll(void);
float HAL_Imu_Yaw(void);

enum
{
	IMU_ANGLE_PITCH,
	IMU_ANGLE_ROLL,
	IMU_ANGLE_YAW,
	IMU_ANGLE_COUNT
};

float HAL_Imu_Angle(int angle);

int16_t * HAL_Imu_Raw_Sensor_Data_Offset(void);

void HAL_Imu_Gyr_Drift_Calibration(void);
void HAL_Imu_Mag_Calibration(void);

#endif /* APPLICATION_USER_HAL_IMU_H_ */
