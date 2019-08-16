/*
 * threshold.h
 *
 *  Created on: 3 janv. 2016
 *      Author: Patrick
 */

#ifndef APPLICATION_USER_APP_THRESHOLD_H_
#define APPLICATION_USER_APP_THRESHOLD_H_

void HAL_Threshold_Init(void);
void HAL_Threshold_Process(void);

enum
{
	THRESHOLD_ANGLE_PITCH,
	THRESHOLD_ANGLE_ROLL,
	THRESHOLD_ANGLE_COUNT
};

float HAL_Threshold_Set(int axe);
void HAL_Threshold_Get(int axe, float angle);

enum
{
	THRESHOLD_STATUS_OK = 0,
	THRESHOLD_STATUS_ALERT = 1
};

int HAL_Threshold_Status(int axe);

#endif /* APPLICATION_USER_APP_THRESHOLD_H_ */
